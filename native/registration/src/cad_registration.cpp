#include "cad_registration.h"

#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>

#include <Eigen/Eigenvalues>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

namespace {

using Cloud = pcl::PointCloud<pcl::PointXYZ>;
using FeatureCloud = pcl::PointCloud<pcl::FPFHSignature33>;
using SearchTree = pcl::search::KdTree<pcl::PointXYZ>;
using Clock = std::chrono::steady_clock;

struct Candidate {
  CadRegStrategy strategy = CADREG_STRATEGY_INITIAL;
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  double rmse = std::numeric_limits<double>::infinity();
  double inlier_ratio = 0.0;
  double target_coverage = 0.0;
  double score = 0.0;
  bool converged = false;
  bool accepted = false;
  double shared_coarse_ms = 0.0;
  double refinement_ms = 0.0;
  double quality_ms = 0.0;
};

struct RankedSeed {
  Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
  double score = 0.0;
};

void SetIdentity(double matrix[16]) {
  std::fill(matrix, matrix + 16, 0.0);
  matrix[0] = matrix[5] = matrix[10] = matrix[15] = 1.0;
}

void SetError(CadRegResult *result, CadRegStatus status, const char *message) {
  result->status = status;
  std::snprintf(result->error_message, CADREG_ERROR_MESSAGE_SIZE, "%s",
                message);
}

bool IsValidView(const CadRegPointCloud *view) {
  if (!view || !view->xyz || view->point_count < 3)
    return false;
  const uint64_t stride = view->xyz_stride_bytes == 0 ? 3u * sizeof(double)
                                                      : view->xyz_stride_bytes;
  return stride >= 3u * sizeof(double);
}

Cloud::Ptr CopyCloud(const CadRegPointCloud &view) {
  Cloud::Ptr cloud(new Cloud);
  cloud->reserve(static_cast<size_t>(view.point_count));
  const uint64_t stride =
      view.xyz_stride_bytes == 0 ? 3u * sizeof(double) : view.xyz_stride_bytes;
  const unsigned char *bytes =
      reinterpret_cast<const unsigned char *>(view.xyz);
  for (uint64_t i = 0; i < view.point_count; ++i) {
    double xyz[3];
    std::memcpy(xyz, bytes + i * stride, sizeof(xyz));
    if (std::isfinite(xyz[0]) && std::isfinite(xyz[1]) &&
        std::isfinite(xyz[2])) {
      cloud->push_back(pcl::PointXYZ(static_cast<float>(xyz[0]),
                                     static_cast<float>(xyz[1]),
                                     static_cast<float>(xyz[2])));
    }
  }
  cloud->width = static_cast<uint32_t>(cloud->size());
  cloud->height = 1;
  cloud->is_dense = true;
  return cloud;
}

Cloud::Ptr Downsample(const Cloud::Ptr &input, double voxel) {
  if (voxel <= 0.0)
    return input;
  pcl::VoxelGrid<pcl::PointXYZ> filter;
  filter.setInputCloud(input);
  const float leaf = static_cast<float>(voxel);
  filter.setLeafSize(leaf, leaf, leaf);
  Cloud::Ptr output(new Cloud);
  filter.filter(*output);
  return output;
}

Eigen::Matrix4f ReadMatrix(const double *values) {
  Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();
  if (!values)
    return matrix;
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col)
      matrix(row, col) = static_cast<float>(values[row * 4 + col]);
  return matrix;
}

bool IsFiniteTransform(const Eigen::Matrix4f &matrix) {
  return matrix.allFinite() &&
         (matrix.row(3) - Eigen::RowVector4f(0, 0, 0, 1)).norm() < 1e-4f;
}

void WriteMatrix(const Eigen::Matrix4f &matrix, double values[16]) {
  for (int row = 0; row < 4; ++row)
    for (int col = 0; col < 4; ++col)
      values[row * 4 + col] = static_cast<double>(matrix(row, col));
}

void CalculateQuality(const Cloud &aligned, const Cloud::ConstPtr &target,
                      pcl::KdTreeFLANN<pcl::PointXYZ> *target_tree,
                      const CadRegOptions &options, Candidate *candidate) {
  const double max_squared =
      options.max_correspondence_distance * options.max_correspondence_distance;
  double squared_sum = 0.0;
  uint64_t inliers = 0;
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  for (const pcl::PointXYZ &point : aligned) {
    if (target_tree->nearestKSearch(point, 1, indices, distances) > 0 &&
        distances[0] <= max_squared) {
      squared_sum += distances[0];
      ++inliers;
    }
  }
  candidate->inlier_ratio =
      aligned.empty() ? 0.0 : static_cast<double>(inliers) / aligned.size();
  candidate->rmse = inliers == 0
                        ? std::numeric_limits<double>::infinity()
                        : std::sqrt(squared_sum / static_cast<double>(inliers));
  if (options.enable_target_coverage != 0) {
    Cloud::Ptr aligned_ptr(new Cloud(aligned));
    pcl::KdTreeFLANN<pcl::PointXYZ> source_tree;
    source_tree.setInputCloud(aligned_ptr);
    uint64_t covered = 0;
    for (const pcl::PointXYZ &point : *target) {
      if (source_tree.nearestKSearch(point, 1, indices, distances) > 0 &&
          distances[0] <= max_squared)
        ++covered;
    }
    candidate->target_coverage =
        target->empty() ? 0.0 : static_cast<double>(covered) / target->size();
  }
  const double rmse_part =
      std::isfinite(candidate->rmse)
          ? std::max(0.0, 1.0 - candidate->rmse / options.max_rmse)
          : 0.0;
  candidate->score = options.enable_target_coverage != 0
                         ? 0.70 * candidate->inlier_ratio + 0.25 * rmse_part +
                               0.05 * candidate->target_coverage
                         : 0.75 * candidate->inlier_ratio + 0.25 * rmse_part;
  candidate->accepted =
      candidate->converged &&
      candidate->inlier_ratio >= options.min_inlier_ratio &&
      candidate->rmse <= options.max_rmse &&
      (options.enable_target_coverage == 0 ||
       candidate->target_coverage >= options.min_target_coverage);
}

Candidate Refine(const Cloud::Ptr &source, const Cloud::Ptr &target,
                 const Eigen::Matrix4f &initial, CadRegStrategy strategy,
                 const CadRegOptions &options,
                 const std::vector<Cloud::Ptr> &level_sources,
                 const std::vector<Cloud::Ptr> &level_targets,
                 const std::vector<SearchTree::Ptr> &level_target_trees,
                 pcl::KdTreeFLANN<pcl::PointXYZ> *target_tree) {
  Candidate candidate;
  const Clock::time_point refinement_started = Clock::now();
  candidate.strategy = strategy;
  candidate.transform = initial;
  candidate.converged = true;
  const uint32_t level_count =
      options.icp_level_count == 0 ? 1 : options.icp_level_count;
  for (uint32_t level_index = 0; level_index < level_count; ++level_index) {
    CadRegIcpLevel level{};
    if (options.icp_level_count == 0) {
      level.voxel_size = options.voxel_size;
      level.max_correspondence_distance = options.max_correspondence_distance;
      level.max_iterations = options.max_iterations;
    } else {
      level = options.icp_levels[level_index];
    }
    Cloud::Ptr level_source = level_sources[level_index];
    Cloud::Ptr level_target = level_targets[level_index];
    if (level_source->size() < 3 || level_target->size() < 3) {
      candidate.converged = false;
      break;
    }
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
    icp.setInputSource(level_source);
    icp.setInputTarget(level_target);
    icp.setSearchMethodTarget(level_target_trees[level_index], true);
    icp.setMaxCorrespondenceDistance(level.max_correspondence_distance);
    icp.setMaximumIterations(static_cast<int>(std::min<uint32_t>(
        level.max_iterations,
        static_cast<uint32_t>(std::numeric_limits<int>::max()))));
    icp.setTransformationEpsilon(1e-8);
    icp.setEuclideanFitnessEpsilon(1e-7);
    Cloud aligned_level;
    icp.align(aligned_level, candidate.transform);
    candidate.transform = icp.getFinalTransformation();
    if (!icp.hasConverged()) {
      candidate.converged = false;
      break;
    }
  }
  Cloud aligned;
  pcl::transformPointCloud(*source, aligned, candidate.transform);
  candidate.refinement_ms = std::chrono::duration<double, std::milli>(
                                Clock::now() - refinement_started)
                                .count();
  const Clock::time_point quality_started = Clock::now();
  CalculateQuality(aligned, target, target_tree, options, &candidate);
  candidate.quality_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - quality_started)
          .count();
  return candidate;
}

double ScoreSeed(const Cloud::Ptr &source, const Eigen::Matrix4f &transform,
                 pcl::KdTreeFLANN<pcl::PointXYZ> *target_tree,
                 double max_distance) {
  if (source->empty())
    return 0.0;
  const float maximum_squared = static_cast<float>(max_distance * max_distance);
  uint64_t inliers = 0;
  double squared_sum = 0.0;
  std::vector<int> indices(1);
  std::vector<float> distances(1);
  for (const pcl::PointXYZ &point : *source) {
    const Eigen::Vector4f input(point.x, point.y, point.z, 1.0f);
    const Eigen::Vector4f output = transform * input;
    const pcl::PointXYZ transformed(output.x(), output.y(), output.z());
    if (target_tree->nearestKSearch(transformed, 1, indices, distances) > 0 &&
        distances[0] <= maximum_squared) {
      ++inliers;
      squared_sum += distances[0];
    }
  }
  if (inliers == 0)
    return 0.0;
  const double ratio =
      static_cast<double>(inliers) / static_cast<double>(source->size());
  const double rmse = std::sqrt(squared_sum / static_cast<double>(inliers));
  return 0.8 * ratio + 0.2 * std::max(0.0, 1.0 - rmse / max_distance);
}

std::vector<Eigen::Matrix4f> PcaTransforms(const Cloud &source,
                                           const Cloud &target) {
  auto frame = [](const Cloud &cloud, Eigen::Vector3f *center,
                  Eigen::Matrix3f *axes) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(cloud, centroid);
    *center = centroid.head<3>();
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(cloud, centroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);
    if (solver.info() != Eigen::Success)
      throw std::runtime_error("PCA eigensolver failed");
    axes->col(0) = solver.eigenvectors().col(2);
    axes->col(1) = solver.eigenvectors().col(1);
    axes->col(2) = solver.eigenvectors().col(0);
    if (axes->determinant() < 0.0f)
      axes->col(2) *= -1.0f;
  };
  Eigen::Vector3f source_center, target_center;
  Eigen::Matrix3f source_axes, target_axes;
  frame(source, &source_center, &source_axes);
  frame(target, &target_center, &target_axes);
  std::vector<Eigen::Matrix4f> output;
  for (int sx : {-1, 1})
    for (int sy : {-1, 1})
      for (int sz : {-1, 1}) {
        Eigen::Matrix3f signs = Eigen::Matrix3f::Zero();
        signs.diagonal() << static_cast<float>(sx), static_cast<float>(sy),
            static_cast<float>(sz);
        const Eigen::Matrix3f rotation =
            target_axes * signs * source_axes.transpose();
        if (rotation.determinant() < 0.0f)
          continue;
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
        transform.block<3, 3>(0, 0) = rotation;
        transform.block<3, 1>(0, 3) = target_center - rotation * source_center;
        output.push_back(transform);
      }
  return output;
}

FeatureCloud::Ptr ComputeFeatures(const Cloud::Ptr &cloud, double voxel) {
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimation;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  normal_estimation.setInputCloud(cloud);
  normal_estimation.setSearchMethod(tree);
  normal_estimation.setRadiusSearch(std::max(voxel * 2.5, 1e-6));
  normal_estimation.compute(*normals);
  FeatureCloud::Ptr features(new FeatureCloud);
  pcl::FPFHEstimation<pcl::PointXYZ, pcl::Normal, pcl::FPFHSignature33> fpfh;
  fpfh.setInputCloud(cloud);
  fpfh.setInputNormals(normals);
  fpfh.setSearchMethod(tree);
  fpfh.setRadiusSearch(std::max(voxel * 5.0, 1e-6));
  fpfh.compute(*features);
  return features;
}

std::vector<Eigen::Matrix4f>
FpfhTransforms(const Cloud::Ptr &source_input, const Cloud::Ptr &target_input,
               const Cloud::Ptr &cached_target_feature_cloud,
               const FeatureCloud::Ptr &cached_target_features,
               const CadRegOptions &options) {
  Cloud::Ptr source = Downsample(source_input, options.feature_voxel_size);
  Cloud::Ptr target =
      cached_target_feature_cloud
          ? cached_target_feature_cloud
          : Downsample(target_input, options.feature_voxel_size);
  std::vector<Eigen::Matrix4f> transforms;
  if (source->size() < 10 || target->size() < 10)
    return transforms;
  FeatureCloud::Ptr source_features =
      ComputeFeatures(source, options.feature_voxel_size);
  FeatureCloud::Ptr target_features =
      cached_target_features
          ? cached_target_features
          : ComputeFeatures(target, options.feature_voxel_size);
  for (uint32_t attempt = 0; attempt < options.ransac_attempts; ++attempt) {
    pcl::SampleConsensusPrerejective<pcl::PointXYZ, pcl::PointXYZ,
                                     pcl::FPFHSignature33>
        align;
    align.setInputSource(source);
    align.setSourceFeatures(source_features);
    align.setInputTarget(target);
    align.setTargetFeatures(target_features);
    align.setMaximumIterations(options.ransac_max_iterations);
    align.setNumberOfSamples(3);
    align.setCorrespondenceRandomness(8);
    align.setSimilarityThreshold(0.85f);
    const double coarse_distance =
        options.icp_level_count > 0
            ? options.icp_levels[0].max_correspondence_distance
            : options.max_correspondence_distance;
    align.setMaxCorrespondenceDistance(static_cast<float>(coarse_distance));
    align.setInlierFraction(
        static_cast<float>(std::min(options.min_inlier_ratio, 0.20)));
    Cloud aligned;
    align.align(aligned);
    const Eigen::Matrix4f transform = align.getFinalTransformation();
    if (align.hasConverged() && IsFiniteTransform(transform))
      transforms.push_back(transform);
    if (transforms.size() >= options.max_candidates_per_strategy)
      break;
  }
  return transforms;
}

void SortCandidates(std::vector<Candidate> *candidates) {
  std::sort(candidates->begin(), candidates->end(),
            [](const Candidate &left, const Candidate &right) {
              if (left.accepted != right.accepted)
                return left.accepted > right.accepted;
              return left.score > right.score;
            });
}

void DeduplicateCandidates(std::vector<Candidate> *candidates, double voxel) {
  SortCandidates(candidates);
  std::vector<Candidate> unique;
  const float translation_tolerance =
      static_cast<float>(std::max(voxel * 0.25, 1e-3));
  for (const Candidate &candidate : *candidates) {
    bool duplicate = false;
    for (const Candidate &accepted : unique) {
      const float translation = (candidate.transform.block<3, 1>(0, 3) -
                                 accepted.transform.block<3, 1>(0, 3))
                                    .norm();
      const Eigen::Matrix3f relative =
          candidate.transform.block<3, 3>(0, 0).transpose() *
          accepted.transform.block<3, 3>(0, 0);
      const float cosine =
          std::max(-1.0f, std::min(1.0f, (relative.trace() - 1.0f) * 0.5f));
      const float angle_degrees = std::acos(cosine) * 57.29577951308232f;
      if (translation <= translation_tolerance && angle_degrees <= 0.5f) {
        duplicate = true;
        break;
      }
    }
    if (!duplicate)
      unique.push_back(candidate);
  }
  candidates->swap(unique);
}

bool ValidateOptions(const CadRegOptions *options) {
  if (!options || options->struct_size != sizeof(CadRegOptions) ||
      options->abi_version != CADREG_ABI_VERSION ||
      options->mode < CADREG_MODE_SINGLE ||
      options->mode > CADREG_MODE_ENSEMBLE || options->strategy_mask == 0 ||
      (options->strategy_mask &
       ~(CADREG_STRATEGY_INITIAL | CADREG_STRATEGY_PCA |
         CADREG_STRATEGY_FPFH_RANSAC)) != 0 ||
      options->max_correspondence_distance <= 0.0 ||
      options->max_iterations == 0 || options->max_rmse <= 0.0 ||
      !std::isfinite(options->max_correspondence_distance) ||
      !std::isfinite(options->voxel_size) ||
      !std::isfinite(options->max_rmse) ||
      !std::isfinite(options->min_inlier_ratio) ||
      !std::isfinite(options->ambiguity_score_margin) ||
      options->min_inlier_ratio < 0.0 || options->min_inlier_ratio > 1.0 ||
      options->ambiguity_score_margin < 0.0 ||
      options->icp_level_count > CADREG_MAX_ICP_LEVELS ||
      options->ransac_attempts == 0 ||
      options->max_candidates_per_strategy == 0 ||
      options->max_refined_candidates_per_strategy == 0 ||
      options->max_refined_candidates_per_strategy >
          options->max_candidates_per_strategy ||
      options->min_target_coverage < 0.0 || options->min_target_coverage > 1.0)
    return false;
  for (uint32_t i = 0; i < options->icp_level_count; ++i) {
    const CadRegIcpLevel &level = options->icp_levels[i];
    if (level.voxel_size < 0.0 || !(level.max_correspondence_distance > 0.0) ||
        level.max_iterations == 0 || !std::isfinite(level.voxel_size) ||
        !std::isfinite(level.max_correspondence_distance))
      return false;
  }
  if ((options->strategy_mask & CADREG_STRATEGY_FPFH_RANSAC) != 0 &&
      (!(options->feature_voxel_size > 0.0) ||
       !std::isfinite(options->feature_voxel_size) ||
       options->ransac_max_iterations == 0))
    return false;
  return true;
}

CadRegStatus
RegisterPrepared(const Cloud::Ptr &source, const Cloud::Ptr &target,
                 const Eigen::Matrix4f &initial, const CadRegOptions &options,
                 const std::vector<Cloud::Ptr> *cached_target_levels,
                 const std::vector<SearchTree::Ptr> *cached_target_level_trees,
                 const Cloud::Ptr &cached_target_feature_cloud,
                 const FeatureCloud::Ptr &cached_target_features,
                 CadRegResult *result) {
  std::vector<Candidate> candidates;
  const uint32_t level_count =
      options.icp_level_count == 0 ? 1 : options.icp_level_count;
  std::vector<Cloud::Ptr> level_sources;
  std::vector<Cloud::Ptr> level_targets;
  std::vector<SearchTree::Ptr> local_target_level_trees;
  level_sources.reserve(level_count);
  level_targets.reserve(level_count);
  for (uint32_t i = 0; i < level_count; ++i) {
    const double voxel = options.icp_level_count == 0
                             ? options.voxel_size
                             : options.icp_levels[i].voxel_size;
    level_sources.push_back(voxel == options.voxel_size
                                ? source
                                : Downsample(source, voxel));
    if (cached_target_levels && cached_target_levels->size() == level_count)
      level_targets.push_back((*cached_target_levels)[i]);
    else
      level_targets.push_back(Downsample(target, voxel));
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> target_tree;
  target_tree.setInputCloud(target);
  const std::vector<SearchTree::Ptr> *level_target_trees =
      cached_target_level_trees;
  if (!level_target_trees || level_target_trees->size() != level_count) {
    local_target_level_trees.reserve(level_count);
    for (const Cloud::Ptr &level_target : level_targets) {
      SearchTree::Ptr tree(new SearchTree);
      tree->setInputCloud(level_target);
      local_target_level_trees.push_back(tree);
    }
    level_target_trees = &local_target_level_trees;
  }
  pcl::KdTreeFLANN<pcl::PointXYZ> coarse_target_tree;
  coarse_target_tree.setInputCloud(level_targets.front());
  const CadRegStrategy ordered[] = {CADREG_STRATEGY_INITIAL,
                                    CADREG_STRATEGY_PCA,
                                    CADREG_STRATEGY_FPFH_RANSAC};
  for (CadRegStrategy strategy : ordered) {
    if ((options.strategy_mask & static_cast<uint32_t>(strategy)) == 0)
      continue;
    const Clock::time_point coarse_started = Clock::now();
    std::vector<Eigen::Matrix4f> seeds;
    if (strategy == CADREG_STRATEGY_INITIAL) {
      seeds.push_back(initial);
    } else if (strategy == CADREG_STRATEGY_PCA) {
      seeds = PcaTransforms(*source, *target);
    } else {
      seeds = FpfhTransforms(source, target, cached_target_feature_cloud,
                             cached_target_features, options);
    }
    const double strategy_coarse_ms =
        std::chrono::duration<double, std::milli>(Clock::now() - coarse_started)
            .count();
    result->coarse_registration_ms += strategy_coarse_ms;
    if (seeds.size() > options.max_candidates_per_strategy)
      seeds.resize(options.max_candidates_per_strategy);
    std::vector<RankedSeed> ranked_seeds;
    ranked_seeds.reserve(seeds.size());
    const double seed_distance =
        options.icp_level_count == 0
            ? options.max_correspondence_distance
            : options.icp_levels[0].max_correspondence_distance;
    for (const Eigen::Matrix4f &seed : seeds) {
      ranked_seeds.push_back(
          {seed, ScoreSeed(level_sources.front(), seed, &coarse_target_tree,
                           seed_distance)});
    }
    std::sort(ranked_seeds.begin(), ranked_seeds.end(),
              [](const RankedSeed &left, const RankedSeed &right) {
                return left.score > right.score;
              });
    if (ranked_seeds.size() > options.max_refined_candidates_per_strategy)
      ranked_seeds.resize(options.max_refined_candidates_per_strategy);
    for (const RankedSeed &ranked_seed : ranked_seeds) {
      Candidate candidate =
          Refine(source, target, ranked_seed.transform, strategy, options,
                 level_sources, level_targets, *level_target_trees,
                 &target_tree);
      candidate.shared_coarse_ms = strategy_coarse_ms;
      result->refinement_ms += candidate.refinement_ms;
      result->quality_ms += candidate.quality_ms;
      candidates.push_back(std::move(candidate));
    }
    SortCandidates(&candidates);
    const bool accepted = !candidates.empty() && candidates.front().accepted;
    if (options.mode == CADREG_MODE_SINGLE ||
        (options.mode == CADREG_MODE_CASCADE && accepted))
      break;
  }
  DeduplicateCandidates(&candidates, options.voxel_size);
  result->candidate_count = static_cast<uint32_t>(candidates.size());
  result->accepted_candidate_count = static_cast<uint32_t>(std::count_if(
      candidates.begin(), candidates.end(),
      [](const Candidate &candidate) { return candidate.accepted; }));
  result->diagnostic_count = static_cast<uint32_t>(
      std::min<size_t>(candidates.size(), CADREG_MAX_CANDIDATE_DIAGNOSTICS));
  for (uint32_t index = 0; index < result->diagnostic_count; ++index) {
    const Candidate &candidate = candidates[index];
    CadRegCandidateDiagnostic &diagnostic = result->diagnostics[index];
    diagnostic.strategy = candidate.strategy;
    diagnostic.converged = candidate.converged ? 1u : 0u;
    diagnostic.accepted = candidate.accepted ? 1u : 0u;
    diagnostic.rank = index + 1;
    WriteMatrix(candidate.transform, diagnostic.source_to_target);
    diagnostic.rmse = candidate.rmse;
    diagnostic.inlier_ratio = candidate.inlier_ratio;
    diagnostic.target_coverage = candidate.target_coverage;
    diagnostic.score = candidate.score;
    diagnostic.shared_coarse_ms = candidate.shared_coarse_ms;
    diagnostic.refinement_ms = candidate.refinement_ms;
    diagnostic.quality_ms = candidate.quality_ms;
    diagnostic.candidate_elapsed_ms =
        candidate.refinement_ms + candidate.quality_ms;
  }
  if (candidates.empty()) {
    SetError(result, CADREG_STATUS_NOT_CONVERGED,
             "no strategy produced a registration candidate");
    return result->status;
  }
  const Candidate &best = candidates.front();
  result->converged = best.converged ? 1u : 0u;
  result->selected_strategy = best.strategy;
  result->rmse = best.rmse;
  result->inlier_ratio = best.inlier_ratio;
  result->target_coverage = best.target_coverage;
  result->score = best.score;
  result->second_best_score = candidates.size() > 1 ? candidates[1].score : 0.0;
  WriteMatrix(best.transform, result->source_to_target);
  if (!best.converged) {
    SetError(result, CADREG_STATUS_NOT_CONVERGED,
             "best candidate did not converge");
  } else if (!best.accepted) {
    SetError(result, CADREG_STATUS_QUALITY_REJECTED,
             "best candidate failed the quality gate");
  } else if (options.mode == CADREG_MODE_ENSEMBLE && candidates.size() > 1 &&
             candidates[1].accepted &&
             best.score - candidates[1].score <
                 options.ambiguity_score_margin) {
    SetError(result, CADREG_STATUS_AMBIGUOUS,
             "top candidates are too close to distinguish");
  } else {
    result->status = CADREG_STATUS_SUCCESS;
  }
  return result->status;
}

} // namespace

struct CadRegHandleImpl {
  Cloud::Ptr target_raw;
  Cloud::Ptr target_prepared;
  double prepared_voxel = std::numeric_limits<double>::quiet_NaN();
  std::vector<double> level_voxels;
  std::vector<Cloud::Ptr> target_levels;
  std::vector<SearchTree::Ptr> target_level_trees;
  double feature_voxel = std::numeric_limits<double>::quiet_NaN();
  Cloud::Ptr target_feature_cloud;
  FeatureCloud::Ptr target_features;
  std::string last_error;
};

extern "C" {

uint32_t CADREG_CALL cadreg_get_abi_version(void) { return CADREG_ABI_VERSION; }

void CADREG_CALL cadreg_default_options(CadRegOptions *options) {
  if (!options)
    return;
  *options = {};
  options->struct_size = sizeof(CadRegOptions);
  options->abi_version = CADREG_ABI_VERSION;
  options->max_correspondence_distance = 5.0;
  options->max_iterations = 100;
  options->voxel_size = 1.0;
  options->mode = CADREG_MODE_CASCADE;
  options->strategy_mask = CADREG_STRATEGY_INITIAL | CADREG_STRATEGY_PCA |
                           CADREG_STRATEGY_FPFH_RANSAC;
  options->feature_voxel_size = 5.0;
  options->ransac_max_iterations = 50000;
  options->min_inlier_ratio = 0.60;
  options->max_rmse = 2.0;
  options->ambiguity_score_margin = 0.03;
  options->icp_level_count = 3;
  options->icp_levels[0] = {5.0, 30.0, 40, 0};
  options->icp_levels[1] = {2.0, 10.0, 60, 0};
  options->icp_levels[2] = {0.8, 2.5, 80, 0};
  options->ransac_attempts = 4;
  options->max_candidates_per_strategy = 4;
  options->max_refined_candidates_per_strategy = 2;
  options->enable_target_coverage = 0;
  options->min_target_coverage = 0.20;
}

void CADREG_CALL cadreg_init_result(CadRegResult *result) {
  if (!result)
    return;
  *result = {};
  result->struct_size = sizeof(CadRegResult);
  result->abi_version = CADREG_ABI_VERSION;
  result->status = CADREG_STATUS_INTERNAL_ERROR;
  SetIdentity(result->source_to_target);
  result->rmse = std::numeric_limits<double>::infinity();
}

CadRegHandle CADREG_CALL cadreg_create(void) {
  try {
    return new CadRegHandleImpl;
  } catch (...) {
    return nullptr;
  }
}

void CADREG_CALL cadreg_destroy(CadRegHandle handle) { delete handle; }

CadRegStatus CADREG_CALL cadreg_set_target(CadRegHandle handle,
                                           const CadRegPointCloud *target) {
  if (!handle || !IsValidView(target))
    return CADREG_STATUS_INVALID_ARGUMENT;
  try {
    Cloud::Ptr copied = CopyCloud(*target);
    if (copied->size() < 3) {
      handle->last_error = "target contains fewer than three finite points";
      return CADREG_STATUS_INVALID_ARGUMENT;
    }
    handle->target_raw = copied;
    handle->target_prepared.reset();
    handle->level_voxels.clear();
    handle->target_levels.clear();
    handle->target_level_trees.clear();
    handle->target_feature_cloud.reset();
    handle->target_features.reset();
    handle->feature_voxel = std::numeric_limits<double>::quiet_NaN();
    handle->prepared_voxel = std::numeric_limits<double>::quiet_NaN();
    handle->last_error.clear();
    return CADREG_STATUS_SUCCESS;
  } catch (const std::exception &error) {
    handle->last_error = error.what();
    return CADREG_STATUS_INTERNAL_ERROR;
  } catch (...) {
    handle->last_error = "unknown target preparation error";
    return CADREG_STATUS_INTERNAL_ERROR;
  }
}

CadRegStatus CADREG_CALL
cadreg_register_source(CadRegHandle handle, const CadRegPointCloud *source_view,
                       const double *initial_values,
                       const CadRegOptions *options, CadRegResult *result) {
  const Clock::time_point started = Clock::now();
  if (!result || result->struct_size != sizeof(CadRegResult) ||
      result->abi_version != CADREG_ABI_VERSION)
    return CADREG_STATUS_INVALID_ARGUMENT;
  cadreg_init_result(result);
  try {
    if (!handle || !handle->target_raw) {
      SetError(result, CADREG_STATUS_INVALID_ARGUMENT,
               "set a target point cloud before registration");
    } else if (!IsValidView(source_view)) {
      SetError(result, CADREG_STATUS_INVALID_ARGUMENT,
               "source requires at least three XYZ points");
    } else if (!ValidateOptions(options)) {
      SetError(result, CADREG_STATUS_INVALID_ARGUMENT,
               "invalid options, mode, strategy mask, or quality gate");
    } else {
      const Eigen::Matrix4f initial = ReadMatrix(initial_values);
      if (!IsFiniteTransform(initial)) {
        SetError(result, CADREG_STATUS_INVALID_ARGUMENT,
                 "initial matrix is not a finite 4x4 transform");
      } else {
        const Clock::time_point preprocessing_started = Clock::now();
        Cloud::Ptr source =
            Downsample(CopyCloud(*source_view), options->voxel_size);
        Cloud::Ptr target;
        if (handle->target_prepared &&
            handle->prepared_voxel == options->voxel_size) {
          target = handle->target_prepared;
          result->target_cache_hit = 1;
        } else {
          target = Downsample(handle->target_raw, options->voxel_size);
          handle->target_prepared = target;
          handle->prepared_voxel = options->voxel_size;
          handle->level_voxels.clear();
          handle->target_levels.clear();
          handle->target_level_trees.clear();
        }
        const uint32_t level_count =
            options->icp_level_count == 0 ? 1 : options->icp_level_count;
        std::vector<double> desired_voxels;
        desired_voxels.reserve(level_count);
        for (uint32_t i = 0; i < level_count; ++i)
          desired_voxels.push_back(options->icp_level_count == 0
                                       ? options->voxel_size
                                       : options->icp_levels[i].voxel_size);
        if (handle->level_voxels != desired_voxels ||
            handle->target_levels.size() != level_count) {
          handle->target_levels.clear();
          handle->target_level_trees.clear();
          handle->target_levels.reserve(level_count);
          for (double voxel : desired_voxels)
            handle->target_levels.push_back(
                voxel == options->voxel_size ? target
                                              : Downsample(target, voxel));
          handle->target_level_trees.reserve(level_count);
          for (const Cloud::Ptr &level_target : handle->target_levels) {
            SearchTree::Ptr tree(new SearchTree);
            tree->setInputCloud(level_target);
            handle->target_level_trees.push_back(tree);
          }
          handle->level_voxels = desired_voxels;
        } else {
          result->target_cache_hit = 1;
        }
        if ((options->strategy_mask & CADREG_STRATEGY_FPFH_RANSAC) != 0 &&
            (!handle->target_feature_cloud ||
             handle->feature_voxel != options->feature_voxel_size)) {
          handle->target_feature_cloud =
              Downsample(handle->target_raw, options->feature_voxel_size);
          handle->target_features = ComputeFeatures(
              handle->target_feature_cloud, options->feature_voxel_size);
          handle->feature_voxel = options->feature_voxel_size;
        }
        result->preprocessing_ms = std::chrono::duration<double, std::milli>(
                                       Clock::now() - preprocessing_started)
                                       .count();
        result->source_points_used = source->size();
        result->target_points_used = target->size();
        if (source->size() < 3 || target->size() < 3) {
          SetError(result, CADREG_STATUS_EMPTY_AFTER_DOWNSAMPLING,
                   "too few points remain after downsampling");
        } else {
          RegisterPrepared(source, target, initial, *options,
                           &handle->target_levels,
                           &handle->target_level_trees,
                           handle->target_feature_cloud,
                           handle->target_features, result);
        }
      }
    }
  } catch (const std::exception &error) {
    SetError(result, CADREG_STATUS_INTERNAL_ERROR, error.what());
  } catch (...) {
    SetError(result, CADREG_STATUS_INTERNAL_ERROR, "unknown internal error");
  }
  result->elapsed_ms =
      std::chrono::duration<double, std::milli>(Clock::now() - started).count();
  if (handle && result->error_message[0])
    handle->last_error = result->error_message;
  return result->status;
}

const char *CADREG_CALL cadreg_last_error(CadRegHandle handle) {
  return handle ? handle->last_error.c_str() : "invalid handle";
}

CadRegStatus CADREG_CALL cadreg_register(const CadRegPointCloud *source_view,
                                         const CadRegPointCloud *target_view,
                                         const double *initial_values,
                                         const CadRegOptions *options,
                                         CadRegResult *result) {
  CadRegHandle handle = cadreg_create();
  if (!handle)
    return CADREG_STATUS_INTERNAL_ERROR;
  const CadRegStatus target_status = cadreg_set_target(handle, target_view);
  CadRegStatus status = target_status;
  if (target_status == CADREG_STATUS_SUCCESS) {
    status = cadreg_register_source(handle, source_view, initial_values,
                                    options, result);
  } else if (result && result->struct_size == sizeof(CadRegResult) &&
             result->abi_version == CADREG_ABI_VERSION) {
    cadreg_init_result(result);
    SetError(result, target_status, cadreg_last_error(handle));
  }
  cadreg_destroy(handle);
  return status;
}

} // extern "C"
