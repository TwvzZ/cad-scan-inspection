#include "cad_inspection.h"

#include <cmath>
#include <cstdint>
#include <iomanip>
#include <iostream>
#include <random>
#include <vector>

int main() {
  std::vector<double> xyz;
  std::mt19937 random(7);
  std::normal_distribution<double> noise(0.0, 0.025);
  for (int y = -50; y <= 50; ++y) {
    for (int x = -100; x <= 100; ++x) {
      double z = 0.001 * x - 0.0005 * y + noise(random);
      const bool burr = (x + 65) * (x + 65) + (y - 25) * (y - 25) < 16;
      /* Deliberately larger than the remaining nominal surface. */
      const bool block = x > -55 && x < 90 && y > -42 && y < 42;
      if (burr)
        z += 3.0;
      if (block)
        z += 2.0;
      xyz.push_back(x);
      xyz.push_back(y);
      xyz.push_back(z);
    }
  }

  CadInspectPointCloud cloud{xyz.data(), xyz.size() / 3, 0};
  CadInspectOptions options{};
  cadinspect_default_options(&options);
  options.u_min = -101;
  options.u_max = 101;
  options.v_min = -51;
  options.v_max = 51;
  options.detection_normal_min = -1;
  options.detection_normal_max = 5;
  options.reference_normal_min = -0.4;
  options.reference_normal_max = 0.4;
  options.edge_margin = 2;
  options.positive_defect_threshold = 0.4;
  options.negative_defect_threshold = 0.4;
  options.defect_cluster_cell_size = 1.0;
  options.min_defect_points = 4;
  options.min_defect_area = 3.0;
  options.max_reference_rmse = 0.08;

  std::vector<CadInspectDefect> defects(options.max_output_defects);
  std::vector<uint32_t> labels(cloud.point_count);
  CadInspectBuffers buffers{};
  cadinspect_init_buffers(&buffers);
  buffers.defects = defects.data();
  buffers.defect_capacity = static_cast<uint32_t>(defects.size());
  buffers.point_labels = labels.data();
  buffers.point_label_capacity = labels.size();
  CadInspectResult result{};
  cadinspect_init_result(&result);
  CadInspectStatus status =
      cadinspect_analyze(&cloud, &options, &buffers, &result);

  std::cout << std::fixed << std::setprecision(6)
            << "status=" << static_cast<int>(status)
            << " message=" << result.error_message << "\n"
            << "input=" << result.input_point_count
            << " detection=" << result.detection_roi_point_count
            << " reference=" << result.reference_roi_point_count << "\n"
            << "plane=" << result.reference_plane.coefficients[0] << ","
            << result.reference_plane.coefficients[1] << ","
            << result.reference_plane.coefficients[2] << ","
            << result.reference_plane.coefficients[3] << "\n"
            << "plane_rmse=" << result.reference_plane.rmse
            << " inlier=" << result.reference_plane.inlier_ratio
            << " coverage=" << result.reference_plane.grid_coverage
            << " reliable=" << result.reference_plane.reliable << "\n"
            << "flatness_total=" << result.flatness.least_squares_peak_to_valley
            << " flatness_robust=" << result.flatness.robust_peak_to_valley
            << " flatness_min_zone=" << result.flatness.minimum_zone_flatness
            << "\n"
            << "defects=" << result.defect_count
            << " positive_points=" << result.positive_defect_point_count
            << " negative_points=" << result.negative_defect_point_count
            << " elapsed=" << result.elapsed_ms << " ms\n"
            << "ransac_iterations=" << result.ransac_iterations_used
            << " ransac_points=" << result.ransac_evaluation_point_count
            << " flatness_working=" << result.flatness_working_point_count
            << "\n";
  std::cout << "plane_candidates=" << result.plane_candidate_count
            << " selected=" << result.selected_plane_candidate << "\n";
  for (uint32_t i = 0; i < result.plane_candidate_count; ++i) {
    const auto &p = result.plane_candidates[i];
    std::cout << "  plane#" << (i + 1) << " offset=" << p.nominal_offset
              << " rmse=" << p.rmse << " ratio=" << p.point_ratio
              << " coverage=" << p.grid_coverage << " accepted=" << p.accepted
              << " selected=" << p.selected << "\n";
  }
  for (uint32_t i = 0; i < result.defect_count; ++i) {
    const auto &d = defects[i];
    std::cout << "  #" << d.id << " type=" << d.type
              << " points=" << d.point_count << " area=" << d.projected_area
              << " min=" << d.minimum_height << " max=" << d.maximum_height
              << " mean=" << d.mean_height << "\n";
  }
  return status == CADINSPECT_STATUS_SUCCESS ? 0 : 1;
}
