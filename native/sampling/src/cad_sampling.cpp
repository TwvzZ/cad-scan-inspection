#include "cad_sampling.h"

#if defined(_WIN32)
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#endif

#include <BRepClass3d.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <BRep_Tool.hxx>
#include <IFSelect_ReturnStatus.hxx>
#include <Poly_Triangulation.hxx>
#include <STEPControl_Reader.hxx>
#include <TopAbs_Orientation.hxx>
#include <TopAbs_ShapeEnum.hxx>
#include <TopExp.hxx>
#include <TopExp_Explorer.hxx>
#include <TopLoc_Location.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Shell.hxx>
#include <TopoDS_Solid.hxx>
#include <gp_Pnt.hxx>
#include <gp_Trsf.hxx>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iterator>
#include <limits>
#include <memory>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <vector>

namespace {

using Clock = std::chrono::steady_clock;

struct Vec3 {
  double x;
  double y;
  double z;
};

struct Triangle {
  Vec3 a;
  Vec3 b;
  Vec3 c;
  Vec3 normal;
  double area;
  uint32_t face_id;
};

struct SamplePoint {
  Vec3 position;
  Vec3 normal;
  uint32_t face_id;
};

struct BvhNode {
  Vec3 minimum;
  Vec3 maximum;
  uint32_t begin;
  uint32_t end;
  int left;
  int right;
};

struct VoxelKey {
  int64_t x;
  int64_t y;
  int64_t z;

  bool operator==(const VoxelKey &other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct VoxelKeyHash {
  size_t operator()(const VoxelKey &key) const {
    size_t hash = std::hash<int64_t>{}(key.x);
    hash ^= std::hash<int64_t>{}(key.y) + static_cast<size_t>(0x9e3779b9) +
            (hash << 6) + (hash >> 2);
    hash ^= std::hash<int64_t>{}(key.z) + static_cast<size_t>(0x9e3779b9) +
            (hash << 6) + (hash >> 2);
    return hash;
  }
};

double ElapsedMs(const Clock::time_point &start) {
  return std::chrono::duration<double, std::milli>(Clock::now() - start)
      .count();
}

Vec3 Subtract(const Vec3 &a, const Vec3 &b) {
  return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 Cross(const Vec3 &a, const Vec3 &b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

double Length(const Vec3 &value) {
  return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

double Dot(const Vec3 &a, const Vec3 &b) {
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vec3 FromPoint(const gp_Pnt &point) {
  return {point.X(), point.Y(), point.Z()};
}

Vec3 Minimum(const Vec3 &a, const Vec3 &b) {
  return {std::min(a.x, b.x), std::min(a.y, b.y), std::min(a.z, b.z)};
}

Vec3 Maximum(const Vec3 &a, const Vec3 &b) {
  return {std::max(a.x, b.x), std::max(a.y, b.y), std::max(a.z, b.z)};
}

double Component(const Vec3 &value, int axis) {
  return axis == 0 ? value.x : (axis == 1 ? value.y : value.z);
}

int BuildBvhNode(const std::vector<Triangle> &triangles,
                 std::vector<uint32_t> *indices, std::vector<BvhNode> *nodes,
                 uint32_t begin, uint32_t end) {
  BvhNode node{};
  node.minimum = {std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity(),
                  std::numeric_limits<double>::infinity()};
  node.maximum = {-std::numeric_limits<double>::infinity(),
                  -std::numeric_limits<double>::infinity(),
                  -std::numeric_limits<double>::infinity()};
  Vec3 centroid_min = node.minimum;
  Vec3 centroid_max = node.maximum;
  for (uint32_t i = begin; i < end; ++i) {
    const Triangle &t = triangles[(*indices)[i]];
    node.minimum = Minimum(node.minimum, Minimum(t.a, Minimum(t.b, t.c)));
    node.maximum = Maximum(node.maximum, Maximum(t.a, Maximum(t.b, t.c)));
    const Vec3 c{(t.a.x + t.b.x + t.c.x) / 3.0, (t.a.y + t.b.y + t.c.y) / 3.0,
                 (t.a.z + t.b.z + t.c.z) / 3.0};
    centroid_min = Minimum(centroid_min, c);
    centroid_max = Maximum(centroid_max, c);
  }
  node.begin = begin;
  node.end = end;
  node.left = -1;
  node.right = -1;
  const int node_index = static_cast<int>(nodes->size());
  nodes->push_back(node);
  if (end - begin <= 8)
    return node_index;
  const Vec3 extent = Subtract(centroid_max, centroid_min);
  const int axis = extent.y > extent.x ? (extent.z > extent.y ? 2 : 1)
                                       : (extent.z > extent.x ? 2 : 0);
  const uint32_t middle = begin + (end - begin) / 2;
  std::nth_element(indices->begin() + begin, indices->begin() + middle,
                   indices->begin() + end, [&](uint32_t lhs, uint32_t rhs) {
                     const Triangle &a = triangles[lhs];
                     const Triangle &b = triangles[rhs];
                     const Vec3 ca{a.a.x + a.b.x + a.c.x, a.a.y + a.b.y + a.c.y,
                                   a.a.z + a.b.z + a.c.z};
                     const Vec3 cb{b.a.x + b.b.x + b.c.x, b.a.y + b.b.y + b.c.y,
                                   b.a.z + b.b.z + b.c.z};
                     return Component(ca, axis) < Component(cb, axis);
                   });
  (*nodes)[node_index].left =
      BuildBvhNode(triangles, indices, nodes, begin, middle);
  (*nodes)[node_index].right =
      BuildBvhNode(triangles, indices, nodes, middle, end);
  return node_index;
}

bool RayBox(const Vec3 &origin, const Vec3 &direction, const BvhNode &node,
            double maximum_distance) {
  double near_value = 0.0;
  double far_value = maximum_distance;
  for (int axis = 0; axis < 3; ++axis) {
    const double o = Component(origin, axis);
    const double d = Component(direction, axis);
    const double low = Component(node.minimum, axis);
    const double high = Component(node.maximum, axis);
    if (std::abs(d) < 1e-15) {
      if (o < low || o > high)
        return false;
      continue;
    }
    double t0 = (low - o) / d;
    double t1 = (high - o) / d;
    if (t0 > t1)
      std::swap(t0, t1);
    near_value = std::max(near_value, t0);
    far_value = std::min(far_value, t1);
    if (near_value > far_value)
      return false;
  }
  return true;
}

bool RayTriangle(const Vec3 &origin, const Vec3 &direction,
                 const Triangle &triangle, double *distance) {
  const Vec3 edge1 = Subtract(triangle.b, triangle.a);
  const Vec3 edge2 = Subtract(triangle.c, triangle.a);
  const Vec3 p = Cross(direction, edge2);
  const double determinant = Dot(edge1, p);
  if (std::abs(determinant) < 1e-12)
    return false;
  const double inverse = 1.0 / determinant;
  const Vec3 offset = Subtract(origin, triangle.a);
  const double u = Dot(offset, p) * inverse;
  if (u < 0.0 || u > 1.0)
    return false;
  const Vec3 q = Cross(offset, edge1);
  const double v = Dot(direction, q) * inverse;
  if (v < 0.0 || u + v > 1.0)
    return false;
  const double t = Dot(edge2, q) * inverse;
  if (t <= 0.0)
    return false;
  *distance = t;
  return true;
}

bool HasOccluder(const std::vector<Triangle> &triangles,
                 const std::vector<uint32_t> &indices,
                 const std::vector<BvhNode> &nodes, const Vec3 &origin,
                 const Vec3 &direction, double maximum_distance,
                 std::vector<int> *stack) {
  stack->clear();
  stack->push_back(0);
  while (!stack->empty()) {
    const int index = stack->back();
    stack->pop_back();
    const BvhNode &node = nodes[index];
    if (!RayBox(origin, direction, node, maximum_distance))
      continue;
    if (node.left >= 0) {
      stack->push_back(node.left);
      stack->push_back(node.right);
      continue;
    }
    for (uint32_t i = node.begin; i < node.end; ++i) {
      double distance = 0.0;
      if (RayTriangle(origin, direction, triangles[indices[i]], &distance) &&
          distance < maximum_distance)
        return true;
    }
  }
  return false;
}

void SetError(CadSampleResult *result, CadSampleStatus status,
              const char *message) {
  result->status = status;
  std::snprintf(result->error_message, CADSAMPLE_ERROR_MESSAGE_SIZE, "%s",
                message);
}

bool ValidResultHeader(const CadSampleResult *result) {
  return result && result->struct_size == sizeof(CadSampleResult) &&
         result->abi_version == CADSAMPLE_ABI_VERSION;
}

std::string NativePathFromUtf8(const char *path_utf8) {
#if defined(_WIN32)
  const int wide_size = MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS,
                                            path_utf8, -1, nullptr, 0);
  if (wide_size <= 0)
    throw std::invalid_argument("invalid UTF-8 path");
  std::wstring wide(static_cast<size_t>(wide_size), L'\0');
  MultiByteToWideChar(CP_UTF8, MB_ERR_INVALID_CHARS, path_utf8, -1, &wide[0],
                      wide_size);
  const int ansi_size =
      WideCharToMultiByte(CP_ACP, WC_NO_BEST_FIT_CHARS, wide.c_str(), -1,
                          nullptr, 0, nullptr, nullptr);
  if (ansi_size <= 0)
    throw std::invalid_argument("path conversion failed");
  std::string native(static_cast<size_t>(ansi_size), '\0');
  BOOL used_default = FALSE;
  WideCharToMultiByte(CP_ACP, WC_NO_BEST_FIT_CHARS, wide.c_str(), -1,
                      &native[0], ansi_size, nullptr, &used_default);
  if (used_default)
    throw std::invalid_argument(
        "STEP path contains characters unsupported by the system code page");
  native.resize(std::strlen(native.c_str()));
  return native;
#else
  return std::string(path_utf8);
#endif
}

std::vector<Triangle> BuildTriangles(const TopoDS_Shape &shape,
                                     const CadSampleOptions &options,
                                     uint64_t *face_count,
                                     uint64_t *selected_face_count,
                                     double *total_area) {
  const double radians =
      options.angular_deflection_deg * 3.14159265358979323846 / 180.0;
  BRepMesh_IncrementalMesh mesher(shape, options.linear_deflection,
                                  options.relative_deflection != 0, radians,
                                  options.parallel_meshing != 0);
  if (!mesher.IsDone())
    throw std::runtime_error("OpenCASCADE triangulation did not complete");

  std::vector<Triangle> triangles;
  uint64_t faces = 0;
  uint64_t selected_faces = 0;
  double area_sum = 0.0;
  TopTools_IndexedMapOfShape outer_faces;
  if (options.surface_mode != CADSAMPLE_SURFACE_ALL_FACES) {
    for (TopExp_Explorer solid_it(shape, TopAbs_SOLID); solid_it.More();
         solid_it.Next()) {
      const TopoDS_Solid solid = TopoDS::Solid(solid_it.Current());
      const TopoDS_Shell shell = BRepClass3d::OuterShell(solid);
      if (!shell.IsNull())
        TopExp::MapShapes(shell, TopAbs_FACE, outer_faces);
    }
  }
  const bool filter_outer =
      options.surface_mode != CADSAMPLE_SURFACE_ALL_FACES &&
      outer_faces.Extent() > 0;
  for (TopExp_Explorer explorer(shape, TopAbs_FACE); explorer.More();
       explorer.Next()) {
    ++faces;
    if (faces > std::numeric_limits<uint32_t>::max())
      throw std::runtime_error("face count exceeds uint32 range");
    const TopoDS_Face face = TopoDS::Face(explorer.Current());
    if (filter_outer && !outer_faces.Contains(face))
      continue;
    ++selected_faces;
    TopLoc_Location location;
    const Handle(Poly_Triangulation) mesh =
        BRep_Tool::Triangulation(face, location);
    if (mesh.IsNull())
      continue;
    const gp_Trsf transform = location.Transformation();
    for (Standard_Integer index = 1; index <= mesh->NbTriangles(); ++index) {
      Standard_Integer ia = 0, ib = 0, ic = 0;
      mesh->Triangle(index).Get(ia, ib, ic);
      if (face.Orientation() == TopAbs_REVERSED)
        std::swap(ib, ic);
      gp_Pnt pa = mesh->Node(ia).Transformed(transform);
      gp_Pnt pb = mesh->Node(ib).Transformed(transform);
      gp_Pnt pc = mesh->Node(ic).Transformed(transform);
      Triangle triangle{};
      triangle.a = FromPoint(pa);
      triangle.b = FromPoint(pb);
      triangle.c = FromPoint(pc);
      const Vec3 cross = Cross(Subtract(triangle.b, triangle.a),
                               Subtract(triangle.c, triangle.a));
      const double cross_length = Length(cross);
      triangle.area = 0.5 * cross_length;
      if (!(triangle.area > 0.0) || !std::isfinite(triangle.area))
        continue;
      triangle.normal = {cross.x / cross_length, cross.y / cross_length,
                         cross.z / cross_length};
      triangle.face_id = static_cast<uint32_t>(faces);
      area_sum += triangle.area;
      triangles.push_back(triangle);
    }
  }
  *face_count = faces;
  *selected_face_count = selected_faces;
  *total_area = area_sum;
  return triangles;
}

void BuildAliasTable(const std::vector<Triangle> &triangles, double total_area,
                     std::vector<double> *probability,
                     std::vector<size_t> *alias) {
  const size_t triangle_count = triangles.size();
  probability->assign(triangle_count, 1.0);
  alias->assign(triangle_count, 0);
  std::vector<size_t> underfull;
  std::vector<size_t> overfull;
  underfull.reserve(triangle_count);
  overfull.reserve(triangle_count);
  for (size_t i = 0; i < triangle_count; ++i) {
    (*probability)[i] =
        triangles[i].area * static_cast<double>(triangle_count) / total_area;
    if ((*probability)[i] < 1.0)
      underfull.push_back(i);
    else
      overfull.push_back(i);
  }
  while (!underfull.empty() && !overfull.empty()) {
    const size_t low = underfull.back();
    underfull.pop_back();
    const size_t high = overfull.back();
    overfull.pop_back();
    (*alias)[low] = high;
    (*probability)[high] =
        (*probability)[high] + (*probability)[low] - 1.0;
    if ((*probability)[high] < 1.0)
      underfull.push_back(high);
    else
      overfull.push_back(high);
  }
  for (size_t index : underfull)
    (*probability)[index] = 1.0;
  for (size_t index : overfull)
    (*probability)[index] = 1.0;
}

void GeneratePoints(
    const std::vector<Triangle> &triangles,
    const std::vector<double> &probability,
    const std::vector<size_t> &alias,
    const std::vector<uint32_t> &bvh_indices,
    const std::vector<BvhNode> &bvh_nodes, const CadSampleOptions &options,
    double *visibility_elapsed_ms, std::vector<SamplePoint> *points) {
  const size_t triangle_count = triangles.size();
  std::mt19937_64 random(options.random_seed);
  std::uniform_int_distribution<size_t> triangle_distribution(
      0, triangle_count - 1);
  std::uniform_real_distribution<double> unit(0.0, 1.0);
  points->clear();
  points->reserve(static_cast<size_t>(options.target_point_count));
  const bool visible_only = options.surface_mode == CADSAMPLE_SURFACE_VISIBLE;
  const Clock::time_point visibility_started = Clock::now();
  const uint64_t maximum_candidates =
      options.target_point_count >
              std::numeric_limits<uint64_t>::max() /
                  static_cast<uint64_t>(visible_only
                                            ? options.visibility_oversample_factor
                                            : 1u)
          ? std::numeric_limits<uint64_t>::max()
          : options.target_point_count *
                static_cast<uint64_t>(visible_only
                                          ? options.visibility_oversample_factor
                                          : 1u);
  const Vec3 camera{options.camera_position[0], options.camera_position[1],
                    options.camera_position[2]};
  Vec3 view_direction{options.view_direction[0], options.view_direction[1],
                      options.view_direction[2]};
  if (visible_only &&
      options.projection_mode == CADSAMPLE_PROJECTION_ORTHOGRAPHIC) {
    const double view_length = Length(view_direction);
    view_direction = {view_direction.x / view_length,
                      view_direction.y / view_length,
                      view_direction.z / view_length};
  }
  double tolerance = options.visibility_tolerance;
  const double minimum_facing_cosine = std::cos(
      options.max_incidence_angle_deg * 3.14159265358979323846 / 180.0);
  const Vec3 orthographic_toward_sensor{-view_direction.x,
                                        -view_direction.y,
                                        -view_direction.z};
  std::vector<int> traversal_stack;
  if (visible_only)
    traversal_stack.reserve(64);
  if (!(tolerance > 0.0)) {
    const BvhNode &root = visible_only ? bvh_nodes.front() : BvhNode{};
    tolerance =
        visible_only
            ? std::max(1e-7,
                       Length(Subtract(root.maximum, root.minimum)) * 1e-8)
            : 0.0;
  }
  for (uint64_t i = 0;
       i < maximum_candidates &&
       points->size() < static_cast<size_t>(options.target_point_count);
       ++i) {
    const size_t column = triangle_distribution(random);
    const size_t triangle_index =
        unit(random) < probability[column] ? column : alias[column];
    const Triangle &triangle = triangles[triangle_index];
    const double root = std::sqrt(unit(random));
    const double v = root * (1.0 - unit(random));
    const double w = root - v;
    const double u = 1.0 - root;
    const Vec3 position{u * triangle.a.x + v * triangle.b.x + w * triangle.c.x,
                        u * triangle.a.y + v * triangle.b.y + w * triangle.c.y,
                        u * triangle.a.z + v * triangle.b.z + w * triangle.c.z};
    if (visible_only) {
      if (options.projection_mode == CADSAMPLE_PROJECTION_ORTHOGRAPHIC) {
        const Vec3 &toward_sensor = orthographic_toward_sensor;
        if (Dot(triangle.normal, toward_sensor) < minimum_facing_cosine)
          continue;
        const Vec3 ray_origin{position.x + toward_sensor.x * tolerance,
                              position.y + toward_sensor.y * tolerance,
                              position.z + toward_sensor.z * tolerance};
        if (HasOccluder(triangles, bvh_indices, bvh_nodes, ray_origin,
                        toward_sensor, std::numeric_limits<double>::infinity(),
                        &traversal_stack))
          continue;
      } else {
        const Vec3 to_point = Subtract(position, camera);
        const double distance = Length(to_point);
        if (!(distance > tolerance))
          continue;
        const Vec3 direction{to_point.x / distance, to_point.y / distance,
                             to_point.z / distance};
        const Vec3 toward_sensor{-direction.x, -direction.y, -direction.z};
        if (Dot(triangle.normal, toward_sensor) < minimum_facing_cosine)
          continue;
        if (HasOccluder(triangles, bvh_indices, bvh_nodes, camera, direction,
                        distance - tolerance, &traversal_stack))
          continue;
      }
    }
    points->push_back({position, triangle.normal, triangle.face_id});
  }
  *visibility_elapsed_ms = visible_only ? ElapsedMs(visibility_started) : 0.0;
}
void VoxelDownsample(std::vector<SamplePoint> *points, double voxel_size) {
  if (!(voxel_size > 0.0))
    return;
  std::unordered_set<VoxelKey, VoxelKeyHash> occupied;
  occupied.reserve(points->size());
  std::vector<SamplePoint> filtered;
  filtered.reserve(points->size());
  for (const SamplePoint &point : *points) {
    const double scaled_x = std::floor(point.position.x / voxel_size);
    const double scaled_y = std::floor(point.position.y / voxel_size);
    const double scaled_z = std::floor(point.position.z / voxel_size);
    if (scaled_x < static_cast<double>(std::numeric_limits<int64_t>::min()) ||
        scaled_x > static_cast<double>(std::numeric_limits<int64_t>::max()) ||
        scaled_y < static_cast<double>(std::numeric_limits<int64_t>::min()) ||
        scaled_y > static_cast<double>(std::numeric_limits<int64_t>::max()) ||
        scaled_z < static_cast<double>(std::numeric_limits<int64_t>::min()) ||
        scaled_z > static_cast<double>(std::numeric_limits<int64_t>::max()))
      throw std::runtime_error("voxel index exceeds int64 range");
    const VoxelKey key{static_cast<int64_t>(scaled_x),
                       static_cast<int64_t>(scaled_y),
                       static_cast<int64_t>(scaled_z)};
    if (occupied.insert(key).second)
      filtered.push_back(point);
  }
  points->swap(filtered);
}

} // namespace

struct CadSampleHandleImpl {
  TopoDS_Shape shape;
  bool loaded = false;
  double last_load_elapsed_ms = 0.0;
  bool mesh_cached = false;
  double mesh_linear_deflection = 0.0;
  double mesh_angular_deflection_deg = 0.0;
  uint32_t mesh_relative_deflection = 0;
  uint32_t mesh_parallel_meshing = 0;
  uint32_t mesh_surface_mode = 0;
  std::vector<Triangle> triangles;
  std::vector<double> alias_probability;
  std::vector<size_t> alias_index;
  std::vector<uint32_t> bvh_indices;
  std::vector<BvhNode> bvh_nodes;
  uint64_t face_count = 0;
  uint64_t selected_face_count = 0;
  double surface_area = 0.0;

  bool sample_cached = false;
  uint64_t sample_target_point_count = 0;
  uint32_t sample_random_seed = 0;
  double sample_voxel_size = 0.0;
  uint32_t sample_surface_mode = 0;
  uint32_t sample_projection_mode = 0;
  double sample_camera_position[3]{};
  double sample_view_direction[3]{};
  double sample_max_incidence_angle_deg = 0.0;
  double sample_visibility_tolerance = 0.0;
  uint32_t sample_visibility_oversample_factor = 0;
  double last_visibility_elapsed_ms = 0.0;
  std::vector<SamplePoint> sampled_points;
};

namespace {

bool SameMeshOptions(const CadSampleHandleImpl &handle,
                     const CadSampleOptions &options) {
  return handle.mesh_cached &&
         handle.mesh_linear_deflection == options.linear_deflection &&
         handle.mesh_angular_deflection_deg == options.angular_deflection_deg &&
         handle.mesh_relative_deflection == options.relative_deflection &&
         handle.mesh_parallel_meshing == options.parallel_meshing &&
         handle.mesh_surface_mode == options.surface_mode;
}

bool SameSampleOptions(const CadSampleHandleImpl &handle,
                       const CadSampleOptions &options) {
  return handle.sample_cached && SameMeshOptions(handle, options) &&
         handle.sample_target_point_count == options.target_point_count &&
         handle.sample_random_seed == options.random_seed &&
         handle.sample_voxel_size == options.voxel_size &&
         handle.sample_surface_mode == options.surface_mode &&
         handle.sample_projection_mode == options.projection_mode &&
         handle.sample_camera_position[0] == options.camera_position[0] &&
         handle.sample_camera_position[1] == options.camera_position[1] &&
         handle.sample_camera_position[2] == options.camera_position[2] &&
         handle.sample_view_direction[0] == options.view_direction[0] &&
         handle.sample_view_direction[1] == options.view_direction[1] &&
         handle.sample_view_direction[2] == options.view_direction[2] &&
         handle.sample_max_incidence_angle_deg ==
             options.max_incidence_angle_deg &&
         handle.sample_visibility_tolerance == options.visibility_tolerance &&
         handle.sample_visibility_oversample_factor ==
             options.visibility_oversample_factor;
}

} // namespace

extern "C" {

uint32_t CADSAMPLE_CALL cadsample_get_abi_version(void) {
  return CADSAMPLE_ABI_VERSION;
}

void CADSAMPLE_CALL cadsample_default_options(CadSampleOptions *options) {
  if (!options)
    return;
  *options = {};
  options->struct_size = sizeof(CadSampleOptions);
  options->abi_version = CADSAMPLE_ABI_VERSION;
  options->target_point_count = 100000;
  options->linear_deflection = 0.1;
  options->angular_deflection_deg = 10.0;
  options->relative_deflection = 0;
  options->parallel_meshing = 1;
  options->random_seed = 1;
  options->voxel_size = 0.0;
  options->surface_mode = CADSAMPLE_SURFACE_OUTER_SHELL;
  options->projection_mode = CADSAMPLE_PROJECTION_ORTHOGRAPHIC;
  options->camera_position[0] = 0.0;
  options->camera_position[1] = 0.0;
  options->camera_position[2] = 0.0;
  options->view_direction[0] = 0.0;
  options->view_direction[1] = 0.0;
  options->view_direction[2] = -1.0;
  options->max_incidence_angle_deg = 75.0;
  options->visibility_tolerance = 0.0;
  options->visibility_oversample_factor = 8;
}

void CADSAMPLE_CALL cadsample_init_buffers(CadSampleBuffers *buffers) {
  if (!buffers)
    return;
  *buffers = {};
  buffers->struct_size = sizeof(CadSampleBuffers);
  buffers->abi_version = CADSAMPLE_ABI_VERSION;
}

void CADSAMPLE_CALL cadsample_init_result(CadSampleResult *result) {
  if (!result)
    return;
  *result = {};
  result->struct_size = sizeof(CadSampleResult);
  result->abi_version = CADSAMPLE_ABI_VERSION;
  result->status = CADSAMPLE_STATUS_INTERNAL_ERROR;
}

CadSampleHandle CADSAMPLE_CALL cadsample_create(void) {
  try {
    return new CadSampleHandleImpl;
  } catch (...) {
    return nullptr;
  }
}

void CADSAMPLE_CALL cadsample_destroy(CadSampleHandle handle) { delete handle; }

CadSampleStatus CADSAMPLE_CALL cadsample_load_step(CadSampleHandle handle,
                                                   const char *path_utf8,
                                                   CadSampleResult *result) {
  if (!ValidResultHeader(result))
    return CADSAMPLE_STATUS_INVALID_ARGUMENT;
  cadsample_init_result(result);
  const Clock::time_point started = Clock::now();
  try {
    if (!handle || !path_utf8 || path_utf8[0] == '\0') {
      SetError(result, CADSAMPLE_STATUS_INVALID_ARGUMENT,
               "handle and non-empty UTF-8 path are required");
    } else {
      STEPControl_Reader reader;
      const std::string native_path = NativePathFromUtf8(path_utf8);
      if (reader.ReadFile(native_path.c_str()) != IFSelect_RetDone) {
        SetError(result, CADSAMPLE_STATUS_FILE_READ_FAILED,
                 "OpenCASCADE failed to read the STEP file");
      } else if (reader.TransferRoots() <= 0) {
        SetError(result, CADSAMPLE_STATUS_NO_SHAPE,
                 "STEP file contains no transferable root shape");
      } else {
        TopoDS_Shape shape = reader.OneShape();
        if (shape.IsNull()) {
          SetError(result, CADSAMPLE_STATUS_NO_SHAPE,
                   "STEP transfer produced an empty shape");
        } else {
          handle->shape = shape;
          handle->loaded = true;
          handle->mesh_cached = false;
          handle->sample_cached = false;
          handle->triangles.clear();
          handle->alias_probability.clear();
          handle->alias_index.clear();
          handle->bvh_indices.clear();
          handle->bvh_nodes.clear();
          handle->sampled_points.clear();
          result->status = CADSAMPLE_STATUS_SUCCESS;
        }
      }
    }
  } catch (const std::invalid_argument &error) {
    SetError(result, CADSAMPLE_STATUS_INVALID_ARGUMENT, error.what());
  } catch (const std::exception &error) {
    SetError(result, CADSAMPLE_STATUS_INTERNAL_ERROR, error.what());
  } catch (...) {
    SetError(result, CADSAMPLE_STATUS_INTERNAL_ERROR,
             "unknown STEP loading error");
  }
  result->load_elapsed_ms = ElapsedMs(started);
  if (handle)
    handle->last_load_elapsed_ms = result->load_elapsed_ms;
  return result->status;
}

CadSampleStatus CADSAMPLE_CALL
cadsample_generate(CadSampleHandle handle, const CadSampleOptions *options,
                   CadSampleBuffers *buffers, CadSampleResult *result) {
  if (!ValidResultHeader(result))
    return CADSAMPLE_STATUS_INVALID_ARGUMENT;
  cadsample_init_result(result);
  const Clock::time_point started = Clock::now();
  try {
    if (!handle || !handle->loaded) {
      SetError(result, CADSAMPLE_STATUS_NO_SHAPE,
               "load a STEP shape before sampling");
    } else if (!options || options->struct_size != sizeof(CadSampleOptions) ||
               options->abi_version != CADSAMPLE_ABI_VERSION ||
               options->target_point_count == 0 ||
               options->target_point_count >
                   static_cast<uint64_t>(std::numeric_limits<size_t>::max() /
                                         sizeof(SamplePoint)) ||
               !(options->linear_deflection > 0.0) ||
               !(options->angular_deflection_deg > 0.0) ||
               options->angular_deflection_deg >= 180.0 ||
               !std::isfinite(options->linear_deflection) ||
               !std::isfinite(options->angular_deflection_deg) ||
               !std::isfinite(options->voxel_size) ||
               options->surface_mode > CADSAMPLE_SURFACE_VISIBLE ||
               options->projection_mode > CADSAMPLE_PROJECTION_ORTHOGRAPHIC ||
               !std::isfinite(options->camera_position[0]) ||
               !std::isfinite(options->camera_position[1]) ||
               !std::isfinite(options->camera_position[2]) ||
               !std::isfinite(options->view_direction[0]) ||
               !std::isfinite(options->view_direction[1]) ||
               !std::isfinite(options->view_direction[2]) ||
               !std::isfinite(options->max_incidence_angle_deg) ||
               !(options->max_incidence_angle_deg > 0.0) ||
               !(options->max_incidence_angle_deg < 90.0) ||
               (options->surface_mode == CADSAMPLE_SURFACE_VISIBLE &&
                options->projection_mode == CADSAMPLE_PROJECTION_ORTHOGRAPHIC &&
                !(Length({options->view_direction[0],
                          options->view_direction[1],
                          options->view_direction[2]}) > 0.0)) ||
               !std::isfinite(options->visibility_tolerance) ||
               options->visibility_tolerance < 0.0 ||
               options->visibility_oversample_factor < 1 ||
               options->visibility_oversample_factor > 32 ||
               options->target_point_count >
                   std::numeric_limits<uint64_t>::max() /
                       std::max<uint32_t>(
                           1, options->visibility_oversample_factor)) {
      SetError(result, CADSAMPLE_STATUS_INVALID_ARGUMENT,
               "invalid options, point count, or meshing deflection");
    } else {
      if (SameMeshOptions(*handle, *options)) {
        result->mesh_cache_hit = 1;
      } else {
        const Clock::time_point mesh_started = Clock::now();
        handle->triangles =
            BuildTriangles(handle->shape, *options, &handle->face_count,
                           &handle->selected_face_count, &handle->surface_area);
        handle->mesh_linear_deflection = options->linear_deflection;
        handle->mesh_angular_deflection_deg = options->angular_deflection_deg;
        handle->mesh_relative_deflection = options->relative_deflection;
        handle->mesh_parallel_meshing = options->parallel_meshing;
        handle->mesh_surface_mode = options->surface_mode;
        BuildAliasTable(handle->triangles, handle->surface_area,
                        &handle->alias_probability, &handle->alias_index);
        handle->bvh_indices.clear();
        handle->bvh_nodes.clear();
        if (options->surface_mode == CADSAMPLE_SURFACE_VISIBLE) {
          if (handle->triangles.size() >
              std::numeric_limits<uint32_t>::max())
            throw std::runtime_error(
                "visible mode supports at most uint32 triangle count");
          handle->bvh_indices.resize(handle->triangles.size());
          for (size_t i = 0; i < handle->triangles.size(); ++i)
            handle->bvh_indices[i] = static_cast<uint32_t>(i);
          handle->bvh_nodes.reserve(handle->triangles.size() * 2);
          BuildBvhNode(handle->triangles, &handle->bvh_indices,
                       &handle->bvh_nodes, 0,
                       static_cast<uint32_t>(handle->triangles.size()));
        }
        result->triangulation_elapsed_ms = ElapsedMs(mesh_started);
        handle->mesh_cached = true;
        handle->sample_cached = false;
        handle->sampled_points.clear();
      }
      result->face_count = handle->face_count;
      result->selected_triangle_count = handle->triangles.size();
      result->selected_surface_area = handle->surface_area;
      result->surface_area = handle->surface_area;
      result->triangle_count = handle->triangles.size();
      if (handle->triangles.empty() || !(handle->surface_area > 0.0)) {
        SetError(result, CADSAMPLE_STATUS_MESH_FAILED,
                 "shape produced no non-degenerate triangles");
      } else {
        if (SameSampleOptions(*handle, *options)) {
          result->sample_cache_hit = 1;
        } else {
          const Clock::time_point generation_started = Clock::now();
          GeneratePoints(
              handle->triangles, handle->alias_probability,
              handle->alias_index, handle->bvh_indices, handle->bvh_nodes,
              *options, &handle->last_visibility_elapsed_ms,
              &handle->sampled_points);
          result->generation_elapsed_ms = ElapsedMs(generation_started);
          const Clock::time_point voxel_started = Clock::now();
          VoxelDownsample(&handle->sampled_points, options->voxel_size);
          result->voxel_elapsed_ms = ElapsedMs(voxel_started);
          handle->sample_target_point_count = options->target_point_count;
          handle->sample_random_seed = options->random_seed;
          handle->sample_voxel_size = options->voxel_size;
          handle->sample_surface_mode = options->surface_mode;
          handle->sample_projection_mode = options->projection_mode;
          std::copy(std::begin(options->camera_position),
                    std::end(options->camera_position),
                    handle->sample_camera_position);
          std::copy(std::begin(options->view_direction),
                    std::end(options->view_direction),
                    handle->sample_view_direction);
          handle->sample_max_incidence_angle_deg =
              options->max_incidence_angle_deg;
          handle->sample_visibility_tolerance = options->visibility_tolerance;
          handle->sample_visibility_oversample_factor =
              options->visibility_oversample_factor;
          handle->sample_cached = true;
        }
        result->visibility_elapsed_ms = handle->last_visibility_elapsed_ms;
        const std::vector<SamplePoint> &points = handle->sampled_points;
        result->required_point_capacity = points.size();
        if (!buffers) {
          SetError(result, CADSAMPLE_STATUS_BUFFER_TOO_SMALL,
                   "query completed; provide output buffers");
        } else if (buffers->struct_size != sizeof(CadSampleBuffers) ||
                   buffers->abi_version != CADSAMPLE_ABI_VERSION ||
                   !buffers->xyz || buffers->capacity_points < points.size()) {
          SetError(result, CADSAMPLE_STATUS_BUFFER_TOO_SMALL,
                   "XYZ buffer capacity is smaller than required");
        } else {
          for (size_t i = 0; i < points.size(); ++i) {
            const SamplePoint &point = points[i];
            buffers->xyz[i * 3 + 0] = point.position.x;
            buffers->xyz[i * 3 + 1] = point.position.y;
            buffers->xyz[i * 3 + 2] = point.position.z;
            if (buffers->normals) {
              buffers->normals[i * 3 + 0] = point.normal.x;
              buffers->normals[i * 3 + 1] = point.normal.y;
              buffers->normals[i * 3 + 2] = point.normal.z;
            }
            if (buffers->face_ids)
              buffers->face_ids[i] = point.face_id;
          }
          result->point_count = points.size();
          result->status = CADSAMPLE_STATUS_SUCCESS;
        }
      }
    }
  } catch (const std::exception &error) {
    SetError(result, CADSAMPLE_STATUS_INTERNAL_ERROR, error.what());
  } catch (...) {
    SetError(result, CADSAMPLE_STATUS_INTERNAL_ERROR, "unknown sampling error");
  }
  result->load_elapsed_ms = handle ? handle->last_load_elapsed_ms : 0.0;
  result->sample_elapsed_ms = ElapsedMs(started);
  return result->status;
}

} // extern "C"
