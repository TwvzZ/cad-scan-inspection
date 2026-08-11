#include "cad_inspection.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>
#include <queue>
#include <random>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace {
using Clock = std::chrono::steady_clock;

struct Vec3 {
  double x, y, z;
};
struct LocalPoint {
  Vec3 world;
  double u, v, w;
  uint64_t input_index;
};
struct Plane {
  Vec3 n;
  double d;
  Vec3 center;
};
struct CellKey {
  int64_t u, v;
  bool operator==(const CellKey &o) const { return u == o.u && v == o.v; }
};
struct CellHash {
  size_t operator()(const CellKey &k) const {
    size_t h = std::hash<int64_t>{}(k.u);
    return h ^ (std::hash<int64_t>{}(k.v) + 0x9e3779b9u + (h << 6) + (h >> 2));
  }
};
struct DefectCellKey {
  int64_t u, v;
  int sign;
  bool operator==(const DefectCellKey &o) const {
    return u == o.u && v == o.v && sign == o.sign;
  }
};
struct DefectCellHash {
  size_t operator()(const DefectCellKey &k) const {
    size_t h = CellHash{}({k.u, k.v});
    return h ^ (std::hash<int>{}(k.sign) + 0x9e3779b9u + (h << 6) + (h >> 2));
  }
};
thread_local uint32_t g_ransac_iterations_used = 0;
thread_local uint32_t g_ransac_evaluation_count = 0;
thread_local uint64_t g_flatness_working_count = 0;

double Elapsed(const Clock::time_point &t) {
  return std::chrono::duration<double, std::milli>(Clock::now() - t).count();
}
Vec3 Add(Vec3 a, Vec3 b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
Vec3 Sub(Vec3 a, Vec3 b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
Vec3 Mul(Vec3 a, double s) { return {a.x * s, a.y * s, a.z * s}; }
double Dot(Vec3 a, Vec3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
Vec3 Cross(Vec3 a, Vec3 b) {
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
double Norm(Vec3 a) { return std::sqrt(Dot(a, a)); }
Vec3 Normalize(Vec3 a) {
  double n = Norm(a);
  if (!(n > 1e-12))
    throw std::invalid_argument("zero-length axis");
  return Mul(a, 1.0 / n);
}
double Clamp(double x, double a, double b) {
  return std::max(a, std::min(b, x));
}

void SetError(CadInspectResult *r, CadInspectStatus s, const char *m) {
  r->status = s;
  std::snprintf(r->error_message, CADINSPECT_ERROR_MESSAGE_SIZE, "%s", m);
}

/* Jacobi eigen decomposition for a symmetric 3x3; returns smallest eigenvector.
 */
Vec3 SmallestEigenvector(double a[3][3]) {
  double v[3][3]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
  for (int iter = 0; iter < 32; ++iter) {
    int p = 0, q = 1;
    if (std::abs(a[0][2]) > std::abs(a[p][q])) {
      p = 0;
      q = 2;
    }
    if (std::abs(a[1][2]) > std::abs(a[p][q])) {
      p = 1;
      q = 2;
    }
    if (std::abs(a[p][q]) < 1e-14)
      break;
    const double phi = 0.5 * std::atan2(2 * a[p][q], a[q][q] - a[p][p]);
    const double c = std::cos(phi), s = std::sin(phi);
    const double app = c * c * a[p][p] - 2 * s * c * a[p][q] + s * s * a[q][q];
    const double aqq = s * s * a[p][p] + 2 * s * c * a[p][q] + c * c * a[q][q];
    for (int k = 0; k < 3; ++k)
      if (k != p && k != q) {
        const double akp = c * a[k][p] - s * a[k][q];
        const double akq = s * a[k][p] + c * a[k][q];
        a[k][p] = a[p][k] = akp;
        a[k][q] = a[q][k] = akq;
      }
    a[p][p] = app;
    a[q][q] = aqq;
    a[p][q] = a[q][p] = 0;
    for (int k = 0; k < 3; ++k) {
      double vp = c * v[k][p] - s * v[k][q];
      double vq = s * v[k][p] + c * v[k][q];
      v[k][p] = vp;
      v[k][q] = vq;
    }
  }
  int index = 0;
  if (a[1][1] < a[index][index])
    index = 1;
  if (a[2][2] < a[index][index])
    index = 2;
  return Normalize({v[0][index], v[1][index], v[2][index]});
}

Plane FitPlane(const std::vector<LocalPoint> &points,
               const std::vector<size_t> &ids, Vec3 nominal) {
  if (ids.size() < 3)
    throw std::runtime_error("fewer than three plane points");
  Vec3 c{0, 0, 0};
  for (size_t i : ids)
    c = Add(c, points[i].world);
  c = Mul(c, 1.0 / ids.size());
  double m[3][3]{};
  for (size_t i : ids) {
    Vec3 d = Sub(points[i].world, c);
    m[0][0] += d.x * d.x;
    m[0][1] += d.x * d.y;
    m[0][2] += d.x * d.z;
    m[1][1] += d.y * d.y;
    m[1][2] += d.y * d.z;
    m[2][2] += d.z * d.z;
  }
  m[1][0] = m[0][1];
  m[2][0] = m[0][2];
  m[2][1] = m[1][2];
  Vec3 n = SmallestEigenvector(m);
  if (Dot(n, nominal) < 0)
    n = Mul(n, -1);
  return {n, -Dot(n, c), c};
}

double Distance(const Plane &p, Vec3 x) { return Dot(p.n, x) + p.d; }
double AngleDeg(Vec3 a, Vec3 b) {
  return std::acos(Clamp(Dot(Normalize(a), Normalize(b)), -1, 1)) *
         57.29577951308232;
}

Plane RansacPlane(const std::vector<LocalPoint> &points,
                  const std::vector<size_t> &reference,
                  const CadInspectOptions &o, Vec3 nominal,
                  std::vector<size_t> *inliers,
                  uint32_t *iterations_used = nullptr,
                  uint32_t *evaluation_count = nullptr) {
  std::mt19937_64 rng(o.random_seed);
  std::vector<size_t> evaluation = reference;
  if (o.ransac_evaluation_limit > 0 &&
      evaluation.size() > o.ransac_evaluation_limit) {
    std::shuffle(evaluation.begin(), evaluation.end(), rng);
    evaluation.resize(o.ransac_evaluation_limit);
  }
  g_ransac_evaluation_count += static_cast<uint32_t>(evaluation.size());
  if (evaluation_count)
    *evaluation_count = static_cast<uint32_t>(evaluation.size());
  std::uniform_int_distribution<size_t> pick(0, evaluation.size() - 1);
  size_t best_count = 0;
  double best_error = std::numeric_limits<double>::infinity();
  Plane best{};
  const double cos_limit =
      std::cos(o.max_normal_angle_deg * 3.14159265358979323846 / 180.0);
  uint32_t adaptive_limit = o.ransac_max_iterations;
  uint32_t iteration = 0;
  for (; iteration < adaptive_limit; ++iteration) {
    size_t ia = evaluation[pick(rng)], ib = evaluation[pick(rng)],
           ic = evaluation[pick(rng)];
    if (ia == ib || ia == ic || ib == ic)
      continue;
    Vec3 a = points[ia].world, b = points[ib].world, c = points[ic].world;
    Vec3 cross = Cross(Sub(b, a), Sub(c, a));
    double length = Norm(cross);
    if (length < 1e-10)
      continue;
    Vec3 n = Mul(cross, 1.0 / length);
    if (Dot(n, nominal) < 0)
      n = Mul(n, -1);
    if (Dot(n, nominal) < cos_limit)
      continue;
    Plane plane{n, -Dot(n, a), a};
    size_t count = 0;
    double error = 0;
    for (size_t id : evaluation) {
      double d = std::abs(Distance(plane, points[id].world));
      if (d <= o.plane_inlier_distance) {
        ++count;
        error += d * d;
      }
    }
    if (count > best_count || (count == best_count && error < best_error)) {
      best_count = count;
      best_error = error;
      best = plane;
      const double ratio = static_cast<double>(count) / evaluation.size();
      const double success = ratio * ratio * ratio;
      if (success > 0 && success < 1) {
        const double required =
            std::log(1.0 - o.ransac_confidence) / std::log(1.0 - success);
        adaptive_limit =
            std::min(adaptive_limit,
                     static_cast<uint32_t>(std::max(1.0, std::ceil(required))));
      } else if (success >= 1) {
        adaptive_limit = 1;
      }
    }
  }
  g_ransac_iterations_used += iteration;
  if (iterations_used)
    *iterations_used = iteration;
  if (best_count < 3)
    throw std::runtime_error("RANSAC found no constrained plane");
  inliers->clear();
  for (size_t id : reference)
    if (std::abs(Distance(best, points[id].world)) <= o.plane_inlier_distance)
      inliers->push_back(id);
  for (int pass = 0; pass < 2; ++pass) {
    best = FitPlane(points, *inliers, nominal);
    std::vector<size_t> next;
    next.reserve(reference.size());
    for (size_t id : reference)
      if (std::abs(Distance(best, points[id].world)) <= o.plane_inlier_distance)
        next.push_back(id);
    if (next.size() < 3)
      break;
    inliers->swap(next);
  }
  return best;
}

CadInspectPlaneCandidate EvaluatePlaneCandidate(
    const std::vector<LocalPoint> &points, const std::vector<size_t> &inliers,
    const Plane &plane, const CadInspectOptions &o, Vec3 nominal,
    Vec3 nominal_origin, size_t total_reference_points, bool wide_mode) {
  CadInspectPlaneCandidate output{};
  output.coefficients[0] = plane.n.x;
  output.coefficients[1] = plane.n.y;
  output.coefficients[2] = plane.n.z;
  output.coefficients[3] = plane.d;
  output.normal[0] = plane.n.x;
  output.normal[1] = plane.n.y;
  output.normal[2] = plane.n.z;
  output.centroid[0] = plane.center.x;
  output.centroid[1] = plane.center.y;
  output.centroid[2] = plane.center.z;
  output.normal_angle_deg = AngleDeg(plane.n, nominal);
  output.nominal_offset = Dot(nominal, Sub(plane.center, nominal_origin));
  output.inlier_count = inliers.size();
  output.point_ratio =
      total_reference_points == 0
          ? 0.0
          : static_cast<double>(inliers.size()) / total_reference_points;
  double squared = 0.0, absolute = 0.0, maximum = 0.0;
  std::vector<uint8_t> occupied(
      static_cast<size_t>(o.coverage_grid_u) * o.coverage_grid_v, 0);
  for (size_t id : inliers) {
    const double error = std::abs(Distance(plane, points[id].world));
    squared += error * error;
    absolute += error;
    maximum = std::max(maximum, error);
    const uint32_t gu = std::min(
        o.coverage_grid_u - 1,
        static_cast<uint32_t>((points[id].u - o.u_min) / (o.u_max - o.u_min) *
                              o.coverage_grid_u));
    const uint32_t gv = std::min(
        o.coverage_grid_v - 1,
        static_cast<uint32_t>((points[id].v - o.v_min) / (o.v_max - o.v_min) *
                              o.coverage_grid_v));
    occupied[static_cast<size_t>(gv) * o.coverage_grid_u + gu] = 1;
  }
  if (!inliers.empty()) {
    output.rmse = std::sqrt(squared / inliers.size());
    output.mean_abs_error = absolute / inliers.size();
    output.max_abs_error = maximum;
  }
  output.grid_coverage = static_cast<double>(std::count(
                             occupied.begin(), occupied.end(), uint8_t{1})) /
                         occupied.size();
  const double required_ratio =
      wide_mode ? o.min_candidate_point_ratio : o.min_reference_inlier_ratio;
  output.accepted = inliers.size() >= o.min_reference_points &&
                    output.point_ratio >= required_ratio &&
                    output.grid_coverage >= o.min_reference_grid_coverage &&
                    output.rmse <= o.max_reference_rmse &&
                    output.normal_angle_deg <= o.max_normal_angle_deg;
  return output;
}

struct Range {
  double low, high;
};
Range QuantileRange(std::vector<double> values, double trim) {
  std::sort(values.begin(), values.end());
  size_t n = values.size();
  size_t lo = static_cast<size_t>(std::floor(trim * n));
  size_t hi = n - 1 - lo;
  if (hi < lo)
    hi = lo;
  return {values[lo], values[hi]};
}
double ResidualRange(const std::vector<LocalPoint> &points, double a, double b,
                     double *low = nullptr, double *high = nullptr) {
  double mn = std::numeric_limits<double>::infinity(), mx = -mn;
  for (const auto &p : points) {
    double r = p.w - a * p.u - b * p.v;
    mn = std::min(mn, r);
    mx = std::max(mx, r);
  }
  if (low)
    *low = mn;
  if (high)
    *high = mx;
  return mx - mn;
}

void ComputeFlatness(const std::vector<LocalPoint> &detection, Vec3 u, Vec3 v,
                     Vec3 n, const CadInspectOptions &o,
                     CadInspectFlatnessResult *r,
                     uint64_t *working_count = nullptr) {
  r->evaluated_point_count = detection.size();
  std::vector<size_t> all(detection.size());
  for (size_t i = 0; i < all.size(); ++i)
    all[i] = i;
  Plane ls = FitPlane(detection, all, n);
  std::vector<double> deviations;
  deviations.reserve(detection.size());
  double mn = std::numeric_limits<double>::infinity(), mx = -mn;
  for (const auto &p : detection) {
    double d = Distance(ls, p.world);
    deviations.push_back(d);
    mn = std::min(mn, d);
    mx = std::max(mx, d);
  }
  r->least_squares_min_deviation = mn;
  r->least_squares_max_deviation = mx;
  r->least_squares_peak_to_valley = mx - mn;
  Range robust = QuantileRange(deviations, o.flatness_trim_fraction);
  r->robust_min_deviation = robust.low;
  r->robust_max_deviation = robust.high;
  r->robust_peak_to_valley = robust.high - robust.low;
  /* Preserve the low/high point of each UV cell for minimum-zone search. */
  std::vector<LocalPoint> working;
  if (o.flatness_working_grid_size > 0) {
    struct Extremes {
      size_t low = 0, high = 0;
      bool set = false;
    };
    std::unordered_map<CellKey, Extremes, CellHash> extrema;
    for (size_t i = 0; i < detection.size(); ++i) {
      const auto &p = detection[i];
      CellKey key{
          static_cast<int64_t>(std::floor(p.u / o.flatness_working_grid_size)),
          static_cast<int64_t>(std::floor(p.v / o.flatness_working_grid_size))};
      auto &e = extrema[key];
      if (!e.set) {
        e.low = e.high = i;
        e.set = true;
      } else {
        if (p.w < detection[e.low].w)
          e.low = i;
        if (p.w > detection[e.high].w)
          e.high = i;
      }
    }
    working.reserve(extrema.size() * 2);
    for (const auto &item : extrema) {
      working.push_back(detection[item.second.low]);
      if (item.second.high != item.second.low)
        working.push_back(detection[item.second.high]);
    }
  } else
    working = detection;
  g_flatness_working_count = working.size();
  if (working_count)
    *working_count = working.size();
  /* Pattern-search the convex residual range in local slope space. */
  double a = 0, b = 0;
  double span_u = std::max(1e-9, o.u_max - o.u_min),
         span_v = std::max(1e-9, o.v_max - o.v_min);
  double step = std::max((mx - mn) / std::min(span_u, span_v), 1e-5);
  double best = ResidualRange(working, a, b);
  uint32_t iteration = 0;
  for (; iteration < o.minimum_zone_max_iterations &&
         step > o.minimum_zone_tolerance;
       ++iteration) {
    bool improved = false;
    double ba = a, bb = b;
    const double da[8]{step, -step, 0, 0, step, step, -step, -step};
    const double db[8]{0, 0, step, -step, step, -step, step, -step};
    for (int k = 0; k < 8; ++k) {
      double value = ResidualRange(working, a + da[k], b + db[k]);
      if (value < best) {
        best = value;
        ba = a + da[k];
        bb = b + db[k];
        improved = true;
      }
    }
    a = ba;
    b = bb;
    if (!improved)
      step *= 0.5;
  }
  r->minimum_zone_flatness = best;
  Vec3 mz = Normalize(Sub(n, Add(Mul(u, a), Mul(v, b))));
  if (Dot(mz, n) < 0)
    mz = Mul(mz, -1);
  r->minimum_zone_normal[0] = mz.x;
  r->minimum_zone_normal[1] = mz.y;
  r->minimum_zone_normal[2] = mz.z;
  r->minimum_zone_iterations = iteration;
  r->minimum_zone_converged = step <= o.minimum_zone_tolerance ? 1u : 0u;
}

bool Valid(const CadInspectPointCloud *c, const CadInspectOptions *o,
           CadInspectResult *r) {
  if (!c || !c->xyz || c->point_count < 3 || !o ||
      o->struct_size != sizeof(*o) ||
      o->abi_version != CADINSPECT_ABI_VERSION) {
    SetError(r, CADINSPECT_STATUS_INVALID_ARGUMENT,
             "invalid cloud or ABI/options header");
    return false;
  }
  const bool ranges = o->u_min < o->u_max && o->v_min < o->v_max &&
                      o->detection_normal_min < o->detection_normal_max &&
                      o->reference_normal_min < o->reference_normal_max;
  if (!ranges || o->edge_margin < 0 || o->ransac_max_iterations == 0 ||
      o->reference_mode < CADINSPECT_REFERENCE_NARROW_ROI ||
      o->reference_mode > CADINSPECT_REFERENCE_WIDE_MULTIPLANE ||
      o->max_plane_candidates == 0 ||
      o->max_plane_candidates > CADINSPECT_MAX_PLANE_CANDIDATES ||
      o->min_candidate_point_ratio < 0 || o->min_candidate_point_ratio > 1 ||
      !(o->ransac_confidence > 0 && o->ransac_confidence < 1) ||
      !(o->plane_inlier_distance > 0) ||
      !(o->max_normal_angle_deg > 0 && o->max_normal_angle_deg < 90) ||
      o->min_reference_points < 3 || o->min_reference_inlier_ratio < 0 ||
      o->min_reference_inlier_ratio > 1 || o->coverage_grid_u == 0 ||
      o->coverage_grid_v == 0 || o->min_reference_grid_coverage < 0 ||
      o->min_reference_grid_coverage > 1 || !(o->max_reference_rmse > 0) ||
      !(o->positive_defect_threshold > 0) ||
      !(o->negative_defect_threshold > 0) ||
      !(o->defect_cluster_cell_size > 0) || o->min_defect_points == 0 ||
      o->max_output_defects == 0 || o->flatness_trim_fraction < 0 ||
      o->flatness_trim_fraction >= 0.5 ||
      !std::isfinite(o->flatness_working_grid_size) ||
      o->minimum_zone_max_iterations == 0 || !(o->minimum_zone_tolerance > 0)) {
    SetError(r, CADINSPECT_STATUS_INVALID_ARGUMENT,
             "invalid ROI, plane, flatness, or defect options");
    return false;
  }
  return true;
}
} // namespace

extern "C" {
uint32_t CADINSPECT_CALL cadinspect_get_abi_version(void) {
  return CADINSPECT_ABI_VERSION;
}
void CADINSPECT_CALL cadinspect_default_options(CadInspectOptions *o) {
  if (!o)
    return;
  *o = {};
  o->struct_size = sizeof(*o);
  o->abi_version = CADINSPECT_ABI_VERSION;
#if defined(CADINSPECT_DEFAULT_WIDE)
  o->reference_mode = CADINSPECT_REFERENCE_WIDE_MULTIPLANE;
#else
  o->reference_mode = CADINSPECT_REFERENCE_NARROW_ROI;
#endif
  o->max_plane_candidates = 4;
  o->min_candidate_point_ratio = 0.03;
  o->frame.axis_u[0] = 1;
  o->frame.axis_v[1] = 1;
  o->frame.nominal_normal[2] = 1;
  o->u_min = -50;
  o->u_max = 50;
  o->v_min = -50;
  o->v_max = 50;
  o->detection_normal_min = -5;
  o->detection_normal_max = 20;
  o->reference_normal_min = -1;
  o->reference_normal_max = 0.5;
  o->edge_margin = 1;
  o->ransac_max_iterations = 1000;
  o->random_seed = 1;
  o->ransac_evaluation_limit = 5000;
  o->ransac_confidence = 0.999;
  o->plane_inlier_distance = 0.15;
  o->max_normal_angle_deg = 5;
  o->min_reference_points = 100;
  o->min_reference_inlier_ratio = 0.5;
  o->coverage_grid_u = 10;
  o->coverage_grid_v = 10;
  o->min_reference_grid_coverage = 0.2;
  o->max_reference_rmse = 0.1;
  o->positive_defect_threshold = 0.3;
  o->negative_defect_threshold = 0.3;
  o->defect_cluster_cell_size = 0.5;
  o->min_defect_points = 5;
  o->min_defect_area = 0.25;
  o->max_output_defects = 256;
  o->flatness_trim_fraction = 0.001;
  o->flatness_working_grid_size = 0.5;
  o->minimum_zone_max_iterations = 100;
  o->minimum_zone_tolerance = 1e-7;
}
void CADINSPECT_CALL cadinspect_init_buffers(CadInspectBuffers *b) {
  if (!b)
    return;
  *b = {};
  b->struct_size = sizeof(*b);
  b->abi_version = CADINSPECT_ABI_VERSION;
}
void CADINSPECT_CALL cadinspect_init_result(CadInspectResult *r) {
  if (!r)
    return;
  *r = {};
  r->struct_size = sizeof(*r);
  r->abi_version = CADINSPECT_ABI_VERSION;
  r->status = CADINSPECT_STATUS_INTERNAL_ERROR;
}

CadInspectStatus CADINSPECT_CALL
cadinspect_analyze(const CadInspectPointCloud *c, const CadInspectOptions *o,
                   CadInspectBuffers *b, CadInspectResult *r) {
  if (!r || r->struct_size != sizeof(*r) ||
      r->abi_version != CADINSPECT_ABI_VERSION)
    return CADINSPECT_STATUS_INVALID_ARGUMENT;
  cadinspect_init_result(r);
  const auto started = Clock::now();
  g_ransac_iterations_used = 0;
  g_ransac_evaluation_count = 0;
  g_flatness_working_count = 0;
  try {
    if (!Valid(c, o, r)) {
      r->elapsed_ms = Elapsed(started);
      return r->status;
    }
    Vec3 origin{o->frame.origin[0], o->frame.origin[1], o->frame.origin[2]};
    Vec3 n = Normalize({o->frame.nominal_normal[0], o->frame.nominal_normal[1],
                        o->frame.nominal_normal[2]});
    Vec3 u0{o->frame.axis_u[0], o->frame.axis_u[1], o->frame.axis_u[2]};
    Vec3 u = Normalize(Sub(u0, Mul(n, Dot(u0, n))));
    Vec3 v = Normalize(Cross(n, u));
    Vec3 requested_v{o->frame.axis_v[0], o->frame.axis_v[1],
                     o->frame.axis_v[2]};
    if (Dot(v, requested_v) < 0) {
      v = Mul(v, -1);
      u = Mul(u, -1);
    }
    r->input_point_count = c->point_count;
    const uint64_t stride =
        c->xyz_stride_bytes ? c->xyz_stride_bytes : 3 * sizeof(double);
    if (stride < 3 * sizeof(double)) {
      SetError(r, CADINSPECT_STATUS_INVALID_ARGUMENT,
               "XYZ stride is too small");
      r->elapsed_ms = Elapsed(started);
      return r->status;
    }
    const auto roi_started = Clock::now();
    std::vector<LocalPoint> detection;
    std::vector<size_t> reference;
    detection.reserve(static_cast<size_t>(c->point_count));
    const unsigned char *bytes =
        reinterpret_cast<const unsigned char *>(c->xyz);
    for (uint64_t i = 0; i < c->point_count; ++i) {
      double xyz[3];
      std::memcpy(xyz, bytes + i * stride, sizeof(xyz));
      if (!std::isfinite(xyz[0]) || !std::isfinite(xyz[1]) ||
          !std::isfinite(xyz[2]))
        continue;
      ++r->finite_point_count;
      Vec3 world{xyz[0], xyz[1], xyz[2]}, delta = Sub(world, origin);
      double lu = Dot(delta, u), lv = Dot(delta, v), lw = Dot(delta, n);
      if (lu < o->u_min || lu > o->u_max || lv < o->v_min || lv > o->v_max ||
          lw < o->detection_normal_min || lw > o->detection_normal_max)
        continue;
      size_t index = detection.size();
      detection.push_back({world, lu, lv, lw, i});
      bool edge =
          lu < o->u_min + o->edge_margin || lu > o->u_max - o->edge_margin ||
          lv < o->v_min + o->edge_margin || lv > o->v_max - o->edge_margin;
      if (edge) {
        ++r->rejected_edge_point_count;
        continue;
      }
      if (o->reference_mode == CADINSPECT_REFERENCE_WIDE_MULTIPLANE ||
          (lw >= o->reference_normal_min && lw <= o->reference_normal_max))
        reference.push_back(index);
    }
    r->detection_roi_point_count = detection.size();
    r->reference_roi_point_count = reference.size();
    r->roi_elapsed_ms = Elapsed(roi_started);
    if (detection.size() < 3) {
      SetError(r, CADINSPECT_STATUS_EMPTY_DETECTION_ROI,
               "detection ROI contains fewer than three points");
      r->elapsed_ms = Elapsed(started);
      return r->status;
    }
    if (reference.size() < o->min_reference_points) {
      SetError(r, CADINSPECT_STATUS_INSUFFICIENT_REFERENCE,
               "narrow reference ROI has insufficient points");
      r->elapsed_ms = Elapsed(started);
      return r->status;
    }
    const auto plane_started = Clock::now();
    const bool wide_mode =
        o->reference_mode == CADINSPECT_REFERENCE_WIDE_MULTIPLANE;
    std::vector<size_t> remaining = reference;
    std::vector<std::vector<size_t>> candidate_inliers;
    std::vector<Plane> candidate_planes;
    const uint32_t requested_candidates =
        wide_mode ? o->max_plane_candidates : 1u;
    for (uint32_t candidate_index = 0;
         candidate_index < requested_candidates &&
         remaining.size() >= o->min_reference_points;
         ++candidate_index) {
      std::vector<size_t> current_inliers;
      Plane current_plane{};
      try {
        current_plane =
            RansacPlane(detection, remaining, *o, n, &current_inliers);
      } catch (const std::exception &) {
        break;
      }
      CadInspectPlaneCandidate evaluated =
          EvaluatePlaneCandidate(detection, current_inliers, current_plane, *o,
                                 n, origin, reference.size(), wide_mode);
      r->plane_candidates[r->plane_candidate_count++] = evaluated;
      candidate_planes.push_back(current_plane);
      candidate_inliers.push_back(current_inliers);
      if (!wide_mode)
        break;
      std::vector<uint8_t> removed(detection.size(), 0);
      for (size_t id : current_inliers)
        removed[id] = 1;
      std::vector<size_t> next;
      next.reserve(remaining.size() - current_inliers.size());
      for (size_t id : remaining)
        if (!removed[id])
          next.push_back(id);
      if (next.size() == remaining.size())
        break;
      remaining.swap(next);
    }
    size_t selected = std::numeric_limits<size_t>::max();
    double closest_offset = std::numeric_limits<double>::infinity();
    for (size_t i = 0; i < r->plane_candidate_count; ++i) {
      const CadInspectPlaneCandidate &candidate = r->plane_candidates[i];
      if (!candidate.accepted)
        continue;
      const double distance = std::abs(candidate.nominal_offset);
      if (distance < closest_offset) {
        closest_offset = distance;
        selected = i;
      }
    }
    if (selected == std::numeric_limits<size_t>::max()) {
      SetError(
          r, CADINSPECT_STATUS_PLANE_REJECTED,
          "no plane candidate passed point, coverage, RMSE, and normal gates");
      r->plane_elapsed_ms = Elapsed(plane_started);
      r->elapsed_ms = Elapsed(started);
      return r->status;
    }
    r->plane_candidates[selected].selected = 1;
    r->selected_plane_candidate = static_cast<uint32_t>(selected + 1);
    const Plane plane = candidate_planes[selected];
    const std::vector<size_t> &inliers = candidate_inliers[selected];
    const CadInspectPlaneCandidate &selected_diagnostic =
        r->plane_candidates[selected];
    auto &pr = r->reference_plane;
    pr.candidate_count = reference.size();
    pr.inlier_count = inliers.size();
    pr.inlier_ratio = selected_diagnostic.point_ratio;
    pr.normal[0] = plane.n.x;
    pr.normal[1] = plane.n.y;
    pr.normal[2] = plane.n.z;
    pr.coefficients[0] = plane.n.x;
    pr.coefficients[1] = plane.n.y;
    pr.coefficients[2] = plane.n.z;
    pr.coefficients[3] = plane.d;
    pr.centroid[0] = plane.center.x;
    pr.centroid[1] = plane.center.y;
    pr.centroid[2] = plane.center.z;
    pr.normal_angle_deg = selected_diagnostic.normal_angle_deg;
    pr.nominal_offset = selected_diagnostic.nominal_offset;
    pr.rmse = selected_diagnostic.rmse;
    pr.mean_abs_error = selected_diagnostic.mean_abs_error;
    pr.max_abs_error = selected_diagnostic.max_abs_error;
    pr.total_grid_cells = o->coverage_grid_u * o->coverage_grid_v;
    pr.grid_coverage = selected_diagnostic.grid_coverage;
    pr.occupied_grid_cells = static_cast<uint32_t>(
        std::llround(pr.grid_coverage * pr.total_grid_cells));
    pr.reliable = selected_diagnostic.accepted;
    r->plane_elapsed_ms = Elapsed(plane_started);
    if (!pr.reliable) {
      SetError(r, CADINSPECT_STATUS_PLANE_REJECTED,
               "reference plane failed inlier, coverage, RMSE, or normal gate");
      r->elapsed_ms = Elapsed(started);
      return r->status;
    }
    const auto flat_started = Clock::now();
    ComputeFlatness(detection, u, v, n, *o, &r->flatness);
    r->flatness_elapsed_ms = Elapsed(flat_started);
    const auto defect_started = Clock::now();
    std::unordered_map<DefectCellKey, std::vector<size_t>, DefectCellHash>
        cells;
    for (size_t i = 0; i < detection.size(); ++i) {
      double e = Distance(plane, detection[i].world);
      int sign = e >= o->positive_defect_threshold
                     ? 1
                     : (e <= -o->negative_defect_threshold ? -1 : 0);
      if (!sign)
        continue;
      if (sign > 0)
        ++r->positive_defect_point_count;
      else
        ++r->negative_defect_point_count;
      DefectCellKey key{static_cast<int64_t>(std::floor(
                            detection[i].u / o->defect_cluster_cell_size)),
                        static_cast<int64_t>(std::floor(
                            detection[i].v / o->defect_cluster_cell_size)),
                        sign};
      cells[key].push_back(i);
    }
    struct DetectedDefect {
      CadInspectDefect value{};
      std::vector<uint64_t> input_indices;
    };
    std::vector<DetectedDefect> defects;
    std::unordered_map<DefectCellKey, uint8_t, DefectCellHash> visited_cells;
    for (const auto &seed_item : cells) {
      const DefectCellKey seed = seed_item.first;
      if (visited_cells.find(seed) != visited_cells.end())
        continue;
      std::queue<DefectCellKey> q;
      q.push(seed);
      visited_cells[seed] = 1;
      std::vector<size_t> cluster;
      uint64_t cluster_cell_count = 0;
      while (!q.empty()) {
        DefectCellKey cell = q.front();
        q.pop();
        ++cluster_cell_count;
        const auto points_in_cell = cells.find(cell);
        if (points_in_cell != cells.end())
          cluster.insert(cluster.end(), points_in_cell->second.begin(),
                         points_in_cell->second.end());
        for (int du = -1; du <= 1; ++du)
          for (int dv = -1; dv <= 1; ++dv) {
            DefectCellKey neighbor{cell.u + du, cell.v + dv, cell.sign};
            if (cells.find(neighbor) == cells.end() ||
                visited_cells.find(neighbor) != visited_cells.end())
              continue;
            visited_cells[neighbor] = 1;
            q.push(neighbor);
          }
      }
      double area = cluster_cell_count * o->defect_cluster_cell_size *
                    o->defect_cluster_cell_size;
      if (cluster.size() < o->min_defect_points || area < o->min_defect_area)
        continue;
      DetectedDefect detected;
      CadInspectDefect &d = detected.value;
      d.type = seed.sign > 0 ? CADINSPECT_DEFECT_POSITIVE
                             : CADINSPECT_DEFECT_NEGATIVE;
      d.point_count = cluster.size();
      d.projected_area = area;
      d.maximum_height = -std::numeric_limits<double>::infinity();
      d.minimum_height = std::numeric_limits<double>::infinity();
      d.local_bounds_min[0] = d.local_bounds_min[1] = d.local_bounds_min[2] =
          std::numeric_limits<double>::infinity();
      d.local_bounds_max[0] = d.local_bounds_max[1] = d.local_bounds_max[2] =
          -std::numeric_limits<double>::infinity();
      Vec3 center{0, 0, 0};
      double sum = 0;
      detected.input_indices.reserve(cluster.size());
      for (size_t point_index : cluster) {
        const LocalPoint &p = detection[point_index];
        detected.input_indices.push_back(p.input_index);
        double h = Distance(plane, p.world);
        center = Add(center, p.world);
        sum += h;
        d.maximum_height = std::max(d.maximum_height, h);
        d.minimum_height = std::min(d.minimum_height, h);
        double vals[3]{p.u, p.v, h};
        for (int k = 0; k < 3; ++k) {
          d.local_bounds_min[k] = std::min(d.local_bounds_min[k], vals[k]);
          d.local_bounds_max[k] = std::max(d.local_bounds_max[k], vals[k]);
        }
      }
      center = Mul(center, 1.0 / cluster.size());
      d.centroid[0] = center.x;
      d.centroid[1] = center.y;
      d.centroid[2] = center.z;
      d.mean_height = sum / cluster.size();
      d.estimated_volume = area * d.mean_height;
      defects.push_back(std::move(detected));
    }
    std::sort(defects.begin(), defects.end(),
              [](const DetectedDefect &a, const DetectedDefect &b) {
                return a.value.projected_area > b.value.projected_area;
              });
    if (defects.size() > o->max_output_defects)
      defects.resize(o->max_output_defects);
    r->required_defect_capacity = static_cast<uint32_t>(defects.size());
    r->defect_count = static_cast<uint32_t>(defects.size());
    if (b && b->point_labels && b->point_label_capacity >= c->point_count)
      std::fill(b->point_labels, b->point_labels + c->point_count, 0u);
    bool too_small = false;
    if (!defects.empty() &&
        (!b || b->struct_size != sizeof(*b) ||
         b->abi_version != CADINSPECT_ABI_VERSION || !b->defects ||
         b->defect_capacity < defects.size()))
      too_small = true;
    uint32_t copy_count = too_small ? 0 : static_cast<uint32_t>(defects.size());
    for (uint32_t i = 0; i < copy_count; ++i) {
      defects[i].value.id = i + 1;
      b->defects[i] = defects[i].value;
    }
    if (!too_small && b && b->point_labels &&
        b->point_label_capacity >= c->point_count) {
      for (uint32_t di = 0; di < copy_count; ++di) {
        for (uint64_t input_index : defects[di].input_indices)
          b->point_labels[input_index] = di + 1;
      }
    }
    r->defect_elapsed_ms = Elapsed(defect_started);
    if (too_small)
      SetError(r, CADINSPECT_STATUS_BUFFER_TOO_SMALL,
               "defect output buffer is smaller than required");
    else
      r->status = CADINSPECT_STATUS_SUCCESS;
  } catch (const std::invalid_argument &e) {
    SetError(r, CADINSPECT_STATUS_INVALID_ARGUMENT, e.what());
  } catch (const std::exception &e) {
    SetError(r, CADINSPECT_STATUS_INTERNAL_ERROR, e.what());
  } catch (...) {
    SetError(r, CADINSPECT_STATUS_INTERNAL_ERROR, "unknown inspection error");
  }
  r->ransac_iterations_used = g_ransac_iterations_used;
  r->ransac_evaluation_point_count = g_ransac_evaluation_count;
  r->flatness_working_point_count = g_flatness_working_count;
  r->elapsed_ms = Elapsed(started);
  return r->status;
}
} // extern C
