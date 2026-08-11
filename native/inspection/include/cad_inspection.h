#ifndef CAD_INSPECTION_H
#define CAD_INSPECTION_H

#include <stdint.h>

#if defined(_WIN32)
#  if defined(CADINSPECT_BUILDING_DLL)
#    define CADINSPECT_API __declspec(dllexport)
#  else
#    define CADINSPECT_API __declspec(dllimport)
#  endif
#  define CADINSPECT_CALL __cdecl
#else
#  define CADINSPECT_API __attribute__((visibility("default")))
#  define CADINSPECT_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define CADINSPECT_ABI_VERSION 3u
#define CADINSPECT_ERROR_MESSAGE_SIZE 512u
#define CADINSPECT_MAX_PLANE_CANDIDATES 8u

typedef enum CadInspectStatus {
    CADINSPECT_STATUS_SUCCESS = 0,
    CADINSPECT_STATUS_INVALID_ARGUMENT = 1,
    CADINSPECT_STATUS_EMPTY_DETECTION_ROI = 2,
    CADINSPECT_STATUS_INSUFFICIENT_REFERENCE = 3,
    CADINSPECT_STATUS_PLANE_REJECTED = 4,
    CADINSPECT_STATUS_BUFFER_TOO_SMALL = 5,
    CADINSPECT_STATUS_INTERNAL_ERROR = 6
} CadInspectStatus;

typedef struct CadInspectPointCloud {
    const double* xyz;
    uint64_t point_count;
    uint64_t xyz_stride_bytes; /* 0 means 3*sizeof(double). */
} CadInspectPointCloud;

typedef enum CadInspectReferenceMode {
    CADINSPECT_REFERENCE_NARROW_ROI = 0,
    CADINSPECT_REFERENCE_WIDE_MULTIPLANE = 1
} CadInspectReferenceMode;

/* CAD-local oriented ROI. Axes are normalized and orthogonalized internally. */
typedef struct CadInspectFrame {
    double origin[3];
    double axis_u[3];
    double axis_v[3];
    double nominal_normal[3];
} CadInspectFrame;

typedef struct CadInspectOptions {
    uint32_t struct_size;
    uint32_t abi_version;
    CadInspectFrame frame;
    CadInspectReferenceMode reference_mode;
    uint32_t max_plane_candidates;
    double min_candidate_point_ratio;
    double u_min;
    double u_max;
    double v_min;
    double v_max;
    /* High ROI used for defects and flatness. */
    double detection_normal_min;
    double detection_normal_max;
    /* Narrow slab used only to find the reference plane. */
    double reference_normal_min;
    double reference_normal_max;
    double edge_margin;
    uint32_t ransac_max_iterations;
    uint32_t random_seed;
    /* Uniform representative subset used during hypothesis scoring; 0=all. */
    uint32_t ransac_evaluation_limit;
    double ransac_confidence;
    double plane_inlier_distance;
    double max_normal_angle_deg;
    uint64_t min_reference_points;
    double min_reference_inlier_ratio;
    /* Reference points must occupy this fraction of an XY grid. */
    uint32_t coverage_grid_u;
    uint32_t coverage_grid_v;
    double min_reference_grid_coverage;
    double max_reference_rmse;
    /* Signed residual thresholds relative to the fitted plane. */
    double positive_defect_threshold;
    double negative_defect_threshold;
    double defect_cluster_cell_size;
    uint64_t min_defect_points;
    double min_defect_area;
    uint32_t max_output_defects;
    /* Percent removed from each tail for the diagnostic robust flatness. */
    double flatness_trim_fraction;
    /* Grid extrema are used by minimum-zone iteration; <=0 uses all points. */
    double flatness_working_grid_size;
    uint32_t minimum_zone_max_iterations;
    double minimum_zone_tolerance;
} CadInspectOptions;

typedef struct CadInspectPlaneCandidate {
    double coefficients[4];
    double normal[3];
    double centroid[3];
    double normal_angle_deg;
    double nominal_offset;
    double rmse;
    double mean_abs_error;
    double max_abs_error;
    uint64_t inlier_count;
    double point_ratio;
    double grid_coverage;
    uint32_t accepted;
    uint32_t selected;
} CadInspectPlaneCandidate;

typedef struct CadInspectPlaneResult {
    /* Ax+By+Cz+D=0; ABC is a unit normal aligned with nominal_normal. */
    double coefficients[4];
    double normal[3];
    double centroid[3];
    double normal_angle_deg;
    double nominal_offset;
    double rmse;
    double mean_abs_error;
    double max_abs_error;
    uint64_t candidate_count;
    uint64_t inlier_count;
    double inlier_ratio;
    uint32_t occupied_grid_cells;
    uint32_t total_grid_cells;
    double grid_coverage;
    uint32_t reliable;
} CadInspectPlaneResult;

typedef struct CadInspectFlatnessResult {
    uint64_t evaluated_point_count;
    /* Full high ROI; defects are deliberately included. */
    double least_squares_peak_to_valley;
    double least_squares_min_deviation;
    double least_squares_max_deviation;
    /* Diagnostic noise-resistant value, not a replacement for total value. */
    double robust_peak_to_valley;
    double robust_min_deviation;
    double robust_max_deviation;
    /* Iteratively optimized minimum-zone approximation. */
    double minimum_zone_flatness;
    double minimum_zone_normal[3];
    uint32_t minimum_zone_converged;
    uint32_t minimum_zone_iterations;
} CadInspectFlatnessResult;

typedef enum CadInspectDefectType {
    CADINSPECT_DEFECT_POSITIVE = 1,
    CADINSPECT_DEFECT_NEGATIVE = 2
} CadInspectDefectType;

typedef struct CadInspectDefect {
    uint32_t id;
    CadInspectDefectType type;
    uint64_t point_count;
    double projected_area;
    double maximum_height;
    double minimum_height;
    double mean_height;
    double estimated_volume;
    double centroid[3];
    double local_bounds_min[3]; /* u, v, signed plane residual */
    double local_bounds_max[3];
} CadInspectDefect;

typedef struct CadInspectBuffers {
    uint32_t struct_size;
    uint32_t abi_version;
    uint32_t defect_capacity;
    CadInspectDefect* defects;
    /* Optional, one value per input point: 0=none, otherwise defect id. */
    uint64_t point_label_capacity;
    uint32_t* point_labels;
} CadInspectBuffers;

typedef struct CadInspectResult {
    uint32_t struct_size;
    uint32_t abi_version;
    CadInspectStatus status;
    uint64_t input_point_count;
    uint64_t finite_point_count;
    uint64_t detection_roi_point_count;
    uint64_t reference_roi_point_count;
    uint64_t rejected_edge_point_count;
    uint32_t plane_candidate_count;
    uint32_t selected_plane_candidate;
    CadInspectPlaneCandidate
        plane_candidates[CADINSPECT_MAX_PLANE_CANDIDATES];
    CadInspectPlaneResult reference_plane;
    CadInspectFlatnessResult flatness;
    uint32_t defect_count;
    uint32_t required_defect_capacity;
    uint64_t positive_defect_point_count;
    uint64_t negative_defect_point_count;
    uint32_t ransac_iterations_used;
    uint32_t ransac_evaluation_point_count;
    uint64_t flatness_working_point_count;
    double roi_elapsed_ms;
    double plane_elapsed_ms;
    double flatness_elapsed_ms;
    double defect_elapsed_ms;
    double elapsed_ms;
    char error_message[CADINSPECT_ERROR_MESSAGE_SIZE];
} CadInspectResult;

CADINSPECT_API uint32_t CADINSPECT_CALL cadinspect_get_abi_version(void);
CADINSPECT_API void CADINSPECT_CALL cadinspect_default_options(
    CadInspectOptions* options);
CADINSPECT_API void CADINSPECT_CALL cadinspect_init_buffers(
    CadInspectBuffers* buffers);
CADINSPECT_API void CADINSPECT_CALL cadinspect_init_result(
    CadInspectResult* result);
CADINSPECT_API CadInspectStatus CADINSPECT_CALL cadinspect_analyze(
    const CadInspectPointCloud* cloud,
    const CadInspectOptions* options,
    CadInspectBuffers* buffers,
    CadInspectResult* result);

#ifdef __cplusplus
}
#endif
#endif
