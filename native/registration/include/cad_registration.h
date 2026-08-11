#ifndef CAD_REGISTRATION_H
#define CAD_REGISTRATION_H

#include <stddef.h>
#include <stdint.h>

#if defined(_WIN32)
#  if defined(CADREG_BUILDING_DLL)
#    define CADREG_API __declspec(dllexport)
#  else
#    define CADREG_API __declspec(dllimport)
#  endif
#  define CADREG_CALL __cdecl
#else
#  define CADREG_API __attribute__((visibility("default")))
#  define CADREG_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define CADREG_ABI_VERSION 5u
#define CADREG_ERROR_MESSAGE_SIZE 512u
#define CADREG_MAX_ICP_LEVELS 4u
#define CADREG_MAX_CANDIDATE_DIAGNOSTICS 16u

typedef struct CadRegHandleImpl* CadRegHandle;

typedef enum CadRegStatus {
    CADREG_STATUS_SUCCESS = 0,
    CADREG_STATUS_NOT_CONVERGED = 1,
    CADREG_STATUS_INVALID_ARGUMENT = 2,
    CADREG_STATUS_EMPTY_AFTER_DOWNSAMPLING = 3,
    CADREG_STATUS_INTERNAL_ERROR = 4,
    CADREG_STATUS_QUALITY_REJECTED = 5,
    CADREG_STATUS_AMBIGUOUS = 6
} CadRegStatus;

typedef enum CadRegMode {
    CADREG_MODE_SINGLE = 0,
    CADREG_MODE_CASCADE = 1,
    CADREG_MODE_ENSEMBLE = 2
} CadRegMode;

typedef enum CadRegStrategy {
    CADREG_STRATEGY_INITIAL = 1,
    CADREG_STRATEGY_PCA = 2,
    CADREG_STRATEGY_FPFH_RANSAC = 4
} CadRegStrategy;

/* XYZ is interleaved double precision data. A zero stride means 3*sizeof(double). */
typedef struct CadRegPointCloud {
    const double* xyz;
    uint64_t point_count;
    uint64_t xyz_stride_bytes;
} CadRegPointCloud;

typedef struct CadRegIcpLevel {
    double voxel_size;
    double max_correspondence_distance;
    uint32_t max_iterations;
    uint32_t reserved;
} CadRegIcpLevel;

typedef struct CadRegOptions {
    uint32_t struct_size;
    uint32_t abi_version;
    double max_correspondence_distance;
    uint32_t max_iterations;
    double voxel_size; /* <= 0 disables downsampling. */
    CadRegMode mode;
    /* Bitwise OR of CadRegStrategy. Cascade order: initial, PCA, FPFH. */
    uint32_t strategy_mask;
    double feature_voxel_size;
    uint32_t ransac_max_iterations;
    double min_inlier_ratio;
    double max_rmse;
    double ambiguity_score_margin;
    uint32_t icp_level_count;
    CadRegIcpLevel icp_levels[CADREG_MAX_ICP_LEVELS];
    uint32_t ransac_attempts;
    uint32_t max_candidates_per_strategy;
    uint32_t enable_target_coverage;
    /* Cheaply rank seeds, then refine at most this many per strategy. */
    uint32_t max_refined_candidates_per_strategy;
    double min_target_coverage;
} CadRegOptions;

typedef struct CadRegCandidateDiagnostic {
    CadRegStrategy strategy;
    uint32_t converged;
    uint32_t accepted;
    uint32_t rank;
    /* Row-major source -> target transform for this candidate. */
    double source_to_target[16];
    double rmse;
    double inlier_ratio;
    double target_coverage;
    double score;
    /*
     * Seed generation is shared by candidates of the same strategy.
     * This value is repeated and must not be summed across candidates.
     */
    double shared_coarse_ms;
    double refinement_ms;
    double quality_ms;
    /* Per-candidate work only: refinement_ms + quality_ms. */
    double candidate_elapsed_ms;
} CadRegCandidateDiagnostic;

typedef struct CadRegResult {
    uint32_t struct_size;
    uint32_t abi_version;
    CadRegStatus status;
    uint32_t converged;
    /* Row-major matrix: p_target = transform * p_source. */
    double source_to_target[16];
    double rmse;
    double inlier_ratio;
    double elapsed_ms;
    uint64_t source_points_used;
    uint64_t target_points_used;
    CadRegStrategy selected_strategy;
    uint32_t candidate_count;
    uint32_t accepted_candidate_count;
    double score;
    double second_best_score;
    double target_coverage;
    uint32_t target_cache_hit;
    uint32_t reserved;
    double preprocessing_ms;
    double coarse_registration_ms;
    double refinement_ms;
    double quality_ms;
    uint32_t diagnostic_count;
    uint32_t diagnostic_reserved;
    CadRegCandidateDiagnostic
        diagnostics[CADREG_MAX_CANDIDATE_DIAGNOSTICS];
    char error_message[CADREG_ERROR_MESSAGE_SIZE];
} CadRegResult;

CADREG_API uint32_t CADREG_CALL cadreg_get_abi_version(void);
CADREG_API void CADREG_CALL cadreg_default_options(CadRegOptions* options);
CADREG_API void CADREG_CALL cadreg_init_result(CadRegResult* result);

/*
 * Production API: upload the stable CAD/reference target once, then register
 * many source scans without repeated target copying. A handle is not safe for
 * concurrent calls; use one handle per worker thread.
 */
CADREG_API CadRegHandle CADREG_CALL cadreg_create(void);
CADREG_API void CADREG_CALL cadreg_destroy(CadRegHandle handle);
CADREG_API CadRegStatus CADREG_CALL cadreg_set_target(
    CadRegHandle handle,
    const CadRegPointCloud* target);
CADREG_API CadRegStatus CADREG_CALL cadreg_register_source(
    CadRegHandle handle,
    const CadRegPointCloud* source,
    const double* initial_source_to_target,
    const CadRegOptions* options,
    CadRegResult* result);
CADREG_API const char* CADREG_CALL cadreg_last_error(CadRegHandle handle);

/*
 * Compatibility API. It creates a temporary context, so repeated production
 * calls should prefer cadreg_set_target + cadreg_register_source.
 * initial_source_to_target may be NULL. When supplied it points to 16 row-major
 * doubles. Input memory is borrowed only for the duration of this call.
 */
CADREG_API CadRegStatus CADREG_CALL cadreg_register(
    const CadRegPointCloud* source,
    const CadRegPointCloud* target,
    const double* initial_source_to_target,
    const CadRegOptions* options,
    CadRegResult* result);

#ifdef __cplusplus
}
#endif

#endif
