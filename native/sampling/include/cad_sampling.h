#ifndef CAD_SAMPLING_H
#define CAD_SAMPLING_H

#include <stdint.h>

#if defined(_WIN32)
#  if defined(CADSAMPLE_BUILDING_DLL)
#    define CADSAMPLE_API __declspec(dllexport)
#  else
#    define CADSAMPLE_API __declspec(dllimport)
#  endif
#  define CADSAMPLE_CALL __cdecl
#else
#  define CADSAMPLE_API __attribute__((visibility("default")))
#  define CADSAMPLE_CALL
#endif

#ifdef __cplusplus
extern "C" {
#endif

#define CADSAMPLE_ABI_VERSION 5u
#define CADSAMPLE_ERROR_MESSAGE_SIZE 512u

typedef struct CadSampleHandleImpl* CadSampleHandle;

typedef enum CadSampleStatus {
    CADSAMPLE_STATUS_SUCCESS = 0,
    CADSAMPLE_STATUS_INVALID_ARGUMENT = 1,
    CADSAMPLE_STATUS_FILE_READ_FAILED = 2,
    CADSAMPLE_STATUS_NO_SHAPE = 3,
    CADSAMPLE_STATUS_MESH_FAILED = 4,
    CADSAMPLE_STATUS_BUFFER_TOO_SMALL = 5,
    CADSAMPLE_STATUS_INTERNAL_ERROR = 6
} CadSampleStatus;

typedef enum CadSampleSurfaceMode {
    /* Compatibility/debug mode: sample every triangulated STEP face. */
    CADSAMPLE_SURFACE_ALL_FACES = 0,
    /* Sample only the outer shell of solids; this is the industrial default. */
    CADSAMPLE_SURFACE_OUTER_SHELL = 1,
    /* Outer shell points that face the camera and are not occluded. */
    CADSAMPLE_SURFACE_VISIBLE = 2
} CadSampleSurfaceMode;

typedef enum CadSampleProjectionMode {
    /* Rays originate from camera_position (area/point camera model). */
    CADSAMPLE_PROJECTION_PERSPECTIVE = 0,
    /* Parallel rays; recommended for line-scan laser profilers. */
    CADSAMPLE_PROJECTION_ORTHOGRAPHIC = 1
} CadSampleProjectionMode;

typedef struct CadSampleOptions {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t target_point_count;
    double linear_deflection;
    double angular_deflection_deg;
    uint32_t relative_deflection;
    uint32_t parallel_meshing;
    uint32_t random_seed;
    /* <= 0 disables post-sampling voxel downsampling. */
    double voxel_size;
    uint32_t surface_mode;
    uint32_t projection_mode;
    /* Used by perspective visibility, in CAD/model coordinates. */
    double camera_position[3];
    /*
     * Orthographic ray travel direction from the sensor toward the object,
     * in CAD/model coordinates. It is normalized internally.
     * Examples: top view (0,0,-1), front view (0,-1,0).
     */
    double view_direction[3];
    /*
     * Reject grazing surfaces whose normal-to-sensor angle exceeds this
     * value. Valid range is (0, 90), default 75 degrees.
     */
    double max_incidence_angle_deg;
    /* > 0 overrides the automatic visibility ray tolerance. */
    double visibility_tolerance;
    /* Candidate multiplier for visible sampling; valid range [1, 32]. */
    uint32_t visibility_oversample_factor;
} CadSampleOptions;

/*
 * Arrays are allocated by the caller:
 *   xyz     : capacity_points * 3 doubles (required)
 *   normals : capacity_points * 3 doubles (optional)
 *   face_ids: capacity_points uint32 values (optional)
 */
typedef struct CadSampleBuffers {
    uint32_t struct_size;
    uint32_t abi_version;
    uint64_t capacity_points;
    double* xyz;
    double* normals;
    uint32_t* face_ids;
} CadSampleBuffers;

typedef struct CadSampleResult {
    uint32_t struct_size;
    uint32_t abi_version;
    CadSampleStatus status;
    uint64_t required_point_capacity;
    uint64_t point_count;
    uint64_t face_count;
    uint64_t triangle_count;
    uint64_t selected_triangle_count;
    double surface_area;
    double selected_surface_area;
    double load_elapsed_ms;
    double triangulation_elapsed_ms;
    double generation_elapsed_ms;
    double voxel_elapsed_ms;
    double visibility_elapsed_ms;
    double sample_elapsed_ms;
    uint32_t mesh_cache_hit;
    uint32_t sample_cache_hit;
    char error_message[CADSAMPLE_ERROR_MESSAGE_SIZE];
} CadSampleResult;

CADSAMPLE_API uint32_t CADSAMPLE_CALL cadsample_get_abi_version(void);
CADSAMPLE_API void CADSAMPLE_CALL cadsample_default_options(
    CadSampleOptions* options);
CADSAMPLE_API void CADSAMPLE_CALL cadsample_init_buffers(
    CadSampleBuffers* buffers);
CADSAMPLE_API void CADSAMPLE_CALL cadsample_init_result(
    CadSampleResult* result);

CADSAMPLE_API CadSampleHandle CADSAMPLE_CALL cadsample_create(void);
CADSAMPLE_API void CADSAMPLE_CALL cadsample_destroy(CadSampleHandle handle);

/* path_utf8 is copied during the call; no input pointer is retained. */
CADSAMPLE_API CadSampleStatus CADSAMPLE_CALL cadsample_load_step(
    CadSampleHandle handle,
    const char* path_utf8,
    CadSampleResult* result);

/*
 * Call with buffers == NULL to query required_point_capacity.
 * The loaded STEP shape is retained by the handle and can be sampled repeatedly.
 */
CADSAMPLE_API CadSampleStatus CADSAMPLE_CALL cadsample_generate(
    CadSampleHandle handle,
    const CadSampleOptions* options,
    CadSampleBuffers* buffers,
    CadSampleResult* result);

#ifdef __cplusplus
}
#endif

#endif
