#include "cad_sampling.h"

#include <cstdio>
#include <vector>

int main(int argc, char** argv) {
    if (argc != 2) {
        std::fprintf(stderr, "usage: cadsample_example model.step\n");
        return 2;
    }

    CadSampleHandle handle = cadsample_create();
    if (!handle) return 3;
    CadSampleResult result{};
    cadsample_init_result(&result);
    CadSampleStatus status = cadsample_load_step(handle, argv[1], &result);
    if (status != CADSAMPLE_STATUS_SUCCESS) {
        std::fprintf(stderr, "load failed: %s\n", result.error_message);
        cadsample_destroy(handle);
        return 4;
    }

    CadSampleOptions options{};
    cadsample_default_options(&options);
    status = cadsample_generate(handle, &options, nullptr, &result);
    if (status != CADSAMPLE_STATUS_BUFFER_TOO_SMALL) {
        std::fprintf(stderr, "query failed: %s\n", result.error_message);
        cadsample_destroy(handle);
        return 5;
    }

    std::vector<double> xyz(result.required_point_capacity * 3);
    std::vector<double> normals(result.required_point_capacity * 3);
    std::vector<uint32_t> face_ids(result.required_point_capacity);
    CadSampleBuffers buffers{};
    cadsample_init_buffers(&buffers);
    buffers.capacity_points = result.required_point_capacity;
    buffers.xyz = xyz.data();
    buffers.normals = normals.data();
    buffers.face_ids = face_ids.data();
    status = cadsample_generate(handle, &options, &buffers, &result);
    std::printf("status=%d points=%llu faces=%llu triangles=%llu "
                "area=%.6f load=%.3fms sample=%.3fms\n",
                static_cast<int>(status),
                static_cast<unsigned long long>(result.point_count),
                static_cast<unsigned long long>(result.face_count),
                static_cast<unsigned long long>(result.triangle_count),
                result.surface_area, result.load_elapsed_ms,
                result.sample_elapsed_ms);
    if (status != CADSAMPLE_STATUS_SUCCESS)
        std::fprintf(stderr, "%s\n", result.error_message);
    cadsample_destroy(handle);
    return status == CADSAMPLE_STATUS_SUCCESS ? 0 : 6;
}
