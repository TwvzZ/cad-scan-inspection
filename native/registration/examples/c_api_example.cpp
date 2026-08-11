#include "cad_registration.h"

#include <array>
#include <cstdio>

int main() {
    const std::array<double, 15> source_xyz{
        0, 0, 0, 10, 0, 0, 0, 10, 0, 0, 0, 10, 5, 5, 5
    };
    const std::array<double, 15> target_xyz{
        2, 3, 4, 12, 3, 4, 2, 13, 4, 2, 3, 14, 7, 8, 9
    };
    const CadRegPointCloud source{source_xyz.data(), 5, 0};
    const CadRegPointCloud target{target_xyz.data(), 5, 0};

    CadRegOptions options{};
    cadreg_default_options(&options);
    options.max_correspondence_distance = 20.0;
    options.max_iterations = 100;
    options.voxel_size = 0.0;

    CadRegResult result{};
    cadreg_init_result(&result);
    const CadRegStatus status =
        cadreg_register(&source, &target, nullptr, &options, &result);

    std::printf("status=%d converged=%u rmse=%.6f inliers=%.3f time=%.3f ms\n",
        
                static_cast<int>(status), result.converged, result.rmse,
                result.inlier_ratio, result.elapsed_ms);
    std::printf("translation = [%.6f, %.6f, %.6f]\n",
                result.source_to_target[3],
                result.source_to_target[7],
                result.source_to_target[11]);
    if (result.error_message[0] != '\0') {
        std::printf("error: %s\n", result.error_message);
    }
    return status == CADREG_STATUS_SUCCESS ? 0 : 1;
}
