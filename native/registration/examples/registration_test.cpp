#include "cad_registration.h"

#if defined(_WIN32)
#  ifndef NOMINMAX
#    define NOMINMAX
#  endif
#  include <windows.h>
#endif

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <sstream>
#include <string>
#include <vector>

namespace {

struct Point {
    double x;
    double y;
    double z;
};

using Cloud = std::vector<Point>;

enum class PlyScalar {
    Int8, UInt8, Int16, UInt16, Int32, UInt32, Float32, Float64
};

PlyScalar ParsePlyScalar(const std::string& text) {
    if (text == "char" || text == "int8") return PlyScalar::Int8;
    if (text == "uchar" || text == "uint8") return PlyScalar::UInt8;
    if (text == "short" || text == "int16") return PlyScalar::Int16;
    if (text == "ushort" || text == "uint16") return PlyScalar::UInt16;
    if (text == "int" || text == "int32") return PlyScalar::Int32;
    if (text == "uint" || text == "uint32") return PlyScalar::UInt32;
    if (text == "float" || text == "float32") return PlyScalar::Float32;
    if (text == "double" || text == "float64") return PlyScalar::Float64;
    throw std::runtime_error("unsupported PLY scalar type: " + text);
}

template <typename T>
T ReadBinary(std::istream& input) {
    T value{};
    input.read(reinterpret_cast<char*>(&value), sizeof(value));
    if (!input) throw std::runtime_error("unexpected end of binary PLY");
    return value;
}

double ReadBinaryScalar(std::istream& input, PlyScalar type) {
    switch (type) {
        case PlyScalar::Int8: return ReadBinary<int8_t>(input);
        case PlyScalar::UInt8: return ReadBinary<uint8_t>(input);
        case PlyScalar::Int16: return ReadBinary<int16_t>(input);
        case PlyScalar::UInt16: return ReadBinary<uint16_t>(input);
        case PlyScalar::Int32: return ReadBinary<int32_t>(input);
        case PlyScalar::UInt32: return ReadBinary<uint32_t>(input);
        case PlyScalar::Float32: return ReadBinary<float>(input);
        case PlyScalar::Float64: return ReadBinary<double>(input);
        default: throw std::runtime_error("invalid PLY scalar type");
    }
}

double ParseDouble(const std::string& text, const char* name,
                   bool allow_zero = false) {
    char* end = nullptr;
    errno = 0;
    const double value = std::strtod(text.c_str(), &end);
    if (errno != 0 || end == text.c_str() || *end != '\0' ||
        !std::isfinite(value) ||
        (allow_zero ? value < 0.0 : value <= 0.0))
        throw std::invalid_argument(std::string("invalid ") + name);
    return value;
}

uint32_t ParseUInt32(const std::string& text, const char* name) {
    char* end = nullptr;
    errno = 0;
    const unsigned long value = std::strtoul(text.c_str(), &end, 10);
    if (errno != 0 || end == text.c_str() || *end != '\0' ||
        value == 0 || value > std::numeric_limits<uint32_t>::max())
        throw std::invalid_argument(std::string("invalid ") + name);
    return static_cast<uint32_t>(value);
}

CadRegMode ParseMode(const std::string& text) {
    if (text == "single") return CADREG_MODE_SINGLE;
    if (text == "cascade") return CADREG_MODE_CASCADE;
    if (text == "ensemble") return CADREG_MODE_ENSEMBLE;
    throw std::invalid_argument(
        "mode must be single, cascade, or ensemble");
}

uint32_t ParseStrategies(const std::string& text) {
    if (text == "initial") return CADREG_STRATEGY_INITIAL;
    if (text == "pca") return CADREG_STRATEGY_PCA;
    if (text == "fpfh") return CADREG_STRATEGY_FPFH_RANSAC;
    if (text == "all")
        return CADREG_STRATEGY_INITIAL |
               CADREG_STRATEGY_PCA |
               CADREG_STRATEGY_FPFH_RANSAC;
    throw std::invalid_argument(
        "strategy must be initial, pca, fpfh, or all");
}

const char* StatusName(CadRegStatus status) {
    switch (status) {
        case CADREG_STATUS_SUCCESS: return "SUCCESS";
        case CADREG_STATUS_NOT_CONVERGED: return "NOT_CONVERGED";
        case CADREG_STATUS_INVALID_ARGUMENT: return "INVALID_ARGUMENT";
        case CADREG_STATUS_EMPTY_AFTER_DOWNSAMPLING:
            return "EMPTY_AFTER_DOWNSAMPLING";
        case CADREG_STATUS_INTERNAL_ERROR: return "INTERNAL_ERROR";
        case CADREG_STATUS_QUALITY_REJECTED: return "QUALITY_REJECTED";
        case CADREG_STATUS_AMBIGUOUS: return "AMBIGUOUS";
        default: return "UNKNOWN";
    }
}

const char* StrategyName(CadRegStrategy strategy) {
    switch (strategy) {
        case CADREG_STRATEGY_INITIAL: return "INITIAL";
        case CADREG_STRATEGY_PCA: return "PCA";
        case CADREG_STRATEGY_FPFH_RANSAC: return "FPFH_RANSAC";
        default: return "UNKNOWN";
    }
}

Cloud LoadPly(const std::filesystem::path& path) {
    if (!std::filesystem::is_regular_file(path))
        throw std::invalid_argument("PLY file does not exist: " +
                                    path.u8string());
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("could not open PLY");
    std::string line;
    if (!std::getline(input, line))
        throw std::runtime_error("file is not a PLY");
    if (!line.empty() && line.back() == '\r') line.pop_back();
    if (line != "ply")
        throw std::runtime_error("file is not a PLY");
    bool ascii = false;
    bool binary_little = false;
    bool in_vertex = false;
    uint64_t vertex_count = 0;
    std::vector<std::pair<PlyScalar, std::string>> properties;
    while (std::getline(input, line)) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        std::istringstream tokens(line);
        std::string keyword;
        tokens >> keyword;
        if (keyword == "format") {
            std::string format;
            tokens >> format;
            ascii = format == "ascii";
            binary_little = format == "binary_little_endian";
            if (!ascii && !binary_little)
                throw std::runtime_error("only ASCII and little-endian PLY are supported");
        } else if (keyword == "element") {
            std::string name;
            uint64_t count = 0;
            tokens >> name >> count;
            in_vertex = name == "vertex";
            if (in_vertex) vertex_count = count;
        } else if (keyword == "property" && in_vertex) {
            std::string type, name;
            tokens >> type;
            if (type == "list")
                throw std::runtime_error("list property is invalid for PLY vertices");
            tokens >> name;
            properties.emplace_back(ParsePlyScalar(type), name);
        } else if (keyword == "end_header") {
            break;
        }
    }
    if ((!ascii && !binary_little) || vertex_count == 0 || properties.empty())
        throw std::runtime_error("incomplete PLY header");
    int x_index = -1, y_index = -1, z_index = -1;
    for (size_t i = 0; i < properties.size(); ++i) {
        if (properties[i].second == "x") x_index = static_cast<int>(i);
        if (properties[i].second == "y") y_index = static_cast<int>(i);
        if (properties[i].second == "z") z_index = static_cast<int>(i);
    }
    if (x_index < 0 || y_index < 0 || z_index < 0)
        throw std::runtime_error("PLY has no XYZ properties");
    Cloud cloud;
    cloud.reserve(static_cast<size_t>(vertex_count));
    std::vector<double> values(properties.size());
    for (uint64_t vertex = 0; vertex < vertex_count; ++vertex) {
        for (size_t property = 0; property < properties.size(); ++property) {
            if (ascii) {
                if (!(input >> values[property]))
                    throw std::runtime_error("unexpected end of ASCII PLY");
            } else {
                values[property] =
                    ReadBinaryScalar(input, properties[property].first);
            }
        }
        const Point point{
            values[static_cast<size_t>(x_index)],
            values[static_cast<size_t>(y_index)],
            values[static_cast<size_t>(z_index)]
        };
        if (std::isfinite(point.x) && std::isfinite(point.y) &&
            std::isfinite(point.z))
            cloud.push_back(point);
    }
    if (cloud.size() < 3)
        throw std::runtime_error("PLY has fewer than three finite points");
    return cloud;
}

std::vector<double> ToDoubleXyz(const Cloud& cloud) {
    std::vector<double> xyz;
    xyz.resize(cloud.size() * 3);
    for (size_t i = 0; i < cloud.size(); ++i) {
        xyz[i * 3 + 0] = cloud[i].x;
        xyz[i * 3 + 1] = cloud[i].y;
        xyz[i * 3 + 2] = cloud[i].z;
    }
    return xyz;
}

Cloud TransformCloud(const Cloud& source, const double matrix[16]) {
    Cloud output;
    output.resize(source.size());
    for (size_t i = 0; i < source.size(); ++i) {
        const double x = source[i].x;
        const double y = source[i].y;
        const double z = source[i].z;
        output[i].x =
            matrix[0] * x + matrix[1] * y + matrix[2] * z + matrix[3];
        output[i].y =
            matrix[4] * x + matrix[5] * y + matrix[6] * z + matrix[7];
        output[i].z =
            matrix[8] * x + matrix[9] * y + matrix[10] * z + matrix[11];
    }
    return output;
}

void SavePly(const std::filesystem::path& path, const Cloud& cloud) {
    std::ofstream output(path, std::ios::binary);
    if (!output) throw std::runtime_error("could not create aligned PLY");
    output << "ply\n"
           << "format binary_little_endian 1.0\n"
           << "comment generated by cadreg_test\n"
           << "element vertex " << cloud.size() << "\n"
           << "property double x\n"
           << "property double y\n"
           << "property double z\n"
           << "end_header\n";
    for (const Point& point : cloud) {
        output.write(reinterpret_cast<const char*>(&point.x), sizeof(double));
        output.write(reinterpret_cast<const char*>(&point.y), sizeof(double));
        output.write(reinterpret_cast<const char*>(&point.z), sizeof(double));
    }
    if (!output) throw std::runtime_error("failed while writing aligned PLY");
}

void PrintUsage(const char* program) {
    std::cout
        << "Usage:\n  " << program
        << " source_scan.ply target_cad.ply aligned_source.ply"
           " [mode] [strategy] [max_distance] [iterations]"
           " [voxel_size] [feature_voxel] [initial_matrix.txt|-]"
           " [ransac_iterations] [ransac_attempts] [max_candidates]"
           " [max_refined] [fast|balanced|accuracy]\n\n"
        << "Defaults:\n"
        << "  mode=cascade, strategy=all, max_distance=5,\n"
        << "  iterations=100, voxel_size=1, feature_voxel=5\n\n"
        << "Modes: single | cascade | ensemble\n"
        << "Strategies: initial | pca | fpfh | all\n";
}

std::vector<double> LoadInitialMatrix(const std::filesystem::path& path) {
    std::ifstream input(path);
    if (!input)
        throw std::runtime_error("could not open initial matrix file");
    std::vector<double> matrix;
    double value = 0.0;
    while (input >> value) matrix.push_back(value);
    if (matrix.size() != 16)
        throw std::runtime_error(
            "initial matrix file must contain exactly 16 numbers");
    for (double item : matrix) {
        if (!std::isfinite(item))
            throw std::runtime_error("initial matrix contains non-finite value");
    }
    return matrix;
}

int Run(const std::vector<std::string>& args) {
    if (args.size() < 4 || args.size() > 16) {
        PrintUsage(args.empty() ? "cadreg_test" : args[0].c_str());
        return 2;
    }
    const std::filesystem::path source_path =
        std::filesystem::u8path(args[1]);
    const std::filesystem::path target_path =
        std::filesystem::u8path(args[2]);
    const std::filesystem::path output_path =
        std::filesystem::u8path(args[3]);

    CadRegOptions options{};
    cadreg_default_options(&options);
    if (args.size() > 4) options.mode = ParseMode(args[4]);
    if (args.size() > 5) options.strategy_mask = ParseStrategies(args[5]);
    if (args.size() > 6)
        options.max_correspondence_distance =
            ParseDouble(args[6], "max_distance");
    if (args.size() > 7)
        options.max_iterations = ParseUInt32(args[7], "iterations");
    if (args.size() > 8)
        options.voxel_size = ParseDouble(args[8], "voxel_size", true);
    if (args.size() > 9)
        options.feature_voxel_size =
            ParseDouble(args[9], "feature_voxel");
    std::vector<double> initial_matrix;
    if (args.size() > 10 && args[10] != "-")
        initial_matrix = LoadInitialMatrix(std::filesystem::u8path(args[10]));
    if (args.size() > 11)
        options.ransac_max_iterations =
            ParseUInt32(args[11], "ransac_iterations");
    if (args.size() > 12)
        options.ransac_attempts =
            ParseUInt32(args[12], "ransac_attempts");
    if (args.size() > 13)
        options.max_candidates_per_strategy =
            ParseUInt32(args[13], "max_candidates");
    if (args.size() > 14)
        options.max_refined_candidates_per_strategy =
            ParseUInt32(args[14], "max_refined");
    const std::string preset = args.size() > 15 ? args[15] : "balanced";
    options.icp_level_count = 3;
    if (preset == "fast") {
        options.icp_levels[0] = {
            std::max(options.voxel_size * 8.0, options.voxel_size),
            options.max_correspondence_distance * 8.0,
            std::min<uint32_t>(options.max_iterations, 15), 0
        };
        options.icp_levels[1] = {
            std::max(options.voxel_size * 3.0, options.voxel_size),
            options.max_correspondence_distance * 3.0,
            std::min<uint32_t>(options.max_iterations, 20), 0
        };
    } else if (preset == "balanced") {
        options.icp_levels[0] = {
            std::max(options.voxel_size * 6.0, options.voxel_size),
            options.max_correspondence_distance * 12.0,
            std::min<uint32_t>(options.max_iterations, 40), 0
        };
        options.icp_levels[1] = {
            std::max(options.voxel_size * 2.5, options.voxel_size),
            options.max_correspondence_distance * 4.0,
            std::min<uint32_t>(options.max_iterations, 60), 0
        };
    } else if (preset == "accuracy") {
        options.icp_levels[0] = {
            std::max(options.voxel_size * 4.0, options.voxel_size),
            options.max_correspondence_distance * 8.0,
            std::min<uint32_t>(options.max_iterations, 50), 0
        };
        options.icp_levels[1] = {
            std::max(options.voxel_size * 2.0, options.voxel_size),
            options.max_correspondence_distance * 3.0,
            std::min<uint32_t>(options.max_iterations, 80), 0
        };
    } else {
        throw std::invalid_argument(
            "ICP preset must be fast, balanced, or accuracy");
    }
    options.icp_levels[2] = {
        options.voxel_size,
        options.max_correspondence_distance,
        preset == "fast"
            ? std::min<uint32_t>(options.max_iterations, 30)
            : options.max_iterations,
        0
    };

    std::cout << "Loading source scan: " << source_path.u8string() << "\n";
    Cloud source = LoadPly(source_path);
    std::cout << "Loading target CAD:  " << target_path.u8string() << "\n";
    Cloud target = LoadPly(target_path);
    std::vector<double> source_xyz = ToDoubleXyz(source);
    std::vector<double> target_xyz = ToDoubleXyz(target);
    const CadRegPointCloud source_view{
        source_xyz.data(), static_cast<uint64_t>(source.size()), 0
    };
    const CadRegPointCloud target_view{
        target_xyz.data(), static_cast<uint64_t>(target.size()), 0
    };
    CadRegHandle handle = cadreg_create();
    if (!handle) throw std::runtime_error("could not create registration context");
    struct HandleGuard {
        CadRegHandle value;
        ~HandleGuard() { cadreg_destroy(value); }
    } guard{handle};
    CadRegStatus status = cadreg_set_target(handle, &target_view);
    if (status != CADREG_STATUS_SUCCESS)
        throw std::runtime_error(
            std::string("target preparation failed: ") +
            cadreg_last_error(handle));
    CadRegResult result{};
    cadreg_init_result(&result);
    status = cadreg_register_source(
        handle, &source_view,
        initial_matrix.empty() ? nullptr : initial_matrix.data(),
        &options, &result);

    std::cout << std::fixed << std::setprecision(8)
              << "\nRegistration result\n"
              << "  status:        " << StatusName(status)
              << " (" << static_cast<int>(status) << ")\n"
              << "  converged:     " << (result.converged ? "yes" : "no")
              << "\n"
              << "  strategy:      " << StrategyName(result.selected_strategy)
              << "\n"
              << "  RMSE:          " << result.rmse << "\n"
              << "  inlier ratio:  " << result.inlier_ratio << "\n"
              << "  score:         " << result.score << "\n"
              << "  second score:  " << result.second_best_score << "\n"
              << "  candidates:    " << result.candidate_count << "\n"
              << "  accepted:      " << result.accepted_candidate_count << "\n"
              << "  source used:   " << result.source_points_used << "\n"
              << "  target used:   " << result.target_points_used << "\n"
              << "  target cached: "
              << (result.target_cache_hit ? "yes" : "no") << "\n"
              << "  target cover:  " << result.target_coverage << "\n"
              << "  preprocess:    " << result.preprocessing_ms << " ms\n"
              << "  coarse:        " << result.coarse_registration_ms << " ms\n"
              << "  refine:        " << result.refinement_ms << " ms\n"
              << "  quality:       " << result.quality_ms << " ms\n"
              << "  elapsed:       " << result.elapsed_ms << " ms\n"
              << "  message:       "
              << (result.error_message[0] ? result.error_message : "-")
              << "\n\n"
              << "source_to_target (row-major)\n";
    for (int row = 0; row < 4; ++row) {
        std::cout << "  ";
        for (int col = 0; col < 4; ++col)
            std::cout << std::setw(14)
                      << result.source_to_target[row * 4 + col] << " ";
        std::cout << "\n";
    }
    std::cout << "\nCandidate diagnostics (sorted by score):\n";
    for (uint32_t index = 0; index < result.diagnostic_count; ++index) {
        const CadRegCandidateDiagnostic& item = result.diagnostics[index];
        std::cout << "  #" << item.rank
                  << " strategy=" << StrategyName(item.strategy)
                  << " converged=" << (item.converged ? "yes" : "no")
                  << " accepted=" << (item.accepted ? "yes" : "no")
                  << " score=" << item.score
                  << " rmse=" << item.rmse
                  << " inlier=" << item.inlier_ratio
                  << " shared_coarse=" << item.shared_coarse_ms << " ms"
                  << " refine=" << item.refinement_ms << " ms"
                  << " quality=" << item.quality_ms << " ms"
                  << " candidate=" << item.candidate_elapsed_ms << " ms\n";
    }

    Cloud aligned = TransformCloud(source, result.source_to_target);
    if (!output_path.parent_path().empty())
        std::filesystem::create_directories(output_path.parent_path());
    SavePly(output_path, aligned);
    std::cout << "\nAligned source written to: "
              << output_path.u8string() << "\n";
    return status == CADREG_STATUS_SUCCESS ||
           status == CADREG_STATUS_AMBIGUOUS ? 0 : 3;
}

}  // namespace

#if defined(_WIN32)
namespace {
std::string Utf8FromWide(const wchar_t* value) {
    const int size = WideCharToMultiByte(
        CP_UTF8, 0, value, -1, nullptr, 0, nullptr, nullptr);
    if (size <= 0) throw std::runtime_error("argument conversion failed");
    std::string result(static_cast<size_t>(size), '\0');
    WideCharToMultiByte(CP_UTF8, 0, value, -1, &result[0], size,
                        nullptr, nullptr);
    result.resize(static_cast<size_t>(size - 1));
    return result;
}
}  // namespace

int wmain(int argc, wchar_t** argv) {
    try {
        std::vector<std::string> args;
        args.reserve(static_cast<size_t>(argc));
        for (int i = 0; i < argc; ++i) args.push_back(Utf8FromWide(argv[i]));
        return Run(args);
    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << "\n";
        return 1;
    }
}
#else
int main(int argc, char** argv) {
    try {
        return Run(std::vector<std::string>(argv, argv + argc));
    } catch (const std::exception& error) {
        std::cerr << "ERROR: " << error.what() << "\n";
        return 1;
    }
}
#endif
