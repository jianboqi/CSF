// ======================================================================================
// CSFCLI - Command-line tool for CSF ground filtering
//
// Input:  PCD file (ASCII format)
// Output: LAS 1.2 file with ground/non-ground classification
//
// Usage: csf_cli --input <pcd_file> --output <las_file> [--config <params.cfg>]
// ======================================================================================

#include <chrono>
#include <cstring>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "Cfg.h"
#include "../src/CSF.h"
#include "../src/PCDReader.h"
#include "../src/LASWriter.h"

static void print_usage(const char* prog) {
    std::cout << "Usage: " << prog
              << " --input <pcd_file> --output <las_file> [--config <params.cfg>]"
              << std::endl;
    std::cout << "Options:" << std::endl;
    std::cout << "  --input   <file>   Input PCD file (ASCII format)" << std::endl;
    std::cout << "  --output  <file>   Output LAS 1.2 file" << std::endl;
    std::cout << "  --config  <file>   Optional CSF parameter config file" << std::endl;
    std::cout << "  --help             Show this help" << std::endl;
}

int main(int argc, char* argv[]) {
    std::string input_path;
    std::string output_path;
    std::string config_path;

    // --- Parse command line ---
    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--help") == 0 ||
            std::strcmp(argv[i], "-h") == 0) {
            print_usage(argv[0]);
            return 0;
        } else if (std::strcmp(argv[i], "--input") == 0 && i + 1 < argc) {
            input_path = argv[++i];
        } else if (std::strcmp(argv[i], "--output") == 0 && i + 1 < argc) {
            output_path = argv[++i];
        } else if (std::strcmp(argv[i], "--config") == 0 && i + 1 < argc) {
            config_path = argv[++i];
        } else {
            std::cerr << "Unknown argument: " << argv[i] << std::endl;
            print_usage(argv[0]);
            return 1;
        }
    }

    if (input_path.empty() || output_path.empty()) {
        std::cerr << "Error: --input and --output are required." << std::endl;
        print_usage(argv[0]);
        return 1;
    }

    // --- Configure CSF parameters ---
    CSF csf;

    if (!config_path.empty()) {
        Cfg cfg;
        std::string val;

        if (cfg.readConfigFile(config_path.c_str(), "slop_smooth", val)) {
            if (val == "true" || val == "True" || val == "1")
                csf.params.bSloopSmooth = true;
            else if (val == "false" || val == "False" || val == "0")
                csf.params.bSloopSmooth = false;
        }
        if (cfg.readConfigFile(config_path.c_str(), "class_threshold", val))
            csf.params.class_threshold = std::atof(val.c_str());
        if (cfg.readConfigFile(config_path.c_str(), "cloth_resolution", val))
            csf.params.cloth_resolution = std::atof(val.c_str());
        if (cfg.readConfigFile(config_path.c_str(), "iterations", val))
            csf.params.interations = std::atoi(val.c_str());
        if (cfg.readConfigFile(config_path.c_str(), "rigidness", val))
            csf.params.rigidness = std::atoi(val.c_str());
        if (cfg.readConfigFile(config_path.c_str(), "time_step", val))
            csf.params.time_step = std::atof(val.c_str());

        std::cout << "CSF parameters loaded from: " << config_path << std::endl;
    } else {
        std::cout << "Using default CSF parameters." << std::endl;
    }

    // --- Read PCD ---
    std::cout << "Reading PCD file: " << input_path << std::endl;
    csf::PointCloud pc;
    if (!read_pcd(input_path, pc)) {
        std::cerr << "Error: failed to read PCD file." << std::endl;
        return 1;
    }
    if (pc.empty()) {
        std::cerr << "Error: PCD file contains no points." << std::endl;
        return 1;
    }

    csf.setPointCloud(pc);

    // --- Run filtering ---
    std::cout << "Running cloth simulation filter..." << std::endl;
    std::vector<int> groundIndexes, offGroundIndexes;

    auto t_start = std::chrono::steady_clock::now();
    csf.do_filtering(groundIndexes, offGroundIndexes, false);
    auto t_end = std::chrono::steady_clock::now();

    double infer_time_ms =
        std::chrono::duration<double, std::milli>(t_end - t_start).count();

    std::cout << "Filtering complete." << std::endl;
    std::cout << "  Ground points:     " << groundIndexes.size() << std::endl;
    std::cout << "  Non-ground points: " << offGroundIndexes.size() << std::endl;
    std::cout << "  Inference time:    " << infer_time_ms << " ms" << std::endl;

    // --- Write LAS ---
    std::cout << "Writing LAS file: " << output_path << std::endl;
    if (!write_las(output_path, csf.getPointCloud(), groundIndexes, offGroundIndexes)) {
        std::cerr << "Error: failed to write LAS file." << std::endl;
        return 1;
    }

    // --- Write runtime JSON ---
    std::string json_path = output_path;
    size_t ext_pos = json_path.rfind('.');
    if (ext_pos != std::string::npos)
        json_path = json_path.substr(0, ext_pos);
    json_path += "_runtime.json";

    std::ofstream json_out(json_path.c_str());
    if (json_out.is_open()) {
        json_out << "{\"infer_time_ms\": " << infer_time_ms << "}" << std::endl;
        json_out.close();
        std::cout << "Runtime written to: " << json_path << std::endl;
    }

    std::cout << "Done." << std::endl;
    return 0;
}
