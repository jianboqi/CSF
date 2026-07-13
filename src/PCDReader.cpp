// ======================================================================================
// PCDReader - Lightweight ASCII PCD file reader for CSF
// ======================================================================================

#include "PCDReader.h"

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

// Trim leading and trailing whitespace
static std::string trim(const std::string& s) {
    size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start])))
        ++start;
    size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1])))
        --end;
    return s.substr(start, end - start);
}

bool read_pcd(const std::string& fname, csf::PointCloud& pointcloud) {
    std::ifstream fin(fname.c_str(), std::ios::in);
    if (!fin.is_open()) {
        std::cerr << "PCDReader: cannot open file: " << fname << std::endl;
        return false;
    }

    // --- Parse PCD header ---
    std::string line;
    std::string version;
    std::vector<std::string> fields;
    std::vector<int> sizes;
    std::vector<char> types;
    std::vector<int> counts;
    int width = 0;
    int height = 0;
    int points = 0;
    std::string data_type;
    bool header_done = false;

    while (!header_done && std::getline(fin, line)) {
        // Remove trailing carriage return for Windows files
        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#')
            continue;

        std::istringstream iss(trimmed);
        std::string keyword;
        iss >> keyword;

        if (keyword == "VERSION") {
            iss >> version;
        } else if (keyword == "FIELDS") {
            std::string field;
            while (iss >> field)
                fields.push_back(field);
        } else if (keyword == "SIZE") {
            int s;
            while (iss >> s)
                sizes.push_back(s);
        } else if (keyword == "TYPE") {
            char t;
            while (iss >> t)
                types.push_back(t);
        } else if (keyword == "COUNT") {
            int c;
            while (iss >> c)
                counts.push_back(c);
        } else if (keyword == "WIDTH") {
            iss >> width;
        } else if (keyword == "HEIGHT") {
            iss >> height;
        } else if (keyword == "POINTS") {
            iss >> points;
        } else if (keyword == "DATA") {
            iss >> data_type;
            header_done = true;
        }
    }

    if (!header_done) {
        std::cerr << "PCDReader: incomplete header in " << fname << std::endl;
        return false;
    }

    if (data_type != "ascii") {
        std::cerr << "PCDReader: unsupported DATA type '" << data_type
                  << "'. Only 'ascii' is supported." << std::endl;
        return false;
    }

    // Determine total point count
    int total_points = points;
    if (total_points <= 0) {
        total_points = width * height;
    }
    if (total_points <= 0) {
        std::cerr << "PCDReader: cannot determine point count" << std::endl;
        return false;
    }

    // Find indices of x, y, z fields (case-insensitive)
    int x_idx = -1, y_idx = -1, z_idx = -1;
    for (size_t i = 0; i < fields.size(); ++i) {
        std::string f_lower = fields[i];
        std::transform(f_lower.begin(), f_lower.end(), f_lower.begin(),
                       [](unsigned char c) { return std::tolower(c); });
        if (f_lower == "x")
            x_idx = static_cast<int>(i);
        else if (f_lower == "y")
            y_idx = static_cast<int>(i);
        else if (f_lower == "z")
            z_idx = static_cast<int>(i);
    }

    if (x_idx < 0 || y_idx < 0 || z_idx < 0) {
        std::cerr << "PCDReader: PCD file must contain x, y, z fields. Found:";
        for (const auto& f : fields)
            std::cerr << " " << f;
        std::cerr << std::endl;
        return false;
    }

    int num_fields = static_cast<int>(fields.size());

    // --- Read data lines ---
    pointcloud.clear();
    pointcloud.reserve(total_points);

    int line_count = 0;
    while (line_count < total_points && std::getline(fin, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        std::string trimmed = trim(line);
        if (trimmed.empty())
            continue;

        std::istringstream iss(trimmed);
        std::vector<std::string> tokens;
        std::string token;
        while (iss >> token)
            tokens.push_back(token);

        if (static_cast<int>(tokens.size()) < num_fields) {
            std::cerr << "PCDReader: line " << (line_count + 1)
                      << " has insufficient fields (" << tokens.size()
                      << " < " << num_fields << "), skipping" << std::endl;
            continue;
        }

        double x_val = atof(tokens[x_idx].c_str());
        double y_val = atof(tokens[y_idx].c_str());
        double z_val = atof(tokens[z_idx].c_str());

        // Apply the same coordinate transform as XYZReader:
        // x -> x,  y -> -z,  z -> y
        csf::Point pt;
        pt.x = x_val;
        pt.y = -z_val;
        pt.z = y_val;
        pointcloud.push_back(pt);

        ++line_count;
    }

    std::cout << "PCDReader: loaded " << pointcloud.size()
              << " points from " << fname << std::endl;
    return true;
}
