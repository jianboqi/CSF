// ======================================================================================
// PCDReader - Lightweight ASCII + binary PCD file reader for CSF
// ======================================================================================

#include "PCDReader.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// --- helpers ---

static std::string trim(const std::string& s) {
    size_t start = 0;
    while (start < s.size() && std::isspace(static_cast<unsigned char>(s[start])))
        ++start;
    size_t end = s.size();
    while (end > start && std::isspace(static_cast<unsigned char>(s[end - 1])))
        --end;
    return s.substr(start, end - start);
}

// Compute byte offset to a field within a binary point record
static int field_offset(int field_idx,
                         const std::vector<int>& sizes,
                         const std::vector<int>& counts) {
    int off = 0;
    for (int i = 0; i < field_idx; ++i) {
        int cnt = (i < static_cast<int>(counts.size())) ? counts[i] : 1;
        off += sizes[i] * cnt;
    }
    return off;
}

// Read a 4-byte float from a buffer at a given offset
static float read_float32(const char* buf, int off) {
    float val;
    std::memcpy(&val, buf + off, sizeof(float));
    return val;
}

// Read an 8-byte double from a buffer at a given offset
static double read_float64(const char* buf, int off) {
    double val;
    std::memcpy(&val, buf + off, sizeof(double));
    return val;
}

// Read a 4-byte int32 from a buffer at a given offset
static std::int32_t read_int32(const char* buf, int off) {
    std::int32_t val;
    std::memcpy(&val, buf + off, sizeof(std::int32_t));
    return val;
}

// Read a 4-byte uint32 from a buffer at a given offset
static std::uint32_t read_uint32(const char* buf, int off) {
    std::uint32_t val;
    std::memcpy(&val, buf + off, sizeof(std::uint32_t));
    return val;
}

// --- header parser (shared) ---

struct PCDHeader {
    std::string version;
    std::vector<std::string> fields;
    std::vector<int> sizes;
    std::vector<char> types;
    std::vector<int> counts;
    int width = 0;
    int height = 0;
    int points = 0;
    std::string data_type;
};

static bool parse_pcd_header(std::ifstream& fin, PCDHeader& hdr) {
    std::string line;
    while (std::getline(fin, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        std::string trimmed = trim(line);
        if (trimmed.empty() || trimmed[0] == '#')
            continue;

        std::istringstream iss(trimmed);
        std::string keyword;
        iss >> keyword;

        if (keyword == "VERSION") {
            iss >> hdr.version;
        } else if (keyword == "FIELDS") {
            std::string f;
            while (iss >> f) hdr.fields.push_back(f);
        } else if (keyword == "SIZE") {
            int s;
            while (iss >> s) hdr.sizes.push_back(s);
        } else if (keyword == "TYPE") {
            char t;
            while (iss >> t) hdr.types.push_back(t);
        } else if (keyword == "COUNT") {
            int c;
            while (iss >> c) hdr.counts.push_back(c);
        } else if (keyword == "WIDTH") {
            iss >> hdr.width;
        } else if (keyword == "HEIGHT") {
            iss >> hdr.height;
        } else if (keyword == "POINTS") {
            iss >> hdr.points;
        } else if (keyword == "DATA") {
            iss >> hdr.data_type;
            return true;  // header done
        }
    }
    return false;  // never reached DATA line
}

// --- field index lookup ---

static bool find_xyz_indices(const std::vector<std::string>& fields,
                              int& x_idx, int& y_idx, int& z_idx) {
    x_idx = y_idx = z_idx = -1;
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
        for (const auto& f : fields) std::cerr << " " << f;
        std::cerr << std::endl;
        return false;
    }
    return true;
}

// --- coordinate transform (matches XYZReader convention) ---

static csf::Point make_point(double px, double py, double pz) {
    csf::Point pt;
    pt.x = px;
    pt.y = -pz;   // PCD z → CSF y (negated)
    pt.z = py;    // PCD y → CSF z
    return pt;
}

// ======================================================================
//  ASCII reader
// ======================================================================

static bool read_pcd_ascii(std::ifstream& fin,
                            const PCDHeader& hdr,
                            int total_points,
                            int x_idx, int y_idx, int z_idx,
                            csf::PointCloud& pointcloud) {
    int num_fields = static_cast<int>(hdr.fields.size());
    pointcloud.clear();
    pointcloud.reserve(total_points);

    std::string line;
    int line_count = 0;
    while (line_count < total_points && std::getline(fin, line)) {
        if (!line.empty() && line.back() == '\r')
            line.pop_back();

        std::string trimmed = trim(line);
        if (trimmed.empty()) continue;

        std::istringstream iss(trimmed);
        std::vector<std::string> tokens;
        std::string token;
        while (iss >> token) tokens.push_back(token);

        if (static_cast<int>(tokens.size()) < num_fields) continue;

        double xv = atof(tokens[x_idx].c_str());
        double yv = atof(tokens[y_idx].c_str());
        double zv = atof(tokens[z_idx].c_str());
        pointcloud.push_back(make_point(xv, yv, zv));
        ++line_count;
    }
    return true;
}

// ======================================================================
//  Binary reader
// ======================================================================

static bool read_pcd_binary(std::ifstream& fin,
                             const PCDHeader& hdr,
                             int total_points,
                             int x_idx, int y_idx, int z_idx,
                             csf::PointCloud& pointcloud) {
    pointcloud.clear();
    pointcloud.reserve(total_points);

    // Compute point stride (bytes per point record)
    int stride = 0;
    for (size_t i = 0; i < hdr.fields.size(); ++i) {
        int cnt = (i < hdr.counts.size()) ? hdr.counts[i] : 1;
        stride += hdr.sizes[i] * cnt;
    }

    std::vector<char> buf(stride);

    int x_off = field_offset(x_idx, hdr.sizes, hdr.counts);
    int y_off = field_offset(y_idx, hdr.sizes, hdr.counts);
    int z_off = field_offset(z_idx, hdr.sizes, hdr.counts);

    char x_type = hdr.types[x_idx];
    char y_type = hdr.types[y_idx];
    char z_type = hdr.types[z_idx];

    int x_size = hdr.sizes[x_idx];
    int y_size = hdr.sizes[y_idx];
    int z_size = hdr.sizes[z_idx];

    // Helper to read a numeric value from packed binary
    auto read_val = [](char type, int size, const char* ptr) -> double {
        switch (type) {
        case 'F':  // float
            if (size == 4) return static_cast<double>(read_float32(ptr, 0));
            if (size == 8) return read_float64(ptr, 0);
            break;
        case 'I':  // signed int
            if (size == 1) return static_cast<double>(static_cast<std::int8_t>(ptr[0]));
            if (size == 2) { std::int16_t v; std::memcpy(&v, ptr, 2); return static_cast<double>(v); }
            if (size == 4) return static_cast<double>(read_int32(ptr, 0));
            if (size == 8) { std::int64_t v; std::memcpy(&v, ptr, 8); return static_cast<double>(v); }
            break;
        case 'U':  // unsigned int
            if (size == 1) return static_cast<double>(static_cast<std::uint8_t>(ptr[0]));
            if (size == 2) { std::uint16_t v; std::memcpy(&v, ptr, 2); return static_cast<double>(v); }
            if (size == 4) return static_cast<double>(read_uint32(ptr, 0));
            if (size == 8) { std::uint64_t v; std::memcpy(&v, ptr, 8); return static_cast<double>(v); }
            break;
        default:
            break;
        }
        return 0.0;
    };

    for (int i = 0; i < total_points; ++i) {
        fin.read(buf.data(), stride);
        if (fin.gcount() < stride) {
            std::cerr << "PCDReader: unexpected EOF at point " << i << std::endl;
            break;
        }

        double xv = read_val(x_type, x_size, buf.data() + x_off);
        double yv = read_val(y_type, y_size, buf.data() + y_off);
        double zv = read_val(z_type, z_size, buf.data() + z_off);
        pointcloud.push_back(make_point(xv, yv, zv));
    }
    return true;
}

// ======================================================================
//  Public entry point
// ======================================================================

bool read_pcd(const std::string& fname, csf::PointCloud& pointcloud) {
    std::ifstream fin(fname.c_str(), std::ios::in | std::ios::binary);
    if (!fin.is_open()) {
        std::cerr << "PCDReader: cannot open file: " << fname << std::endl;
        return false;
    }

    // 1. Parse header
    PCDHeader hdr;
    if (!parse_pcd_header(fin, hdr)) {
        std::cerr << "PCDReader: incomplete header in " << fname << std::endl;
        return false;
    }

    // 2. Validate data type
    if (hdr.data_type != "ascii" && hdr.data_type != "binary") {
        std::cerr << "PCDReader: unsupported DATA type '" << hdr.data_type
                  << "'. Only 'ascii' and 'binary' are supported." << std::endl;
        return false;
    }

    // 3. Determine total point count
    int total_points = hdr.points;
    if (total_points <= 0) total_points = hdr.width * hdr.height;
    if (total_points <= 0) {
        std::cerr << "PCDReader: cannot determine point count" << std::endl;
        return false;
    }

    // 4. Locate x/y/z fields
    int x_idx, y_idx, z_idx;
    if (!find_xyz_indices(hdr.fields, x_idx, y_idx, z_idx))
        return false;

    // 5. Read data
    bool ok = false;
    if (hdr.data_type == "ascii")
        ok = read_pcd_ascii(fin, hdr, total_points, x_idx, y_idx, z_idx, pointcloud);
    else
        ok = read_pcd_binary(fin, hdr, total_points, x_idx, y_idx, z_idx, pointcloud);

    if (ok) {
        std::cout << "PCDReader: loaded " << pointcloud.size()
                  << " points (" << hdr.data_type << ") from " << fname << std::endl;
    }
    return ok;
}
