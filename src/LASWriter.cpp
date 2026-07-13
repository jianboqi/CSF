// ======================================================================================
// LASWriter - Minimal LAS 1.2 point cloud writer for CSF
// ======================================================================================

#include "LASWriter.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <iostream>
#include <limits>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// LAS 1.2 binary structures — packed to ensure correct byte layout
// ---------------------------------------------------------------------------
#pragma pack(push, 1)

struct LASHeader12 {
    char     fileSignature[4];      //  0: "LASF"
    uint16_t fileSourceId;          //  4
    uint16_t globalEncoding;        //  6
    uint32_t projectIdGuid1;        //  8
    uint16_t projectIdGuid2;        // 12
    uint16_t projectIdGuid3;        // 14
    char     projectIdGuid4[8];     // 16
    uint8_t  versionMajor;          // 24
    uint8_t  versionMinor;          // 25
    char     systemId[32];          // 26
    char     generatingSoftware[32];// 58
    uint16_t creationDayOfYear;     // 90
    uint16_t creationYear;          // 92
    uint16_t headerSize;            // 94
    uint32_t offsetToPointData;     // 96
    uint32_t numVLRs;               // 100
    uint8_t  pointDataFormat;       // 104
    uint16_t pointDataRecordLen;    // 105
    uint32_t legacyNumPoints;       // 107
    uint32_t legacyNumPointsByReturn[5]; // 111 (5 * 4 = 20 bytes)
    double   xScale;                // 131
    double   yScale;                // 139
    double   zScale;                // 147
    double   xOffset;               // 155
    double   yOffset;               // 163
    double   zOffset;               // 171
    double   maxX;                  // 179
    double   minX;                  // 187
    double   maxY;                  // 195
    double   minY;                  // 203
    double   maxZ;                  // 211
    double   minZ;                  // 219
};
// Total: 227 bytes

struct LASPointRecord0 {
    int32_t  x;               //  0
    int32_t  y;               //  4
    int32_t  z;               //  8
    uint16_t intensity;       // 12
    uint8_t  returnByte;      // 14: bits 0-2 return number, bits 3-5 num returns, bit 6 scan dir, bit 7 edge
    uint8_t  classification;  // 15
    int8_t   scanAngleRank;   // 16
    uint8_t  userData;        // 17
    uint16_t pointSourceId;   // 18
};
// Total: 20 bytes

#pragma pack(pop)

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static double clamp_scaled(double val, double scale, double offset) {
    double scaled = (val - offset) / scale;
    if (scaled > static_cast<double>(INT32_MAX))
        return static_cast<double>(INT32_MAX);
    if (scaled < static_cast<double>(INT32_MIN))
        return static_cast<double>(INT32_MIN);
    return scaled;
}

static void set_header_string(char* dst, size_t dst_size, const std::string& src) {
    std::memset(dst, 0, dst_size);
    std::strncpy(dst, src.c_str(), dst_size - 1);
}

// ---------------------------------------------------------------------------
// Main write function
// ---------------------------------------------------------------------------

bool write_las(const std::string& fname,
               const csf::PointCloud& points,
               const std::vector<int>& ground_idx,
               const std::vector<int>& offground_idx) {

    std::ofstream out(fname.c_str(), std::ios::binary);
    if (!out.is_open()) {
        std::cerr << "LASWriter: cannot open output file: " << fname << std::endl;
        return false;
    }

    int n = static_cast<int>(points.size());
    if (n == 0) {
        std::cerr << "LASWriter: no points to write" << std::endl;
        return false;
    }

    // Build classification lookup: 0 = unset, 1 = unclassified, 2 = ground
    std::vector<uint8_t> classification(n, 1); // default: unclassified
    for (int idx : ground_idx) {
        if (idx >= 0 && idx < n)
            classification[idx] = 2; // ground
    }
    for (int idx : offground_idx) {
        if (idx >= 0 && idx < n)
            classification[idx] = 1; // unclassified
    }

    // PCD and LAS both use Z+ = up. CSF internal also Z+ = up.
    // Identity transform — no reverse mapping needed.
    double minX =  std::numeric_limits<double>::max();
    double maxX = -std::numeric_limits<double>::max();
    double minY =  std::numeric_limits<double>::max();
    double maxY = -std::numeric_limits<double>::max();
    double minZ =  std::numeric_limits<double>::max();
    double maxZ = -std::numeric_limits<double>::max();

    for (const auto& pt : points) {
        double ox = pt.x;
        double oy = pt.y;
        double oz = pt.z;

        if (ox < minX) minX = ox;
        if (ox > maxX) maxX = ox;
        if (oy < minY) minY = oy;
        if (oy > maxY) maxY = oy;
        if (oz < minZ) minZ = oz;
        if (oz > maxZ) maxZ = oz;
    }

    // Compute scale and offset
    double scaleX = (maxX - minX) / static_cast<double>(INT32_MAX - 1);
    double scaleY = (maxY - minY) / static_cast<double>(INT32_MAX - 1);
    double scaleZ = (maxZ - minZ) / static_cast<double>(INT32_MAX - 1);

    // Avoid zero scale
    if (scaleX < 1e-16) scaleX = 0.001;
    if (scaleY < 1e-16) scaleY = 0.001;
    if (scaleZ < 1e-16) scaleZ = 0.001;

    double offsetX = minX;
    double offsetY = minY;
    double offsetZ = minZ;

    // --- Build and write LAS 1.2 header ---
    LASHeader12 hdr;
    std::memset(&hdr, 0, sizeof(hdr));

    hdr.fileSignature[0] = 'L';
    hdr.fileSignature[1] = 'A';
    hdr.fileSignature[2] = 'S';
    hdr.fileSignature[3] = 'F';
    hdr.fileSourceId       = 0;
    hdr.globalEncoding     = 0;
    hdr.versionMajor       = 1;
    hdr.versionMinor       = 2;
    set_header_string(hdr.systemId,           sizeof(hdr.systemId),           "CSF");
    set_header_string(hdr.generatingSoftware, sizeof(hdr.generatingSoftware), "CSF PCD-to-LAS CLI");
    hdr.creationDayOfYear  = 1;
    hdr.creationYear       = 2026;
    hdr.headerSize         = static_cast<uint16_t>(sizeof(LASHeader12));
    hdr.offsetToPointData  = static_cast<uint32_t>(sizeof(LASHeader12));
    hdr.numVLRs            = 0;
    hdr.pointDataFormat    = 0;
    hdr.pointDataRecordLen = static_cast<uint16_t>(sizeof(LASPointRecord0));
    hdr.legacyNumPoints    = static_cast<uint32_t>(n);
    for (int i = 0; i < 5; ++i)
        hdr.legacyNumPointsByReturn[i] = 0;
    hdr.legacyNumPointsByReturn[0] = static_cast<uint32_t>(n);

    hdr.xScale  = scaleX;
    hdr.yScale  = scaleY;
    hdr.zScale  = scaleZ;
    hdr.xOffset = offsetX;
    hdr.yOffset = offsetY;
    hdr.zOffset = offsetZ;
    hdr.maxX    = maxX;
    hdr.minX    = minX;
    hdr.maxY    = maxY;
    hdr.minY    = minY;
    hdr.maxZ    = maxZ;
    hdr.minZ    = minZ;

    out.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));

    // --- Write point records ---
    for (int i = 0; i < n; ++i) {
        double ox = points[i].x;
        double oy = points[i].y;
        double oz = points[i].z;

        LASPointRecord0 rec;
        std::memset(&rec, 0, sizeof(rec));

        rec.x              = static_cast<int32_t>(std::llround(clamp_scaled(ox, scaleX, offsetX)));
        rec.y              = static_cast<int32_t>(std::llround(clamp_scaled(oy, scaleY, offsetY)));
        rec.z              = static_cast<int32_t>(std::llround(clamp_scaled(oz, scaleZ, offsetZ)));
        rec.intensity      = 0;
        rec.returnByte     = 0x01; // return number 1, num returns 1
        rec.classification = classification[i];
        rec.scanAngleRank  = 0;
        rec.userData       = 0;
        rec.pointSourceId  = 0;

        out.write(reinterpret_cast<const char*>(&rec), sizeof(rec));
    }

    out.close();
    std::cout << "LASWriter: wrote " << n << " points to " << fname << std::endl;
    return true;
}
