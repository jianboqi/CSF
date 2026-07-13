// ======================================================================================
// PCDReader - Lightweight PCD file reader for CSF
//
// Supports both ASCII and binary PCD formats (no PCL dependency).
//
// Coordinate convention matches XYZReader:
//   point.x = PCD x,  point.y = -PCD z,  point.z = PCD y
// ======================================================================================

#ifndef PCD_READER_H_
#define PCD_READER_H_

#include <string>
#include "point_cloud.h"

/// Read an ASCII or binary PCD file and populate a csf::PointCloud.
/// Returns true on success, false on failure.
bool read_pcd(const std::string& fname, csf::PointCloud& pointcloud);

#endif // PCD_READER_H_
