// ======================================================================================
// LASWriter - Minimal LAS 1.2 point cloud writer for CSF
//
// Writes LAS 1.2 format with Point Data Record Format 0.
// Coordinate transform: reverses the CSF internal convention back to
// original (X, Y, Z) space.
// ======================================================================================

#ifndef LAS_WRITER_H_
#define LAS_WRITER_H_

#include <string>
#include <vector>
#include "point_cloud.h"

/// Write a LAS 1.2 file (format 0) with classification labels.
///
/// @param fname       Output .las file path
/// @param points      Original point cloud (in CSF internal coordinates)
/// @param ground_idx  Indices of ground-classified points
/// @param offground_idx Indices of non-ground (unclassified) points
/// @return true on success, false on failure
bool write_las(const std::string& fname,
               const csf::PointCloud& points,
               const std::vector<int>& ground_idx,
               const std::vector<int>& offground_idx);

#endif // LAS_WRITER_H_
