#ifndef XYZ_READER_H_
#define XYZ_READER_H_

#include <string>
#include <vector>
using namespace std;

#include "point_cloud.h"

//从fname文件中读取点云，将点云数据存储在pointcloud中
void read_xyz(string fname, csf::PointCloud &pointcloud);


#endif