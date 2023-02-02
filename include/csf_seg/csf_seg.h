// ======================================================================================
// Copyright 2017 State Key Laboratory of Remote Sensing Science,
// Institute of Remote Sensing Science and Engineering, Beijing Normal University

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ======================================================================================

// #######################################################################################
// #                                                                                     #
// #            CSF: Airborne LiDAR filtering based on Cloth Simulation                  #
// #                                                                                     #
// #  Please cite the following paper, If you use this software in your work.            #
// #                                                                                     #
// #  Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. An Easy-to-Use Airborne LiDAR  #
// #  Data Filtering Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501.   #
// #                               (http://ramm.bnu.edu.cn/)                             #
// #                                                                                     #
// #                      Wuming Zhang; Jianbo Qi; Peng Wan; Hongtao Wang                #
// #                                                                                     #
// #                      contact us: 2009zwm@gmail.com; wpqjbzwm@126.com                #
// #                                                                                     #
// #######################################################################################

// cloth simulation filter for airborne lidar filtering
#ifndef _CSF_GROUND_SEG_H_
#define _CSF_GROUND_SEG_H_

#include <vector>
#include <string>
#include "point_cloud.h"

#include <yaml-cpp/yaml.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/extract_indices.h>

struct CSFSegParams
{
  CSFSegParams()
    : slope_smooth(true), time_step(0.65), class_threshold(0.5), cloth_resolution(0.5), rigidness(3), iterations(500)
  {
  }
  // refer to the website:http://ramm.bnu.edu.cn/projects/CSF/ for the setting of these paramters
  bool slope_smooth;
  double time_step;
  double class_threshold;
  double cloth_resolution;
  int rigidness;
  int iterations;
};

class CSFGroundSeg
{
public:
  CSFGroundSeg();
  ~CSFGroundSeg();

  void segmentGround(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& in_points,
                     pcl::PointCloud<pcl::PointXYZI>& out_groundless_points,
                     pcl::PointCloud<pcl::PointXYZI>& out_ground_points);
  void segmentGround(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& in_points,
                     pcl::PointCloud<pcl::PointXYZI>& out_upper_ground_points,
                     pcl::PointCloud<pcl::PointXYZI>& out_lower_ground_points,
                     pcl::PointCloud<pcl::PointXYZI>& out_ground_points);
  void getGroundSurface(pcl::PointCloud<pcl::PointXYZ>& ground_surface);
  void paramInitialize(const std::string& config_file);

private:
  csf::PointCloud point_cloud;
  pcl::PointCloud<pcl::PointXYZ> ground_surface_;
  CSFSegParams params_;

  void setPointCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_cloud);
  void doFiltering(std::vector<int>& groundIndexes, std::vector<int>& offGroundIndexes);
  void doFiltering(std::vector<int>& groundIndexes, std::vector<int>& upperGroundIndexes,
                   std::vector<int>& lowerGroundIndexes);
};

#endif  // ifndef _CSF_GROUND_SEG_H_
