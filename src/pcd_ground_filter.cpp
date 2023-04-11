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

#include <vector>
#include <locale.h>
#include <time.h>
#include <cstdlib>
#include <cstring>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

#include <csf_seg/csf_seg.h>

#include <boost/filesystem.hpp>

int main(int argc, char* argv[])
{
  std::string input_pcd, config_file, surface_output;
  if (argc >= 4)
  {
    input_pcd = argv[1];
    config_file = argv[2];
    pcl::PointCloud<pcl::PointXYZI>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI> ground_cloud, lower_ground_cloud, upper_ground_cloud;

    namespace fs = boost::filesystem;
    std::string input_pcd_name = fs::path(input_pcd).relative_path().string() + fs::path(input_pcd).stem().string();
    // Import input point cloud
    if (pcl::io::loadPCDFile(input_pcd.c_str(), *input_cloud) == -1)
    {
      std::cerr << "Error: Cannot load input PCD: " + input_pcd << std::endl;
      exit(1);
    }
    std::cout << "PCD file loaded!" << std::endl;

    // step 1 Initialize the object
    CSFGroundSeg csf;

    // step 2 parameter setting
    csf.paramInitialize(config_file);

    clock_t start, end;
    start = clock();

    // step 3 segmentGround
    csf.segmentGround(input_cloud, upper_ground_cloud, lower_ground_cloud, ground_cloud);

    end = clock();
    double dur = (double)(end - start);
    printf("Use Time:%f\n", (dur / CLOCKS_PER_SEC));

    std::string ground_pcd_name = input_pcd_name + "_ground_cloud.pcd";
    if (pcl::io::savePCDFileBinary(ground_pcd_name, ground_cloud))
    {
      std::cerr << "Error: Cannot save PCD: " + ground_pcd_name << std::endl;
      exit(1);
    }

    int separate_upper_lower_ground = std::stoi(argv[3]);
    if (separate_upper_lower_ground == 1)
    {
      std::string upper_ground_pcd_name = input_pcd_name + "_upper_ground_cloud.pcd";
      if (pcl::io::savePCDFileBinary(upper_ground_pcd_name, upper_ground_cloud))
      {
        std::cerr << "Error: Cannot save PCD: " + upper_ground_pcd_name << std::endl;
        exit(1);
      }
      std::string lower_ground_pcd_name = input_pcd_name + "_lower_ground_cloud.pcd";
      if (pcl::io::savePCDFileBinary(lower_ground_pcd_name, lower_ground_cloud))
      {
        std::cerr << "Error: Cannot save PCD: " + lower_ground_pcd_name << std::endl;
        exit(1);
      }
    }
    else
    {
      std::string non_ground_pcd_name = input_pcd_name + "_non_ground_cloud.pcd";
      pcl::PointCloud<pcl::PointXYZI> non_ground_cloud = upper_ground_cloud + lower_ground_cloud;
      if (pcl::io::savePCDFileBinary(non_ground_pcd_name, non_ground_cloud))
      {
        std::cerr << "Error: Cannot save PCD: " + non_ground_pcd_name << std::endl;
        exit(1);
      }
    }

    if (argc > 4)
    {
      std::string surface_output = argv[4];
      if (surface_output == "surface")
      {
        pcl::PointCloud<pcl::PointXYZ> ground_surface;
        csf.getGroundSurface(ground_surface);
        std::string surface_pcd_name = input_pcd_name + "_ground_surface.pcd";
        if (pcl::io::savePCDFileBinary(surface_pcd_name, ground_surface))
        {
          std::cerr << "Error: Cannot save PCD: " + surface_pcd_name << std::endl;
          exit(1);
        }
      }
    }

    return 0;
  }
  else
  {
    std::cerr << "Error: Invalid Arguments" << std::endl;
    exit(1);
  }
}