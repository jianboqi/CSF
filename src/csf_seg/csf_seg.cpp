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

#include "csf_seg/csf_seg.h"
#include "csf_seg/vec3.h"
#include "csf_seg/cloth.h"
#include "csf_seg/rasterization.h"
#include "csf_seg/c2cdist.h"
#include <fstream>

CSFGroundSeg::CSFGroundSeg()
{
}

CSFGroundSeg::~CSFGroundSeg()
{
}

void CSFGroundSeg::segmentGround(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& in_points,
                                 pcl::PointCloud<pcl::PointXYZI>& out_groundless_points,
                                 pcl::PointCloud<pcl::PointXYZI>& out_ground_points)
{
  // Set input point cloud
  setPointCloud(in_points);

  // Get ground and non-ground points indices
  std::vector<int> csf_ground_indices, csf_non_ground_indices;
  doFiltering(csf_ground_indices, csf_non_ground_indices);

  // Separate input Point cloud into ground and non-ground
  pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices());
  ground_indices->indices = csf_ground_indices;

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(in_points);
  extract.setIndices(ground_indices);
  extract.setNegative(false);
  extract.filter(out_ground_points);
  extract.setNegative(true);
  extract.filter(out_groundless_points);
}

void CSFGroundSeg::segmentGround(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& in_points,
                                 pcl::PointCloud<pcl::PointXYZI>& out_upper_ground_points,
                                 pcl::PointCloud<pcl::PointXYZI>& out_lower_ground_points,
                                 pcl::PointCloud<pcl::PointXYZI>& out_ground_points)
{
  // Set input point cloud
  setPointCloud(in_points);

  // Get ground and non-ground points indices
  std::vector<int> csf_ground_indices, csf_upper_ground_indices, csf_lower_ground_indices;
  doFiltering(csf_ground_indices, csf_upper_ground_indices, csf_lower_ground_indices);

  // Separate input Point cloud into ground and non-ground
  pcl::PointIndices::Ptr ground_indices(new pcl::PointIndices());
  ground_indices->indices = csf_ground_indices;
  pcl::PointIndices::Ptr upper_ground_indices(new pcl::PointIndices());
  upper_ground_indices->indices = csf_upper_ground_indices;
  pcl::PointIndices::Ptr lower_ground_indices(new pcl::PointIndices());
  lower_ground_indices->indices = csf_lower_ground_indices;

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(in_points);
  extract.setIndices(ground_indices);
  extract.filter(out_ground_points);
  extract.setIndices(upper_ground_indices);
  extract.filter(out_upper_ground_points);
  extract.setIndices(lower_ground_indices);
  extract.filter(out_lower_ground_points);
}

void CSFGroundSeg::getGroundSurface(pcl::PointCloud<pcl::PointXYZ>& ground_surface)
{
  ground_surface = ground_surface_;
}

void CSFGroundSeg::paramInitialize(const std::string& config_file)
{
  std::cout << "Loading CSFGroundSeg params" << std::endl;
  try
  {
    YAML::Node conf = YAML::LoadFile(config_file);
    params_.slope_smooth = conf["csf_seg"]["slope_smooth"].as<bool>();
    params_.time_step = conf["csf_seg"]["time_step"].as<double>();
    params_.class_threshold = conf["csf_seg"]["class_threshold"].as<double>();
    params_.cloth_resolution = conf["csf_seg"]["cloth_resolution"].as<double>();
    params_.rigidness = conf["csf_seg"]["rigidness"].as<int>();
    params_.iterations = conf["csf_seg"]["iterations"].as<int>();

    // Print the parameters
    std::cout << "Slope smoothing: " << params_.slope_smooth << '\n';
    std::cout << "Time step: " << params_.time_step << '\n';
    std::cout << "Class threshold: " << params_.class_threshold << '\n';
    std::cout << "Cloth resolution: " << params_.cloth_resolution << '\n';
    std::cout << "Rigidness: " << params_.rigidness << '\n';
    std::cout << "Iterations: " << params_.iterations << '\n';
  }
  catch (YAML::Exception& e)
  {
    std::cerr << "\033[31;1mYAML Error: " << e.what() << "\033[m" << std::endl;
    exit(3);
  }
}

void CSFGroundSeg::paramInitialize(const CSFSegParams& params)
{
  std::cout << "Set CSFGroundSeg params" << std::endl;
  params_ = params;

}

void CSFGroundSeg::setPointCloud(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr& input_cloud)
{
  // Clear the point cloud
  this->point_cloud.resize(0);
  // Fill the CSF point cloud with points from PCL point cloud
  for (size_t i = 0; i < input_cloud->points.size(); i++)
  {
    csf::Point point;
    point.x = input_cloud->points[i].x;
    point.y = -input_cloud->points[i].z;
    point.z = input_cloud->points[i].y;
    point_cloud.push_back(point);
  }
}

void CSFGroundSeg::doFiltering(std::vector<int>& groundIndexes, std::vector<int>& offGroundIndexes)
{
  // Terrain
  std::cout << "Configuring terrain..." << std::endl;
  csf::Point bbMin, bbMax;
  point_cloud.computeBoundingBox(bbMin, bbMax);

  double cloth_y_height = 0.05;

  int clothbuffer_d = 2;
  Vec3 origin_pos(bbMin.x - clothbuffer_d * params_.cloth_resolution, bbMax.y + cloth_y_height,
                  bbMin.z - clothbuffer_d * params_.cloth_resolution);

  int width_num = static_cast<int>(std::floor((bbMax.x - bbMin.x) / params_.cloth_resolution)) + 2 * clothbuffer_d;

  int height_num = static_cast<int>(std::floor((bbMax.z - bbMin.z) / params_.cloth_resolution)) + 2 * clothbuffer_d;

  std::cout << "Configuring cloth..." << std::endl;
  std::cout << "width: " << width_num << " "
            << "height: " << height_num << std::endl;

  Cloth cloth(origin_pos, width_num, height_num, params_.cloth_resolution, params_.cloth_resolution, 0.3, 9999,
              params_.rigidness, params_.time_step);

  std::cout << "Rasterizing..." << std::endl;
  Rasterization::RasterTerrian(cloth, point_cloud, cloth.getHeightvals());

  double time_step2 = params_.time_step * params_.time_step;
  double gravity = 0.2;

  std::cout << "Simulating..." << std::endl;
  cloth.addForce(Vec3(0, -gravity, 0) * time_step2);

  // boost::progress_display pd(params_.iteration);
  for (int i = 0; i < params_.iterations; i++)
  {
    double maxDiff = cloth.timeStep();
    cloth.terrCollision();
    // params_.class_threshold / 100
    if ((maxDiff != 0) && (maxDiff < 0.005))
    {
      // early stop
      break;
    }
    // pd++;
  }

  if (params_.slope_smooth)
  {
    std::cout << "Post processing..." << std::endl;
    cloth.movableFilter();
  }

  // Export ground surface
  ground_surface_.clear();
  for (int i = 0; i < cloth.getSize(); i++)
  {
    Particle* particle = cloth.getParticle1d(i);

    pcl::PointXYZ point;
    point.x = particle->getPos().f[0];
    point.y = particle->getPos().f[2];
    point.z = -particle->getPos().f[1];
    ground_surface_.push_back(point);
  }

  c2cdist c2c(params_.class_threshold);
  c2c.calCloud2CloudDist(cloth, point_cloud, groundIndexes, offGroundIndexes);
}

void CSFGroundSeg::doFiltering(std::vector<int>& groundIndexes, std::vector<int>& upperGroundIndexes,
                               std::vector<int>& lowerGroundIndexes)
{
  // Terrain
  std::cout << "Configuring terrain..." << std::endl;
  csf::Point bbMin, bbMax;
  point_cloud.computeBoundingBox(bbMin, bbMax);

  double cloth_y_height = 0.05;

  int clothbuffer_d = 2;
  Vec3 origin_pos(bbMin.x - clothbuffer_d * params_.cloth_resolution, bbMax.y + cloth_y_height,
                  bbMin.z - clothbuffer_d * params_.cloth_resolution);

  int width_num = static_cast<int>(std::floor((bbMax.x - bbMin.x) / params_.cloth_resolution)) + 2 * clothbuffer_d;

  int height_num = static_cast<int>(std::floor((bbMax.z - bbMin.z) / params_.cloth_resolution)) + 2 * clothbuffer_d;

  std::cout << "Configuring cloth..." << std::endl;
  std::cout << "width: " << width_num << " "
            << "height: " << height_num << std::endl;

  Cloth cloth(origin_pos, width_num, height_num, params_.cloth_resolution, params_.cloth_resolution, 0.3, 9999,
              params_.rigidness, params_.time_step);

  std::cout << "Rasterizing..." << std::endl;
  Rasterization::RasterTerrian(cloth, point_cloud, cloth.getHeightvals());

  double time_step2 = params_.time_step * params_.time_step;
  double gravity = 0.2;

  std::cout << "Simulating..." << std::endl;
  cloth.addForce(Vec3(0, -gravity, 0) * time_step2);

  // boost::progress_display pd(params_.iteration);
  int current =0;
  for (int i = 0; i < params_.iterations; i++)
  {
    std::cout << "Progress " << current++ << "/" << params_.iterations << "\r" << std::flush;

    double maxDiff = cloth.timeStep();
    cloth.terrCollision();
    // params_.class_threshold / 100
    if ((maxDiff != 0) && (maxDiff < 0.005))
    {
      // early stop
      break;
    }
    // pd++;
  }
    std::cout << "Progress " << current++ << "/" << params_.iterations << std::endl;

  if (params_.slope_smooth)
  {
    std::cout << "Post processing..." << std::endl;
    cloth.movableFilter();
  }

  // Export ground surface
  ground_surface_.clear();
  for (int i = 0; i < cloth.getSize(); i++)
  {
    Particle* particle = cloth.getParticle1d(i);

    pcl::PointXYZ point;
    point.x = particle->getPos().f[0];
    point.y = particle->getPos().f[2];
    point.z = -particle->getPos().f[1];
    ground_surface_.push_back(point);
  }

  c2cdist c2c(params_.class_threshold);
  c2c.calCloud2CloudDist(cloth, point_cloud, groundIndexes, upperGroundIndexes, lowerGroundIndexes);
}
