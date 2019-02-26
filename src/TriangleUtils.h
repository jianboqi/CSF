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

#ifndef _TRIANGLEUTILS_H_
#define _TRIANGLEUTILS_H_


#define TRI_EPSILON 0.000001

#define CROSS(dest, v1, v2) \
			dest[0] = v1[1] * v2[2] - v1[2] * v2[1]; \
			dest[1] = v1[2] * v2[0] - v1[0] * v2[2]; \
			dest[2] = v1[0] * v2[1] - v1[1] * v2[0];

#define DOT(v1,v2) (v1[0] * v2[0] + v1[1]*v2[1] + v1[2] * v2[2])

#define SUB(dest, v1, v2)  \
			dest[0] = v1[0] - v2[0];  \
			dest[1] = v1[1] - v2[1];  \
			dest[2] = v1[2] - v2[2];
/**
*ray-triangle intersection algorithm from the paper"Fast, Minimum Storage Ray/Triangle Intersection"
*orig: ray origin
*dir: ray direction
*vert0/1/2: triangle vertices
*t(out): distance between ray and triangle
*return: ray is intersected with triangle or not
*/
bool intersect_triangle(double orig[3], double dir[3],
	double vert0[3], double vert1[3], double vert2[3],
	double *t) {
	double edge1[3], edge2[3], tvec[3], pvec[3], qvec[3];
	double det, inv_det;
	double u, v;
	// find the vectors for two edges sharing vert0
	SUB(edge1, vert1, vert0);
	SUB(edge2, vert2, vert0);

	//Calculating the determinant- also used to calculate U paramters
	CROSS(pvec, dir, edge2);

	//If determinant is near zero, ray lies in plane of triangle
	det = DOT(edge1, pvec);
	if (det > -TRI_EPSILON && det < TRI_EPSILON)
		return false;
	inv_det = 1.0 / det;

	//calcualte vector from ray origin to vert0 
	SUB(tvec, orig, vert0);
	// calcualte U paramter
	u = DOT(tvec, pvec) * inv_det;
	if (u < 0.0 || u > 1.0)
		return false;

	//prepare to test v parameter
	CROSS(qvec, tvec, edge1);

	//calcualte V parameter
	v = DOT(dir, qvec)*inv_det;
	if (v < 0.0 || u + v > 1.0)
		return false;

	//ray-triange distance
	*t = DOT(edge2, qvec)*inv_det;
	return true;
}


void computeMinimumHeightForParticleGivenLiDARList(Particle *pcur, const csf::PointCloud& pc, Cloth & cloth,
	const std::vector<int> correspondingLidarPointList, int querying_window_size_half) {
	if (correspondingLidarPointList.size() < 3) {
		return;
	}
	
	//first find the a nearest point sets and project them into plane for triangulation
	//If a particle's correspondingLidarPointList is sufficient for triangulation, we use it
	std::vector<double> coords;
	std::vector<double> coords_y;//the height value for each LiDAR point
	//do triangulation
	//triangulation happens here
	coords.resize(correspondingLidarPointList.size() * 2);
	coords_y.resize(correspondingLidarPointList.size() * 2);
	for (int k = 0; k < correspondingLidarPointList.size(); k++) {
		csf::Point point = pc[correspondingLidarPointList[k]];
		coords[2 * k] = point.x;
		coords[2 * k + 1] = point.z;
		coords_y[2 * k] = point.y;
		coords_y[2 * k + 1] = point.y;
	}

	std::vector<std::size_t> d_triangles;
	try {
		delaunator::Delaunator d(coords);
		d_triangles = d.triangles;
	}
	catch (const runtime_error& error) {
		return;
	}

	//test nearby particle for intersection
	for (int w = -querying_window_size_half; w <= querying_window_size_half; w++) {
		for (int h = -querying_window_size_half; h <= querying_window_size_half; h++) {
			int cloth_coord_x = w + pcur->pos_x;
			int cloth_coord_y = h + pcur->pos_y;
			if (cloth_coord_x >= 0 && cloth_coord_x < cloth.num_particles_width &&
				cloth_coord_y >= 0 && cloth_coord_y < cloth.num_particles_height) {
				Particle *p_neighbor = cloth.getParticle(cloth_coord_x, cloth_coord_y);
				if (!p_neighbor->isInterpolated) {
					//for each triangle
					//d.triangles stores the starting position of each vertex in coords 
					//i.e. [vertex1, vertex2, vertex3, vertex1, vertex2, ...]

					double rayDir[3] = { 0,-1,0 };//only calcualte vertical distance
					bool particleFinishInterpolation = false;
					for (std::size_t tindex = 0; tindex < d_triangles.size(); tindex += 3) {
						double tDist = 0;
						double vert0[3] = { coords[2 * d_triangles[tindex]],coords_y[2 * d_triangles[tindex]],coords[2 * d_triangles[tindex] + 1] };
						double vert1[3] = { coords[2 * d_triangles[tindex + 1]],coords_y[2 * d_triangles[tindex + 1]],coords[2 * d_triangles[tindex + 1] + 1] };
						double vert2[3] = { coords[2 * d_triangles[tindex + 2]],coords_y[2 * d_triangles[tindex + 2]],coords[2 * d_triangles[tindex + 2] + 1] };
						bool isIntersected = intersect_triangle(p_neighbor->getPos().f, rayDir, vert0, vert1, vert2, &tDist);
						if (isIntersected) {
							if (tDist < p_neighbor->interpolated_pointHeight) {
								p_neighbor->interpolated_pointHeight = p_neighbor->getPos().f[1] - tDist;
							}
							particleFinishInterpolation = true;
							break;
						}

					}
					if(particleFinishInterpolation)
						p_neighbor->isInterpolated = true;
				}
			}
		}
	}
}

void computeMinimumHeightForParticleByTriangulation(Particle *pcur,
													const csf::PointCloud& pc,
													Cloth & cloth,
													double rasterization_window_size,
	                                                int downsampling_window_num) {
	
	//Define a window size to include more LiDAR point cloud
	double window_size = rasterization_window_size;//meter
	int sub_window_num = downsampling_window_num; //sub window number for downsampling point cloud

	std::vector<int> merged_correspondingLidarPointList;

	//Find all the pacticle that are within this window
	int steps = int(window_size / cloth.step_x);
	int half_steps = int(steps / 2.0);

	// For each particle, we have built a relative large triangle mesh, which can be used for
	// nerghboring particle, instead of builing one for each of them. This could save some time.
	//Usually an order 
	int querying_window_size_half = (half_steps-2) > 0 ? (half_steps - 2) : 0;

	for (int w = -half_steps; w <= half_steps; w++) {
		for (int h = -half_steps; h <= half_steps; h++) {
			int cloth_coord_x = w + pcur->pos_x;
			int cloth_coord_y = h + pcur->pos_y;
			if (cloth_coord_x >= 0 && cloth_coord_x < cloth.num_particles_width &&
				cloth_coord_y >= 0 && cloth_coord_y < cloth.num_particles_height) {
				Particle *p_neighbor = cloth.getParticle(cloth_coord_x, cloth_coord_y);
				merged_correspondingLidarPointList.insert(merged_correspondingLidarPointList.end(),
					p_neighbor->correspondingLidarPointList.begin(),
					p_neighbor->correspondingLidarPointList.end());
			}
		}
	}
	//filtering high lidar points, such as trees and building to reduce the number of points for triangulation
	//A griding method is used to only keep the highest value for each sub grid in the rasterization window
	std::vector<int> subsampled_correspondingLidarPointList;
	int window_x_left = -half_steps + pcur->pos_x;
	int window_z_up =   -half_steps + pcur->pos_y;
	if (window_x_left < 0) window_x_left = 0;
	if (window_z_up < 0)   window_z_up = 0;
	Vec3 window_left_up_corner = cloth.getParticle(window_x_left, window_z_up)->getPos();
	//This is the acutal window size due to the int()
	//+ cloth.step_x because the corresponding point cloud is also lying in a buffer, hals step_x in each side
	double real_window_size = half_steps * 2 * cloth.step_x + cloth.step_x;
	vector<double> hightestYsInASubWindow(sub_window_num*sub_window_num, MIN_INF);
	vector<int> hightestYsPointIndex(sub_window_num*sub_window_num, MIN_INF);
	for (int i = 0; i < merged_correspondingLidarPointList.size(); i++) {
		csf::Point point = pc[merged_correspondingLidarPointList[i]];
		int x_cord_in_subwindow = int((point.x - window_left_up_corner.f[0]+ cloth.step_x*0.5) / real_window_size * sub_window_num);
		x_cord_in_subwindow = (x_cord_in_subwindow >= sub_window_num) ? (sub_window_num - 1): x_cord_in_subwindow;
		x_cord_in_subwindow = (x_cord_in_subwindow < 0) ? 0 : x_cord_in_subwindow;
		int z_cord_in_subwindow = int((point.z - window_left_up_corner.f[2] + cloth.step_x*0.5) / real_window_size * sub_window_num);
		z_cord_in_subwindow = (z_cord_in_subwindow >= sub_window_num) ? (sub_window_num - 1): z_cord_in_subwindow;
		z_cord_in_subwindow = (z_cord_in_subwindow < 0) ? 0 : z_cord_in_subwindow;
		if (point.y > hightestYsInASubWindow[z_cord_in_subwindow*sub_window_num + x_cord_in_subwindow]) {
			hightestYsInASubWindow[z_cord_in_subwindow*sub_window_num + x_cord_in_subwindow] = point.y;
			hightestYsPointIndex[z_cord_in_subwindow*sub_window_num + x_cord_in_subwindow] = merged_correspondingLidarPointList[i];
		}
	}

	for (int i = 0; i < hightestYsPointIndex.size(); i++) {
		if(hightestYsPointIndex[i]>=0)
			subsampled_correspondingLidarPointList.push_back(hightestYsPointIndex[i]);
	}


	computeMinimumHeightForParticleGivenLiDARList(pcur, pc, cloth, subsampled_correspondingLidarPointList, querying_window_size_half);
}

#endif // ifndef _TRIANGLEUTILS_H_
