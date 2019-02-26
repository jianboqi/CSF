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

#include "Rasterization.h"
#include <queue>
#include <fstream>
#include "delaunator.h"
#include "TriangleUtils.h"

double Rasterization::findHeightValByScanline(Particle *p, Cloth& cloth) {
    int xpos = p->pos_x;
    int ypos = p->pos_y;

    for (int i = xpos + 1; i < cloth.num_particles_width; i++) {
        double crresHeight = cloth.getParticle(i, ypos)->nearestPointHeight;

        if (crresHeight > MIN_INF)
            return crresHeight;
    }

    for (int i = xpos - 1; i >= 0; i--) {
        double crresHeight = cloth.getParticle(i, ypos)->nearestPointHeight;

        if (crresHeight > MIN_INF)
            return crresHeight;
    }

    for (int j = ypos - 1; j >= 0; j--) {
        double crresHeight = cloth.getParticle(xpos, j)->nearestPointHeight;

        if (crresHeight > MIN_INF)
            return crresHeight;
    }

    for (int j = ypos + 1; j < cloth.num_particles_height; j++) {
        double crresHeight = cloth.getParticle(xpos, j)->nearestPointHeight;

        if (crresHeight > MIN_INF)
            return crresHeight;
    }

    return findHeightValByNeighbor(p, cloth);
}


double Rasterization::findHeightValByNeighbor(Particle *p, Cloth& cloth) {
    queue<Particle *>  nqueue;
    vector<Particle *> pbacklist;
    int neiborsize = p->neighborsList.size();

    for (int i = 0; i < neiborsize; i++) {
        p->isVisited = true;
        nqueue.push(p->neighborsList[i]);
    }

    // iterate over the nqueue
    while (!nqueue.empty()) {
        Particle *pneighbor = nqueue.front();
        nqueue.pop();
        pbacklist.push_back(pneighbor);

        if (pneighbor->nearestPointHeight > MIN_INF) {
            for (std::size_t i = 0; i < pbacklist.size(); i++)
                pbacklist[i]->isVisited = false;

            while (!nqueue.empty()) {
                Particle *pp = nqueue.front();
                pp->isVisited = false;
                nqueue.pop();
            }

            return pneighbor->nearestPointHeight;
        } else {
            int nsize = pneighbor->neighborsList.size();

            for (int i = 0; i < nsize; i++) {
                Particle *ptmp = pneighbor->neighborsList[i];

                if (!ptmp->isVisited) {
                    ptmp->isVisited = true;
                    nqueue.push(ptmp);
                }
            }
        }
    }

    return MIN_INF;
}

void Rasterization::RasterTerrian(Cloth          & cloth,
                                  csf::PointCloud& pc,
                                  vector<double> & heightVal,
								  int rasterization_mode,
								  double rasterization_window_size,
	                              int downsampling_window_num) {

    for (std::size_t i = 0; i < pc.size(); i++) {
        double pc_x = pc[i].x;
        double pc_z = pc[i].z;

        double deltaX = pc_x - cloth.origin_pos.f[0];
        double deltaZ = pc_z - cloth.origin_pos.f[2];
        int    col    = int(deltaX / cloth.step_x + 0.5);
        int    row    = int(deltaZ / cloth.step_y + 0.5);

        if ((col >= 0) && (row >= 0)) {
            Particle *pt = cloth.getParticle(col, row);
            pt->correspondingLidarPointList.push_back(i);
            double pc2particleDist = SQUARE_DIST(
                pc_x, pc_z,
                pt->getPos().f[0],
                pt->getPos().f[2]
            );

            if (pc2particleDist < pt->tmpDist) {
                pt->tmpDist            = pc2particleDist;
                pt->nearestPointHeight = pc[i].y;
                pt->nearestPointIndex  = i;
            }
        }
    }

	#pragma omp parallel for
	for (int i = 0; i < cloth.getSize(); i++) {
		Particle *pcur = cloth.getParticle1d(i);
		if (!pcur->isInterpolated) {
			if (rasterization_mode == 0) {  //Method 1: simple nearest point
				// If a Particle does not have a corresponding LiDAR point, nearestPointHeight is initialized as MIN_INF
				double    nearestHeight = pcur->nearestPointHeight;
				if (nearestHeight > MIN_INF) {
					pcur->interpolated_pointHeight = nearestHeight;
				}
				else {
					pcur->interpolated_pointHeight = findHeightValByScanline(pcur, cloth);
				}
			}
			else if (rasterization_mode == 1) {  //Method 2: 2.5D triangulation
				computeMinimumHeightForParticleByTriangulation(pcur, pc, cloth, rasterization_window_size, downsampling_window_num);
			}
			else if (rasterization_mode == 2) {  //Method 3: second-degree polynomial fitting method: Z = a.X2 + b.X + c.XY + d.Y + e.Y2 + f
													// to be implemented
													/////
			}
		}
		
	}

	heightVal.resize(cloth.getSize());
    #pragma omp parallel for
    for (int i = 0; i < cloth.getSize(); i++) {
        Particle *pcur          = cloth.getParticle1d(i);
		if (pcur->interpolated_pointHeight == MAX_INF) {
			heightVal[i] = MIN_INF;
		}
		else {
			heightVal[i] = pcur->interpolated_pointHeight;
		}
    }

}
