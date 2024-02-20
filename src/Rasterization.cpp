// ======================================================================================
// Copyright 2017 State Key Laboratory of Remote Sensing Science,
// Institute of Remote Sensing Science and Engineering, Beijing Normal
// University

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

// find height by scanning the nearest particles in the same row and column
double Rasterization::findHeightValByScanline(Particle *p, Cloth &cloth) {
  int xpos = p->pos_x;
  int ypos = p->pos_y;

  for (int i = xpos + 1; i < cloth.num_particles_width; i++) {
    const double crresHeight = cloth.getParticle(i, ypos)->nearest_point_height;

    if (crresHeight > MIN_INF)
      return crresHeight;
  }

  for (int i = xpos - 1; i >= 0; i--) {
    const double crresHeight = cloth.getParticle(i, ypos)->nearest_point_height;

    if (crresHeight > MIN_INF)
      return crresHeight;
  }

  for (int j = ypos - 1; j >= 0; j--) {
    const double crresHeight = cloth.getParticle(xpos, j)->nearest_point_height;

    if (crresHeight > MIN_INF)
      return crresHeight;
  }

  for (int j = ypos + 1; j < cloth.num_particles_height; j++) {
    const double crresHeight = cloth.getParticle(xpos, j)->nearest_point_height;

    if (crresHeight > MIN_INF)
      return crresHeight;
  }

  return findHeightValByNeighbor(p);
}

// find height by Region growing around the current particle
double Rasterization::findHeightValByNeighbor(Particle *p) {
  std::queue<Particle *> nqueue;
  std::vector<Particle *> pbacklist;

  // TODO RJ: this algorithm left the visited flag of some particles to "true"
  // it should be reseted to "false" after the algorithm is done because this
  // flag is reused in the cloth simultation. This is a bug with minor
  // consequences it seems to apply only to particles with id 0,1,
  // particle_number-1 and particle_number-2

  // initialize the queue with the neighbors of the current particle
  for (auto neighbor : p->neighborsList) {
    p->is_visited = true;
    nqueue.push(neighbor);
  }

  // iterate over a queue of neighboring particles
  while (!nqueue.empty()) {
    Particle *pneighbor = nqueue.front();
    nqueue.pop();
    pbacklist.push_back(pneighbor);

    // if the current enqueued particle has a height defined, we return it
    if (pneighbor->nearest_point_height > MIN_INF) {

      // reset the visited flag for all the particles in the backlist...
      for (auto p : pbacklist) {
        p->is_visited = false;
      };

      //... And reset the visited flag for all the particles in the queue
      while (!nqueue.empty()) {
        Particle *pp = nqueue.front();
        pp->is_visited = false;
        nqueue.pop();
      }

      // return the height value
      return pneighbor->nearest_point_height;

    } else { // else we schedule to visit the neighbors of the current neighbor
      for (auto ptmp : pneighbor->neighborsList) {
        if (!ptmp->is_visited) {
          ptmp->is_visited = true;
          nqueue.push(ptmp);
        }
      }
    }
  }

  return MIN_INF;
}

void Rasterization::Rasterize(Cloth &cloth, const csf::PointCloud &pc,
                              std::vector<double> &heightVal) {

  for (const auto &point : pc) {

    const double delta_x = point.x - cloth.origin_pos.f[0];
    const double delta_z = point.z - cloth.origin_pos.f[2];
    const int col = int(delta_x / cloth.step_x + 0.5);
    const int row = int(delta_z / cloth.step_y + 0.5);

    if ((col >= 0) && (row >= 0)) {
      Particle *particle = cloth.getParticle(col, row);
      const double point_to_particle_dist = SQUARE_DIST(
          point.x, point.z, particle->initial_pos.f[0], particle->initial_pos.f[2]);

      if (point_to_particle_dist < particle->tmp_dist) {
        particle->tmp_dist = point_to_particle_dist;
        particle->nearest_point_height = point.y;
      }
    }
  }

  heightVal.resize(cloth.getSize());
  for (int i = 0; i < cloth.getSize(); i++) {
    Particle *pcur = cloth.getParticle1d(i);
    const double nearest_height = pcur->nearest_point_height;

    if (nearest_height > MIN_INF) {
      heightVal[i] = nearest_height;
    } else {
      // fill height value for cells without height value yet
      heightVal[i] = findHeightValByScanline(pcur, cloth);
    }
  }
}
