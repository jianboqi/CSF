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

#include "Cloth.h"
#include <fstream>

Cloth::Cloth(const Vec3 &_origin_pos, int _num_particles_width,
             int _num_particles_height, double _step_x, double _step_y,
             double _smoothThreshold, double _heightThreshold, int rigidness,
             double time_step)
    : constraint_iterations(rigidness), time_step(time_step),
      smoothThreshold(_smoothThreshold), heightThreshold(_heightThreshold),
      origin_pos(_origin_pos), step_x(_step_x), step_y(_step_y),
      num_particles_width(_num_particles_width),
      num_particles_height(_num_particles_height) {

  constexpr double gravity = 0.2; // TODO: make it a parameter
  double time_step2 = time_step * time_step;
  const double displacement = -gravity * time_step2;

  // We are essentially using this vector as an array with room for
  // num_particles_width*num_particles_height particles
  particles.reserve(num_particles_width * num_particles_height);
  // creating particles in a grid of particles from (0,0,0) to
  // (width,-height,0) creating particles in a grid

  for (int i = 0; i < num_particles_height; i++) { // arrange in Row Major order
    for (int j = 0; j < num_particles_width; j++) {
      // insert particle in column i at j'th row
      particles.emplace_back(Vec3(origin_pos.f[0] + j * step_x, origin_pos.f[1],
                                  origin_pos.f[2] + i * step_y),
                             displacement, j, i);
    }
  }

  // Connecting immediate neighbor particles with constraints
  // (distance 1 and sqrt(2) in the grid)
  for (int x = 0; x < num_particles_width; x++) {
    for (int y = 0; y < num_particles_height; y++) {
      if (x < num_particles_width - 1)
        makeConstraint(getParticle(x, y), getParticle(x + 1, y));

      if (y < num_particles_height - 1)
        makeConstraint(getParticle(x, y), getParticle(x, y + 1));

      if ((x < num_particles_width - 1) && (y < num_particles_height - 1))
        makeConstraint(getParticle(x, y), getParticle(x + 1, y + 1));

      if ((x < num_particles_width - 1) && (y < num_particles_height - 1))
        makeConstraint(getParticle(x + 1, y), getParticle(x, y + 1));
    }
  }

  // Connecting secondary neighbors with constraints (distance 2 and sqrt(4) in
  // the grid)
  for (int x = 0; x < num_particles_width; x++) {
    for (int y = 0; y < num_particles_height; y++) {
      if (x < num_particles_width - 2)
        makeConstraint(getParticle(x, y), getParticle(x + 2, y));

      if (y < num_particles_height - 2)
        makeConstraint(getParticle(x, y), getParticle(x, y + 2));

      if ((x < num_particles_width - 2) && (y < num_particles_height - 2))
        makeConstraint(getParticle(x, y), getParticle(x + 2, y + 2));

      if ((x < num_particles_width - 2) && (y < num_particles_height - 2))
        makeConstraint(getParticle(x + 2, y), getParticle(x, y + 2));
    }
  }
}

double Cloth::timeStep() {
  int particleCount = static_cast<int>(particles.size());
#ifdef CSF_USE_OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < particleCount; i++) {
    particles[i].timeStep();
  }

  for (int j = 0; j < particleCount; j++) {
    particles[j].satisfyConstraintSelf(constraint_iterations);
  }

  double max_diff = 0;
  for (int i = 0; i < particleCount; i++) {
    if (particles[i].isMovable()) {

      max_diff = std::max(
          max_diff, fabs(particles[i].previous_height - particles[i].height));
    }
  }

  return max_diff;
}

void Cloth::terrCollision() {
  const int particleCount = static_cast<int>(particles.size());

#ifdef CSF_USE_OPENMP
#pragma omp parallel for
#endif
  for (int i = 0; i < particleCount; i++) {
    const double displacement = particles[i].height;

    if (displacement < height_values[i]) {
      particles[i].offsetPos(height_values[i] - displacement);
      particles[i].makeUnmovable();
    }
  }
}

void Cloth::movableFilter() {

  for (int x = 0; x < num_particles_width; x++) {
    for (int y = 0; y < num_particles_height; y++) {
      Particle *ptc = getParticle(x, y);

      if (ptc->isMovable() && !ptc->is_visited) {
        std::queue<int> queue;
        std::vector<XY> connected; // store the connected component
        std::vector<std::vector<int>> neighbors;
        int sum = 1;
        int index = y * num_particles_width + x;

        // visit the init node
        connected.emplace_back(x, y);
        particles[index].is_visited = true;

        // enqueue the init node
        queue.push(index);

        while (!queue.empty()) {
          const Particle &ptc_f = particles[queue.front()];
          queue.pop();
          int cur_x = ptc_f.pos_x;
          int cur_y = ptc_f.pos_y;
          std::vector<int> neighbor;

          if (cur_x > 0) {
            Particle *ptc_left = getParticle(cur_x - 1, cur_y);

            if (ptc_left->isMovable()) {
              if (!ptc_left->is_visited) {
                ptc_left->is_visited = true;
                connected.emplace_back(cur_x - 1, cur_y);
                queue.push(num_particles_width * cur_y + cur_x - 1);
                neighbor.push_back(sum);
                ptc_left->c_pos = sum++;
              } else {
                neighbor.push_back(ptc_left->c_pos);
              }
            }
          }

          if (cur_x < num_particles_width - 1) {
            Particle *ptc_right = getParticle(cur_x + 1, cur_y);

            if (ptc_right->isMovable()) {
              if (!ptc_right->is_visited) {
                ptc_right->is_visited = true;
                connected.emplace_back(cur_x + 1, cur_y);
                queue.push(num_particles_width * cur_y + cur_x + 1);
                neighbor.push_back(sum);
                ptc_right->c_pos = sum++;
              } else {
                neighbor.push_back(ptc_right->c_pos);
              }
            }
          }

          if (cur_y > 0) {
            Particle *ptc_bottom = getParticle(cur_x, cur_y - 1);

            if (ptc_bottom->isMovable()) {
              if (!ptc_bottom->is_visited) {
                ptc_bottom->is_visited = true;
                connected.emplace_back(cur_x, cur_y - 1);
                queue.push(num_particles_width * (cur_y - 1) + cur_x);
                neighbor.push_back(sum);
                ptc_bottom->c_pos = sum++;
              } else {
                neighbor.push_back(ptc_bottom->c_pos);
              }
            }
          }

          if (cur_y < num_particles_height - 1) {
            Particle *ptc_top = getParticle(cur_x, cur_y + 1);

            if (ptc_top->isMovable()) {
              if (!ptc_top->is_visited) {
                ptc_top->is_visited = true;
                connected.emplace_back(cur_x, cur_y + 1);
                queue.push(num_particles_width * (cur_y + 1) + cur_x);
                neighbor.push_back(sum);
                ptc_top->c_pos = sum++;
              } else {
                neighbor.push_back(ptc_top->c_pos);
              }
            }
          }
          neighbors.push_back(neighbor);
        }

        if (sum > max_particle_for_post_processing) {
          std::vector<int> edgePoints = findUnmovablePoint(connected);
          handle_slop_connected(edgePoints, connected, neighbors);
        }
      }
    }
  }
}

std::vector<int> Cloth::findUnmovablePoint(const std::vector<XY> & connected) {
  std::vector<int> edgePoints;

  for (size_t i = 0; i < connected.size(); i++) {
    int x = connected[i].x;
    int y = connected[i].y;
    int index = y * num_particles_width + x;
    Particle *ptc = getParticle(x, y);

    if (x > 0) {
      Particle *ptc_x = getParticle(x - 1, y);

      if (!ptc_x->isMovable()) {
        int index_ref = y * num_particles_width + x - 1;

        if ((fabs(height_values[index] - height_values[index_ref]) <
             smoothThreshold) &&
            (ptc->height - height_values[index] < heightThreshold)) {
          particles[index].offsetPos(height_values[index] - ptc->height);
          ptc->makeUnmovable();
          edgePoints.push_back(i);
          continue;
        }
      }
    }

    if (x < num_particles_width - 1) {
      Particle *ptc_x = getParticle(x + 1, y);

      if (!ptc_x->isMovable()) {
        int index_ref = y * num_particles_width + x + 1;

        if ((fabs(height_values[index] - height_values[index_ref]) <
             smoothThreshold) &&
            (ptc->height - height_values[index] < heightThreshold)) {
          particles[index].offsetPos(height_values[index] - ptc->height);
          ptc->makeUnmovable();
          edgePoints.push_back(i);
          continue;
        }
      }
    }

    if (y > 0) {
      Particle *ptc_y = getParticle(x, y - 1);

      if (!ptc_y->isMovable()) {
        int index_ref = (y - 1) * num_particles_width + x;

        if ((fabs(height_values[index] - height_values[index_ref]) <
             smoothThreshold) &&
            (ptc->height- height_values[index] < heightThreshold)) {
          particles[index].offsetPos(height_values[index] - ptc->height);
          ptc->makeUnmovable();
          edgePoints.push_back(i);
          continue;
        }
      }
    }

    if (y < num_particles_height - 1) {
      Particle *ptc_y = getParticle(x, y + 1);

      if (!ptc_y->isMovable()) {
        int index_ref = (y + 1) * num_particles_width + x;

        if ((fabs(height_values[index] - height_values[index_ref]) <
             smoothThreshold) &&
            (ptc->height - height_values[index] < heightThreshold)) {
          particles[index].offsetPos(height_values[index] - ptc->height);
          ptc->makeUnmovable();
          edgePoints.push_back(i);
          continue;
        }
      }
    }
  }

  return edgePoints;
}

void Cloth::handle_slop_connected(
    const std::vector<int> &edgePoints, const std::vector<XY> &connected,
    const std::vector<std::vector<int>> &neighbors) {
  std::vector<bool> visited;

  for (std::size_t i = 0; i < connected.size(); i++)
    visited.push_back(false);

  std::queue<int> queue;

  for (size_t i = 0; i < edgePoints.size(); i++) {
    queue.push(edgePoints[i]);
    visited[edgePoints[i]] = true;
  }

  while (!queue.empty()) {
    int index = queue.front();
    queue.pop();

    int index_center =
        connected[index].y * num_particles_width + connected[index].x;

    for (size_t i = 0; i < neighbors[index].size(); i++) {
      int index_neighbor =
          connected[neighbors[index][i]].y * num_particles_width +
          connected[neighbors[index][i]].x;

      if ((fabs(height_values[index_center] - height_values[index_neighbor]) <
           smoothThreshold) &&
          (fabs(particles[index_neighbor].height -
                height_values[index_neighbor]) < heightThreshold)) {
        particles[index_neighbor].offsetPos(
            height_values[index_neighbor] -
            particles[index_neighbor].height);
        particles[index_neighbor].makeUnmovable();

        if (visited[neighbors[index][i]] == false) {
          queue.push(neighbors[index][i]);
          visited[neighbors[index][i]] = true;
        }
      }
    }
  }
}

std::vector<double> Cloth::toVector() {
  std::vector<double> clothCoordinates;
  clothCoordinates.reserve(particles.size() * 3);
  for (auto &particle : particles) {
    clothCoordinates.push_back(particle.initial_pos.f[0]);
    clothCoordinates.push_back(particle.initial_pos.f[2]);
    clothCoordinates.push_back(-particle.height);
  }
  return clothCoordinates;
}

void Cloth::saveToFile(std::string path) {
  std::string filepath = "cloth_nodes.txt";

  if (path == "") {
    filepath = "cloth_nodes.txt";
  } else {
    filepath = path;
  }

  std::ofstream f1(filepath.c_str());

  if (!f1)
    return;

  for (std::size_t i = 0; i < particles.size(); i++) {
    f1 << std::fixed << std::setprecision(8) << particles[i].initial_pos.f[0]
       << "	" << particles[i].initial_pos.f[2] << "	"
       << -particles[i].height << std::endl;
  }

  f1.close();
}

void Cloth::saveMovableToFile(std::string path) {
  std::string filepath = "cloth_movable.txt";

  if (path == "") {
    filepath = "cloth_movable.txt";
  } else {
    filepath = path;
  }

  std::ofstream f1(filepath.c_str());

  if (!f1)
    return;

  for (std::size_t i = 0; i < particles.size(); i++) {
    if (particles[i].isMovable()) {
      f1 << std::fixed << std::setprecision(8) << particles[i].initial_pos.f[0]
         << "	" << particles[i].initial_pos.f[2] << "	"
         << -particles[i].height << std::endl;
    }
  }

  f1.close();
}
