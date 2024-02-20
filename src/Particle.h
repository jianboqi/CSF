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

#pragma once

#include "Vec3.h"
#include <vector>
/* Some physics constants */

#define MAX_INF 9999999999.0
#define MIN_INF -9999999999.0

/* The particle class represents a particle of mass that can move
 * around in 3D space*/
static constexpr double singleMove1[15] = {
    0,       0.3,     0.51,    0.657,   0.7599,  0.83193, 0.88235, 0.91765,
    0.94235, 0.95965, 0.97175, 0.98023, 0.98616, 0.99031, 0.99322};
static constexpr double doubleMove1[15] = {
    0,      0.3,    0.42,   0.468, 0.4872, 0.4949, 0.498, 0.4992,
    0.4997, 0.4999, 0.4999, 0.5,   0.5,    0.5,    0.5};

class Particle {

  /*
  Initially, we have to make modifications of particle positions for each
  constraint(constraintTimes = rigidness), However, to save computation time, we
  precomputed the total displacement of a particle for all constraintTimes.
  For singleMove1, which means one of the two particles is unmovable, then we
  move the other one only: if constraintTimes = 0: singleMove1 = 0 if
  constraintTimes = 1: singleMove1 = 0.3, i.e., each time we move 0.3 (scale
  factor of the total distance) for a particle towards the other one if
  constraintTimes = 2: singleMove1 = (1-0.3)*0.3+0.3 = 0.51 if constraintTimes =
  3: singleMove1 = (1-0.51)*0.3+0.51 = 0.657
  ...

  For doubleMove1, we move both of the two particles towards each other.
  if constraintTimes = 0: singleMove2 = 0
  if constraintTimes = 1: singleMove2 = 0.3, i.e., each time we move 0.3 (scale
  factor of the total distance) for the two particles towards each other if
  constraintTimes = 2: singleMove2 = (1-0.3*2)*0.3+0.3 = 0.42 if constraintTimes
  = 3: singleMove2 = (1-0.42*2)*0.3+0.42 = 0.468
  ...
  */

  static constexpr double damping =
      0.01; // how much to damp the cloth simulation each frame
  static constexpr double one_minus_damping = 1.0 - damping;

private:
  bool movable; // can the particle move or not ? used to pin parts of the cloth
  const double displacement; // the displacement of the particle in z(y)
                             // direction at each step
  // it's g*dt^2, where g is the gravity, dt is the time step

public:
  // These two memebers are used in the process of edge smoothing after
  // the cloth simulation step.

  // The initial position of the particle in the cloth
  // this is basically only use at export time and it could
  // simply be genrated on the fly given the particle position
  // on the grid
  const Vec3 initial_pos;

  int pos_x; // position in the cloth grid
  int pos_y;

  bool is_visited;
  double tmp_dist;

  // position in a connected component
  int c_pos;

  std::vector<Particle *> neighborsList;
  // store height of the particle at previous time step
  double previous_height;
  // store height of the particle
  double height;

  // store the height of the nearest point in the point cloud
  // this is used in the process of rasterization init the height_values
  // array
  double nearest_point_height;

  void satisfyConstraintSelf(int constraintTimes);

public:
  Particle(const Vec3 &pos, const double displacement, const int pos_x,
           const int pos_y)
      : movable(true), displacement(displacement), initial_pos(pos),
        pos_x(pos_x), pos_y(pos_y) {
    is_visited = false;
    c_pos = 0;
    tmp_dist = MAX_INF;
    height = initial_pos.f[1];
    nearest_point_height = MIN_INF;
    previous_height = initial_pos.f[1];
  }

  bool isMovable() { return movable; }

  /* This is one of the important methods, where the time is
   * progressed a single step size (TIME_STEPSIZE) The method is
   * called by Cloth.time_step()*/
  void timeStep();

  void offsetPos(double offset) {
    if (movable)
      height += offset;
  }

  void makeUnmovable() { movable = false; }

  void printself(std::string s = "") {
    std::cout << s << ": " << initial_pos.f[0] << " movable:  " << this->movable
              << std::endl;
  }
};