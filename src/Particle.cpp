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

#include "Particle.h"

/* This is one of the important methods, where the time is progressed
 *  a single step size (TIME_STEPSIZE) The method is called by
 *  Cloth.time_step() Given the equation "force = mass * acceleration"
 *  the next position is found through verlet integration*/
void Particle::timeStep() {
  if (movable) {
    const double temp_height = height;
    height =
        height + (height - previous_height) * one_minus_damping + displacement;
    previous_height = temp_height;
  }
}

void Particle::satisfyConstraintSelf(int constraintTimes) {
  Particle *p1 = this;

  for (std::size_t i = 0; i < neighborsList.size(); i++) {
    Particle *p2 = neighborsList[i];
    const double correction_factor = p2->height - p1->height;

    if (p1->isMovable() && p2->isMovable()) {
      // Lets make it half that length, so that we can move BOTH p1 and p2.
      const double correction_factor_half =
          correction_factor *
          (constraintTimes > 14 ? 0.5 : doubleMove1[constraintTimes]);
      p1->offsetPos(correction_factor_half);
      p2->offsetPos(-correction_factor_half);
    } else if (p1->isMovable() && !p2->isMovable()) {
      const double correction_factor_half =
          correction_factor *
          (constraintTimes > 14 ? 1 : singleMove1[constraintTimes]);
      p1->offsetPos(correction_factor_half);
    } else if (!p1->isMovable() && p2->isMovable()) {
      const double correction_factor_half =
          correction_factor *
          (constraintTimes > 14 ? 1 : singleMove1[constraintTimes]);
      p2->offsetPos(-correction_factor_half);
    }
  }
}
