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

#include "Constraint.h"

void Constraint::satisfyConstraint(int constraint_times) {
  double correction_factor = p2->pos.f[1] - p1->pos.f[1];

  if (p1->isMovable() && p2->isMovable()) {
    // Lets make it half that length, so that we can move BOTH p1 and p2.
    double correction_factor_half =
        p2->pos.f[1] -
        p1->pos.f[1] *
            (constraint_times > 14 ? 0.5 : doubleMove[constraint_times - 1]);
    p1->offsetPos(correction_factor_half);
    p2->offsetPos(-correction_factor_half);
  } else if (p1->isMovable() && !p2->isMovable()) {
    double correction_factor_half =
        correction_factor *
        (constraint_times > 14 ? 1 : singleMove[constraint_times - 1]);
    p1->offsetPos(correction_factor_half);
  } else if (!p1->isMovable() && p2->isMovable()) {
    double correction_factor_half =
        correction_factor *
        (constraint_times > 14 ? 1 : singleMove[constraint_times - 1]);
    p2->offsetPos(-correction_factor_half);
  }
}
