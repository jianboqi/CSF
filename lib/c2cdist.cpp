#include "c2cdist.h"
#include <cmath>


void c2cdist::calCloud2CloudDist(Cloth           & cloth,
                                 csf::PointCloud & pc,
                                 std::vector<int>& groundIndexes,
                                 std::vector<int>& offGroundIndexes) {
    groundIndexes.resize(0);
    offGroundIndexes.resize(0);

    for (std::size_t i = 0; i < pc.size(); i++) {
        double pc_x = pc[i].x;
        double pc_z = pc[i].z;

        double deltaX = pc_x - cloth.origin_pos.f[0];
        double deltaZ = pc_z - cloth.origin_pos.f[2];

        int col0 = int(deltaX / cloth.step_x);
        int row0 = int(deltaZ / cloth.step_y);
        int col1 = col0 + 1;
        int row1 = row0;
        int col2 = col0 + 1;
        int row2 = row0 + 1;
        int col3 = col0;
        int row3 = row0 + 1;

        double subdeltaX = (deltaX - col0 * cloth.step_x) / cloth.step_x;
        double subdeltaZ = (deltaZ - row0 * cloth.step_y) / cloth.step_y;

        double fxy
            = cloth.getParticle(col0, row0)->pos.f[1] * (1 - subdeltaX) * (1 - subdeltaZ) +
              cloth.getParticle(col3, row3)->pos.f[1] * (1 - subdeltaX) * subdeltaZ +
              cloth.getParticle(col2, row2)->pos.f[1] * subdeltaX * subdeltaZ +
              cloth.getParticle(col1, row1)->pos.f[1] * subdeltaX * (1 - subdeltaZ);
        float height_var = fxy - pc[i].y;

        if (std::fabs(height_var) < class_treshold) {
            groundIndexes.push_back(i);
        } else {
            offGroundIndexes.push_back(i);
        }
    }
}
