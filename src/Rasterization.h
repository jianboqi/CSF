#ifndef _KNN_H_
#define _KNN_H_

#include "point_cloud.h"
#include "Cloth.h"

#define SQUARE_DIST(x1, y1, x2, y2) \
    (((x1) - (x2)) * ((x1) - (x2)) + ((y1) - (y2)) * ((y1) - (y2)))


class Rasterization {
public:

    Rasterization() {}
    ~Rasterization() {}

    // for a cloth particle, if no corresponding lidar point are found.
    // the heightval are set as its neighbor's
    double static findHeightValByNeighbor(Particle *p, Cloth& cloth);
    double static findHeightValByScanline(Particle *p, Cloth& cloth);

    void static   RasterTerrian(Cloth          & cloth,
                                csf::PointCloud& pc,
                                vector<double> & heightVal);
};

#endif // ifndef _KNN_H_
