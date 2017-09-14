// #######################################################################################
// #                                                                                     #
// #            CSF: Airborne LiDAR filtering based on Cloth Simulation                  #
// #                                                                                     #
// #  Please cite the following paper, If you use this software in your work.            #
// #                                                                                     #
// #  Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. An Easy-to-Use Airborne LiDAR  #
// #  Data Filtering Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501.   #
// #                                                                                     #
// #                                     Copyright ©                                     #
// #               RAMM laboratory, School of Geography, Beijing Normal University       #
// #                               (http://ramm.bnu.edu.cn/)                             #
// #                                                                                     #
// #                      Wuming Zhang; Jianbo Qi; Peng Wan; Hongtao Wang                #
// #                                                                                     #
// #                      contact us: 2009zwm@gmail.com; wpqjbzwm@126.com                #
// #                                                                                     #
// #######################################################################################

#ifndef _POINT_CLOUD_H_
#define _POINT_CLOUD_H_

#include <vector>

namespace csf {

struct Point {
    union {
        struct {
            float x;
            float y;
            float z;
        };
        float u[3];
    };

    Point() : x(0), y(0), z(0) {}
};

class PointCloud : public std::vector<Point>{
public:

    void computeBoundingBox(Point& bbMin, Point& bbMax);
};

}


#endif // ifndef _POINT_CLOUD_H_
