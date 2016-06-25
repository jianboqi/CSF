#ifndef _C2CDIST_H_
#define _C2CDIST_H_


#include "Cloth.h"

//计算点云与点云之间的距离

class c2cdist
{
public:
	c2cdist(double threshold):class_treshold(threshold){}
	~c2cdist(){}
public:
	vector<int> calCloud2CloudDist(Cloth& cloth, PointCloud &pc);
private:
	double class_treshold;//
};



#endif