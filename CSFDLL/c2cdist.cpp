#include "c2cdist.h"
#include <cmath>


vector<int> c2cdist::calCloud2CloudDist(Cloth& cloth, PointCloud &pc)
{
	vector<int> re;
	//找到每个激光雷达点到布料直接的距离，用该距离阈值来对点云进行分类
//#pragma omp parallel for
	for (int i = 0; i < cloth.getSize(); i++)
	{
		Particle * pt = cloth.getParticle1d(i);
		std::vector<int> nearestLidarList = pt->correspondingLidarPointList;
		for (int j = 0; j < nearestLidarList.size(); j++)
		{
			float height_var = pt->getPos().f[1] - pc[nearestLidarList[j]].y;
			if (std::fabs(height_var) < class_treshold)
			{
//#pragma omp critical
				re.push_back(nearestLidarList[j]);
			}
		}
	}
	return re;
}



//vector<int> c2cdist::calCloud2CloudDist(Cloth cloth, PointCloud &pc)
//{
//	vector<int> re;
//
//	typedef KDTreeSingleIndexAdaptor<
//		L2_Simple_Adaptor<float,Cloth>,
//		Cloth,
//		2> my_kdtree_t;
//
//	my_kdtree_t index(2,cloth,KDTreeSingleIndexAdaptorParams(50000 /* max leaf */));
//	index.buildIndex();
//
//	const size_t num_results = 5;
//	std::vector<size_t>   ret_index(num_results);
//	std::vector<float> out_dist_sqr(num_results);
//	for (int i = 0; i < pc.size(); i++)
//	{
//		const float query_pt[3] = {pc[i].x,0,pc[i].z};
//		index.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);
//		float height_var = cloth.getParticle1d(ret_index[0])->getPos().f[1] - pc[i].y;
//		if (std::fabs(height_var) < class_treshold)
//		{
//			re.push_back(i);
//		}
//	}
//	return re;
//}


//vector<int> c2cdist::calCloud2CloudDist(Cloth cloth, PointCloud &pc)
//{
//	vector<int> re;
//
//	const unsigned int N = 3;
//	list<Point_d> points_2d;
//	map<string, double >mapstring;
//	/// maping coordinates xy->z  to query the height value of each point
//	for (int i = 0; i < cloth.getSize(); i++)
//	{
//		ostringstream ostrx, ostrz;
//		ostrx << cloth.getParticle1d(i)->getPos().f[0];
//		ostrz << cloth.getParticle1d(i)->getPos().f[2];
//		mapstring.insert(pair<string, double>(ostrx.str() + ostrz.str(), cloth.getParticle1d(i)->getPos().f[1]));
//		points_2d.push_back(Point_d( cloth.getParticle1d(i)->getPos().f[0], cloth.getParticle1d(i)->getPos().f[2]));
//	}
//	Tree tree(points_2d.begin(), points_2d.end());
//	// step two  query the nearest point of cloth for each terr point
//	double height_var=0;
//	for (int i = 0; i < pc.size(); i++)
//	{
//		Point_d query(pc[i].x,pc[i].z);
//		Neighbor_search search(tree, query, N);
//		double search_min = 0;
//		for (Neighbor_search::iterator it = search.begin(); it != search.end(); it++)
//		{
//			ostringstream ostrx, ostrz;
//			ostrx << it->first.x();
//			ostrz << it->first.y();
//			double y = mapstring[ostrx.str() + ostrz.str()];
//			search_min = search_min + y / double(N);
//		//	if (y > search_min){ search_min = y; }
//		}
//		height_var = search_min - pc[i].y;
//		if (std::fabs(height_var) < class_treshold)
//		{
//			re.push_back(i);
//		}
//	}
//
//	return re;
//}