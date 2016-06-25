#include "KNN.h"



void Rasterlization::RasterTerrian(Cloth &cloth, PointCloud &pc, vector<double> &heightVal)
{
	//首先对每个lidar点找到在布料网格中对应的节点，并记录下来
//#pragma omp parallel for
	for (int i = 0; i < pc.size(); i++)
	{
		double pc_x = pc[i].x;
		double pc_z = pc[i].z;
		//将该坐标与布料的左上角坐标相减
		double deltaX = pc_x - cloth.origin_pos1.f[0];
		double deltaZ = pc_z - cloth.origin_pos1.f[2];
		int col = int(deltaX / cloth.cloth_resolution + 0.5);
		int row = int(deltaZ / cloth.cloth_resolution + 0.5);
		if (col >= 0 && row >= 0)
		{
			Particle * pt = cloth.getParticle(col, row);
//#pragma omp critical
			pt->correspondingLidarPointList.push_back(i);
			double pc2particleDist = distSquare(pc_x, pc_z, pt->getPos().f[0], pt->getPos().f[2]);
			if (pc2particleDist < pt->tmpDist)
			{
				pt->tmpDist = pc2particleDist;
				pt->nearestPointHeight = pc[i].y;
				pt->nearestPointIndex = i;
			}
		}
	}
	heightVal.resize(cloth.getSize());
#pragma omp parallel for
	for (int i = 0; i < cloth.getSize(); i++)
	{
		heightVal[i] = cloth.getParticle1d(i)->nearestPointHeight;
	}
	
}



//void Rasterlization::RasterTerrian(Cloth cloth, PointCloud &pc, vector<double> &heightVal)
//{
//	typedef KDTreeSingleIndexAdaptor<
//		L2_Simple_Adaptor<float,PointCloud>,
//		PointCloud,
//		2> my_kdtree_t;
//
//	my_kdtree_t index(2, pc, KDTreeSingleIndexAdaptorParams(10 /* max leaf */));
//	index.buildIndex();
//	
//	const size_t num_results = 1;
//	heightVal.resize(cloth.getSize());
//	string outfile = "nearest-nano.txt";
//	ofstream fout(outfile);
//	for (int i = 0; i < cloth.getSize(); i++)
//	{
//		std::vector<size_t>   ret_index(num_results);
//		std::vector<float> out_dist_sqr(num_results);
//		const float query_pt[3] = {cloth.getParticle1d(i)->getPos().f[0],cloth.getParticle1d(i)->getPos().f[1], cloth.getParticle1d(i)->getPos().f[2]};
//		index.knnSearch(&query_pt[0], num_results, &ret_index[0], &out_dist_sqr[0]);
//		heightVal[i] = pc[ret_index[0]].y;
//		fout << cloth.getParticle1d(i)->getPos().f[0] << " " << cloth.getParticle1d(i)->getPos().f[2] << " " << pc[ret_index[0]].x << " " << pc[ret_index[0]].z << " " << out_dist_sqr[0] << " " << heightVal[i] <<" "<< ret_index[0]<< endl;
//	}
//}

//void Rasterlization::RasterTerrian(Cloth cloth, PointCloud &pc, vector<double> &heightVal)
//{
//	//首先建立映射xz->y 即通过xz的坐标能够找到y的坐标，因为所有点云不可能有xz重合的
//	map<string, double >mapstring;
//	list<Point_d> points_2d;
//	for (int i = 0; i < pc.size(); i++){
//		ostringstream ostrx, ostrz;
//		ostrx << pc[i].x;
//		ostrz << pc[i].z;
//		mapstring.insert(pair<string, double>(ostrx.str() + ostrz.str(), pc[i].y));
//		points_2d.push_back(Point_d(pc[i].x, pc[i].z));
//	}
//	Tree tree(points_2d.begin(), points_2d.end());
//
//	heightVal.resize(cloth.getSize());
//	for (int i = 0; i < cloth.getSize(); i++)
//	{
//		Point_d query(cloth.getParticle1d(i)->getPos().f[0], cloth.getParticle1d(i)->getPos().f[2]);
//		Neighbor_search search(tree, query, N);
//		double search_max = -9999;
//		for (Neighbor_search::iterator it = search.begin(); it != search.end(); it++)
//		{
//			ostringstream ostrx, ostrz;
//			ostrx << it->first.x();
//			ostrz << it->first.y();
//			double y = mapstring[ostrx.str() + ostrz.str()];
//			if (y > search_max){ search_max = y; }
//		}
//		heightVal[i] = search_max;
//	}
//
//}