#include "Rasterization.h"

void Rasterlization::RasterTerrian(Cloth &cloth, wl::PointCloud &pc, vector<double> &heightVal)
{
	//首先对每个lidar点找到在布料网格中对应的节点，并记录下来
	double tmp;
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
			pt->correspondingLidarPointList.push_back(i);
			double pc2particleDist = SQUARE_DIST(pc_x, pc_z, pt->getPos().f[0], pt->getPos().f[2]);
			if (pc2particleDist < pt->tmpDist)
			{			
				pt->tmpDist = pc2particleDist;
				pt->nearestPointHeight = pc[i].y;
				pt->nearestPointIndex = i;
				tmp = pc[i].y;
			}
		}
	}
	heightVal.resize(cloth.getSize());
//#pragma omp parallel for
	for (int i = 0; i < cloth.getSize(); i++)
	{
		double nearestHeight = cloth.getParticle1d(i)->nearestPointHeight;
		if (nearestHeight > MIN_INF)
		{
			heightVal[i] = nearestHeight;
			tmp = nearestHeight;
		}
		else
		{
			heightVal[i] = tmp;
		}
	}
	
}