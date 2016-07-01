#include "Rasterization.h"
#include <queue>

double Rasterlization::findHeightValByScanline(Particle *p, Cloth &cloth)
{
	int xpos = p->pos_x;
	int ypos = p->pos_y;
	//横向向右扫描
	for (int i = xpos+1; i < cloth.num_particles_width; i++)
	{
		double crresHeight = cloth.getParticle(i, ypos)->nearestPointHeight;
		if (crresHeight > MIN_INF) 
			return crresHeight;
	}
	//横向向左扫描
	for (int i = xpos - 1; i >=0 ; i--)
	{
		double crresHeight = cloth.getParticle(i, ypos)->nearestPointHeight;
		if (crresHeight > MIN_INF)
			return crresHeight;
	}
	//纵向向上扫描
	for (int j = ypos - 1; j >= 0; j--)
	{
		double crresHeight = cloth.getParticle(xpos, j)->nearestPointHeight;
		if (crresHeight > MIN_INF)
			return crresHeight;
	}
	//纵向向下扫描
	for (int j = ypos + 1; j < cloth.num_particles_height; j++)
	{
		double crresHeight = cloth.getParticle(xpos, j)->nearestPointHeight;
		if (crresHeight > MIN_INF)
			return crresHeight;
	}

	return findHeightValByNeighbor(p, cloth);


}

double Rasterlization::findHeightValByNeighbor(Particle *p, Cloth &cloth)
{
	queue<Particle*> nqueue;
	vector<Particle *> pbacklist;
	int neiborsize = p->neighborsList.size();
	for (int i = 0; i < neiborsize; i++)
	{
		p->isVisited = true;
		nqueue.push(p->neighborsList[i]);
	}
	
	//iterate over the nqueue
	while (!nqueue.empty())
	{
		Particle *pneighbor = nqueue.front();
		nqueue.pop();
		pbacklist.push_back(pneighbor);
		if (pneighbor->nearestPointHeight > MIN_INF)
		{
			for (int i = 0; i < pbacklist.size(); i++)
				pbacklist[i]->isVisited = false;
			while (!nqueue.empty())
			{
				Particle *pp = nqueue.front();
				pp->isVisited = false;
				nqueue.pop();
			}
			return pneighbor->nearestPointHeight;
		}
		else
		{
			int nsize = pneighbor->neighborsList.size();
			for (int i = 0; i < nsize; i++)
			{
				Particle *ptmp = pneighbor->neighborsList[i];
				if (!ptmp->isVisited)
				{
					ptmp->isVisited = true;
					nqueue.push(ptmp);
				}
			}
				
		}
	}
	return MIN_INF;
}

void Rasterlization::RasterTerrian(Cloth &cloth, csf::PointCloud &pc, vector<double> &heightVal)
{

	//首先对每个lidar点找到在布料网格中对应的节点，并记录下来
	for (int i = 0; i < pc.size(); i++)
	{
		double pc_x = pc[i].x;
		double pc_z = pc[i].z;
		//将该坐标与布料的左上角坐标相减
		double deltaX = pc_x - cloth.origin_pos.f[0];
		double deltaZ = pc_z - cloth.origin_pos.f[2];
		int col = int(deltaX / cloth.step_x + 0.5);
		int row = int(deltaZ / cloth.step_y + 0.5);
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
			}
		}
	}
	heightVal.resize(cloth.getSize());
//#pragma omp parallel for
	for (int i = 0; i < cloth.getSize(); i++)
	{
		Particle *pcur = cloth.getParticle1d(i);
		double nearestHeight = pcur->nearestPointHeight;
		if (nearestHeight > MIN_INF)
		{
			heightVal[i] = nearestHeight;
		}
		else
		{
			heightVal[i] = findHeightValByScanline(pcur, cloth);
		}
		
	}
	
}