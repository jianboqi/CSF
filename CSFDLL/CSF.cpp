//------------------ SimpleDLL.cpp ----------------

//注意此处的宏定义需要写在#include "SimpleDLL.h"之前
//以完成在dll项目内部使用__declspec(dllexport)导出
//在dll项目外部使用时，用__declspec(dllimport)导入
#define DLL_IMPLEMENT

#include "CSF.h"
#include "XYZReader.h"
#include "Terrian.h"
#include "Vec3.h"
#include "Cloth.h"
#include "KNN.h"
#include "c2cdist.h"
//#include "Cloth.h"

CSF::CSF()
{
	params.k_nearest_points = 1;
	params.bSloopSmooth = true;
	params.time_step = 0.65;
	params.class_threshold=0.5;
	params.cloth_resolution = 1;
	params.rigidness = 3;
	params.interations = 500;
}
CSF::~CSF()
{
}


void CSF::setPointCloud(vector< LASPoint > points)
{

}

void CSF::setPointCloud(PointCloud &pc)
{
	point_cloud.resize(pc.size());
	#pragma omp parallel for
	for (int i=0;i<pc.size();i++)
	{
		LASPoint las;
		las.x = pc[i].x;
		las.y = -pc[i].z;
		las.z = pc[i].y;
		point_cloud[i]=las;
	}
}

void CSF::readPointsFromFile(string filename)
{
	read_xyz(filename,this->point_cloud);
}

vector<int> CSF::do_filtering()
{
	vector<int> re;//存储最后分类结果

	//首先从现有创建terrian
	cout<<"Configuring terrain..."<<endl;
	Terrian terr(point_cloud);
	//布料初始位置距离地面最高点的位置
	double cloth_y_height = 0.05;
	//计算布料中节点个数
	double clothbuffer_d = 4;//布料边缘缓冲区大小
	Vec3 origin_pos1(terr.cube[0] - clothbuffer_d, terr.cube[3] + cloth_y_height, terr.cube[4] - clothbuffer_d);
	int width_num = (terr.cube[1] - terr.cube[0] + clothbuffer_d * 2) / params.cloth_resolution;
	int height_num = (terr.cube[5] - terr.cube[4] + clothbuffer_d * 2) /params.cloth_resolution;
	cout<<"Configuring cloth..."<<endl;
	Cloth cloth1(terr.cube[1] - terr.cube[0] + clothbuffer_d * 2, terr.cube[5] - terr.cube[4] + clothbuffer_d * 2, width_num, height_num, origin_pos1, 0.3, 9999, params.rigidness, params.time_step, params.cloth_resolution); // one Cloth object of the Cloth class
	//
	Rasterlization raster(params.k_nearest_points);
	vector<double> heightvals;
	cout<<"KNN..."<<endl;
	raster.RasterTerrian(cloth1, point_cloud, heightvals);
	cloth1.setheightvals(heightvals);

	double time_step2 = params.time_step*params.time_step;
	double gravity = 0.2;
	bool flag = false;
	cout<<"Starting simulation..."<<endl;
//	boost::progress_display pd(params.interations);
	for (int i = 0; i < params.interations; i++)
	{
		cloth1.addForce(Vec3(0, -gravity, 0)*time_step2);
		double maxDiff = cloth1.timeStep();
		cloth1.terrCollision(heightvals, &terr, flag);

		if (maxDiff != 0 && maxDiff < params.class_threshold / 100)
		{
			//early stop
			break;
		}

//		pd++;
	}
	//边坡后处理
	if(params.bSloopSmooth)
	{
		cout<<"post handle..."<<endl;
		cloth1.movableFilter();
	}

	cloth1.saveToFile();

	//分类
	c2cdist c2c(params.class_threshold);
	re = c2c.calCloud2CloudDist(cloth1,point_cloud);
	return re;
}

void CSF::saveGroundPoints(vector<int> grp, string path)
{
	string filepath = "terr_ground.txt";
	if (path == "")
	{
		filepath = "terr_ground.txt";
	}
	else
	{
		filepath = path;
	}
	ofstream f1(filepath, ios::out);
	if (!f1)return;
	for (size_t i = 0; i < grp.size(); i++)
	{
		f1 << fixed << setprecision(8) << point_cloud[grp[i]].x<< "	" << point_cloud[grp[i]].z << "	" << -point_cloud[grp[i]].y << endl;
	}
	f1.close();
}