//------------------ SimpleDLL.cpp ----------------

//注意此处的宏定义需要写在#include "SimpleDLL.h"之前
//以完成在dll项目内部使用__declspec(dllexport)导出
//在dll项目外部使用时，用__declspec(dllimport)导入
#define DLL_IMPLEMENT

#include "CSF.h"
#include "XYZReader.h"
#include "Vec3.h"
#include "Cloth.h"
#include "Rasterization.h"
#include "c2cdist.h"
//#include "Cloth.h"

CSF::CSF()
{
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


void CSF::setPointCloud(vector< Point > points)
{

}

void CSF::setPointCloud(PointCloud &pc)
{
	point_cloud.resize(pc.size());
	#pragma omp parallel for
	for (int i=0;i<pc.size();i++)
	{
		Point las;
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
	wl::Point bbMin, bbMax;
	point_cloud.computeBoundingBox(bbMin, bbMax);
	//布料初始位置距离地面最高点的位置
	double cloth_y_height = 0.05;
	//计算布料中节点个数
	int clothbuffer_d = 2;//缓冲区大小，两个网格默认
	Vec3 origin_pos(bbMin.x - clothbuffer_d * params.cloth_resolution,
		bbMax.y + cloth_y_height,
		bbMin.z - clothbuffer_d * params.cloth_resolution);

	int width_num = static_cast<int>(floor((bbMax.x - bbMin.x) / params.cloth_resolution)) + 2 * clothbuffer_d;
	int height_num = static_cast<int>(floor((bbMax.z - bbMin.z) / params.cloth_resolution)) + 2 * clothbuffer_d;
	cout<<"Configuring cloth..."<<endl;
	cout << "width: " << width_num << " " << "height: " << height_num << endl;
	Cloth cloth1(terr.cube[1] - terr.cube[0] + clothbuffer_d * 2, terr.cube[5] - terr.cube[4] + clothbuffer_d * 2, width_num, height_num, origin_pos1, 0.3, 9999, params.rigidness, params.time_step, params.cloth_resolution); // one Cloth object of the Cloth class
	//
	Rasterlization raster;
//	vector<double> heightvals;
	cout<<"KNN..."<<endl;
	raster.RasterTerrian(cloth1, point_cloud, cloth1.heightvals);
//	cloth1.setheightvals(heightvals);

	double time_step2 = params.time_step*params.time_step;
	double gravity = 0.2;
	cout<<"Starting simulation..."<<endl;
	cloth1.addForce(Vec3(0, -gravity, 0)*time_step2);
//	boost::progress_display pd(params.interations);
	for (int i = 0; i < params.interations; i++)
	{
		double maxDiff = cloth1.timeStep();
		cloth1.terrCollision(cloth1.heightvals, &terr);

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

//	cloth1.saveToFile();

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