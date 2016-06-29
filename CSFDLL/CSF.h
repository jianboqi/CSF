
//#######################################################################################
//#                                                                                     #
//#            CSF: Airborne LiDAR filtering based on Cloth Simulation                  #
//#                                                                                     #
//#  Please cite the following paper, If you use this plugin in your work.              #
//#                                                                                     #
//#  Zhang W, Qi J, Wan P, Wang H, Xie D, Wang X, Yan G. An Easy-to-Use Airborne LiDAR  #
//#  Data Filtering Method Based on Cloth Simulation. Remote Sensing. 2016; 8(6):501.   #
//#                                                                                     #
//#                                     Copyright ©                                     #
//#               RAMM laboratory, School of Geography, Beijing Normal University       #
//#                               (http://ramm.bnu.edu.cn/)                             #
//#                                                                                     #
//#                      Wuming Zhang; Jianbo Qi; Peng Wan; Hongtao Wang                #
//#                                                                                     #
//#                      contact us: 2009zwm@gmail.com; wpqjbzwm@126.com                #
//#                                                                                     #
//#######################################################################################



//cloth simulation filter for airborne lidar filtering
#ifndef _CSF_H_
#define _CSF_H_
#include <vector>
#include <string>
#include "point_cloud.h"
using namespace std;


//#ifdef DLL_IMPLEMENT  
//#define DLL_API __declspec(dllexport)  
//#else  
//#define DLL_API __declspec(dllimport)  
//#endif

class /*DLL_API*/  CSF
{
public:
	CSF();
	~CSF();

	//设置点云数据 从vector导入点云 set pointcloud from vector
	void setPointCloud(vector< wl::Point > points);
	//set point cloud from a one-dimentional array. it defines a N*3 point cloud by the given rows.
	void setPointCloud(double *points, int rows);
	//从文件读取点云 主要用于测试  read pointcloud from txt file: (X Y Z) for each line
	void readPointsFromFile(string filename);

	inline wl::PointCloud & getPointCloud(){ return point_cloud; }
	inline const wl::PointCloud & getPointCloud() const{ return point_cloud; }

	//保存点到文件 调试用 save points to file
	void savePoints(vector<int> grp, string path);

	//get size of pointcloud
	size_t size(){return point_cloud.size();}


	//从已有的PointCloud中输入  set pointcloud 
	void setPointCloud(wl::PointCloud &pc);

	//执行滤波处理 得到地面点的在PointCloud 中的序号  
	//do filtering, the results are index of ground points in the original pointcloud
	//groundIndexes gorund points index
	//offGroundIndexes non-ground points index
	void do_filtering(std::vector<int>& groundIndexes, std::vector<int>& offGroundIndexes,bool exportCloth=false);

private:
	 /*class __declspec (dllexport)*/ wl::PointCloud point_cloud;

public:

	struct{
		//refer to the website:http://ramm.bnu.edu.cn/projects/CSF/ for the setting of these paramters
		//是否进行边坡后处理
		bool bSloopSmooth;
		//时间步长
		double time_step;

		//分类阈值
		double class_threshold;

		//布料格网大小
		double cloth_resolution;

		//布料硬度参数
		int rigidness;

		//迭代次数
		int interations;
	}params;
};

#endif