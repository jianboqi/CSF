
//#######################################################################################
//#                                                                                     #
//#                              CLOUDCOMPARE PLUGIN: qCSF                              #
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

#include "point_cloud.h"
#include <iostream>
using namespace wl;
//#pragma once;






//该宏完成在dll项目内部使用__declspec(dllexport)导出
//在dll项目外部使用时，用__declspec(dllimport)导入
//宏DLL_IMPLEMENT在CSF.cpp中定义
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
	void setPointCloud(vector< Point > points);
	//从文件读取点云 主要用于测试  read pointcloud from txt file: (X Y Z) for each line
	void readPointsFromFile(string filename);
	//保存地面点到文件 调试用 save extracted ground points to file
	void saveGroundPoints(vector<int> grp, string path = "");

	//get size of pointcloud
	size_t size(){return point_cloud.size();}


	//从已有的PointCloud中输入  set pointcloud 
	void setPointCloud(PointCloud &pc);

	//执行滤波处理 得到地面点的在PointCloud 中的序号  
	//do filtering, the results are index of ground points in the original pointcloud
	vector<int> do_filtering();

private:
	 /*class __declspec (dllexport)*/ wl::PointCloud point_cloud;

public:

	struct{

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