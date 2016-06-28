
// SimpleDLLTest.cpp : 定义控制台应用程序的入口点。
#include <mex.h>
#include <vector>
#include <string>
#include "CSF.h" 
using namespace std;
#pragma comment(lib, "CSF.lib")

//input: pointcloudfilepath riginess isSlopSmooth cloth_resolution 
void csf_filtering(string filepath,int rigidness, bool isSmooth, double cloth_resolution)
{

	CSF csf;
	//step 1 read point cloud
	
	csf.readPointsFromFile(filepath);

	//备注：在实际使用过程中，点云数据由主程序提供，调用函数为
	//csf.setPointCloud(pc);//pc为PointCloud类

	//step 2 设置参数
	csf.params.bSloopSmooth = isSmooth;
	csf.params.class_threshold = 0.5;
	csf.params.cloth_resolution = cloth_resolution;
	csf.params.interations = 500;
	csf.params.rigidness = rigidness ;
	csf.params.time_step = 0.65;

	//step3 执行滤波,result中储存的是地面点的索引 
	vector<int> result = csf.do_filtering();

	csf.saveGroundPoints(result);
	mexPrintf("OK, Done!\n");
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	char filepath[200];
	mxGetString(prhs[0],filepath,200);
	double resolution = mxGetScalar(prhs[3]);
	bool isSmooth =  mxIsLogicalScalarTrue(prhs[2]);
	int rigidness = (int)mxGetScalar(prhs[1]);

	csf_filtering(filepath,rigidness,isSmooth,resolution);

}


