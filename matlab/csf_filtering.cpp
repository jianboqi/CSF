
// SimpleDLLTest.cpp : ¶¨Òå¿ØÖÆÌ¨Ó¦ÓÃ³ÌÐòµÄÈë¿Úµã¡£
#include <mex.h>
#include <vector>
#include <string>
#include "../src/CSF.h"
using namespace std;
// #pragma comment(lib, "CSF.lib")

//input: pointcloud riginess isSlopSmooth cloth_resolution 
void csf_filtering(double* points
	,int rigidness
	, bool isSmooth
	, double cloth_resolution
	, double class_threshold
	, int interations
	, double time_step
	,int rows
	,std::vector<int>& groundIndex
	,std::vector<int>& nongroundIndex
	,int& groundRows
	,int& nongroundRows
	)
{
	#define A(i,j) points[i+j*rows]

	CSF csf;
	//step 1 read point cloud from N*3 array

	csf.setPointCloud(points,rows);

	//±¸×¢£ºÔÚÊµ¼ÊÊ¹ÓÃ¹ý³ÌÖÐ£¬µãÔÆÊý¾ÝÓÉÖ÷³ÌÐòÌá¹©£¬µ÷ÓÃº¯ÊýÎª
	//csf.setPointCloud(pc);//pcÎªPointCloudÀà

	//step 2 ÉèÖÃ²ÎÊý
	csf.params.bSloopSmooth = isSmooth;
	csf.params.class_threshold = class_threshold;
	csf.params.cloth_resolution = cloth_resolution;
	csf.params.interations = interations;
	csf.params.rigidness = rigidness ;
	csf.params.time_step = time_step;

	//step3 Ö´ÐÐÂË²¨,resultÖÐ´¢´æµÄÊÇµØÃæµãµÄË÷Òý 
	csf.do_filtering(groundIndex,nongroundIndex);
	groundRows = groundIndex.size();
	nongroundRows = nongroundIndex.size();
}

void mexFunction(int nlhs, mxArray *plhs[], int nrhs, const mxArray *prhs[])
{
	double *points = mxGetPr(prhs[0]);
	double resolution = mxGetScalar(prhs[3]);
	bool isSmooth =  mxIsLogicalScalarTrue(prhs[2]);
	int rigidness = (int)mxGetScalar(prhs[1]);
	double class_threshold = mxGetScalar(prhs[4]);
	int interations = (int) mxGetScalar(prhs[5]);
	double time_step = mxGetScalar(prhs[6]);

	int rows = mxGetM(prhs[0]);
	std::vector<int> groundIndex,nongroundIndex;
	int groundRows,nongroundRows;
	csf_filtering(points,rigidness,isSmooth,resolution,class_threshold,interations,time_step,
		rows,groundIndex, nongroundIndex,groundRows,nongroundRows);
	plhs[0] = mxCreateNumericMatrix(groundRows,1, mxINT32_CLASS, mxREAL);
	plhs[1] = mxCreateNumericMatrix(nongroundRows,1, mxINT32_CLASS, mxREAL);

	int* outputgroundMatrix = (int *)mxGetData(plhs[0]);
	int* outputnongroundMatrix = (int *)mxGetData(plhs[1]); 
	for(int i=0;i<groundIndex.size();i++)
		outputgroundMatrix[i] = groundIndex[i]+1;
	for(int i=0;i<nongroundIndex.size();i++)
		outputnongroundMatrix[i] = nongroundIndex[i]+1;

    // Read in the data
    // for (int col=0; col < 3; col++) {
    //     for (int row=0; row < groundRows; row++) {
    //         outputgroundMatrix[row + col*groundRows] = outputBuff[col][row];
    //     }
    // }

}


