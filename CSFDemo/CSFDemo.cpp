// ======================================================================================
// Copyright 2017 State Key Laboratory of Remote Sensing Science, 
// Institute of Remote Sensing Science and Engineering, Beijing Normal University

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
// ======================================================================================


#include <vector>
#include "Cfg.h"
#include "../src/CSF.h" 
#include <locale.h>
#include <time.h>
#include <cstdlib>
#include <cstring>
//using namespace std;

int main(int argc,char* argv[])
{
	//读取文本参数，仅用于调试
	Cfg cfg;
	string slop_smooth;
	cfg.readConfigFile("params.cfg", "slop_smooth", slop_smooth);
	bool ss = false;
	if (slop_smooth == "true" || slop_smooth == "True")
	{
		ss = true;
	}
	else if (slop_smooth == "false" || slop_smooth == "False")
	{
		ss = false;
	}
	else{
		if (atoi(slop_smooth.c_str()) == 0){
			ss = false;
		}
		else
		{
			ss = true;
		}
	}

	string class_threshold;
	cfg.readConfigFile("params.cfg", "class_threshold", class_threshold);
	string cloth_resolution;
	cfg.readConfigFile("params.cfg", "cloth_resolution", cloth_resolution);
	string iterations;
	cfg.readConfigFile("params.cfg", "iterations", iterations);
	string rigidness;
	cfg.readConfigFile("params.cfg", "rigidness", rigidness);
	string time_step;
	cfg.readConfigFile("params.cfg", "time_step", time_step);
	string terr_pointClouds_filepath;
	cfg.readConfigFile("params.cfg", "terr_pointClouds_filepath", terr_pointClouds_filepath);
	string rasterization_mode;
	cfg.readConfigFile("params.cfg", "rasterization_mode", rasterization_mode);
	string rasterization_window_size;
	cfg.readConfigFile("params.cfg", "rasterization_window_size", rasterization_window_size);
	string downsampling_window_num;
	cfg.readConfigFile("params.cfg", "downsampling_window_num", downsampling_window_num);

	//string offsetXstr;
	//string offsetYstr;
	//cfg.readConfigFile("params.cfg", "offsetX", offsetXstr);
	//cfg.readConfigFile("params.cfg", "offsetY", offsetYstr);
	//double offsetX = atof(offsetXstr.c_str());
	//double offsetY = atof(offsetYstr.c_str());


	CSF csf(0);
	//step 1 输入点云
	csf.readPointsFromFile(terr_pointClouds_filepath);

	std::cout << " - Input file: " << terr_pointClouds_filepath << endl;

	clock_t start, end;
	start = clock();

	//备注：在实际使用过程中，点云数据由主程序提供，调用函数为
	//csf.setPointCloud(pc);//pc为PointCloud类

	//step 2 设置参数
	csf.params.bSloopSmooth = ss;
	csf.params.class_threshold = atof(class_threshold.c_str());
	csf.params.cloth_resolution = atof(cloth_resolution.c_str());
	csf.params.interations = atoi(iterations.c_str());
	csf.params.rigidness = atoi(rigidness.c_str());
	csf.params.time_step = atof(time_step.c_str());
	csf.params.rasterization_mode = atoi(rasterization_mode.c_str());
	csf.params.rasterization_window_size = atof(rasterization_window_size.c_str());
	csf.params.downsampling_window_num = atoi(downsampling_window_num.c_str());

	//step3 执行滤波,result中储存的是地面点的索引 
	std::vector<int> groundIndexes, offGroundIndexes;
	if (argc == 2 && strcmp(argv[1], "-c")==0) //export Cloth
	{
		cout << " - Export cloth enabled." << endl;
		csf.do_filtering(groundIndexes, offGroundIndexes, true);
	}else
	{
		csf.do_filtering(groundIndexes, offGroundIndexes, false);
	}
		

	end = clock();
	double dur = (double)(end - start);
	printf(" - Use Time:%f\n", (dur / CLOCKS_PER_SEC));

	csf.savePoints(groundIndexes,"ground.txt");
	csf.savePoints(offGroundIndexes, "non-ground.txt");
//	system("pause");
	return 0;
}