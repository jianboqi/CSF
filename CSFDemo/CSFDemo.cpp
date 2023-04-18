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

int main(int argc,char* argv[])
{
	Cfg cfg;
	std::string slop_smooth;
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

	std::string class_threshold;
	cfg.readConfigFile("params.cfg", "class_threshold", class_threshold);
	std::string cloth_resolution;
	cfg.readConfigFile("params.cfg", "cloth_resolution", cloth_resolution);
	std::string iterations;
	cfg.readConfigFile("params.cfg", "iterations", iterations);
	std::string rigidness;
	cfg.readConfigFile("params.cfg", "rigidness", rigidness);
	std::string time_step;
	cfg.readConfigFile("params.cfg", "time_step", time_step);
	std::string terr_pointClouds_filepath;
	cfg.readConfigFile("params.cfg", "terr_pointClouds_filepath", terr_pointClouds_filepath);

	CSF csf;
	//step 1 Input the point cloud
	csf.readPointsFromFile(terr_pointClouds_filepath);

	clock_t start, end;
	start = clock();

	//In real application, the point cloud may be provided by main program
	//csf.setPointCloud(pc);//pc is PointCloud class

	//step 2 parameter setting
	csf.params.bSloopSmooth = ss;
	csf.params.class_threshold = atof(class_threshold.c_str());
	csf.params.cloth_resolution = atof(cloth_resolution.c_str());
	csf.params.interations = atoi(iterations.c_str());
	csf.params.rigidness = atoi(rigidness.c_str());
	csf.params.time_step = atof(time_step.c_str());

	//step3 do filtering
	std::vector<int> groundIndexes, offGroundIndexes;
	if (argc == 2 && strcmp(argv[1], "-c")==0)
	{
		std::cout << "Export cloth enabled." << std::endl;
		csf.do_filtering(groundIndexes, offGroundIndexes, true);
	}
	else
	{
		csf.do_filtering(groundIndexes, offGroundIndexes, false);
	}
		

	end = clock();
	double dur = (double)(end - start);
	printf("Use Time:%f\n", (dur / CLOCKS_PER_SEC));

	csf.savePoints(groundIndexes,"ground.txt");
	csf.savePoints(offGroundIndexes, "non-ground.txt");

	return 0;
}