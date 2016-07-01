#include "XYZReader.h"
#include <sstream>
#include <iostream>
#include <iomanip> 
#include <fstream>
using namespace std;

void read_xyz(string fname, csf::PointCloud &pointcloud)
{
	ifstream fin(fname.c_str(), ios::in);
	char line[500];
	string x, y, z;
	while (fin.getline(line, sizeof(line)))
	{
		stringstream words(line);
		words >> x;
		words >> y;
		words >> z;
		csf::Point point;
		point.x = atof(x.c_str());
		point.y = -atof(z.c_str());
		point.z = atof(y.c_str());
		pointcloud.push_back(point);
	}

}