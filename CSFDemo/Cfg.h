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

#ifndef _CFG_H_
#define _CFG_H_
#include <iostream>  
#include <string>  
#include <fstream> 
using namespace std;

class Cfg
{
public:
	bool readConfigFile(const char * cfgfilepath, const string & key, string & value)
	{
		fstream cfgFile;
		cfgFile.open(cfgfilepath);//打开文件      
		if (!cfgFile.is_open())
		{
			cout << "can not open cfg file!" << endl;
			return false;
		}
		char tmp[1000];
		while (!cfgFile.eof())//循环读取每一行  
		{
			cfgFile.getline(tmp, 1000);//每行读取前1000个字符，1000个应该足够了  
			string line(tmp);
			size_t pos = line.find('=');//找到每行的“=”号位置，之前是key之后是value  
			if (pos == string::npos) return false;
			string tmpKey = line.substr(0, pos);//取=号之前  
			if (key == tmpKey)
			{
				value = line.substr(pos + 1);//取=号之后  
				return true;
			}
		}
		return false;
	}
};


#endif