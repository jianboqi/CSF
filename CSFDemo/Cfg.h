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

class Cfg
{
public:
	bool readConfigFile(const char * cfgfilepath, const std::string & key, std::string & value)
	{
		fstream cfgFile;
		cfgFile.open(cfgfilepath);//���ļ�      
		if (!cfgFile.is_open())
		{
			std::cout << "can not open cfg file!" << std::endl;
			return false;
		}
		char tmp[1000];
		while (!cfgFile.eof())//ѭ����ȡÿһ��  
		{
			cfgFile.getline(tmp, 1000);//ÿ�ж�ȡǰ1000���ַ���1000��Ӧ���㹻��  
			std::string line(tmp);
			std::size_t pos = line.find('=');//�ҵ�ÿ�еġ�=����λ�ã�֮ǰ��key֮����value  
			if (pos == std::string::npos) return false;
			string tmpKey = line.substr(0, pos);//ȡ=��֮ǰ  
			if (key == tmpKey)
			{
				value = line.substr(pos + 1);//ȡ=��֮��  
				return true;
			}
		}
		return false;
	}
};


#endif