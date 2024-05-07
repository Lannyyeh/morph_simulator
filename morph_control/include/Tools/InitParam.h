#pragma once

#include <iostream>
#include <fstream>

std::map<std::string, std::string> ini;

void iniLoad(std::string iniFileName) {
	std::ifstream in(iniFileName);
	if(!in.is_open()) {
		std::cout<<"[iniLoad] "<<iniFileName<<" not found, use default parameters!"<<std::endl;
		return;
	}
	while(in) {
		std::string line;
		std::getline(in, line);
		if(line.empty() || line[0]=='#') continue;
		std::string key, value;
		size_t eqPos = line.find_first_of("=");
		if(eqPos == std::string::npos || eqPos == 0) {
			std::cout<<"[iniLoad] ignore line:"<<line<<std::endl;
			continue;
		}
		key = line.substr(0,eqPos);
		value = line.substr(eqPos+1);
		//std::cout<<"[iniLoad] "<<key<<"=>"<<value<<std::endl;
		ini[key]=value;
	}
}

template<class T>
T iniGet(std::string key, T default_value) {
	std::map<std::string, std::string>::const_iterator itr=ini.find(key);
	if(itr!=ini.end()) {
		std::stringstream ss;
		ss<<itr->second;
		T ret;
		ss>>ret;
		return ret;
	}
	return default_value;
}

template<> std::string iniGet(std::string key, std::string default_value) {
	std::map<std::string, std::string>::const_iterator itr=ini.find(key);
	if(itr!=ini.end()) {
		return itr->second;
	}
	return default_value;
}

