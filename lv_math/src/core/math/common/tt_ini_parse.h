#ifndef __INI_PARSE_H__
#define __INI_PARSE_H__

#include<iostream>
#include<fstream>
#include<sstream>
#include<vector>
#include<map>
#include<cstdlib>
#include<Eigen/Core>

namespace common{

struct IniNode
{
	IniNode(std::string _root, std::string _key, std::string _value)
	:root(_root), key(_key), value(_value)
	{}

	std::string root;
	std::string key; 
	std::string value;
};

struct SubNode
{
	void InsertElement(std::string key, std::string value)
	{
		sub_node.insert(std::pair<std::string, std::string>(key, value));
	}
	std::map<std::string, std::string> sub_node;
};

class IniParse
{
public:
	bool ReadIni(std::string path);
	
	std::string GetValue(std::string root, std::string key);

	template<class T>
	void GetValue(std::string root, std::string key, T& val)
	{
		std::string str = GetValue(root, key);
		if(str != m_non_value_return){
			std::istringstream iss(str);
			iss >> val;
		}
	}

	template<class T>
	void GetValue(std::string root, std::string key, T* val)
	{
		std::string str = GetValue(root, key);
		if(str != m_non_value_return){
			std::istringstream iss(str);
			iss >> (*val);
		}
	}

	void Clear();

	void StringToMatrix(std::string& str, Eigen::Matrix3d& mat);

	void StringToVector(std::string& str, Eigen::Vector3d& vec);
private:
	std::string& TrimString(std::string& str);
private:
	std::map<std::string, SubNode> m_map_ini;

	const std::string m_non_value_return = "NONE_VALUE_IN_CONFIG_FILE";
};


}

#endif // __INI_PARSE_H__