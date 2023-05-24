#ifndef __LOAD_PCD_H__
#define __LOAD_PCD_H__


#include "types/point3d.h"
#include "common/tt_test.h"
#include <vector>
#include <iostream>
#include <fstream>
#include <sstream>
std::vector<types::Point3D> load_pcd(const std::string& file_name);




#endif // __LOAD_PCD_H__