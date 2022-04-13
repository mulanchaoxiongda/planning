#pragma once

#include <fstream>
#include <iostream>
#include <string>

#include "RobotModel.h"

using namespace std;

class SaveData
{
 public:
     SaveData(const char* FullFilePath);
     ~SaveData();

     ofstream file;
};