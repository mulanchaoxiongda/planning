#include "SaveData.h"
#include <iostream>

using namespace std;

SaveData::SaveData(const char* FullFilePath)
{
    file.open(FullFilePath,ios::out);

    file.precision(8);
    file.flags(ios::right|ios::fixed);
    file.fill('0');
}

SaveData::~SaveData()
{
    file.close();
}