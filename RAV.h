#ifndef RAV_H
#define RAV_H

#include <iostream>
#include "tool.h"

using namespace std;

void outputRVdata_set(RV_DATA* _RV_data);
void outputNLVOdata_set(const string &DATA_set_type, TS_DATA &_TS_data, NLVO_DATA* _NLVO_data);
void outputNLVO_RAVdata_set(RV_DATA* _RV_data, NLVO_DATA* _NLVO_data, RAV_DATA* _RAV_data);
void outputdata_set(const string &DATA_set_type, OS_DATA &_OS_data, TS_DATA &_TS_data, RV_DATA* _RV_data, NLVO_DATA* _NLVO_data, RAV_DATA* _RAV_data);

#endif // RAV_H
