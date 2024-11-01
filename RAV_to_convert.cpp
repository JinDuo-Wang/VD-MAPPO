#include <cstdio>
#include <cstring>
#include <cmath>

#define PI (3.1415926f)
#define T_MAX 10

using namespace std;

///RV集数据
typedef struct _RV_data
{
    //计算RV集所需数据
    double vmax[T_MAX+1]; //最大平均速度
    double vmin[T_MAX+1]; //最小平均速度
    double fimax[T_MAX+1]; //最大转向范围
    double k_OD[T_MAX+1]; //直线OD斜率
    double k_OC[T_MAX+1];//直线OC斜率
    double m[T_MAX+1];//向量OD横坐标
    double n[T_MAX+1];//向量OD纵坐标
    double o[T_MAX+1];//向量OC横坐标
    double p[T_MAX+1];//向量OD纵坐标
    //RV集数据
    int RV_set[1000][2]; //RV集,向量形式
    double RV_v_value_set[1000];//RV集，标量
    double RV_heading_set[1000];//RV集标量对应的偏航角
    int RV_set_num;//RV集中数据的总数量
    int RV_time_index[1000];//RV集中每个数据所对应的时间
    int RV_num_per_sec[T_MAX+1];//RV集中每秒所对应的数据的数量
}RV_DATA;

///NLVO集数据（包括NLVO、NLVOa、NLVOua、NLVOaua）
typedef struct _NLVO_data
{
    //计算NLVO集所需数据
    double relative_position[2];//两艇相对位置
    double k_PB[T_MAX+1];//直线PB斜率
    double k_PA[T_MAX+1];//直线PA斜率
    double c[T_MAX+1];//向量PB横坐标
    double d[T_MAX+1];//向量PB纵坐标
    double e[T_MAX+1];//向量PA横坐标
    double f[T_MAX+1];//向量PB纵坐标
    //NLVO集数据
    int NLVOdata_set[1000000][2];//NLVO集
    int NLVO_num_per_sec[T_MAX+1];//NLVO集中每秒所对应的数据的数量
}NLVO_DATA;

///RAV集数据（包括RAV、RAVa、RAVua、RAVaua）
typedef struct _RAV_data
{
    //计算某一时刻的NLVO集中数据是否在相应时刻的RV集内，若在，则放入RAV集
    //RAV数据集
    int RAV_set[1000][2];//RAV集
    int RAV_set_num;//RAV集中数据的总数量
    int RAV_time_index[1000];//RAV集中每个数据所对应的时间
    int RAV_num_per_sec[T_MAX+1];//RAV集中每秒所对应的数据的数量
    double RAV_v_value_set[1000];//RAV集，标量
    double RAV_heading_set[1000];//RAV集标量对应的偏航角
}RAV_DATA;

///我方艇数据
typedef struct _OS_data
{
    double dLongitude;//OS经度
    double dLatitude;//OS纬度
    double v_vector[2]; //OS速度向量
}OS_DATA;

///敌方艇数据
typedef struct _TS_data
{
    double dLongitude;//TS经度
    double dLatitude;//TS纬度
    double v_vector[T_MAX+1][2]; //TS速度向量，包含当前速度向量和未来10s内的预测速度向量
}TS_DATA;

extern "C" double TransMP(double Latitude);
extern "C" void GetDistAngleFromGpsByHT(double dStartPosLon, double dStartPosLat, double dDestPosLon, double dDestPosLat, float* pfAngle, float* pfDist);
extern "C" void outputRVdata_set(RV_DATA* _RV_data);
extern "C" void outputNLVOdata_set(char *DATA_set_type, TS_DATA &_TS_data, NLVO_DATA* _NLVO_data);
extern "C" void outputNLVO_RAVdata_set(RV_DATA* _RV_data, NLVO_DATA* _NLVO_data, RAV_DATA* _RAV_data);
extern "C" void outputdata_set(char *DATA_set_type, OS_DATA &_OS_data, TS_DATA &_TS_data, RV_DATA* _RV_data, NLVO_DATA* _NLVO_data, RAV_DATA* _RAV_data);

double t0 = 0;//初始时刻
double rsafe = 50;//安全半径
double rattack = 150;//攻击半径
double half_angleattack = PI/4;//攻击角度的一半
double acceleration = 0.6;//艇的最大加速度
double angular_acceleration = PI/30;//艇的最大角加速度
double USV_vmax = 16.9767;//艇的最大速度
static double earthE = 0.0818191;

double TransMP(double Latitude) {
    //求渐长率 从经度到渐长率的转换 要计算
    double LatArc;
    LatArc = fabs(Latitude * PI / 180);
    double Rate = pow(((1 - earthE * sin(LatArc)) / (1 + earthE * sin(LatArc))), earthE / 2);
    double MP = 7915.70447 * log10((tan(PI / 4 + LatArc / 2)) * Rate);
    if (Latitude < 0) MP = (-MP);
    return (MP);
}

void GetDistAngleFromGpsByHT(double dStartPosLon, double dStartPosLat, double dDestPosLon, double dDestPosLat,
                             float *pfAngle, float *pfDist) {
    double aLon1 = dStartPosLon / 180 * PI;
    double aLat1 = dStartPosLat / 180 * PI;
    double aLon2 = dDestPosLon / 180 * PI;
    double aLat2 = dDestPosLat / 180 * PI;

    double dlon = aLon2 - aLon1;
    double dlat = aLat2 - aLat1;

    double a = pow(sin(dlat / 2), 2) + cos(aLat1) * cos(aLat2) * pow(sin(dlon / 2), 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));

    float earthRadiusX = 6378137.0;
    double d = earthRadiusX / 1852 * c;
    *pfDist = d*1852;


    double Dmp;
    double DLon;
    double DLonMin;
    double angleArc;
    double angle;

    /*double deltY = ConvertLatiudeToPlaneY(Lat2) - ConvertLatiudeToPlaneY(Lat1);
    double deltX = ConvertLongitudeToPlaneX(Lon2) - ConvertLongitudeToPlaneX(Lon1);

    if (deltX == 0)
    {
        if (deltY >= 0)
        {
            return 0;
        }
        else
        {
        }
    }

    return atan(deltY / deltX) * 180 / PI + 90; */

    Dmp = TransMP(dDestPosLat) - TransMP(dStartPosLat);

    //Dmp *= 0.00037485564396751882570767595417152; //added by txh
    DLon = dDestPosLon - dStartPosLon;
    if (DLon > 180) DLon = DLon - 360;
    if (DLon < (-180)) DLon = DLon + 360;
    DLonMin = DLon * 60;
    if (Dmp != 0) {
        angleArc = atan(fabs(DLonMin / Dmp));
        angle = angleArc * 180 / PI;
    }
    if (Dmp < 0 && DLonMin <= 0) angle = angle + 180;
    if (Dmp > 0 && DLonMin < 0) angle = 360 - angle;
    if (Dmp < 0 && DLonMin > 0) angle = 180 - angle;
    if (Dmp == 0 && DLonMin >= 0) angle = 90;
    if (Dmp == 0 && DLonMin < 0) angle = 270;
    *pfAngle = angle;

}

void outputRVdata_set(RV_DATA* _RV_data)
{
    int i = 0;
    int case_option = 0;
    int x, y;
    for (int t1 = 1; t1 <= T_MAX; t1++)
    {
        int num = 0;
        int r = (int)(_RV_data->vmax[t1]);
        for(int c = -r; c <= r; c++)
        {
            for (int d = -r; d <= r; d++){
                x = c;
                y = d;
                if(x * x + y * y <= _RV_data->vmax[t1]*_RV_data->vmax[t1] && x * x + y * y >= _RV_data->vmin[t1]*_RV_data->vmin[t1]){
                    if(_RV_data->m[t1] != 0 && _RV_data->o[t1] != 0)
                        case_option = 0;
                    else if(_RV_data->m[t1] == 0 && _RV_data->o[t1] != 0)
                        case_option = 1;
                    else if(_RV_data->m[t1] != 0 && _RV_data->o[t1] == 0)
                        case_option = 2;
                    switch(case_option){
                    case 0:{
                        if(_RV_data->m[t1] > 0 && _RV_data->o[t1] > 0)
                        {
                            if (y >= _RV_data->k_OD[t1] * x &&
                                    y <= _RV_data->k_OC[t1] * x)
                            {
                                _RV_data->RV_set[i][0] = x;
                                _RV_data->RV_set[i][1] = y;
                                _RV_data->RV_time_index[i] = t1;
//                                cout<<"("<<_RV_data->RV_set[i][0]<<","<<_RV_data->RV_set[i][1]<<")"<<endl;
                                i++;
                                num++;
                            }
                        }
                        else if(_RV_data->m[t1] > 0 && _RV_data->o[t1] < 0)
                        {
                            if (y >= _RV_data->k_OD[t1] * x &&
                                    y >= _RV_data->k_OC[t1] * x)
                            {
                                _RV_data->RV_set[i][0] = x;
                                _RV_data->RV_set[i][1] = y;
                                _RV_data->RV_time_index[i] = t1;
//                                cout<<"("<<_RV_data->RV_set[i][0]<<","<<_RV_data->RV_set[i][1]<<")"<<endl;
                                i++;
                                num++;
                            }
                        }
                        else if(_RV_data->m[t1] < 0 && _RV_data->o[t1] > 0)
                        {
                            if (y <= _RV_data->k_OD[t1] * x &&
                                    y <= _RV_data->k_OC[t1] * x)
                            {
                                _RV_data->RV_set[i][0] = x;
                                _RV_data->RV_set[i][1] = y;
                                _RV_data->RV_time_index[i] = t1;
//                                cout<<"("<<_RV_data->RV_set[i][0]<<","<<_RV_data->RV_set[i][1]<<")"<<endl;
                                i++;
                                num++;
                            }
                        }
                        else if(_RV_data->m[t1] < 0 && _RV_data->o[t1] < 0)
                        {
                            if (y <= _RV_data->k_OD[t1] * x &&
                                    y >= _RV_data->k_OC[t1] * x)
                            {
                                _RV_data->RV_set[i][0] = x;
                                _RV_data->RV_set[i][1] = y;
                                _RV_data->RV_time_index[i] = t1;
//                                cout<<"("<<_RV_data->RV_set[i][0]<<","<<_RV_data->RV_set[i][1]<<")"<<endl;
                                i++;
                                num++;
                            }
                        }
                    }
                        break;
                    case 1:{
                        if (_RV_data->o[t1] > 0)
                        {
                            if (y <= _RV_data->k_OC[t1] * x &&
                                    x >=0)
                            {
                                _RV_data->RV_set[i][0] = x;
                                _RV_data->RV_set[i][1] = y;
                                _RV_data->RV_time_index[i] = t1;
//                                cout<<"("<<_RV_data->RV_set[i][0]<<","<<_RV_data->RV_set[i][1]<<")"<<endl;
                                i++;
                                num++;
                            }
                        }
                        else
                        {
                            if (y >= _RV_data->k_OC[t1] * x &&
                                    x <=0)
                            {
                                _RV_data->RV_set[i][0] = x;
                                _RV_data->RV_set[i][1] = y;
                                _RV_data->RV_time_index[i] = t1;
//                                cout<<"("<<_RV_data->RV_set[i][0]<<","<<_RV_data->RV_set[i][1]<<")"<<endl;
                                i++;
                                num++;
                            }
                        }
                    }
                        break;
                    case 2:{
                        if(_RV_data->m[t1] > 0)
                        {
                            if (x >= 0 &&
                                    y >= _RV_data->k_OD[t1] * x)
                            {
                                _RV_data->RV_set[i][0] = x;
                                _RV_data->RV_set[i][1] = y;
                                _RV_data->RV_time_index[i] = t1;
//                                cout<<"("<<_RV_data->RV_set[i][0]<<","<<_RV_data->RV_set[i][1]<<")"<<endl;
                                i++;
                                num++;
                            }
                        }
                        else
                        {
                            if (x <= 0 &&
                                    y <= _RV_data->k_OD[t1] * x)
                            {
                                _RV_data->RV_set[i][0] = x;
                                _RV_data->RV_set[i][1] = y;
                                _RV_data->RV_time_index[i] = t1;
//                                cout<<"("<<_RV_data->RV_set[i][0]<<","<<_RV_data->RV_set[i][1]<<")"<<endl;
                                i++;
                                num++;
                            }
                        }
                    }
                        break;
                    default:
                        break;
                    }
                }
            }
        }
        _RV_data->RV_num_per_sec[t1] = num;
    }
    _RV_data->RV_set_num = i;
    for(int i = 0; i<_RV_data->RV_set_num; i++)
    {
        _RV_data->RV_v_value_set[i] = sqrt(_RV_data->RV_set[i][0]*_RV_data->RV_set[i][0]+_RV_data->RV_set[i][1]*_RV_data->RV_set[i][1]);
        _RV_data->RV_heading_set[i] = atan2(_RV_data->RV_set[i][0], _RV_data->RV_set[i][1]);
    }
}


void outputNLVOdata_set(char *DATA_set_type, TS_DATA &_TS_data, NLVO_DATA* _NLVO_data)
{
    int i = 0;
    int case_option = 0;
    int t1 = 0;
    double a, b;
    int position_x, position_y;
    if(strcmp(DATA_set_type, "RAV") == 0){
        for (t1 = 1; t1 <= T_MAX; t1++)
        {
            double pmove[2];
            double a = pmove[0] = _NLVO_data->relative_position[0]/(t1 - t0) + _TS_data.v_vector[t1][0];
            double b = pmove[1] = _NLVO_data->relative_position[1]/(t1 - t0) + _TS_data.v_vector[t1][1];
            int r = (int)(rattack / (t1 - t0));
            int num = 0;
            for(int m = -r; m <= r; m++)
            {
                if(m + a < 0 && m + a > -1)
                    continue;
                for (int n = -r; n <= r; n++){
                    if(n + b < 0 && n + b > -1)
                        continue;
                    position_x = (int)(m + a);
                    position_y = (int)(n + b);
                    if ((position_x - a)*(position_x - a)+(position_y - b)*(position_y - b) <= (rattack/(t1-t0))*(rattack/(t1-t0)))
                    {
                        _NLVO_data->NLVOdata_set[i][0] = position_x;
                        _NLVO_data->NLVOdata_set[i][1] = position_y;
                        //cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                        i++;
                        num++;
                    }
                }
            }
            _NLVO_data->NLVO_num_per_sec[t1] = num;
        }
    }
    else if(strcmp(DATA_set_type, "RAVa") == 0){
        for (t1 = 1; t1 <= T_MAX; t1++)
        {

            double pmove[2];
            a = pmove[0] = _NLVO_data->relative_position[0]/(t1 - t0) + _TS_data.v_vector[t1][0];
            b = pmove[1] = _NLVO_data->relative_position[1]/(t1 - t0) + _TS_data.v_vector[t1][1];
            int r = (int)(rattack / (t1 - t0));
            int num = 0;
            for(int m = -r; m <= r; m++)
            {
                if(m + a < 0 && m + a > -1)
                    continue;
                for (int n = -r; n <= r; n++){
                    if(n + b < 0 && n + b > -1)
                        continue;
                    position_x = (int)(m + a);
                    position_y = (int)(n + b);
                    if ((position_x - a)*(position_x - a)+(position_y - b)*(position_y - b) <= (rattack/(t1-t0))*(rattack/(t1-t0)))
                    {
                        if(_NLVO_data->c[t1] != 0 && _NLVO_data->e[t1] != 0)
                            case_option = 0;
                        else if(_NLVO_data->c[t1] == 0 && _NLVO_data->e[t1] != 0)
                            case_option = 1;
                        else if(_NLVO_data->c[t1] != 0 && _NLVO_data->e[t1] == 0)
                            case_option = 2;
                        switch(case_option){
                        case 0:{
                            if(_NLVO_data->c[t1] > 0 && _NLVO_data->e[t1] > 0)
                            {
                                if (position_y >= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                        position_y <= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a)
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                            else if(_NLVO_data->c[t1] > 0 && _NLVO_data->e[t1] < 0)
                            {
                                if (position_y >= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                        position_y >= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a)
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                            else if(_NLVO_data->c[t1] < 0 && _NLVO_data->e[t1] > 0)
                            {
                                if (position_y <= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                        position_y <= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a)
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                            else if(_NLVO_data->c[t1] < 0 && _NLVO_data->e[t1] < 0)
                            {
                                if (position_y <= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                        position_y >= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a)
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                        }
                            break;
                        case 1:{
                            if (_NLVO_data->e[t1] > 0)
                            {
                                if (position_x >= a &&
                                        position_y <= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a )
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                            else
                            {
                                if (position_x <= a &&
                                        position_y >= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a)
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                        }
                            break;
                        case 2:{
                            if(_NLVO_data->c[t1] > 0)
                            {
                                if (position_y >= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                        position_x >= a)
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                            else
                            {
                                if (position_y <= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                        position_x <= a)
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                        }
                            break;
                        default:
                            break;
                        }
                    }
                }
            }
            _NLVO_data->NLVO_num_per_sec[t1] = num;
        }
    }
    else if(strcmp(DATA_set_type, "RAVua") == 0){
        for (t1 = 1; t1 <= T_MAX; t1++)
        {
            double pmove[2];
            a = pmove[0] = _NLVO_data->relative_position[0]/(t1 - t0) + _TS_data.v_vector[t1][0];
            b = pmove[1] = _NLVO_data->relative_position[1]/(t1 - t0) + _TS_data.v_vector[t1][1];
            int r = (int)(rattack / (t1 - t0));
            int num = 0;
            for(int m = -r; m <= r; m++)
            {
                if(m + a < 0 && m + a > -1)
                    continue;
                for (int n = -r; n <= r; n++){
                    if(n + b < 0 && n + b > -1)
                        continue;
                    position_x = (int)(m + a);
                    position_y = (int)(n + b);
                    if ((position_x - a)*(position_x - a)+(position_y - b)*(position_y - b) <= (rattack/(t1-t0))*(rattack/(t1-t0)))
                    {
                        if(_NLVO_data->c[t1] != 0 && _NLVO_data->e[t1] != 0)
                            case_option = 0;
                        else if(_NLVO_data->c[t1] == 0 && _NLVO_data->e[t1] != 0)
                            case_option = 1;
                        else if(_NLVO_data->c[t1] != 0 && _NLVO_data->e[t1] == 0)
                            case_option = 2;
                        switch(case_option){
                        case 0:{
                            if(_NLVO_data->c[t1] > 0 && _NLVO_data->e[t1] > 0)
                            {
                                if (! (position_y >= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                       position_y <= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a))
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                            else if(_NLVO_data->c[t1] > 0 && _NLVO_data->e[t1] < 0)
                            {
                                if (! (position_y >= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                       position_y >= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a))
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                            else if(_NLVO_data->c[t1] < 0 && _NLVO_data->e[t1] > 0)
                            {
                                if (! (position_y <= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                       position_y <= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a))
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                            else if(_NLVO_data->c[t1] < 0 && _NLVO_data->e[t1] < 0)
                            {
                                if (! (position_y <= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                       position_y >= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a))
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                        }
                            break;
                        case 1:{
                            if (_NLVO_data->e[t1] > 0)
                            {
                                if (! (position_x >= a &&
                                       position_y <= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a ))
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                            else
                            {
                                if (! (position_x <= a &&
                                       position_y >= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a))
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                        }
                            break;
                        case 2:{
                            if(_NLVO_data->c[t1] > 0)
                            {
                                if (! (position_y >= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                       position_x >= a))
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                            else
                            {
                                if (! (position_y <= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                       position_x <= a))
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                        }
                            break;
                        default:
                            break;
                        }
                    }
                }
            }
            _NLVO_data->NLVO_num_per_sec[t1] = num;
        }
    }
    else if(strcmp(DATA_set_type, "RAVaua") == 0){
        for (t1 = 1; t1 <= T_MAX; t1++)
        {
            double pmove[2];
            a = pmove[0] = _NLVO_data->relative_position[0]/(t1 - t0) + _TS_data.v_vector[t1][0];
            b = pmove[1] = _NLVO_data->relative_position[1]/(t1 - t0) + _TS_data.v_vector[t1][1];
            int r = (int)(rattack / (t1 - t0));
            int num = 0;
            for(int m = -r; m <= r; m++)
            {
                if(m + a < 0 && m + a > -1)
                    continue;
                for (int n = -r; n <= r; n++){
                    if(n + b < 0 && n + b > -1)
                        continue;
                    position_x = (int)(m + a);
                    position_y = (int)(n + b);
                    if ((position_x - a)*(position_x - a)+(position_y - b)*(position_y - b) >= (2*rsafe/(t1-t0))*(2*rsafe/(t1-t0)) &&
                            (position_x - a)*(position_x - a)+(position_y - b)*(position_y - b) <= (rattack/(t1-t0))*(rattack/(t1-t0)))
                    {
                        if(_NLVO_data->c[t1] != 0 && _NLVO_data->e[t1] != 0)
                            case_option = 0;
                        else if(_NLVO_data->c[t1] == 0 && _NLVO_data->e[t1] != 0)
                            case_option = 1;
                        else if(_NLVO_data->c[t1] != 0 && _NLVO_data->e[t1] == 0)
                            case_option = 2;
                        switch(case_option){
                        case 0:{
                            if(_NLVO_data->c[t1] > 0 && _NLVO_data->e[t1] > 0)
                            {
                                if (! (position_y >= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                       position_y <= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a))
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                            else if(_NLVO_data->c[t1] > 0 && _NLVO_data->e[t1] < 0)
                            {
                                if (! (position_y >= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                       position_y >= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a))
                                {

                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
                                    //                            cout << 111 << endl;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                            else if(_NLVO_data->c[t1] < 0 && _NLVO_data->e[t1] > 0)
                            {
                                if (! (position_y <= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                       position_y <= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a))
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                            else if(_NLVO_data->c[t1] < 0 && _NLVO_data->e[t1] < 0)
                            {
                                if (! (position_y <= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                       position_y >= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a))
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                        }
                            break;
                        case 1:{
                            if (_NLVO_data->e[t1] > 0)
                            {
                                if (! (position_x >= a &&
                                       position_y <= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a ))
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                            else
                            {
                                if (! (position_x <= a &&
                                       position_y >= _NLVO_data->k_PA[t1] * position_x + b - _NLVO_data->k_PA[t1] * a))
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                        }
                            break;
                        case 2:{
                            if(_NLVO_data->c[t1] > 0)
                            {
                                if (! (position_y >= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                       position_x >= a))
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
//                                    cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                            else
                            {
                                if (! (position_y <= _NLVO_data->k_PB[t1] * position_x + b - _NLVO_data->k_PB[t1] * a &&
                                       position_x <= a))
                                {
                                    _NLVO_data->NLVOdata_set[i][0] = position_x;
                                    _NLVO_data->NLVOdata_set[i][1] = position_y;
                                    //cout<<"("<<_NLVO_data->NLVOdata_set[i][0]<<","<<_NLVO_data->NLVOdata_set[i][1]<<")"<<endl;
                                    i++;
                                    num++;
                                }
                            }
                        }
                            break;
                        default:
                            break;
                        }
                    }
                }
            }
            _NLVO_data->NLVO_num_per_sec[t1] = num;
        }
    }
}

void outputNLVO_RAVdata_set(RV_DATA* _RV_data, NLVO_DATA* _NLVO_data, RAV_DATA* _RAV_data)
{
    int i = 0;
    int sum = 0;
    int case_option = 0;
    int x, y;
    for (int t1 = 1; t1 <= T_MAX; t1++)
    {
        int num = 0;
        sum += _NLVO_data->NLVO_num_per_sec[t1];
        for (int k = sum - _NLVO_data->NLVO_num_per_sec[t1]; k < sum; k++)
        {
            x = _NLVO_data->NLVOdata_set[k][0];
            y = _NLVO_data->NLVOdata_set[k][1];
            if(x * x + y * y <= _RV_data->vmax[t1]*_RV_data->vmax[t1] && x * x + y * y >= _RV_data->vmin[t1]*_RV_data->vmin[t1]){
                if(_RV_data->m[t1] != 0 && _RV_data->o[t1] != 0)
                    case_option = 0;
                else if(_RV_data->m[t1] == 0 && _RV_data->o[t1] != 0)
                    case_option = 1;
                else if(_RV_data->m[t1] != 0 && _RV_data->o[t1] == 0)
                    case_option = 2;
                switch(case_option){
                case 0:{
                    if(_RV_data->m[t1] > 0 && _RV_data->o[t1] > 0)
                    {
                        if (y >= _RV_data->k_OD[t1] * x &&
                                y <= _RV_data->k_OC[t1] * x)
                        {
                            _RAV_data->RAV_set[i][0] = _NLVO_data->NLVOdata_set[k][0];
                            _RAV_data->RAV_set[i][1] = _NLVO_data->NLVOdata_set[k][1];
                            _RAV_data->RAV_time_index[i] =t1;
//                            cout<<"("<<_RAV_data->RAV_set[i][0]<<","<<_RAV_data->RAV_set[i][1]<<")"<<endl;
                            i++;
                            num++;
                        }
                    }
                    else if(_RV_data->m[t1] > 0 && _RV_data->o[t1] < 0)
                    {
                        if (y >= _RV_data->k_OD[t1] * x &&
                                y >= _RV_data->k_OC[t1] * x)
                        {
                            _RAV_data->RAV_set[i][0] = _NLVO_data->NLVOdata_set[k][0];
                            _RAV_data->RAV_set[i][1] = _NLVO_data->NLVOdata_set[k][1];
                            _RAV_data->RAV_time_index[i] = t1;
//                            cout<<"("<<_RAV_data->RAV_set[i][0]<<","<<_RAV_data->RAV_set[i][1]<<")"<<endl;
                            i++;
                            num++;
                        }
                    }
                    else if(_RV_data->m[t1] < 0 && _RV_data->o[t1] > 0)
                    {
                        if (y <= _RV_data->k_OD[t1] * x &&
                                y <= _RV_data->k_OC[t1] * x)
                        {
                            _RAV_data->RAV_set[i][0] = _NLVO_data->NLVOdata_set[k][0];
                            _RAV_data->RAV_set[i][1] = _NLVO_data->NLVOdata_set[k][1];
                            _RAV_data->RAV_time_index[i] = t1;
//                            cout<<"("<<_RAV_data->RAV_set[i][0]<<","<<_RAV_data->RAV_set[i][1]<<")"<<endl;
                            i++;
                            num++;
                        }
                    }
                    else if(_RV_data->m[t1] < 0 && _RV_data->o[t1] < 0)
                    {
                        if (y <= _RV_data->k_OD[t1] * x &&
                                y >= _RV_data->k_OC[t1] * x)
                        {
                            _RAV_data->RAV_set[i][0] = _NLVO_data->NLVOdata_set[k][0];
                            _RAV_data->RAV_set[i][1] = _NLVO_data->NLVOdata_set[k][1];
                            _RAV_data->RAV_time_index[i] = t1;
//                            cout<<"("<<_RAV_data->RAV_set[i][0]<<","<<_RAV_data->RAV_set[i][1]<<")"<<endl;
                            i++;
                            num++;
                        }
                    }
                }
                    break;
                case 1:{
                    if (_RV_data->o[t1] > 0)
                    {
                        if (y <= _RV_data->k_OC[t1] * x &&
                                x >=0)
                        {
                            _RAV_data->RAV_set[i][0] = _NLVO_data->NLVOdata_set[k][0];
                            _RAV_data->RAV_set[i][1] = _NLVO_data->NLVOdata_set[k][1];
                            _RAV_data->RAV_time_index[i] = t1;
//                            cout<<"("<<_RAV_data->RAV_set[i][0]<<","<<_RAV_data->RAV_set[i][1]<<")"<<endl;
                            i++;
                            num++;
                        }
                    }
                    else
                    {
                        if (y >= _RV_data->k_OC[t1] * x &&
                                x <=0)
                        {
                            _RAV_data->RAV_set[i][0] = _NLVO_data->NLVOdata_set[k][0];
                            _RAV_data->RAV_set[i][1] = _NLVO_data->NLVOdata_set[k][1];
                            _RAV_data->RAV_time_index[i] = t1;
//                            cout<<"("<<_RAV_data->RAV_set[i][0]<<","<<_RAV_data->RAV_set[i][1]<<")"<<endl;
                            i++;
                            num++;
                        }
                    }
                }
                    break;
                case 2:{
                    if(_RV_data->m[t1] > 0)
                    {
                        if (x >= 0 &&
                                y >= _RV_data->k_OD[t1] * x)
                        {
                            _RAV_data->RAV_set[i][0] = _NLVO_data->NLVOdata_set[k][0];
                            _RAV_data->RAV_set[i][1] = _NLVO_data->NLVOdata_set[k][1];
                            _RAV_data->RAV_time_index[i] = t1;
//                            cout<<"("<<_RAV_data->RAV_set[i][0]<<","<<_RAV_data->RAV_set[i][1]<<")"<<endl;
                            i++;
                            num++;
                        }
                    }
                    else
                        if (x <= 0 &&
                                y <= _RV_data->k_OD[t1] * x)
                        {
                            _RAV_data->RAV_set[i][0] = _NLVO_data->NLVOdata_set[k][0];
                            _RAV_data->RAV_set[i][1] = _NLVO_data->NLVOdata_set[k][1];
                            _RAV_data->RAV_time_index[i] = t1;
//                            cout<<"("<<_RAV_data->RAV_set[i][0]<<","<<_RAV_data->RAV_set[i][1]<<")"<<endl;
                            i++;
                            num++;
                        }
                }
                    break;
                default:
                    break;
                }
            }
        }
        _RAV_data->RAV_num_per_sec[t1] = num;
    }
    _RAV_data->RAV_set_num = i;
    for(int i = 0; i < _RAV_data->RAV_set_num; i++)
    {
        _RAV_data->RAV_v_value_set[i] = sqrt(_RAV_data->RAV_set[i][0]*_RAV_data->RAV_set[i][0]+_RAV_data->RAV_set[i][1]*_RAV_data->RAV_set[i][1]);
        _RAV_data->RAV_heading_set[i] = atan2(_RAV_data->RAV_set[i][0], _RAV_data->RAV_set[i][1]);
    }
}

void outputdata_set(char *DATA_set_type, OS_DATA &_OS_data, TS_DATA &_TS_data, RV_DATA* _RV_data, NLVO_DATA* _NLVO_data, RAV_DATA* _RAV_data)
{
    //OS的最大可达平均偏转角
    for (int t = 0; t <= T_MAX; t++)
    {
        _RV_data->fimax[t] = angular_acceleration * t/2;
    }
    //OS的速度值
    double v_value = sqrt(_OS_data.v_vector[0]*_OS_data.v_vector[0]+_OS_data.v_vector[1]*_OS_data.v_vector[1]);
    for (int t = 0; t <= T_MAX; t++)
    {
        //OS的最大平均速度
        _RV_data->vmax[t] = v_value + acceleration * t / 2;
        if(v_value + acceleration * t >= USV_vmax)
        {
            _RV_data->vmax[t] = USV_vmax - (USV_vmax-v_value) * (USV_vmax-v_value) / (2 * acceleration * t);
        }
        //OS的最小平均速度
        _RV_data->vmin[t] = v_value - acceleration * t / 2;
        if(_RV_data->vmin[t] <= 0)
            _RV_data->vmin[t] = 0;
        if(_RV_data->vmin[t] > 0 && v_value + acceleration * t < 0)
            _RV_data->vmin[t] = v_value * v_value / (2 * acceleration * t);
    }
    float pfAngle, pfDist;
    GetDistAngleFromGpsByHT(_OS_data.dLongitude, _OS_data.dLatitude, _TS_data.dLongitude, _TS_data.dLatitude,
                            &pfAngle, &pfDist);
    pfAngle = pfAngle*PI/180;
    //TS相对OS的位置
    _NLVO_data->relative_position[0] = sin(pfAngle) * pfDist;
    _NLVO_data->relative_position[1] = cos(pfAngle) * pfDist;
    //cout<< _NLVO_data->relative_position[0] << ' ' << _NLVO_data->relative_position[1] << endl;
    for (int i = 0; i <= T_MAX; i++){
        _NLVO_data->c[i] = _TS_data.v_vector[i][0] * cos(half_angleattack) + _TS_data.v_vector[i][1] * sin(half_angleattack);
        _NLVO_data->d[i] = _TS_data.v_vector[i][0] * (-sin(half_angleattack)) + _TS_data.v_vector[i][1] * cos(half_angleattack);
        if(_NLVO_data->c[i] != 0)
            _NLVO_data->k_PB[i] = _NLVO_data->d[i]/_NLVO_data->c[i];
        _NLVO_data->e[i] = _TS_data.v_vector[i][0] * cos(half_angleattack) - _TS_data.v_vector[i][1] * sin(half_angleattack);
        _NLVO_data->f[i] = _TS_data.v_vector[i][0] * sin(half_angleattack) + _TS_data.v_vector[i][1] * cos(half_angleattack);
        if(_NLVO_data->e[i] != 0)
            _NLVO_data->k_PA[i] = _NLVO_data->f[i]/_NLVO_data->e[i];
    }
    for (int i = 0; i <= T_MAX; i++){
        _RV_data->m[i] = _OS_data.v_vector[0] * cos(_RV_data->fimax[i]) + _OS_data.v_vector[1] * sin(_RV_data->fimax[i]);
        _RV_data->n[i] = _OS_data.v_vector[0] * (-sin(_RV_data->fimax[i])) + _OS_data.v_vector[1] * cos(_RV_data->fimax[i]);
        if(_RV_data->m[i] != 0)
            _RV_data->k_OD[i] = _RV_data->n[i]/_RV_data->m[i];
        _RV_data->o[i] = _OS_data.v_vector[0] * cos(_RV_data->fimax[i]) - _OS_data.v_vector[1] * sin(_RV_data->fimax[i]);
        _RV_data->p[i] = _OS_data.v_vector[0] * sin(_RV_data->fimax[i]) + _OS_data.v_vector[1] * cos(_RV_data->fimax[i]);
        if(_RV_data->o[i] != 0)
            _RV_data->k_OC[i] = _RV_data->p[i]/_RV_data->o[i];
    }
    if(strcmp(DATA_set_type, "RV")==0){
//        cout << "RV: " << endl;
        outputRVdata_set(_RV_data);
    }
    else if(strcmp(DATA_set_type, "RAV")==0 || strcmp(DATA_set_type, "RAVa")==0 || strcmp(DATA_set_type, "RAVua")==0 || strcmp(DATA_set_type, "RAVaua")==0){
//        cout<<"NLVO: "<<endl;
        outputNLVOdata_set(DATA_set_type, _TS_data, _NLVO_data);
//        cout<< DATA_set_type <<": "<<endl;
        outputNLVO_RAVdata_set(_RV_data, _NLVO_data, _RAV_data);
    }
    else{
        //cout << "DATA_set_type input error!" << endl;
    }
}

int main(){
	OS_DATA os;
	os.dLatitude = 10.5;
        os.dLongitude = 134.5;
        os.v_vector[0] = 8.060551817460404;
        os.v_vector[1] = -2.1393836627905;
	TS_DATA ts;
	ts.dLatitude = 10.499392821722926;
        ts.dLongitude = 134.50285787348932;
        ts.v_vector[0][0] = -6.337605215308168;
        ts.v_vector[0][1] = 5.820715541638695;
        for(int i=1;i<=10;i++){
        	ts.v_vector[i][0] = ts.v_vector[0][0];
        	ts.v_vector[i][1] = ts.v_vector[0][1];
        }
        RV_DATA rv;
        NLVO_DATA nlvo;
        RAV_DATA rav;
	outputdata_set("RV", os, ts, &rv, &nlvo, &rav);
	printf("%d\n", rav.RAV_set_num);
	for (int i=0;i<rav.RAV_set_num;i++){
		printf("%d, %d\n", rav.RAV_set[i][0], rav.RAV_set[i][1]);
	}
}
