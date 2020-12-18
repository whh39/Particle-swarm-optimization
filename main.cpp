#include "PSO.h"
#include <math.h>
#include <iostream>
using namespace std;


//待优化的目标函数
double objFunction(ZPSO_Partical& partical);


int main(void)
{
    //定义求解范围,x在[-10,10]范围，y在[-10,10]范围
    double minPos[] = { -10,-10 };
    double maxPos[] = { 10,10 };
    //定义问题描述参数
    int dimension = 2; //候选解维度
    int particalCount = 200; //粒子群粒子个数
    double globalGuideCoe = 1; //全局引导加速度
    double localGuideCoe = 1; //局部引导加速度
    double maxSpeed = 4; //粒子最大速度
    //构建pso算法
    ZPSO_Algorithm pso(objFunction, minPos, maxPos, dimension, particalCount,
        globalGuideCoe, localGuideCoe, maxSpeed);
    //运行pso算法
    ZPSO_Partical bestPartical; //粒子群最终进化结果
    int generation = 200; //粒子群进化代数
    pso.findMax(generation, bestPartical); //获取最终进化结果
    //输出最终结果
    cout << "the best position for the objFunction is:" << endl;
    cout << "x=" << bestPartical._position[0] << endl;
    cout << "y=" << bestPartical._position[1] << endl;
    cout << "the best fitness for the objFunction is:" << bestPartical._fitness << endl;
    cout << "The run time is:" << (double)clock() / CLOCKS_PER_SEC << "s" << endl;
    return(0);
}


//优化目标函数定义
double objFunction(ZPSO_Partical& partical)
{
    //从partical中读取候选解
    double x = partical._position[0] - 0.3;
    double y = partical._position[1] + 0.1;
    //计算候选解对应的函数值
    double r = sqrt(x * x + y * y);
    double rtn;
    if (r < 1e-8)
        rtn = 1;
    else
        rtn = sin(r) / r;
    return(rtn);
}
