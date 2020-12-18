#include "PSO.h"
#include <math.h>
#include <iostream>
using namespace std;


//���Ż���Ŀ�꺯��
double objFunction(ZPSO_Partical& partical);


int main(void)
{
    //������ⷶΧ,x��[-10,10]��Χ��y��[-10,10]��Χ
    double minPos[] = { -10,-10 };
    double maxPos[] = { 10,10 };
    //����������������
    int dimension = 2; //��ѡ��ά��
    int particalCount = 200; //����Ⱥ���Ӹ���
    double globalGuideCoe = 1; //ȫ���������ٶ�
    double localGuideCoe = 1; //�ֲ��������ٶ�
    double maxSpeed = 4; //��������ٶ�
    //����pso�㷨
    ZPSO_Algorithm pso(objFunction, minPos, maxPos, dimension, particalCount,
        globalGuideCoe, localGuideCoe, maxSpeed);
    //����pso�㷨
    ZPSO_Partical bestPartical; //����Ⱥ���ս������
    int generation = 200; //����Ⱥ��������
    pso.findMax(generation, bestPartical); //��ȡ���ս������
    //������ս��
    cout << "the best position for the objFunction is:" << endl;
    cout << "x=" << bestPartical._position[0] << endl;
    cout << "y=" << bestPartical._position[1] << endl;
    cout << "the best fitness for the objFunction is:" << bestPartical._fitness << endl;
    cout << "The run time is:" << (double)clock() / CLOCKS_PER_SEC << "s" << endl;
    return(0);
}


//�Ż�Ŀ�꺯������
double objFunction(ZPSO_Partical& partical)
{
    //��partical�ж�ȡ��ѡ��
    double x = partical._position[0] - 0.3;
    double y = partical._position[1] + 0.1;
    //�����ѡ���Ӧ�ĺ���ֵ
    double r = sqrt(x * x + y * y);
    double rtn;
    if (r < 1e-8)
        rtn = 1;
    else
        rtn = sin(r) / r;
    return(rtn);
}
