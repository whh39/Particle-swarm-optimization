/************************************************************
 * 头文件名：ZPSOAlgorithm
 * 头文件描述：用户自定义粒子群算法
 * 作者：chikuo
 * 日期：20190706
 * 版本号：ZPSOAlgorithm_2019070601
************************************************************/

/************************************************************
 * 修改日期：20190708
 * 修改人：chikuo
 * 修改信息：引入扰动速度，提高pso算法局部搜索能力
************************************************************/
#ifndef _ZPSOALGORITHM_H
#define _ZPSOALGORITHM_H

#include <stdlib.h>
#include <time.h>
#include <math.h>

//粒子群算法例子个体
class ZPSO_Partical
{
public:
    int _dimension; //粒子维度
    double* _position; //粒子所在位置数组指针
    double* _velocity; //例子速度
    double* _bestPosition; //当前粒子搜索得到过的最优位置
    double _fitness; //例子适应度
    double _bestFitness; //当前粒子搜索得到过的最优适应度
    //构造函数，粒子维度初始化为0
    ZPSO_Partical(void)
    {
        _dimension = 0;
    }
    //析构函数，释放粒子内存
    ~ZPSO_Partical(void)
    {
        if (_dimension)
        {
            delete[]_position;
            delete[]_velocity;
            delete[]_bestPosition;
        }
    }
    //初始化函数，用于为粒子开辟内存空间
    void initial(int dimension)
    {
        if (_dimension != dimension && dimension)
        {
            //需要重新分配内存
            if (_dimension)
            {
                //消除已有内存
                delete[]_position;
                delete[]_velocity;
                delete[]_bestPosition;
            }
            //开辟新内存
            _dimension = dimension;
            _position = new double[_dimension];
            _velocity = new double[_dimension];
            _bestPosition = new double[_dimension];
        }
    }
    //复制函数，用于粒子间的复制操作
    void copy(ZPSO_Partical& partical)
    {
        this->initial(partical._dimension);
        for (int i = 0; i < _dimension; i++)
        {
            _position[i] = partical._position[i];
            _velocity[i] = partical._velocity[i];
            _bestPosition[i] = partical._bestPosition[i];
            _fitness = partical._fitness;
            _bestFitness = partical._bestFitness;
        }
    }
};

//PSO算法
class ZPSO_Algorithm
{
public:
    int _dimension; //粒子群维度
    int _particalCount; //种群粒子数量
    double _globalGuideCoe; //全局最优引导系数
    double _localGuideCoe; //局部最优引导系数
    ZPSO_Partical _globalBestPartical; //搜索过程得到的全局最优粒子
    double* _positionMinValue; //粒子位置的最小界
    double* _positionMaxValue; //粒子位置的最大界
    double _maxSpeed; //粒子允许最大速度
    double (*_fitnessFunction)(ZPSO_Partical&); //粒子适应度函数
    ZPSO_Partical* _particalSet; //粒子集
    /***************************************************************
     * 函数名：ZPSO_Algorithm
     * 函数描述：构造一个PSO算法
     * 输入参数：
     *  objFunction：优化目标函数指针
     *  positionMinValue：解下界，其任一维度注意应低于上界
     *  positionMaxValue：解上界，其任一维度注意应高于下界
     *  dimension：解空间维度
     *  particalCount：种群粒子个数
     *  globalGuideCoe：粒子种群全局最优引导速度因子，默认为2
     *  localGuideCoe：粒子种群个体最优引导速度因子，默认为2
     *  maxSpeed：粒子运动最大速度
     * 输出参数：
     *  ZPSO_Algorithm&：构建得到的PSO算法本身
    ***************************************************************/
    ZPSO_Algorithm(double (*objFunction)(ZPSO_Partical&),
        double* positionMinValue, double* positionMaxValue,
        int dimension, int particalCount,
        double globalGuideCoe = 2, double localGuideCoe = 2,
        double maxSpeed = 1)
    {
        //初始化类内参数并分配内存
        _fitnessFunction = objFunction;
        _dimension = dimension;
        _positionMinValue = new double[_dimension];
        _positionMaxValue = new double[_dimension];
        for (int i = 0; i < _dimension; i++)
        {
            _positionMinValue[i] = positionMinValue[i];
            _positionMaxValue[i] = positionMaxValue[i];
        }
        _particalCount = particalCount;
        _globalGuideCoe = globalGuideCoe;
        _localGuideCoe = localGuideCoe;
        _maxSpeed = maxSpeed;
        _particalSet = new ZPSO_Partical[_particalCount];
        for (int i = 0; i < _particalCount; i++)
            _particalSet[i].initial(_dimension);
        _globalBestPartical.initial(_dimension);
        //配置随机数种子
        srand((unsigned int)time(NULL));
    }
    /***************************************************************
     * 函数名：~ZPSO_Algorithm
     * 函数描述：析构一个PSO算法，释放算法内存
     * 输入参数：void
     * 输出参数：void
    ***************************************************************/
    ~ZPSO_Algorithm(void)
    {
        //释放内存
        delete[]_positionMinValue;
        delete[]_positionMaxValue;
        delete[]_particalSet;
    }
    /***************************************************************
     * 函数名：rand0_1
     * 函数描述：生成一个0-1的均匀分布随机数
     * 输入参数：void
     * 输出参数：
     *  double：在0-1上均匀分布的随机数
    ***************************************************************/
    double rand0_1(void) { return((1.0 * rand()) / RAND_MAX); }
    /***************************************************************
     * 函数名：refresh
     * 函数描述：计算粒子适应度并更新粒子个体最优位置与全局最优位置
     * 输入参数：void
     * 输出参数：void
    ***************************************************************/
    void refresh(void)
    {
        int globalBestParticalIndex = -1;
        for (int i = 0; i < _particalCount; i++)
        {
            //循环遍历所有粒子，更新粒子适应度
            _particalSet[i]._fitness = this->_fitnessFunction(_particalSet[i]);
            if (_particalSet[i]._fitness > _particalSet[i]._bestFitness)
            {
                //更新粒子的个体最优位置
                for (int j = 0; j < _dimension; j++)
                    _particalSet[i]._bestPosition[j] = _particalSet[i]._position[j];
                _particalSet[i]._bestFitness = _particalSet[i]._fitness;
                //是否更新全局最优解
                if (_particalSet[i]._bestFitness > _globalBestPartical._bestFitness)
                    globalBestParticalIndex = i;
            }
        }
        //更新全局最优粒子位置
        if (globalBestParticalIndex != -1)
            _globalBestPartical.copy(_particalSet[globalBestParticalIndex]);
    }
    /***************************************************************
     * 函数名：randomlyInitial
     * 函数描述：随机初始化种群中粒子位置
     * 输入参数：void
     * 输出参数：void
    ***************************************************************/
    void randomlyInitial(void)
    {
        int globalBestParticalIndex = -1;
        //遍历所有粒子

        //初始化第0个粒子与全局最优粒子
        //初始化粒子位置与速度
        double velocityMod = 0;
        //遍历粒子的任一维度
        for (int j = 0; j < _particalSet[0]._dimension; j++)
        {
            //随机初始化粒子位置与最佳位置
            double tempVal = _positionMinValue[j];
            tempVal += rand0_1() * (_positionMaxValue[j] - _positionMinValue[j]);
            _particalSet[0]._position[j] = tempVal;
            _particalSet[0]._bestPosition[j] = tempVal;
            //随机初始化粒子速度
            _particalSet[0]._velocity[j] = rand0_1();
            velocityMod += _particalSet[0]._velocity[j] * _particalSet[0]._velocity[j];
        }
        //粒子速度归化为随机大小v_mod
        double v_mod = rand0_1() * _maxSpeed;
        velocityMod = sqrt(velocityMod);
        for (int j = 0; j < _particalSet[0]._dimension; j++)
            _particalSet[0]._velocity[j] *= (v_mod / velocityMod);

        //更新粒子初代适应度值与最佳适应度值
        _particalSet[0]._fitness = _fitnessFunction(_particalSet[0]);
        _particalSet[0]._bestFitness = _particalSet[0]._fitness;
        _globalBestPartical.copy(_particalSet[0]);

        //初始化1~_particalCount-1个粒子
        for (int i = 1; i < _particalCount; i++)
        {
            velocityMod = 0;
            //初始化粒子位置与速度
            //遍历粒子的任一维度
            for (int j = 0; j < _particalSet[i]._dimension; j++)
            {
                //随机初始化粒子位置与最佳位置
                double tempVal = _positionMinValue[j];
                tempVal += rand0_1() * (_positionMaxValue[j] - _positionMinValue[j]);
                _particalSet[i]._position[j] = tempVal;
                _particalSet[i]._bestPosition[j] = tempVal;
                //随机初始化粒子速度
                _particalSet[i]._velocity[j] = rand0_1();
                velocityMod += _particalSet[i]._velocity[j] * _particalSet[i]._velocity[j];
            }
            //粒子速度归化为随机大小v_mod
            v_mod = rand0_1() * _maxSpeed;
            velocityMod = sqrt(velocityMod);
            for (int j = 0; j < _particalSet[i]._dimension; j++)
                _particalSet[i]._velocity[j] *= (v_mod / velocityMod);

            //更新粒子初代适应度值与最佳适应度值
            _particalSet[i]._fitness = _fitnessFunction(_particalSet[i]);
            _particalSet[i]._bestFitness = _particalSet[i]._fitness;
            if (_particalSet[i]._bestFitness > _globalBestPartical._bestFitness)
                globalBestParticalIndex = i;
        }

        //更新粒子群全局最佳数据
        if (globalBestParticalIndex != -1)
            _globalBestPartical.copy(_particalSet[globalBestParticalIndex]);
    }
    /***************************************************************
     * 函数名：disturbance
     * 函数描述：对粒子速度进行给定大小的扰动
     * 输入参数：
     *  partical：被扰动的粒子对象
     *  relativeVelocityRate：扰动速度大小上限相对于_maxSpeed的比例，默认为0.05
     * 输出参数：void
    ***************************************************************/
    void disturbance(ZPSO_Partical& partical, double relativeVelocityRate = 0.05)
    {
        //生成扰动速度
        double* disturbanceVelocity = new double[_dimension];
        //随机生成扰动速度大小
        double disturbanceVelocityMod = relativeVelocityRate * _maxSpeed * rand0_1();
        double v_mod = 0;
        for (int i = 0; i < _dimension; i++)
        {
            disturbanceVelocity[i] = rand0_1();
            v_mod += disturbanceVelocity[i] * disturbanceVelocity[i];
        }
        v_mod = sqrt(v_mod);
        //扰动速度大小归化到disturbanceVelocityMod
        for (int i = 0; i < _dimension; i++)
            disturbanceVelocity[i] *= (disturbanceVelocityMod / v_mod);
        //扰动粒子速度
        v_mod = 0;
        for (int i = 0; i < _dimension; i++)
        {
            partical._velocity[i] += disturbanceVelocity[i];
            v_mod += partical._velocity[i] * partical._velocity[i];
        }
        v_mod = sqrt(v_mod);
        //粒子速度受限
        if (v_mod > _maxSpeed)
            for (int i = 0; i < _dimension; i++)
                partical._velocity[i] *= (_maxSpeed / v_mod);
        delete[]disturbanceVelocity;
    }
    /***************************************************************
     * 函数名：update
     * 函数描述：更新粒子群粒子位置与适应度
     * 输入参数：
     *  disturbanceRate：粒子速度扰动概率，默认为0.2
     *  disturbanceVelocityCoe：速度扰动因子，表征扰动速度相对_maxSpeed大小
     *                          用于扰动粒子速度以提高局部搜索能力，默认为0.05
     * 输出参数：void
    ***************************************************************/
    void update(double disturbanceRate = 0.2,
        double disturbanceVelocityCoe = 0.05)
    {
        double v_mod;
        //遍历所有粒子
        for (int i = 0; i < _particalCount; i++)
        {
            //遍历所有维度
            v_mod = 0;
            double r1 = rand0_1();
            double r2 = rand0_1();
            for (int j = 0; j < _particalSet[i]._dimension; j++)
            {
                //速度更新
                //全局最优位置加速度
                _particalSet[i]._velocity[j] += _globalGuideCoe * r1 * (_globalBestPartical._bestPosition[j] - _particalSet[i]._position[j]);
                //个体局部最优位置加速度
                _particalSet[i]._velocity[j] += _localGuideCoe * r2 * (_particalSet[i]._bestPosition[j] - _particalSet[i]._position[j]);
                //粒子速度模二
                v_mod += _particalSet[i]._velocity[j] * _particalSet[i]._velocity[j];
            }
            //粒子速度受限
            v_mod = sqrt(v_mod);
            if (v_mod > _maxSpeed)
                for (int j = 0; j < _particalSet[i]._dimension; j++)
                    _particalSet[i]._velocity[j] *= (_maxSpeed / v_mod);
            //对粒子速度进行扰动，提高算法局部搜索能力
            if (rand0_1() < disturbanceRate)
                this->disturbance(_particalSet[i], disturbanceVelocityCoe);
            //位置更新
            for (int j = 0; j < _particalSet[i]._dimension; j++)
            {
                _particalSet[i]._position[j] += _particalSet[i]._velocity[j];
                //粒子位置受限
                if (_particalSet[i]._position[j] < _positionMinValue[j])
                    _particalSet[i]._position[j] = _positionMinValue[j];
                else if (_particalSet[i]._position[j] > _positionMaxValue[j])
                    _particalSet[i]._position[j] = _positionMaxValue[j];
            }
        }
        //更新粒子群适应度
        this->refresh();
    }
    /***************************************************************
     * 函数名：findMax
     * 函数描述：采用粒子群算法搜索最优解
     * 输入参数：
     *  times：粒子群进化次数
     *  bestPartical：进化得到的最优个体
     *  disturbanceRate：粒子速度扰动概率，默认为0.2
     *  disturbanceVelocityCoe：速度扰动因子，表征扰动速度相对_maxSpeed大小
     *                          用于扰动粒子速度以提高局部搜索能力，默认为0.05
     * 输出参数：void
    ***************************************************************/
    void findMax(int times, ZPSO_Partical& bestPartical,
        double disturbanceRate = 0.2,
        double disturbanceVelocityCoe = 0.05)
    {
        this->randomlyInitial();
        for (int i = 0; i < times; i++)this->update(disturbanceRate, disturbanceVelocityCoe);
        bestPartical.copy(_globalBestPartical);
    }
    /***************************************************************
     * 函数名：findMax
     * 函数描述：采用粒子群算法搜索最优解
     * 输入参数：
     *  times：粒子群进化次数
     *  bestParticalInEachLoop：每一次进化中的最优个体数组，
     *                          长度为times+1，由外部调用者提供内存空间
     *  disturbanceRate：粒子速度扰动概率，默认为0.2
     *  disturbanceVelocityCoe：速度扰动因子，表征扰动速度相对_maxSpeed大小
     *                          用于扰动粒子速度以提高局部搜索能力，默认为0.05
     * 输出参数：void
    ***************************************************************/
    void findMax(int times, ZPSO_Partical* bestParticalInEachLoop,
        double disturbanceRate = 0.2,
        double disturbanceVelocityCoe = 0.05)
    {
        this->randomlyInitial();
        for (int i = 1; i <= times; i++)
        {
            this->update(disturbanceRate, disturbanceVelocityCoe);
            bestParticalInEachLoop[i].copy(_globalBestPartical);
        }
    }
};

#endif // _ZPSOALGORITHM_H