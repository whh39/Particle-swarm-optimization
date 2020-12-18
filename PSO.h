/************************************************************
 * ͷ�ļ�����ZPSOAlgorithm
 * ͷ�ļ��������û��Զ�������Ⱥ�㷨
 * ���ߣ�chikuo
 * ���ڣ�20190706
 * �汾�ţ�ZPSOAlgorithm_2019070601
************************************************************/

/************************************************************
 * �޸����ڣ�20190708
 * �޸��ˣ�chikuo
 * �޸���Ϣ�������Ŷ��ٶȣ����pso�㷨�ֲ���������
************************************************************/
#ifndef _ZPSOALGORITHM_H
#define _ZPSOALGORITHM_H

#include <stdlib.h>
#include <time.h>
#include <math.h>

//����Ⱥ�㷨���Ӹ���
class ZPSO_Partical
{
public:
    int _dimension; //����ά��
    double* _position; //��������λ������ָ��
    double* _velocity; //�����ٶ�
    double* _bestPosition; //��ǰ���������õ���������λ��
    double _fitness; //������Ӧ��
    double _bestFitness; //��ǰ���������õ�����������Ӧ��
    //���캯��������ά�ȳ�ʼ��Ϊ0
    ZPSO_Partical(void)
    {
        _dimension = 0;
    }
    //�����������ͷ������ڴ�
    ~ZPSO_Partical(void)
    {
        if (_dimension)
        {
            delete[]_position;
            delete[]_velocity;
            delete[]_bestPosition;
        }
    }
    //��ʼ������������Ϊ���ӿ����ڴ�ռ�
    void initial(int dimension)
    {
        if (_dimension != dimension && dimension)
        {
            //��Ҫ���·����ڴ�
            if (_dimension)
            {
                //���������ڴ�
                delete[]_position;
                delete[]_velocity;
                delete[]_bestPosition;
            }
            //�������ڴ�
            _dimension = dimension;
            _position = new double[_dimension];
            _velocity = new double[_dimension];
            _bestPosition = new double[_dimension];
        }
    }
    //���ƺ������������Ӽ�ĸ��Ʋ���
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

//PSO�㷨
class ZPSO_Algorithm
{
public:
    int _dimension; //����Ⱥά��
    int _particalCount; //��Ⱥ��������
    double _globalGuideCoe; //ȫ����������ϵ��
    double _localGuideCoe; //�ֲ���������ϵ��
    ZPSO_Partical _globalBestPartical; //�������̵õ���ȫ����������
    double* _positionMinValue; //����λ�õ���С��
    double* _positionMaxValue; //����λ�õ�����
    double _maxSpeed; //������������ٶ�
    double (*_fitnessFunction)(ZPSO_Partical&); //������Ӧ�Ⱥ���
    ZPSO_Partical* _particalSet; //���Ӽ�
    /***************************************************************
     * ��������ZPSO_Algorithm
     * ��������������һ��PSO�㷨
     * ���������
     *  objFunction���Ż�Ŀ�꺯��ָ��
     *  positionMinValue�����½磬����һά��ע��Ӧ�����Ͻ�
     *  positionMaxValue�����Ͻ磬����һά��ע��Ӧ�����½�
     *  dimension����ռ�ά��
     *  particalCount����Ⱥ���Ӹ���
     *  globalGuideCoe��������Ⱥȫ�����������ٶ����ӣ�Ĭ��Ϊ2
     *  localGuideCoe��������Ⱥ�������������ٶ����ӣ�Ĭ��Ϊ2
     *  maxSpeed�������˶�����ٶ�
     * ���������
     *  ZPSO_Algorithm&�������õ���PSO�㷨����
    ***************************************************************/
    ZPSO_Algorithm(double (*objFunction)(ZPSO_Partical&),
        double* positionMinValue, double* positionMaxValue,
        int dimension, int particalCount,
        double globalGuideCoe = 2, double localGuideCoe = 2,
        double maxSpeed = 1)
    {
        //��ʼ�����ڲ����������ڴ�
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
        //�������������
        srand((unsigned int)time(NULL));
    }
    /***************************************************************
     * ��������~ZPSO_Algorithm
     * ��������������һ��PSO�㷨���ͷ��㷨�ڴ�
     * ���������void
     * ���������void
    ***************************************************************/
    ~ZPSO_Algorithm(void)
    {
        //�ͷ��ڴ�
        delete[]_positionMinValue;
        delete[]_positionMaxValue;
        delete[]_particalSet;
    }
    /***************************************************************
     * ��������rand0_1
     * ��������������һ��0-1�ľ��ȷֲ������
     * ���������void
     * ���������
     *  double����0-1�Ͼ��ȷֲ��������
    ***************************************************************/
    double rand0_1(void) { return((1.0 * rand()) / RAND_MAX); }
    /***************************************************************
     * ��������refresh
     * ��������������������Ӧ�Ȳ��������Ӹ�������λ����ȫ������λ��
     * ���������void
     * ���������void
    ***************************************************************/
    void refresh(void)
    {
        int globalBestParticalIndex = -1;
        for (int i = 0; i < _particalCount; i++)
        {
            //ѭ�������������ӣ�����������Ӧ��
            _particalSet[i]._fitness = this->_fitnessFunction(_particalSet[i]);
            if (_particalSet[i]._fitness > _particalSet[i]._bestFitness)
            {
                //�������ӵĸ�������λ��
                for (int j = 0; j < _dimension; j++)
                    _particalSet[i]._bestPosition[j] = _particalSet[i]._position[j];
                _particalSet[i]._bestFitness = _particalSet[i]._fitness;
                //�Ƿ����ȫ�����Ž�
                if (_particalSet[i]._bestFitness > _globalBestPartical._bestFitness)
                    globalBestParticalIndex = i;
            }
        }
        //����ȫ����������λ��
        if (globalBestParticalIndex != -1)
            _globalBestPartical.copy(_particalSet[globalBestParticalIndex]);
    }
    /***************************************************************
     * ��������randomlyInitial
     * ���������������ʼ����Ⱥ������λ��
     * ���������void
     * ���������void
    ***************************************************************/
    void randomlyInitial(void)
    {
        int globalBestParticalIndex = -1;
        //������������

        //��ʼ����0��������ȫ����������
        //��ʼ������λ�����ٶ�
        double velocityMod = 0;
        //�������ӵ���һά��
        for (int j = 0; j < _particalSet[0]._dimension; j++)
        {
            //�����ʼ������λ�������λ��
            double tempVal = _positionMinValue[j];
            tempVal += rand0_1() * (_positionMaxValue[j] - _positionMinValue[j]);
            _particalSet[0]._position[j] = tempVal;
            _particalSet[0]._bestPosition[j] = tempVal;
            //�����ʼ�������ٶ�
            _particalSet[0]._velocity[j] = rand0_1();
            velocityMod += _particalSet[0]._velocity[j] * _particalSet[0]._velocity[j];
        }
        //�����ٶȹ黯Ϊ�����Сv_mod
        double v_mod = rand0_1() * _maxSpeed;
        velocityMod = sqrt(velocityMod);
        for (int j = 0; j < _particalSet[0]._dimension; j++)
            _particalSet[0]._velocity[j] *= (v_mod / velocityMod);

        //�������ӳ�����Ӧ��ֵ�������Ӧ��ֵ
        _particalSet[0]._fitness = _fitnessFunction(_particalSet[0]);
        _particalSet[0]._bestFitness = _particalSet[0]._fitness;
        _globalBestPartical.copy(_particalSet[0]);

        //��ʼ��1~_particalCount-1������
        for (int i = 1; i < _particalCount; i++)
        {
            velocityMod = 0;
            //��ʼ������λ�����ٶ�
            //�������ӵ���һά��
            for (int j = 0; j < _particalSet[i]._dimension; j++)
            {
                //�����ʼ������λ�������λ��
                double tempVal = _positionMinValue[j];
                tempVal += rand0_1() * (_positionMaxValue[j] - _positionMinValue[j]);
                _particalSet[i]._position[j] = tempVal;
                _particalSet[i]._bestPosition[j] = tempVal;
                //�����ʼ�������ٶ�
                _particalSet[i]._velocity[j] = rand0_1();
                velocityMod += _particalSet[i]._velocity[j] * _particalSet[i]._velocity[j];
            }
            //�����ٶȹ黯Ϊ�����Сv_mod
            v_mod = rand0_1() * _maxSpeed;
            velocityMod = sqrt(velocityMod);
            for (int j = 0; j < _particalSet[i]._dimension; j++)
                _particalSet[i]._velocity[j] *= (v_mod / velocityMod);

            //�������ӳ�����Ӧ��ֵ�������Ӧ��ֵ
            _particalSet[i]._fitness = _fitnessFunction(_particalSet[i]);
            _particalSet[i]._bestFitness = _particalSet[i]._fitness;
            if (_particalSet[i]._bestFitness > _globalBestPartical._bestFitness)
                globalBestParticalIndex = i;
        }

        //��������Ⱥȫ���������
        if (globalBestParticalIndex != -1)
            _globalBestPartical.copy(_particalSet[globalBestParticalIndex]);
    }
    /***************************************************************
     * ��������disturbance
     * �����������������ٶȽ��и�����С���Ŷ�
     * ���������
     *  partical�����Ŷ������Ӷ���
     *  relativeVelocityRate���Ŷ��ٶȴ�С���������_maxSpeed�ı�����Ĭ��Ϊ0.05
     * ���������void
    ***************************************************************/
    void disturbance(ZPSO_Partical& partical, double relativeVelocityRate = 0.05)
    {
        //�����Ŷ��ٶ�
        double* disturbanceVelocity = new double[_dimension];
        //��������Ŷ��ٶȴ�С
        double disturbanceVelocityMod = relativeVelocityRate * _maxSpeed * rand0_1();
        double v_mod = 0;
        for (int i = 0; i < _dimension; i++)
        {
            disturbanceVelocity[i] = rand0_1();
            v_mod += disturbanceVelocity[i] * disturbanceVelocity[i];
        }
        v_mod = sqrt(v_mod);
        //�Ŷ��ٶȴ�С�黯��disturbanceVelocityMod
        for (int i = 0; i < _dimension; i++)
            disturbanceVelocity[i] *= (disturbanceVelocityMod / v_mod);
        //�Ŷ������ٶ�
        v_mod = 0;
        for (int i = 0; i < _dimension; i++)
        {
            partical._velocity[i] += disturbanceVelocity[i];
            v_mod += partical._velocity[i] * partical._velocity[i];
        }
        v_mod = sqrt(v_mod);
        //�����ٶ�����
        if (v_mod > _maxSpeed)
            for (int i = 0; i < _dimension; i++)
                partical._velocity[i] *= (_maxSpeed / v_mod);
        delete[]disturbanceVelocity;
    }
    /***************************************************************
     * ��������update
     * ������������������Ⱥ����λ������Ӧ��
     * ���������
     *  disturbanceRate�������ٶ��Ŷ����ʣ�Ĭ��Ϊ0.2
     *  disturbanceVelocityCoe���ٶ��Ŷ����ӣ������Ŷ��ٶ����_maxSpeed��С
     *                          �����Ŷ������ٶ�����߾ֲ�����������Ĭ��Ϊ0.05
     * ���������void
    ***************************************************************/
    void update(double disturbanceRate = 0.2,
        double disturbanceVelocityCoe = 0.05)
    {
        double v_mod;
        //������������
        for (int i = 0; i < _particalCount; i++)
        {
            //��������ά��
            v_mod = 0;
            double r1 = rand0_1();
            double r2 = rand0_1();
            for (int j = 0; j < _particalSet[i]._dimension; j++)
            {
                //�ٶȸ���
                //ȫ������λ�ü��ٶ�
                _particalSet[i]._velocity[j] += _globalGuideCoe * r1 * (_globalBestPartical._bestPosition[j] - _particalSet[i]._position[j]);
                //����ֲ�����λ�ü��ٶ�
                _particalSet[i]._velocity[j] += _localGuideCoe * r2 * (_particalSet[i]._bestPosition[j] - _particalSet[i]._position[j]);
                //�����ٶ�ģ��
                v_mod += _particalSet[i]._velocity[j] * _particalSet[i]._velocity[j];
            }
            //�����ٶ�����
            v_mod = sqrt(v_mod);
            if (v_mod > _maxSpeed)
                for (int j = 0; j < _particalSet[i]._dimension; j++)
                    _particalSet[i]._velocity[j] *= (_maxSpeed / v_mod);
            //�������ٶȽ����Ŷ�������㷨�ֲ���������
            if (rand0_1() < disturbanceRate)
                this->disturbance(_particalSet[i], disturbanceVelocityCoe);
            //λ�ø���
            for (int j = 0; j < _particalSet[i]._dimension; j++)
            {
                _particalSet[i]._position[j] += _particalSet[i]._velocity[j];
                //����λ������
                if (_particalSet[i]._position[j] < _positionMinValue[j])
                    _particalSet[i]._position[j] = _positionMinValue[j];
                else if (_particalSet[i]._position[j] > _positionMaxValue[j])
                    _particalSet[i]._position[j] = _positionMaxValue[j];
            }
        }
        //��������Ⱥ��Ӧ��
        this->refresh();
    }
    /***************************************************************
     * ��������findMax
     * ������������������Ⱥ�㷨�������Ž�
     * ���������
     *  times������Ⱥ��������
     *  bestPartical�������õ������Ÿ���
     *  disturbanceRate�������ٶ��Ŷ����ʣ�Ĭ��Ϊ0.2
     *  disturbanceVelocityCoe���ٶ��Ŷ����ӣ������Ŷ��ٶ����_maxSpeed��С
     *                          �����Ŷ������ٶ�����߾ֲ�����������Ĭ��Ϊ0.05
     * ���������void
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
     * ��������findMax
     * ������������������Ⱥ�㷨�������Ž�
     * ���������
     *  times������Ⱥ��������
     *  bestParticalInEachLoop��ÿһ�ν����е����Ÿ������飬
     *                          ����Ϊtimes+1�����ⲿ�������ṩ�ڴ�ռ�
     *  disturbanceRate�������ٶ��Ŷ����ʣ�Ĭ��Ϊ0.2
     *  disturbanceVelocityCoe���ٶ��Ŷ����ӣ������Ŷ��ٶ����_maxSpeed��С
     *                          �����Ŷ������ٶ�����߾ֲ�����������Ĭ��Ϊ0.05
     * ���������void
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