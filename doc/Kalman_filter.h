#ifndef __KALMAN_FILTER_H
#define __KALMAN_FILTER_H

#include "stm32f10x.h"

// 一阶卡尔曼滤波器参数
typedef struct 
{
    float Z_k[50];  // 第k次测量结果
    float E_mea;    // 第k次测量误差
    double X_k[50]; // 第k次估计值
    double K_k[50];  // 第k次卡尔曼增益 
    double E_est[50];   // 第k次估计误差，需要实时更新

}Kalman_InitDef;

// void generate_RandomMesurement(Kalman_InitDef *kalmanStruct);
void get_Actual_Temperature(Kalman_InitDef *kalmanStruct);
void Kalman_Parameter_Init(Kalman_InitDef *kalmanStruct);
float KalmanGain_Calculation(double E_est, double E_mea);
void calculate_X_K(Kalman_InitDef *kalmanStruct);

#endif 

