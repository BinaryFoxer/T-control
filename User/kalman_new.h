#ifndef KALMAN_NEW_H_
#define KALMAN_NEW_H_

#include "stm32f10x.h"                  // Device header

typedef struct {
    double X_prev;   // 上一次的估计值
    double E_est;    // 估计误差
    double E_mea;    // 测量误差（固定值）
} Kalman_InitDef;

void Kalman_Parameter_Init(Kalman_InitDef *kalmanStruct);
double update_Kalman(Kalman_InitDef *kalman, double measurement);

#endif
