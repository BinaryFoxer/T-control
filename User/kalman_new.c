#include "kalman_new.h"

// 卡尔曼滤波器参数初始化
void Kalman_Parameter_Init(Kalman_InitDef *kalmanStruct)
{
    kalmanStruct->X_prev = 28;   // 第一次估计值，不需太精准
    kalmanStruct->E_est = 5;  // 第一次估计误差，同样不需太精准
    kalmanStruct->E_mea = 0.25; // 测量误差，需要根据ADC和传感器计算出精确值
                                // 如果滤波效果不理想，可能是测量误差不准，这里暂时随便赋值
    kalmanStruct->Q = 0.4; // 过程噪声，这里暂时随便赋值
}


// =============================
// 函数名称：卡尔曼滤波输出
// 函数功能：实时计算输入数据的卡尔曼滤波输出
// 入口参数：@Kalman_InitDef
//			 @measurement	第k次测量值，此工程中使用AD_Value
// 函数返回：第k次估计值
// 当前版本：VER1.0
// 修改日期：2025
// 当前作者：BinaryFoxer
// 其它备注：
// ==============================

// 单次卡尔曼更新，返回滤波后的值
double update_Kalman(Kalman_InitDef *kalman, double measurement) 
{
    // 预测阶段
    double X_prior = kalman->X_prev;
    double E_prior = kalman->E_est + kalman->Q;

    // 更新阶段
    double K = E_prior / (E_prior + kalman->E_mea);
    double X_new = X_prior + K * (measurement - X_prior);
    double E_new = (1 - K) * E_prior;
    
    // 保存状态供下次使用
    kalman->X_prev = X_new;
    kalman->E_est = E_new;
    
    return X_new;
    
}
