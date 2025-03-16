// =============================
// 函数名称：一阶卡尔曼滤波算法
// 函数功能：对从传感器采集的50个数据进行卡尔曼滤波
// 入口参数：@Kalman_InitDef
// 函数返回：X_k
// 当前版本：VER1.0
// 修改日期：2025
// 当前作者：BiaryFoxer
// 其它备注：目前是直接对采集到的每个数据进行卡尔曼滤波，后续测试可以考虑
//          先对50个采样值进行均值滤波再进行卡尔曼滤波，精度提高响应速度降低
//          后续查一下stm32 ADC1的采样速率，要注意50个数据进行滤波的时间不能长于40ms
// ==============================


#include "Kalman_filter.h"  
#include "stdlib.h"
#include "stdio.h"
#include "Temper.h"

/*
这里的一阶卡尔曼滤波算法实际上是一个递归算法，
它的核心思想其实仍然是均值滤波，只是每一次会递归更新估计误差增益和卡尔曼增益，
卡尔曼增益实际上是根据测量误差和估计误差决定第k次实验时是更相信测量值还是估计值
*/


// // 滤波效果测试，产生随机测量值
// void generate_RandomMesurement(Kalman_InitDef *kalmanStruct)
// {
//     uint8_t i;
//     for(i = 0; i < 50; i++)
//     {
//         // 每次的测量值随机生成
//         kalmanStruct->Z_k[i] = (float)30 + rand() / RAND_MAX *2 - 1;
//         // 打印测量值供对比
//         printf("Random measurement Z_k = %f\r\n", kalmanStruct->Z_k[i]);

//     }

// }

// 从温度传感器采集温度值
void get_Actual_Temperature(Kalman_InitDef *kalmanStruct)
{
    uint8_t i;
    for(i = 0; i < 50; i++)
    {
        // 采集50次温度传感器的值进行卡尔曼滤波
        // 将电压值映射到-40~40℃，这个范围可能需要根据实际温度传感器的参数修改
        kalmanStruct->Z_k[i] = (float) AD_Value / 4096.0 * 80.0 - 40.0;

    }

}

// 卡尔曼滤波器参数初始化
void Kalman_Parameter_Init(Kalman_InitDef *kalmanStruct)
{
    kalmanStruct->X_k[0] = 20;   // 第一次估计值，不需太精准
    kalmanStruct->E_est[0] = 1;  // 第一次估计误差，同样不需太精准
    kalmanStruct->E_mea = 0.033; // 测量误差，需要根据ADC和传感器计算出精确值
                                // 如果滤波效果不理想，可能是测量误差不准，这里暂时随便赋值
}

// 计算卡尔曼增益
float KalmanGain_Calculation(double E_est, double E_mea)
{
    double result;
    result = E_est / (E_est + E_mea);  
    return result;

}

// 计算第k次估计结果（通过数据融合计算出最优化结果）
void calculate_X_K(Kalman_InitDef *kalmanStruct)
{
    uint8_t i;
    double Kalman_Gain;
    // i从1开始，第一次预估值通过初始化获得
    for (i = 1; i < 50; i++)
    {
        // 计算每次的卡尔曼增益
        Kalman_Gain = KalmanGain_Calculation(kalmanStruct->E_est[i - 1], kalmanStruct->E_mea);
        // 带入公式，计算每次估计值
        // 第k次估计值 = 上一次估计值加上卡尔曼增益 * (第k次测量值 - 上一次估计值)
        kalmanStruct->X_k[i] = kalmanStruct->X_k[i - 1] + Kalman_Gain * (kalmanStruct->Z_k[i] - kalmanStruct->X_k[i - 1]);
        // 更新估计误差
        kalmanStruct->E_est[i] = ((double)1 - Kalman_Gain) * kalmanStruct->E_est[i - 1];
       
        // // 打印参数值供检验
        // printf("Kalman Gain =%f\r\n", Kalman_Gain);
        // printf("X_k =%f\r\n", kalmanStruct->X_k[i]);
        // printf("E_est =%f\r\n", kalmanStruct->E_est[i]);

    }
    
}
