// // 驱动控制测试
// key_num = Key_GetNum();
// if(key_num == 1)
// {
//     PWM += 10;
//     if(PWM > 100)
//     {
//         PWM = 100;
//     }

// }

// else if (key_num == 2)
// {
//     PWM -= 10;
//     if(PWM < -100)
//     {
//         PWM = -100;
//     }

// }

// driver_SetPWM(PWM);
// OLED_Printf(0, 16, OLED_8X16, "PWM:%+04d", PWM);
// OLED_Update();

// // 电位器ADC采集测试
// RP_1 = RP_GetValue(1);
// RP_2 = RP_GetValue(2);
// RP_3 = RP_GetValue(3);
// RP_4 = RP_GetValue(4);

// // 串口测试
// Serial_Printf("%d, %d, %d, %d\r\n", RP_1, RP_2, RP_3, RP_4);

// // 温度传感器ADC采集测试
// OLED_Printf(0, 16, OLED_8X16, "Temp:%d", AD_Value);
// OLED_Update();