#ifndef __KALMAN_H
#define __KALMAN_H


// 卡尔曼滤波器结构体
typedef struct {
    float q;        // 过程噪声协方差
    float r;        // 测量噪声协方差
    float x;        // 状态估计值
    float p;        // 估计误差协方差
    float k;        // 卡尔曼增益
} KalmanFilter;

// 初始化卡尔曼滤波器
void KalmanFilter_Init(KalmanFilter *kf, float q, float r);
// 卡尔曼滤波更新
float KalmanFilter_Update(KalmanFilter *kf, float measurement, float dt) ;

#endif