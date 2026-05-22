#include "ti_msp_dl_config.h"
#include "stdio.h"
#include "Kalman.h" 



// 初始化卡尔曼滤波器
void KalmanFilter_Init(KalmanFilter *kf, float q, float r) {
    kf->q = q;
    kf->r = r;
    kf->x = 0;
    kf->p = 1.0;
}

// 卡尔曼滤波更新
float KalmanFilter_Update(KalmanFilter *kf, float measurement, float dt) {
    // 预测步骤
    // 状态预测：x = x (假设角度匀速变化，无外部输入)
    // 误差协方差预测：p = p + q
    kf->p = kf->p + kf->q * dt;
    
    // 更新步骤
    // 计算卡尔曼增益：k = p / (p + r)
    kf->k = kf->p / (kf->p + kf->r);
    
    // 状态更新：x = x + k * (z - x)
    kf->x = kf->x + kf->k * (measurement - kf->x);
    
    // 误差协方差更新：p = (1 - k) * p
    kf->p = (1 - kf->k) * kf->p;
    
    return kf->x;
}