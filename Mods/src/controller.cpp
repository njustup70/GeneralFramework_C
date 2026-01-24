#include "controller.hpp"

// 构造函数初始化
IVIdentifier::IVIdentifier(float b0_nominal, float fs, float cut_freq, float forget_time) : b0_(b0_nominal) 
{
    // 计算高通滤波系数 alpha = 1 / (1 + 2*pi*fc*dt)
    // 实际上离散形式常用: alpha = rc / (rc + dt)
    float dt = 1.0f / fs;
    float rc = 1.0f / (2.0f * 3.1415926f * cut_freq);
    hpf_alpha_ = rc / (rc + dt);

    // 计算遗忘因子 lambda
    // 记忆时间 T = 1 / (1 - lambda) * dt  => lambda = 1 - dt/T
    // 例如 T=1s, dt=0.001s -> lambda = 0.999
    if (forget_time < 0.01f) forget_time = 0.01f; // 保护
    lambda_ = 1.0f - (dt / forget_time);

    Reset();
}

void IVIdentifier::Reset() {
    theta_iv = 0.0f;
    cov_cross = 0.0f;
    cov_self = 0.0f;
    
    last_r_in_ = 0; last_r_out_ = 0;
    last_u_in_ = 0; last_u_out_ = 0;
    last_z3_in_ = 0; last_z3_out_ = 0;
}

// 核心高通滤波器：y[k] = alpha * (y[k-1] + x[k] - x[k-1])
float IVIdentifier::HighPassFilter(float input, float& last_in, float& last_out)
{
    float output = hpf_alpha_ * (last_out + input - last_in);
    last_in = input;
    last_out = output;
    return output;
}

void IVIdentifier::Update(float r_cmd, float u_ctrl, float z3_obs)
{
    // 我们首先通过高通滤波获取动态分量，相当于去均值
    r_ac = HighPassFilter(r_cmd, last_r_in_, last_r_out_);
    u_ac = HighPassFilter(u_ctrl, last_u_in_, last_u_out_);
    z3_ac = HighPassFilter(z3_obs, last_z3_in_, last_z3_out_);

    // 接着计算相关性矩阵
    cov_cross = lambda_ * cov_cross + r_ac * z3_ac;       // 互相关矩阵
    cov_self = lambda_ * cov_self + r_ac * u_ac;        // 自相关矩阵

    // 为了防止激发模态不足，导致的不可辨识问题，这里利用 参考输入的自相关判断
    const float CALCULATION_THRES = 650.0f; 

    // 激发模态充分的前提下，计算辨识参数 theta_iv
    if (fabs(cov_self) > CALCULATION_THRES)
    {
        float raw_theta = cov_cross / cov_self;
        
        // 一阶低通滤波防止突变
        theta_iv = 0.99f * theta_iv + 0.01f * raw_theta;

        // 工程上，再来一阶低通滤波
        theta_iv_accum_ = 0.999f * theta_iv_accum_ + 0.001f * theta_iv;
    }

    // 计算估计的转动惯量 J_hat_
    float real_b = theta_iv_accum_ + b0_;

    if (real_b < 0.0001f) real_b = 0.0001f;         // 保护数据
    J_hat_ = Kt_ / real_b;
}

float IVIdentifier::GetEstimatedJ(float Kt) const {
    // 原理: theta = b_real - b0
    // b_real = theta + b0
    // J_real = Kt / b_real
    float b_real = theta_iv + b0_;
    
    // 保护：防止分母为0或负数（惯量不可能是负的）
    if (b_real < 0.0001f) b_real = 0.0001f; 
    
    return Kt / b_real;
}