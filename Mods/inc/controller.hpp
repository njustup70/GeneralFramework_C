#pragma once
#include "linear_math.hpp"
#include "bsp_dwt.h"

namespace CtrlUtils
{
    static int Signf(float val)
    {
        if (val > 0) return 1;
        else if (val < 0) return -1;
        else return 0;
    };
}

/**
 * @brief 线性化的微分跟踪器
 * @name Lineared Time Differentiator (Linear TD)
 * @note 其实就是一个临界阻尼的二阶系统，用于平滑阶跃输入并计算其导数
 */
class LinearTD
{
public:
    float v1 = 0.0f;        // 跟踪到的平滑速度 (Tracked Speed)
    float v2 = 0.0f;        // 跟踪信号的微分 (Tracked Acceleration)
    float acc = 0.0f;       // 【新增】跟踪加速度 (前馈项)
    
    // 响应速度因子 r: 越大跟踪越快，但也越容易把噪声传进去。
    // 对于你的500Hz系统，建议尝试 10.0 ~ 50.0
    float r = 20.0f; 
    float dt = 0.002f;

    void Init(float _r, float _dt)
    {
        r = _r;
        dt = _dt;
        v1 = 0.0f;
        v2 = 0.0f;
    }

    /**
     * @brief 计算下一时刻的跟踪值
     * @param target_input 原始的阶跃目标输入
     */
    void Update(float target_input)
    {
        // 误差
        float error = v1 - target_input;
        
        // 计算期望加速度 (临界阻尼公式: a = -r*r*e - 2*r*v)
        // 这里的 v2 就是 dv1/dt
        acc = -r * r * error - 2.0f * r * v2;
        
        // 欧拉积分更新
        v1 += v2 * dt;
        v2 += acc * dt;
    }
};

/**
 * @brief 三阶线性跟踪微分器 (3rd-Order Linear TD)
 * @note 平滑位置、速度、加速度。输出的加速度是连续的。
 */
class LinearTD_3rd
{
    public:
    float v1 = 0.0f; // 位置
    float v2 = 0.0f; // 速度
    float v3 = 0.0f; // 加速度 (系统内部状态)
    
    float r = 20.0f; 
    float dt = 0.002f;

    // === 新增：限制参数 ===
    float max_v = 0.0f; // 最大速度 (0代表不限制)
    float max_a = 0.0f; // 最大加速度 (0代表不限制)

    // 辅助函数：限幅
    private : float fclamp(float val, float limit) 
    {
        if (limit <= 0.0f)  return val; // 0代表不限制
        if (val > limit)    return limit;
        if (val < -limit)   return -limit;
        return val;
    }

    public : void Init(float _r, float _dt, float _max_v, float _max_a)
    {
        r = _r;
        dt = _dt;
        max_v = _max_v;
        max_a = _max_a;
        v1 = 0.0f;
        v2 = 0.0f;
        v3 = 0.0f;
    }

    void Update(float target_input)
    {
        float error = v1 - target_input;
        
        // 计算加加速度 (Jerk)
        float jerk = -r*r*r * error - 3.0f*r*r * v2 - 3.0f*r * v3;
        
        // 积分更新加速度 v3
        v3 += jerk * dt;

        // 硬切断加速度
        v3 = fclamp(v3, max_a); 

        // 积分更新速度 v2
        v2 += v3 * dt;

        // 插入：硬切断速度
        v2 = fclamp(v2, max_v);

        // 积分更新位置 v1
        v1 += v2 * dt;
    }
};

/**
 * @brief 一阶低通滤波器
 * @note 这个跟手写的不同点是，这个是基于截止频率 fc 来设计的，更直观一些
 */
class LowPassFilter 
{
public:
    float y_prev = 0.0f;
    float alpha = 1.0f;         // 低通滤波系数 (0~1)

    // fc: 截止频率 (Hz or rad/s), dt: 周期
    void Init(float cutoff_freq_rad, float dt) 
    {
        // 计算 alpha: alpha = dt / (RC + dt)
        // time_const (RC) = 1 / cutoff_freq_rad
        float time_const = 1.0f / cutoff_freq_rad;
        alpha = dt / (time_const + dt);
    }

    void InitHz(float cutoff_freq_hz, float dt) 
    {
        float omega_c = 2 * PI * cutoff_freq_hz;  // Hz转rad/s
        float time_const = 1.0f / omega_c;
        alpha = dt / (time_const + dt);
    }

    float Filter(float input) 
    {
        // 一阶滤波公式: y = alpha * x + (1-alpha) * y_last
        float y = alpha * input + (1.0f - alpha) * y_prev;
        y_prev = y;
        return y;
    }
};


/**
 * @brief 摩擦补偿器
 */
class FrictionCompensator
{
    public:
    FrictionCompensator() {};

    float fric_static = 0.0f;   // 静摩擦补偿 (电流：A)
    float fric_dynamic = 0.0f;  // 动摩擦补偿 (电流：A)
    float vel_transition_start = 0.0f; // 速度过渡起点 (rad/s)
    float vel_transition_end = 0.0f;   // 速度过渡终点 (rad/s)

    LowPassFilter lpf_fric;     // 限制补偿带宽，放置补偿突变，把ESO干爆
    
    void Init(float F_static, float F_dynamic, float V_tran_start, float V_tran_end)
    {
        fric_static = F_static;
        fric_dynamic = F_dynamic;
        vel_transition_start = V_tran_start;
        vel_transition_end = V_tran_end;
        lpf_fric.Init(5.0f, 0.001f);   // 截止频率12rad/s，周期1ms   
    }

    /**
     * @brief 根据摩擦力模型，获得当前的摩擦力大小
     * @note 用于补偿ESO的模型输入
     */
    float GetFriction(float v_real)
    {
        // 不再需要判断意图，只试图恢复真实物理世界的摩擦力
        float i_total = 0.0f;

        // 若速度很快，直接使用动摩擦补偿
        if (fabs(v_real) > vel_transition_end)
        {
            // 根据符号判定，摩擦力与速度方向相反
            i_total = -fric_dynamic * CtrlUtils::Signf(v_real);
        }
        // 速度在过渡区间，线性插值
        else if (fabs(v_real) > vel_transition_start)
        {
            // 计算线性比
            float ratio = (fabs(v_real) - vel_transition_start) / (vel_transition_end - vel_transition_start);
            // 根据符号判定，摩擦力与速度方向相反
            i_total = -(fric_static + (fric_dynamic - fric_static) * ratio) * CtrlUtils::Signf(v_real);
        }
        // 速度很小，使用静摩擦补偿
        else
        {
            i_total = -fric_static * CtrlUtils::Signf(v_real);
        }

        return i_total;
    }

    float GetSimpleFriction(float v_real)
    {
        // 简单模型：只有静摩擦补偿
        float i_total = 0.0f;

        // 使用静摩擦补偿
        i_total = -fric_static * CtrlUtils::Signf(v_real) * 0.75f;

        return i_total;
    }

    /**
     * @brief 根据真实速度和意图，计算摩擦补偿电流
     */
    float Get_Compensation(float real_u, float real_velo)
    {
        // 构造一个p，表示控制器的能量意图；当p > 0，系统希望加速；反之希望刹车
        float p = real_u * real_velo;
        
        float i_total = 0.0f;

        // 当系统希望加速时，使用正向摩擦补偿
        if (p > 0 && fabs(real_u) > 0.25f)
        {
            // 若速度很快，直接使用动摩擦补偿
            if (fabs(real_velo) > vel_transition_end)
            {
                // 根据符号补偿
                i_total = fric_dynamic * CtrlUtils::Signf(real_velo);
            }
            // 速度在过渡区间，线性插值
            else if (fabs(real_velo) > vel_transition_start)
            {
                // 计算线性比
                float ratio = (fabs(real_velo) - vel_transition_start) / (vel_transition_end - vel_transition_start);
                // 根据符号补偿
                i_total = (fric_static + (fric_dynamic - fric_static) * ratio) * CtrlUtils::Signf(real_velo);
            }
            // 速度很小，使用静摩擦补偿
            else
            {
                i_total = fric_static * CtrlUtils::Signf(real_velo);
            }
        }
        // 当系统希望能量减小，或不变时，实际上摩擦力在帮忙刹车，不进行补偿
        else
        {
            i_total = 0.0f;
        }
        
        // 低通滤波，防止突变
        i_total = lpf_fric.Filter(i_total);

        return i_total;
    };
};


class SquareInjector
{
    public:
    SquareInjector() {};
    float amplitude = 0.0f;     // 方波幅值
    float period = 1.0f;        // 方波周期 (s)

    void Init(float amp, float per)
    {
        amplitude = amp;
        period = per;
    }

    void InitHz(float amp, float freq_hz)
    {
        amplitude = amp;
        period = 1.0f / freq_hz;
    }

    float GetValue(float time_sec)
    {
        float phase = fmod(time_sec, period);
        if (phase < (period / 2.0f))
        {
            return amplitude;
        }
        else
        {
            return -amplitude;
        }
    }

    float AutoGetValue()
    {
        static float start_time = DWT_GetTimeline_Sec();
        float current_time = DWT_GetTimeline_Sec();
        return GetValue(current_time - start_time);
    }
};

template<uint8_t con_dim, uint8_t state_dim, uint8_t meas_dim>
class KalmanObserver
{
    public:
    KalmanObserver() {};
    
    Matrix<state_dim, 1> x;     // 状态向量
    Matrix<meas_dim, 1> y;      // 观测向量估计值 (Output Estimate)

    Matrix<con_dim, 1> u;       // 控制输入向量

    /// @brief 离散状态转移矩阵
    Matrix<state_dim, state_dim> F;
    /// @brief 离散控制输入矩阵
    Matrix<state_dim, con_dim> G;
    /// @brief 观测矩阵
    Matrix<meas_dim, state_dim> H;
    /// @brief 过程噪声协方差矩阵
    Matrix<state_dim, state_dim> Q;
    /// @brief 测量噪声协方差矩阵
    Matrix<meas_dim, meas_dim> R; // 必须正定
    /// @brief 估计误差协方差矩阵
    Matrix<state_dim, state_dim> P;

    /**
     * @brief 卡尔曼滤波迭代步骤
     * @param control_input 当前控制输入
     * @param measurement 当前传感器的实际测量值
     */
    void Observe(const Matrix<con_dim, 1>& control_input, const Matrix<meas_dim, 1>& measurement)
    {
        // 首先进行预测，也就是状态转移
        x = F * x + G * u;
        y = H * x;
        P = F * P * F.transpose() + Q;

        // 获得新息
        Matrix<meas_dim, 1> y_tilde = measurement - y;

        // 计算新息协方差: S = H * P * H^T + R
        Matrix<meas_dim, meas_dim> S = H * P * H.transpose() + R;

        // 岭回归修正
        S = S + Matrix<meas_dim, meas_dim>::identity() * 1e-6f;
        
        // 计算 S 的逆矩阵 S_inv
        Matrix<meas_dim, meas_dim> S_inv;
        
        // 尝试求逆
        if (S.inverse(S_inv))       // 求逆成功，执行更新
        {
            // 计算卡尔曼增益: K = P * H^T * S^-1
            Matrix<state_dim, meas_dim> K = P * H.transpose() * S_inv;

            // 更新状态估计: x_k|k = x_k|k-1 + K * y_tilde
            x = x + K * y_tilde;

            // 更新误差协方差: P_k|k = (I - K * H) * P_k|k-1
            Matrix<state_dim, state_dim> I = Matrix<state_dim, state_dim>::identity();
            P = (I - K * H) * P;
        }
        else
        {
            // 求逆失败 (矩阵奇异)
            // 严重警告：通常意味着 R 设置过小，或者系统进入了不可观测状态。
            // 策略：跳过本次更新，仅信任模型预测。
            // 这样可以防止 x 变成 NaN 或 Inf，保证机器人不会失控。
        }
    }
};


class IVIdentifier {
public:
    /**
     * @brief 构造函数
     * @param b0_nominal  ADRC控制器当前设置的b0值
     * @param fs          采样频率 (Hz)，例如 1000
     * @param cut_freq    高通滤波截止频率 (Hz)，用于去直流，建议 1.0 - 5.0
     * @param forget_time 协方差统计的"记忆时间" (秒)，建议 0.5 - 2.0
     */
    IVIdentifier(float b0_nominal, float fs, float cut_freq, float forget_time);

    /**
     * @brief 1kHz 周期调用更新函数
     * @param r_cmd  当前的参考指令 (速度或位置指令) -> 工具变量
     * @param u_ctrl 当前的控制量 (电流/力矩) -> 回归变量
     * @param z3_obs 当前ESO观测到的总扰动 z3 -> 观测变量
     */
    void Update(float r_cmd, float u_ctrl, float z3_obs);

    // 获取辨识出的 theta (即 b_real - b0)
    float GetTheta() const { return theta_iv; }

    // 获取辨识出的真实惯量 J (单位与 b0 定义一致)
    float GetEstimatedJ(float Kt = 1.0f) const;

    // 获取辨识过程的置信度/激发程度 (分母的大小)
    float GetExcitationLevel() const { return cov_self; }

    // 重置算法状态
    void Reset();

    // 参数
    float b0_;
    float lambda_;       // 遗忘因子 (0 < lambda < 1)
    float hpf_alpha_;    // 高通滤波器系数
    
    // 状态变量
    float theta_iv;     // 最终辨识参数
    float theta_iv_accum_; // 累计和，用于计算平均值
    float cov_cross;      // 分子累加器 (Cov(r, z3))
    float cov_self;      // 分母累加器 (Cov(r, u))

    float Kt_ = 0.01562;          // 转矩常数 (用于计算J)
    float J_hat_ = 0.0f;        // 估计的惯量

    float r_ac;
    float u_ac;
    float z3_ac;

    // 高通滤波器历史状态 (用于去均值)
    float last_r_in_, last_r_out_;
    float last_u_in_, last_u_out_;
    float last_z3_in_, last_z3_out_;

    // 辅助函数：一阶高通滤波
    float HighPassFilter(float input, float& last_in, float& last_out);
};





