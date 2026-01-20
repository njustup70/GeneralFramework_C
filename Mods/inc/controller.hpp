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

template<uint8_t con_dim, uint8_t state_dim, uint8_t meas_dim>
class StateSpaceModel
{
    Matrix<state_dim, 1> x;     // 状态向量
    Matrix<meas_dim, 1> y;      // 测量向量
    Matrix<con_dim, 1> u;       // 控制输入向量

    Matrix<state_dim, state_dim> A;         // 状态转移矩阵
    Matrix<state_dim, con_dim> B;           // 输入矩阵
    Matrix<meas_dim, state_dim> C;          // 观测矩阵
};

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

    void Init(float _r, float _dt)
    {
        r = _r;
        dt = _dt;
        v1 = 0.0f;
        v2 = 0.0f;
        v3 = 0.0f;
    }

    void Update(float target_input)
    {
        float error = v1 - target_input;
        
        // 三阶系统的特征方程系数 (r^3, 3r^2, 3r) 对应二项式系数 1,3,3,1
        // 计算“加加速度” (Jerk)
        float jerk = -r*r*r * error - 3.0f*r*r * v2 - 3.0f*r * v3;
        
        // 欧拉积分
        v1 += v2 * dt;
        v2 += v3 * dt;
        v3 += jerk * dt; // v3 才是输出给外环的平滑加速度
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


class LESO
{
public:
    // 状态变量
    // z1: 估计角度 (rad)
    // z2: 估计角速度 (rad/s)
    // z3: 估计的总扰动加速度 (rad/s^2), 包含了 T_L/J 和 摩擦/J
    float z1 = 0.0f; 
    float z2 = 0.0f;
    float z3 = 0.0f;

    float _base_omega_o = 30.0f; // 基础观测器带宽 (rad/s)

    LowPassFilter eta_filt;

    /**
     * @brief 初始化 ESO
     * @param _omega_o 观测器带宽 (rad/s)。越大越硬，但也越吵。建议从 100~300 开始尝试。
     * @param _J 转动惯量 (kg*m^2)
     * @param _Kt 转矩常数 (N*m/A)
     * @param _dt 控制周期 (s)
     */
    void Init(float _omega_o, float _J, float _Kt, float _dt)
    {
        J = _J;
        dt = _dt;
        // 计算控制增益 b0 = Kt / J
        // 这是将电流(A)转换为角加速度(rad/s^2)的系数
        b0 = _Kt / _J;
        _base_omega_o = _omega_o;

        eta_filt.InitHz(8.0f, dt); 

        // 根据带宽计算 ESO 增益 (极点配置在 -omega_o)
        // L = [3wo, 3wo^2, wo^3]
        beta1 = 3.0f * _omega_o;
        beta2 = 3.0f * _omega_o * _omega_o;
        beta3 = _omega_o * _omega_o * _omega_o;
    }

    void UpdateOmega_O(float _omega_o)
    {
        // 根据带宽计算 ESO 增益 (极点配置在 -omega_o)
        // L = [3wo, 3wo^2, wo^3]
        beta1 = 3.0f * _omega_o;
        beta2 = 3.0f * _omega_o * _omega_o;
        beta3 = _omega_o * _omega_o * _omega_o;
    }

    float alpha = 0.5f;
    float delta = 0.01f; 
    // 定义非线性误差处理函数
    float GetNonlinearError(float raw_err)
    {
        float abs_e = (raw_err > 0) ? raw_err : -raw_err;
        float sign_e = (raw_err > 0) ? 1.0f : (raw_err < 0 ? -1.0f : 0.0f);

        if (abs_e > delta) {
            // 非线性段：注意 C++ 中 powf(0, alpha) 会有开销，这里已做判断
            return powf(abs_e, alpha) * sign_e;
        } else {
            // 线性段：为了保证函数连续性，斜率为 1 / (delta^(1-alpha))
            float linear_gain = 1.0f / powf(delta, 1.0f - alpha);
            return raw_err * linear_gain;
        }
    }

    /**
     * @brief 更新观测器状态
     * @param u_current 实际给电机的电流 (A) -> last_u
     * @param y_measure 测量到的位置 (rad) -> measure_theta_total
     */
    void Observe(float u_current, float y_measure)
    {
        // 1. 计算观测误差 (e = z1 - y)
        float e = z1 - y_measure;

        // 2. 更新状态 (欧拉积分)
        // z1_dot = z2 - beta1 * e
        z1 += (z2 - beta1 * e) * dt;

        // z2_dot = z3 - beta2 * e + b0 * u
        z2 += (z3 - beta2 * e + b0 * u_current) * dt;

        // z3_dot = -beta3 * e
        // z3 += (-beta3 * e) * dt;
        float eta_f = eta_filt.Filter(GetDynamicEta(e, 0.8f));
        z3 += (-beta3 * e - eta * z3) * dt;
    }

    // 获取估计的外部扰动转矩 (N*m)
    // 注意：ADRC 把摩擦力也视为扰动的一部分，所以这里返回的是总扰动
    float GetEstimatedDisturbanceTorque()
    {
        return z3 * J;
    }

    float GetDynamicEta(float error, float wc)
    {
        if (fabs(error) >= wc)
        {
            return 0.0f;
        }
        else if (fabs(error) > 0)
        {
            return eta * (1.001f - fabs(error) / wc);
        }
        else
        {
            return eta;
        }
            
    }

    float beta1, beta2, beta3; // 观测器增益
    float b0; // 系统增益
    float J;  // 惯量
    float dt; // 周期

    float eta = 0.024f; // 衰减系数
};





/**
 * @brief 基于物理模型卡尔曼滤波的自抗扰控制器 (Physics-based ADRC)
 * @details 结构：
 * 1. 观测器：Kalman Filter (状态: theta, omega, T_load)
 * 2. 控制器：P 控制 + 摩擦前馈 + 扰动前馈
 */
class MotorADRC
{
public:
    // 物理参数
    float J;      // 转动惯量 (kg*m^2)
    float B;      // 粘滞摩擦系数 (N*m*s/rad)
    float Kt;     // 转矩常数 (N*m/A)
    float max_current; // 最大电流 (A)

    float coeff_feedforward = 0.55f; // 负载估计系数

    // 卡尔曼观测器实例
    // 1维控制输入(current), 3维状态(theta, omega, T_L), 1维测量(theta)
    KalmanObserver<1, 3, 1> kalman_ob;

    // LESO 实例
    LESO leso;

    SquareInjector square_injector;

    // 输入微分跟踪器
    LinearTD input_td;
    LinearTD_3rd input_td_3rd;

    // 低通滤波器
    LowPassFilter lpf_w, lpf_TL;

    // 控制器参数
    float kp;     // 速度环比例增益
    float kd;     // 【新增】 位置微分增益 (kd)

    FrictionCompensator fric_comp;

    typedef enum ObType
    {
        LESO_SPD,
        LESO_POS,
        KF,
    };

    ObType ob_t;

    /**
     * @brief 初始化控制器
     * @param _J 转动惯量
     * @param _B 粘滞摩擦
     * @param _Kt 转矩常数
     * @param _dt 控制周期 (s)
     */
    void InitKF(float _J, float _B, float _Kt, float _dt, float _max_curr)
    {
        J = _J; B = _B; Kt = _Kt;
        max_current = _max_curr;

        kalman_ob.F = {1.0f, _dt, 0.0f,
                        0.0f, 1.0f - (B / J * _dt), -(1 / J * _dt),
                        0.0f, 0.0f, 1.0f};

        kalman_ob.G = {0.0f, Kt / J * _dt, 0.0f};

        kalman_ob.H = {1.0f, 0.0f, 0.0f};

        kalman_ob.Q = {1e-6f, 0.0f, 0.0f,
                        0.0f, 0.5f, 0.0f,
                        0.0f, 0.0f, 1e-1f};

        kalman_ob.R = {1e-4f};

        // 初始化 P (初始不确定度)
        kalman_ob.P = Matrix<3, 3>::identity() * 100.0f;

        // 控制器增益默认值
        kp = 60.0f; // 这里的 kp 物理意义是带宽，单位 rad/s
        
        ob_t = KF;
    }

    void InitLESO(float wo, float _J, float _B, float _Kt, float _dt, float _max_curr)
    {
        J = _J; B = _B; Kt = _Kt;
        max_current = _max_curr;
        leso.Init(wo, J, Kt, _dt);
        input_td.Init(10.0f, _dt); 
        lpf_w.Init(80.0f, _dt);

        kp = 16.0f; // 这里的 kp 物理意义是带宽，单位 rad/s

        ob_t = LESO_SPD;
    }

    void InitLESO_POS(float wo, float wc, float _J, float _B, float _Kt, float _dt, float _max_curr)
    {
        J = _J; B = _B; Kt = _Kt;
        max_current = _max_curr;
        
        // 1. 初始化 ESO (3阶)
        leso.Init(wo, J, Kt, _dt);
        
        // 2. 初始化 TD
        // r (跟踪因子) 建议设置为 wc 的倍数，或者根据物理系统最大加速度限制设定
        // 这里设为 wc 稍大一点，保证指令不拖后腿
        input_td_3rd.Init(12.5f, _dt); 

        // 3. 计算控制器增益 (根据 wc)
        // 对于二阶系统，临界阻尼配置 (zeta = 1)
        // kp = wc^2
        // kd = 2 * zeta * wc = 2 * wc
        kp = wc * wc;
        kd = 2.0f * 0.75f * wc;

        lpf_TL.InitHz(42.0f, _dt); // 负载转矩的低通滤波器

        fric_comp.Init(0.25f, 0.05f, 40.0f, 120.0f);

        square_injector.InitHz(0.15f, 200.0f);

        ob_t = LESO_POS; // 默认为位置模式，如果是速度模式需单独覆盖
    }

    float Calc_KF(float target_speed, float measure_theta_total)
    {

        // 提取状态
        float theta_hat = kalman_ob.x(0, 0);
        float omega_hat = kalman_ob.x(1, 0);        // 这是极其平滑、无延迟的速度！
        float T_L_hat   = kalman_ob.x(2, 0);        // 这是估计出的外部负载

        // === Step 2: 物理前馈 + 状态反馈控制 ===
        // 我们的目标是让 motor_torque = J*acc + B*omega + T_L
        // 也就是 i * Kt = J * (kp * error) + B * omega + T_L
        
        // 2.1 摩擦补偿 (Viscous Friction Compensation)
        float i_fric = (B * omega_hat) / Kt;
        
        // 2.2 扰动/负载补偿 (Disturbance Rejection) -> ADRC 的灵魂
        float i_load = T_L_hat / Kt * coeff_feedforward;
        
        // 2.3 误差反馈 (P Control) -> 驱动惯量 J 加速
        float error = target_speed - omega_hat;
        float i_acc = (J * kp * error) / Kt;

        // 总电流
        float i_total = i_acc + i_fric + i_load;

        // === Step 3: 限幅与更新 ===
        if (i_total > max_current) i_total = max_current;
        else if (i_total < -max_current) i_total = -max_current;

        
        // Debug 接口：你可以把这些值打印出来看波形
        debug_omega = omega_hat;
        debug_TL = T_L_hat;

        return i_total;
    }

    float Calc_LESO(float target_speed, float measure_theta_total)
    {
        // === 观测器更新严禁在本循环进行！  ===
        // 输入微分跟踪
        input_td.Update(target_speed);

        // 提取状态
        // float theta_hat = leso.z1; // 如果需要位置估计
        float omega_hat = leso.z2;    // 估计速度

        float omega_hat_filtered = lpf_w.Filter(omega_hat);
        
        // 获取扰动转矩 (包含 外部负载 + 粘滞摩擦 + 未建模动态)
        float T_dist_hat = leso.GetEstimatedDisturbanceTorque();

        // === Step 2: 控制律 ===
        // ADRC 的美学：不管是什么扰动（摩擦B还是手捏T_L），统统用 i_dist 抵消掉
        
        // 2.1 扰动补偿 (包含摩擦补偿)
        // 直接用估计出的总扰动除以 Kt 换算成电流
        float i_dist = -(T_dist_hat / Kt); 
        i_dist = lpf_TL.Filter(i_dist); // 对扰动电流进行低通滤波，防止噪声影响控制

        // 2.2 误差反馈 (P Control) -> 驱动纯惯量模型，注意使用微分跟踪器的输出
        float error = input_td.v1 - omega_hat_filtered;
        float i_acc = (J * kp * error) / Kt;

        // 2.3 【新增】加速度前馈 (Acceleration Feedforward)
        // 既然 TD 告诉我们当前需要 v2 的加速度，那根据牛顿第二定律，
        // 我们直接预先给它这个力！这能极大减小滞后。
        float i_feedforward = (J * input_td.v2) / Kt * coeff_feedforward;

        // 总电流 = 加速电流 + 抵消扰动的电流
        // 注意：这里不需要单独算 i_fric 了，因为 leso.z3 已经自动把摩擦力包含在内了！
        // 除非你想要前馈摩擦，否则让 ESO 自己去学摩擦是更鲁棒的做法。
        float i_total = i_acc + i_dist + i_feedforward;

        // === Step 3: 限幅与更新 ===
        if (i_total > max_current) i_total = max_current;
        else if (i_total < -max_current) i_total = -max_current;

        // Debug
        debug_omega = omega_hat_filtered * 60.0f / (2.0f * 3.1415926f); // 转换为rpm;
        debug_TL = T_dist_hat;

        return i_total;
    }

    float fric_viscous_width = 2.0f;

    float Calc_LESO_POS(float target_pos, float measure_theta_total)
    {
        // 首先更新微分跟踪器，获取目标轨迹（包括位置、速度和加速度）
        input_td_3rd.Update(target_pos);

        // 2. 提取 ESO 状态 (需要在外部循环中先调用 leso.Observe)
        float pos_hat = leso.z1;      // 估计位置
        float vel_hat = leso.z2;      // 估计速度 (低噪声)
        float dist_hat = leso.z3;     // 估计的总扰动加速度 (包含负载、摩擦、模型误差)

        // 3. 计算误差
        // 这里的误差是用 TD生成的"安排值" 减去 ESO的"估计值"
        float err_p = input_td_3rd.v1 - pos_hat;
        float err_v = input_td_3rd.v2 - vel_hat;

        // 4. PD 控制器 (u0 - 虚拟控制量: 期望加速度)
        // u0 = kp * ep + kd * ev + acc_feed 
        float u0 = kp * err_p + kd * err_v + input_td_3rd.v3;

        // 5. 扰动补偿 (Disturbance Rejection)
        // 核心公式: u = (u0 - f_hat) / b0
        // dist_hat 是总扰动加速度，除以 b0 转换为电流
        float i_des = lpf_TL.Filter((u0 - dist_hat) / leso.b0);

        // 零速摩擦补偿
        // float i_fric = -fric_comp.GetSimpleFriction(vel_hat);
        // float i_fric = 0.0f;

        // i_des += i_fric;


        // 方波注入
        i_des += square_injector.AutoGetValue();
        
        // 6. 限幅
        if (i_des > max_current) i_des = max_current;
        else if (i_des < -max_current) i_des = -max_current;

        // Debug 赋值
        debug_omega = vel_hat * 60.0f / (2.0f * 3.1415926f); // 转换为rpm;
        debug_TL = dist_hat / leso.b0 * 1000.0f; // 转换为 mA
        debug_ltd_targ_omega = input_td_3rd.v2  * 60.0f / (2.0f * 3.1415926f); // 目标速度
        debug_ltd_targ_pos = input_td_3rd.v1  * 8192.0f / (2.0f * 3.1415926f);   // 目标位置

        debug_current = i_des * 1000.0f; // 转换为 mA

        return i_des;
    }


    /**
     * @brief 计算控制输出
     * @param target_speed 目标速度 (rad/s)
     * @param measure_theta_total 累积测量的绝对角度 (rad, 需处理过零)
     */
    float Calc(float target_speed, float measure_theta_total)
    {
        if (ob_t == LESO_SPD)
        {
            return Calc_LESO(target_speed, measure_theta_total);
        }
        else if (ob_t == LESO_POS)
        {
            return Calc_LESO_POS(target_speed, measure_theta_total);
        }
        else // KF
        {
            return Calc_KF(target_speed, measure_theta_total);
        }
    }

    // 调试用的变量
    float debug_omega;
    float debug_TL;

    float debug_ltd_targ_omega;
    float debug_ltd_targ_pos;

    float debug_current;
};