#pragma once
#include "linear_math.hpp"

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

