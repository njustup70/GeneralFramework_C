#include "MainFrame.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "Monitor.hpp"
#include "std_cpp.h"

/**     测试用      **/
#include "controller.hpp"


StateCore& core = StateCore::GetInstance();
Monitor& monit = Monitor::GetInstance();


StateGraph example_graph("graph_name");
void Action_of_Dege(StateCore* core);


// 测试框架用
MotorDJI test_motor_0;

float K_t = 0.01562f; // 转矩常数
float J = 0.000425f; // 转动惯量
float dt = 0.002f; // 采样时间间隔
float B = 1.56e-4f; // 阻尼系数


/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
    // 配置状态图为简并模式
    example_graph.Degenerate(Action_of_Dege);
    // 向状态机核心注册
    core.RegistGraph(example_graph);

    test_motor_0.Init(&hcan1, 1, Speed_Control);

    // 配置扩张状态卡尔曼观测器（扩张T_L）
    test_motor_0.kalman_ob.F = {1.0f, dt, 0.0f,
                                0.0f, 1.0f - (B / J * dt), -(1 / J * dt),
                                0.0f, 0.0f, 1.0f};

    test_motor_0.kalman_ob.G = {0.0f, K_t / J * dt, 0.0f};

    test_motor_0.kalman_ob.H = {1.0f, 0.0f, 0.0f};

    test_motor_0.kalman_ob.Q = {1e-5f, 0.0f, 0.0f,
                                 0.0f, 0.5f, 0.0f,
                                 0.0f, 0.0f, 10.0f};

    test_motor_0.kalman_ob.R = {1e-5f};
    
    // 配置跟踪器
    monit.Track(test_motor_0.measure.speed_rpm);
    monit.Track(test_motor_0.kalman_rpm); // 卡尔曼估计的速度，rad/s转rpm  
    monit.Perflize();  // 切换高性能模式
}

void Action_of_Dege(StateCore* core)
{
    // 简并模式的动作
}