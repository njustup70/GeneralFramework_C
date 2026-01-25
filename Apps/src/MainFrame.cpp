#include "MainFrame.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "Monitor.hpp"
#include "std_cpp.h"

/**     测试用      **/
#include "signator.hpp"


StateCore& core = StateCore::GetInstance();
Monitor& monit = Monitor::GetInstance();


StateGraph example_graph("graph_name");
void Action_of_Dege(StateCore* core);


// 测试框架用
MotorDJI test_motor_0;

float K_t = 0.01562f; // 转矩常数
float J = 0.000352925847;  // 转动惯量
// float J = 0.00033;  // 转动惯量
float dt = 0.001f; // 采样时间间隔
float B = 1.56e-4f; // 阻尼系数
// float B = 0.0f; // 阻尼系数

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

    test_motor_0.Init(&hcan1, 1, ADRC_SpeedControl);
    test_motor_0.motor_adrc.Init(ADRC::Sec_Ord, 15.0f, 2.7f, J, B, K_t, dt, 18.0f);
    test_motor_0.g_Identifier.b0_ = K_t / J * 5.0f;          // 离散化b0

    test_motor_0.Dynamicle(Dynamic); // 设置为稳定辨识模式

    // test_motor_0.motor_adrc.InitKF(J, B, K_t, dt, 10.0f);
    // test_motor_0.motor_adrc.InitLESO(80.0f, J, B, K_t, dt, 15.0f);
    
    
    // test_motor_0.motor_adrc.Init(ADRC::Sec_Ord, 21.0f, 5.25f, J, B, K_t, dt, 15.0f);
    // test_motor_0.speed_pid.Init(0.015f, 0.0f, 0.0f, 0.0f);
    
    // 配置跟踪器
    monit.Track(test_motor_0.motor_adrc.debug_current);      // 速度跟踪曲线
    monit.Track(test_motor_0.motor_adrc.debug_TL);

    monit.Track(test_motor_0.motor_adrc.debug_omega);        // 位置跟踪曲线
    monit.Track(test_motor_0.measure.total_angle);

    monit.Track(test_motor_0.g_Identifier.rho_ru);                 // 电机的转动惯量
    monit.Track(test_motor_0.g_Identifier.J_hat_);

    monit.Perflize();  // 切换高性能模式
}

void Action_of_Dege(StateCore* core)
{
    // 简并模式的动作
}