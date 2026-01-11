#include "MainFrame.hpp"
#include "System.hpp"
#include "Chassis.hpp"
#include "Monitor.hpp"
#include "std_cpp.h"


StateCore& core = StateCore::GetInstance();

StateGraph example_graph("graph_name");
void Action_of_Dege(StateCore* core);


// 测试框架用
MotorDJI test_motor_0;


/**
 * @brief 程序主入口
 * @warning 严禁阻塞
 */
void MainFrameCpp()
{
    // 配置状态图为简并模式
    example_graph.Degenerate(Action_of_Dege);

    test_motor_0.Init(&hcan1, 1, Speed_Control);

    // 向状态机核心注册
    core.RegistGraph(example_graph);
}

void Action_of_Dege(StateCore* core)
{
    // 简并模式的动作
}