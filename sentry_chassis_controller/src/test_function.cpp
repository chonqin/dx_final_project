#include "sentry_chassis_controller/test_function.h"
/*
    此文件包含了测试各个功能的函数
    用于验证代码的正确性和功能实现
    在主控器里面调用这些测试函数来展示功能
    包含：
        测试四个驱动轮子pid
        测试四个转向舵轮pid
        ...
*/
namespace sentry_chassis_controller {
    /*测试四个驱动轮子pid*/
    void test_wheels_pid(std::array<hardware_interface::JointHandle, 4>& wheel_joints,
                         std::array<control_toolbox::Pid, 4>& wheel_pids,
                         const ros::Duration& period){
        ROS_INFO("Testing wheels PID");                    
        const double target_velocity = 10.0; // 目标速度
        // 对每个轮子进行PID控制测试，i为轮子索引，索引0-3统一。
        for (size_t i = 0; i < wheel_joints.size(); ++i) {
            // 获取当前速度
            double current_velocity = wheel_joints[i].getVelocity();
            // 计算控制输出
            double output = wheel_pids[i].computeCommand(target_velocity-current_velocity, period);
            // 应用控制输出
            wheel_joints[i].setCommand(output);
        }
    }
    /*测试四个转向舵轮pid*/                     
    void test_pivots_pid(std::array<hardware_interface::JointHandle, 4>& pivot_joints,
                         std::array<control_toolbox::Pid, 4>& pivot_pids,
                         const ros::Duration& period){
        ROS_INFO("Testing pivot PID");
        const double target_angle = 3.14/2; // 目标角度
        // 对每个轮子进行PID控制测试，i为轮子索引，索引0-3统一。
        for (size_t i = 0; i < pivot_joints.size(); ++i) {
            // 获取当前速度
            double current_velocity = pivot_joints[i].getVelocity();
            // 计算控制输出
            double output = pivot_pids[i].computeCommand(target_angle-current_velocity, period);
            // 应用控制输出
            pivot_joints[i].setCommand(output);
        }                    
    }
    
    


}