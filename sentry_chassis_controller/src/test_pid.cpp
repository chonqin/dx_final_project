#include "sentry_chassis_controller/test_pid.h"
/*
    此文件包含了测试pid功能的函数
    用于验证代码的正确性和功能实现
    在主控器里面调用测试函数来展示功能
    包含：
        测试四个驱动轮子pid和dynamic_reconfigure
        测试四个转向舵轮pid和dynamic_reconfigure
        ...
*/
namespace sentry_chassis_controller {
    /*
        测试四个驱动轮子pid
        输入:四个轮子的关节句柄数组，四个轮子的pid数组，
            四个轮子的目标速度发布器数组，四个轮子的实际速度发布器数组，控制周期
    */
    void test_wheels_pid(std::array<hardware_interface::JointHandle, 4>& wheel_joints,
                         std::array<control_toolbox::Pid, 4>& wheel_pids,
                         std::array<ros::Publisher, 4>& wheel_target_pub,
                         std::array<ros::Publisher, 4>& wheel_actual_pub,
                         const ros::Duration& period){                   
        const double target_velocity = 8.0; // 目标速度
        std_msgs::Float64 msg;
        // 对每个轮子进行PID控制测试，i为轮子索引，索引0-3统一。
        //for (size_t i = 0; i < 3; ++i) {
            // 获取当前速度
            size_t i = 0;
            double current_velocity = wheel_joints[i].getVelocity();
            double error = target_velocity - current_velocity;
            // 计算控制输出
            double output = wheel_pids[i].computeCommand(error, period);
            static int count = 0;
            if (count++ % 20 == 0) {
                ROS_WARN("=== PID 诊断 ===");
                ROS_WARN("周期 period: %.6f 秒", period.toSec());
                ROS_WARN("误差 error: %.4f rad/s", error);
                ROS_WARN("输出 output: %.4f N·m", output);
            }
            // 应用控制输出
            wheel_joints[i].setCommand(output);
            // 发布目标速度和实际速度
            msg.data = target_velocity;
            wheel_target_pub[i].publish(msg);

            msg.data = current_velocity;
            wheel_actual_pub[i].publish(msg);
        //}
    }
    /*
        测试四个转向舵轮pid
        输入：四个轮子的关节句柄数组，四个轮子的pid数组，
            四个轮子的目标角度发布器数组，四个轮子的实际角度发布器数组，控制周期
    */                     
    void test_pivots_pid(std::array<hardware_interface::JointHandle, 4>& pivot_joints,
                         std::array<control_toolbox::Pid, 4>& pivot_pids,
                         std::array<ros::Publisher, 4>& pivot_target_pub,
                         std::array<ros::Publisher, 4>& pivot_actual_pub,
                         const ros::Duration& period){
        const double target_angle = 3.14/2.0; // 目标角度
        std_msgs::Float64 msg;
        // 对每个轮子进行PID控制测试，i为轮子索引，索引0-3统一。
        for (size_t i = 0; i < 3; ++i) {
            // 获取当前速度
            double current_positon = pivot_joints[i].getPosition();
            double error = target_angle - current_positon;
            // 计算控制输出
            double output = pivot_pids[i].computeCommand(error, period);
            // 应用控制输出
            pivot_joints[i].setCommand(output);
            // 发布目标速度和实际速度
            msg.data = target_angle;
            pivot_target_pub[i].publish(msg); 

            msg.data = current_positon;
            pivot_actual_pub[i].publish(msg);
        }                    
    }
    
    


}