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
    void pid_control(std::array<hardware_interface::JointHandle, 4>& wheel_joints,
                        std::array<hardware_interface::JointHandle, 4>& pivot_joints,
                        std::array<double, 4> &wheel_speed,
                        std::array<double, 4> &steering_angle,
                        std::array<control_toolbox::Pid, 4>& wheel_pids,
                        std::array<control_toolbox::Pid, 4>& pivot_pids,
                        std::array<ros::Publisher, 4>& wheel_target_pub,
                        std::array<ros::Publisher, 4>& wheel_actual_pub,
                        std::array<ros::Publisher, 4>& pivot_target_pub,
                        std::array<ros::Publisher, 4>& pivot_actual_pub,
                        const ros::Duration& period){

        std_msgs::Float64 msg;
        for (size_t i = 0; i < 4; i++) {
            // pid 控制驱动轮速度
            double current_velocity = wheel_joints[i].getVelocity();
            double error = wheel_speed[i] - current_velocity;
            double output = wheel_pids[i].computeCommand(error, period);
            wheel_joints[i].setCommand(output);
            // 发布目标速度和实际速度
            msg.data = wheel_speed[i];
            wheel_target_pub[i].publish(msg);

            msg.data = current_velocity;
            wheel_actual_pub[i].publish(msg);
        }                    
        for( size_t i = 0; i < 4; i++) {
            // pid 控制转向舵轮
            double current_positon = pivot_joints[i].getPosition();
            double error = steering_angle[i] - current_positon;
            double output = pivot_pids[i].computeCommand(error, period);
            pivot_joints[i].setCommand(output);
            // 发布目标速度和实际速度
            msg.data = steering_angle[i];
            pivot_target_pub[i].publish(msg); 

            msg.data = current_positon;
            pivot_actual_pub[i].publish(msg);
        }
    }
    /*
        测试四个驱动轮子pid
        输入:四个轮子的关节句柄数组，四个轮子的pid数组，
            四个轮子的目标速度发布器数组，四个轮子的实际速度发布器数组，控制周期
    */
    void test_wheels_pid(std::array<hardware_interface::JointHandle, 4>& wheel_joints,
                         std::array<control_toolbox::Pid, 4>& wheel_pids,
                         std::array<ros::Publisher, 4>& wheel_target_pub,
                         std::array<ros::Publisher, 4>& wheel_actual_pub,
                         double target_,
                         const ros::Duration& period){                   
        //const double target_velocity = 8.0; // 目标速度
        std_msgs::Float64 msg;
        // 对每个轮子进行PID控制测试，i为轮子索引，索引0-3统一。
        for (size_t i = 0; i < 4; i++) {
            // 获取当前速度
            double current_velocity = wheel_joints[i].getVelocity();
            double error = target_ - current_velocity;
            // 计算控制输出
            double output = wheel_pids[i].computeCommand(error, period);
            // 应用控制输出
            wheel_joints[i].setCommand(output);
            // 发布目标速度和实际速度
            msg.data = target_;
            wheel_target_pub[i].publish(msg);

            msg.data = current_velocity;
            wheel_actual_pub[i].publish(msg);
        }
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
                         double target_,
                         const ros::Duration& period){
        //const double target_angle = 3.14/2.0; // 目标角度
        std_msgs::Float64 msg;
        // 对每个轮子进行PID控制测试，i为轮子索引，索引0-3统一。
        for (size_t i = 0; i < 4; i++) {
            // 获取当前速度
            double current_positon = pivot_joints[i].getPosition();
            double error = target_ - current_positon;
            // 计算控制输出
            double output = pivot_pids[i].computeCommand(error, period);
            // 应用控制输出
            pivot_joints[i].setCommand(output);
            // 发布目标速度和实际速度
            msg.data = target_;
            pivot_target_pub[i].publish(msg); 

            msg.data = current_positon;
            pivot_actual_pub[i].publish(msg);
        }                    
    }
    
    


}