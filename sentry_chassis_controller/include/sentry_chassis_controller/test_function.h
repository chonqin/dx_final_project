#ifndef SENTRY_CHASSIS_CONTROLLER_TEST_FUNCTION_H
#define SENTRY_CHASSIS_CONTROLLER_TEST_FUNCTION_H
/*
    此文件包含了测试各个功能的函数声明
*/
#include <ros/ros.h>
#include <array>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>

namespace sentry_chassis_controller {

    /*测试驱动轮子pid*/
    void test_wheels_pid(std::array<hardware_interface::JointHandle, 4>& wheel_joints,
                         std::array<control_toolbox::Pid, 4>& wheel_pids,
                         const ros::Duration& period);
    /*测试转向舵轮pid*/                     
    void test_pivots_pid(std::array<hardware_interface::JointHandle, 4>& pivot_joints,
                         std::array<control_toolbox::Pid, 4>& pivot_pids,
                         const ros::Duration& period);    
}

#endif // SENTRY_CHASSIS_CONTROLLER_TEST_FUNCTION_H