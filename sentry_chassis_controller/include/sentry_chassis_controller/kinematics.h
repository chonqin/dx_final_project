#ifndef SENTRY_CHASSIS_CONTROLLER_KINEMATICS_H
#define SENTRY_CHASSIS_CONTROLLER_KINEMATICS_H

#include <geometry_msgs/Twist.h>
#include <array>
#include <cmath>
#include <ros/ros.h>
namespace sentry_chassis_controller {
    /*
            Inverse_solutio
            逆运动学：将底盘的速度转化为轮速和转向角度
            输入： 底盘线速度vx, vy和角速度omega
            输出： 四个轮子的速度wheel_speeds和转向角度steering
    */
    /*
            forward_solution
            正运动学：将轮速和转向角度转化为底盘的速度
            输入： 四个轮子的速度wheel_speeds和转向角度steering
            输出： 底盘线速度vx, vy和角速度omega
    */
    void Inverse_solution(double vx, double vy, double omega,double wheel_base_, 
                            double wheel_track_ , double wheel_radius_,
                            std::array<double, 4> &wheel_speed,
                            std::array<double, 4> &steering_angle) ;
    /*逆运动学测试函数，用于update里调用测试*/    
    void test_inverse(double vx, double vy, double omega,double wheel_base_, 
                            double wheel_track_ , double wheel_radius_,
                            std::array<double, 4> &wheel_speed,
                            std::array<double, 4> &steering_angle);

    void forward_solution(const std::array<double, 4> &wheel_speed,
                        const std::array<double, 4> &steering_angle,
                        double wheel_base_, double wheel_track_ , double wheel_radius_,
                        double &vx, double &vy, double &omega);      
 

}
#endif // SENTRY_CHASSIS_CONTROLLER_KINEMATICS_H