#ifndef SENTRY_CHASSIS_CONTROLLER_KINEMATICS_H
#define SENTRY_CHASSIS_CONTROLLER_KINEMATICS_H

#include <geometry_msgs/Twist.h>
#include <array>
#include <cmath>

namespace sentry_chassis_controller {
    // 运动学Kinematics类,包含逆运动学和正运动学函数
    class Kinematics {
        public:
            Kinematics(double wheel_base, double wheel_track);
            ~Kinematics() = default;
            /*
                Kinematics::Inverse_solution
                逆运动学：将底盘的速度转化为轮速和转向角度
                输入： 底盘线速度vx, vy和角速度omega
                输出： 四个轮子的速度wheel_speeds和转向角度steering
            */
            /*
                Kinematics::forward_solution
                正运动学：将轮速和转向角度转化为底盘的速度
                输入： 四个轮子的速度wheel_speeds和转向角度steering
                输出： 底盘线速度vx, vy和角速度omega
            */
            void Inverse_solution(double vx, double vy, double omega,
                                   std::array<double, 4> &wheel_speed,
                                   std::array<double, 4> &steering_angle);
            void forward_solution(const std::array<double, 4> &wheel_speed,
                                   const std::array<double, 4> &steering_angle,
                                   double &vx, double &vy, double &omega);    
        private:
            double wheel_base_;  // 轴距
            double wheel_track_; // 轮距        

    };  
    

}
#endif // SENTRY_CHASSIS_CONTROLLER_KINEMATICS_H