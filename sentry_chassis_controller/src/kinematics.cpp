#include "sentry_chassis_controller/kinematics.h"

namespace sentry_chassis_controller {
    Kinematics::Kinematics(double wheel_base, double wheel_track ,double wheel_radius)
        : wheel_base_(wheel_base), wheel_track_(wheel_track), wheel_radius_(wheel_radius) {}
    /*
        Kinematics::Inverse_solution
        逆运动学：将底盘的速度转化为轮速和转向角度
        输入： 底盘整体线速度vx, vy和角速度omega
        输出： 四个轮子的速度wheel_speed和转向角度steering_angle
    */
    /*
        Kinematics::forward_solution：
        正运动学：将轮速和转向角度转化为底盘的速度
        输入： 四个轮子的速度wheel_speed和转向角度steering_angle
        输出： 底盘线速度vx, vy和角速度omega
    */        
    void Kinematics::Inverse_solution(double vx, double vy, double omega,
                                    std::array<double, 4> &wheel_speed,
                                    std::array<double, 4> &steering_angle) {
        // 计算解算所需投影点到车体中心的距离参数
        const double L = wheel_base_/2.0;
        const double W = wheel_track_/2.0;
        const double R = std::sqrt(L*L + W*W);
        const double a = std::sqrt(2) / 2.0 ;
        // 计算每个轮子的驱动速度和转向角度                              
        for(size_t i = 0; i < 4; i++) {
            wheel_speed[i] = std::sqrt((vx - a* omega * R) ^ 2 +
                                        (vy + a* omega * R) ^ 2) / wheel_radius_;
            steering_angle[i] = std::atan2(vx - a* omega * R, vy + a* omega * R);                         
        }
    }
    void Kinematics::forward_solution(const std::array<double, 4> &wheel_speed,
                                   const std::array<double, 4> &steering_angle,
                                   double &vx, double &vy, double &omega) {
        
        // 计算解算所需的L和W
        const double L = wheel_base_/2.0;
        const double W = wheel_track_/2.0;                            
        // 先计算每个轮子的速度分量
        // 左前轮 (索引 0)
        double vx_fl = wheel_speed[0] * std::cos(steering_angle[0]);
        double vy_fl = wheel_speed[0] * std::sin(steering_angle[0]);
        // 右前轮 (索引 1)
        double vx_fr = wheel_speed[1] * std::cos(steering_angle[1]);
        double vy_fr = wheel_speed[1] * std::sin(steering_angle[1]);
        // 左后轮 (索引 2) 
        double vx_bl = wheel_speed[2] * std::cos(steering_angle[2]);
        double vy_bl = wheel_speed[2] * std::sin(steering_angle[2]);
        // 右后轮 (索引 3)
        double vx_br = wheel_speed[3] * std::cos(steering_angle[3]);
        double vy_br = wheel_speed[3] * std::sin(steering_angle[3]);
        // 计算底盘的线速度和角速度
        vx = (vx_fl + vx_fr + vx_bl + vx_br) / 4.0;
        vy = (vy_fl + vy_fr + vy_bl + vy_br) / 4.0;
        omega = ((-vy_fl + vy_fr - vy_bl + vy_br) / (2.0 * W) +
                 (vx_fl + vx_fr - vx_bl - vx_br) / (2.0 * L)) / 2.0;                                     
    }                


}