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
            wheel_speed[i] = std::sqrt(std::pow(vx - a* omega * R, 2) +
                                    std::pow(vy + a* omega * R, 2)) / wheel_radius_;
            steering_angle[i] = std::atan2(vx - a* omega * R, vy + a* omega * R);                         
        }
    }
    void Kinematics::forward_solution(const std::array<double, 4> &wheel_speed,
                                   const std::array<double, 4> &steering_angle,
                                   double &vx, double &vy, double &omega) {
        // 计算每个轮子的线速度在车体坐标系下的分量
        std::array<double, 4> wx;
        std::array<double, 4> wy;

        const double L = wheel_base_/2.0;
        const double W = wheel_track_/2.0;
        const double R = std::sqrt(L*L + W*W);
        const double a = std::sqrt(2) / 2.0 ;

        for(size_t i = 0;i < 4 ; i++){
            wx[i] = wheel_speed[i] * wheel_radius_ * std::sin(steering_angle[i]);
            wy[i] = wheel_speed[i] * wheel_radius_ * std::cos(steering_angle[i]);
        }
        // 计算车体的线速度和角速度
        vx = (wx[0] + wx[1] + wx[2] + wx[3]) * wheel_radius_ / 4.0 ;
        vy = (wy[0] + wy[1] + wy[2] + wy[3]) * wheel_radius_ / 4.0 ;
        omega = (-wx[0]-wx[1]+wx[2]+wx[3]+wy[0]-wy[1]-wy[2]+wy[3]) 
                * a / R * wheel_radius_ / 4.0 ;
    }                


}