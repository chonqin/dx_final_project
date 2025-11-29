#include "sentry_chassis_controller/kinematics.h"

namespace sentry_chassis_controller {
    /*
        Inverse_solution
        逆运动学：将底盘的速度转化为轮速和转向角度
        输入： 底盘整体线速度vx, vy和角速度omega
        输出： 四个轮子的速度wheel_speed和转向角度steering_angle
    */
    /*
        forward_solution：
        正运动学：将轮速和转向角度转化为底盘的速度
        输入： 四个轮子的速度wheel_speed和转向角度steering_angle
        输出： 底盘线速度vx, vy和角速度omega
    */        
    void Inverse_solution(double vx, double vy, double omega,double wheel_base_, 
                            double wheel_track_ , double wheel_radius_,
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
    void test_inverse(double vx, double vy, double omega,double wheel_base_, 
                            double wheel_track_ , double wheel_radius_,
                            std::array<double, 4> &wheel_speed,
                            std::array<double, 4> &steering_angle) {
        // 调用逆运动学函数计算轮速和转向角度，并存储在对应数组中
        // 这里的vx, vy, omega是从cmd_vel话题接收到的 
        Inverse_solution(vx, vy, omega, wheel_base_, wheel_track_, wheel_radius_, wheel_speed, steering_angle);
        // 输出解算结果
        ROS_INFO("解算得到: wheel_speed=[%.2f, %.2f, %.2f, %.2f], steering_angle=[%.2f, %.2f, %.2f, %.2f]",
                 wheel_speed[0], wheel_speed[1], wheel_speed[2], wheel_speed[3],
                 steering_angle[0], steering_angle[1], steering_angle[2], steering_angle[3]);
    }
    void forward_solution(const std::array<double, 4> &wheel_speed,
                            const std::array<double, 4> &steering_angle,
                            double wheel_base_, double wheel_track_ , double wheel_radius_,
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