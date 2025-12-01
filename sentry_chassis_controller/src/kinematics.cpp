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
    
        const double L = wheel_base_ / 2.0;
        const double W = wheel_track_ / 2.0;
    
        // 左手系：x向前，y向左
        // 索引0-3：左前，右前，左后，右后
        // URDF：x_offset=左右方向(正=右), y_offset=前后方向(正=前)
        double wheel_pos_x[4] = {W, W, -W, -W};   // x方向：左轮x=W，右轮x=-W
        double wheel_pos_y[4] = {L, -L, L, -L};   // y方向：前轮y=L，后轮y=-L
        
        // 计算每个轮子的驱动速度和转向角度
        for(size_t i = 0; i < 4; i++) {
            // 计算该轮子处的速度分量
            // 对于左手系：vx_wheel = vx - omega * y, vy_wheel = vy + omega * x
            double vx_wheel = vx - omega * wheel_pos_y[i];
            double vy_wheel = vy + omega * wheel_pos_x[i];
            
            // 计算轮速（速度大小）
            double wheel_velocity = std::sqrt(vx_wheel * vx_wheel + vy_wheel * vy_wheel);
            wheel_speed[i] = wheel_velocity / wheel_radius_;
            
            // 计算转向角（使用atan2(vy, vx)定义）
            steering_angle[i] = std::atan2(vy_wheel, vx_wheel);
        }
        ROS_INFO("逆运动学输入: vx=%.2f, vy=%.2f, omega=%.2f", vx, vy, omega);
        ROS_INFO("解算结果为：轮速(%.2f, %.2f, %.2f, %.2f), 转向角度(%.2f, %.2f, %.2f, %.2f)",
                      wheel_speed[0], wheel_speed[1], wheel_speed[2], wheel_speed[3],
                      steering_angle[0], steering_angle[1], steering_angle[2], steering_angle[3]);
    }
    void forward_solution(const std::array<double, 4> &wheel_speed,
                        const std::array<double, 4> &steering_angle,
                        double wheel_base_, double wheel_track_ , double wheel_radius_,
                        double &vx, double &vy, double &omega) {

        // 轮子位置
        const double L = wheel_base_ / 2.0;
        const double W = wheel_track_ / 2.0;
    
        // 索引0-3：左前，右前，左后，右后
        double wheel_pos_x[4] = {W, W, -W, -W};   // x方向：左轮x=W，右轮x=-W
        double wheel_pos_y[4] = {L, -L, L, -L};   // y方向：前轮y=L，后轮y=-L

        // 计算每个轮子的速度分量
        std::array<double, 4> vx_wheel;
        std::array<double, 4> vy_wheel;

        for(size_t i = 0; i < 4; i++){
            vx_wheel[i] = wheel_speed[i] * wheel_radius_ * std::cos(steering_angle[i]);
            vy_wheel[i] = wheel_speed[i] * wheel_radius_ * std::sin(steering_angle[i]);
        }
    
        // 计算底盘线速度（平均值）
        vx = (vx_wheel[0] + vx_wheel[1] + vx_wheel[2] + vx_wheel[3]) / 4.0;
        vy = (vy_wheel[0] + vy_wheel[1] + vy_wheel[2] + vy_wheel[3]) / 4.0;
    
        // 计算角速度
        // 利用：vx_wheel = vx - omega * y, vy_wheel = vy + omega * x
        // 可得: omega = (vx - vx_wheel) / (-y)  或  omega = (vy_wheel - vy) / x
        double omega_sum = 0.0;
        // 为避免除以零，为每个轮子选择合适的公式
        omega_sum += (vy_wheel[0] - vy) / wheel_pos_x[0]; // 左前
        omega_sum += (vy_wheel[1] - vy) / wheel_pos_x[1]; // 右前
        omega_sum += (vy_wheel[2] - vy) / wheel_pos_x[2]; // 左后
        omega_sum += (vy_wheel[3] - vy) / wheel_pos_x[3]; // 右后
    
        omega = omega_sum / 4.0;

        ROS_INFO_ONCE("正运动学解算结果：底盘速度vx=%.2f, vy=%.2f, omega=%.2f ", vx, vy, omega);
    }
}