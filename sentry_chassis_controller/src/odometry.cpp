#include "sentry_chassis_controller/odometry.h"

namespace sentry_chassis_controller {
    /*初始化*/
    Odometry::Odometry(ros::NodeHandle& nh, double wheel_base, 
                double wheel_track , double wheel_radius)
        : nh_(nh),
        wheel_base_(wheel_base), 
        wheel_track_(wheel_track), 
        wheel_radius_(wheel_radius),
        x_(0.0), y_(0.0), z_(0.0) {            
        odom_pub = nh_.advertise<nav_msgs::Odometry>("odom", 50);
        
    }
    /*里程计更新函数*/
    void Odometry::update(const ros::Time& time, const ros::Duration& period,
                        const std::array<hardware_interface::JointHandle, 4>& pivot_joints,
                        const std::array<hardware_interface::JointHandle, 4>& wheel_joints) {
        // 获取轮速和转向角度数据
        std::array<double, 4> wheel_speed;
        std::array<double, 4> steering_angle;
        double vx = 0.0, vy = 0.0, omega = 0.0;
        for(size_t i = 0; i < 4; i++) {
            wheel_speed[i] = wheel_joints[i].getVelocity(); // 轮速
            steering_angle[i] = pivot_joints[i].getPosition(); // 转向角度
        }
        // 调用正运动学解算
        forward_solution(wheel_speed, steering_angle, 
                        wheel_radius_, wheel_base_, 
                        wheel_track_, vx, vy, omega);
        // 积分里程计                
        double dt = period.toSec();
        double delta_x = (vx  *std::cos(z_)  - vy * std::sin(z_)) * dt;
        double delta_y = (vx  *std::sin(z_)  + vy * std::cos(z_)) * dt;
        double delta_z = omega * dt;  
        x_ += delta_x;
        y_ += delta_y;
        z_ += delta_z;
        
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";
        odom_trans.transform.translation.x = x_;
        odom_trans.transform.translation.y = y_;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(z_);
        odom_broadcaster.sendTransform(odom_trans);

        // 发布 nav_msgs::Odometry
        nav_msgs::Odometry odom;
        odom.header.stamp = time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_link";

        odom.pose.pose.position.x = x_;
        odom.pose.pose.position.y = y_;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(z_);

        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = omega;

        odom_pub.publish(odom);

    } 
}// namespace sentry_chassis_controller
