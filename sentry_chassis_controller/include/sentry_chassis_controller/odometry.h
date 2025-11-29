// ...existing code...
#ifndef SENTRY_CHASSIS_CONTROLLER_ODOMETRY_H
#define SENTRY_CHASSIS_CONTROLLER_ODOMETRY_H

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <array>
#include <hardware_interface/joint_command_interface.h>
#include "sentry_chassis_controller/kinematics.h"
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>

namespace sentry_chassis_controller{
    
    class Odometry{
        public:
            Odometry(ros::NodeHandle& nh, double wheel_base, 
                double wheel_track , double wheel_radius);

            void update(const ros::Time& time, const ros::Duration& period,
                        const std::array<hardware_interface::JointHandle, 4>& pivot_joints,
                        const std::array<hardware_interface::JointHandle, 4>& wheel_joints);

        private:
            // 里程计位姿
            double x_, y_, z_;
            
            // 车辆参数
            double wheel_base_, wheel_track_, wheel_radius_;

            // ROS接口
            ros::Publisher odom_pub;        
            tf::TransformBroadcaster odom_broadcaster;
            ros::NodeHandle nh_;
    };

} // namespace sentry_chassis_controller

#endif // SENTRY_CHASSIS_CONTROLLER_ODOMETRY_H
