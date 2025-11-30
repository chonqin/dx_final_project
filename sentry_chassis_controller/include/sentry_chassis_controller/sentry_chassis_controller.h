#ifndef SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
#define SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
/*ros官方头文件依赖*/
#include <control_toolbox/pid.h>
#include <pluginlib/class_list_macros.hpp>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <ros/ros.h>
#include <string>
#include <array>
#include <std_msgs/Int32.h>
#include <dynamic_reconfigure/server.h>
#include <sentry_chassis_controller/SentryChassisControllerConfig.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
/*自定义头文件依赖*/
#include "sentry_chassis_controller/kinematics.h"
#include "sentry_chassis_controller/test_pid.h"
#include "sentry_chassis_controller/odometry.h"

namespace sentry_chassis_controller {
    // 定义SentryChassisController类,继承自controller_interface::Controller模板类
    class SentryChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
        public:
            //
            SentryChassisController() = default;
            ~SentryChassisController() override = default;
            // 初始化函数，重写基类的init函数
            bool init(hardware_interface::EffortJointInterface* sentry_chassis_controller, 
                ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;
            // 更新函数，重写基类的update函数        
            void update(const ros::Time& time, const ros::Duration& period) override;

            ros::Subscriber test_mode_sub_;    
            // 定义四个转向关节和四个车轮的句柄数组
            std::array<hardware_interface::JointHandle, 4> pivot_joints_;
            std::array<hardware_interface::JointHandle, 4> wheel_joints_;    

        private:
            double wheel_base_, wheel_track_ , wheel_radius_;// 车轮间距和轴距
            std::string coordinate_system;//坐标系选择
            //测试模式选择，1为测试pid，0为测试逆运动学，3为测试正运动学，等等...    
            int test_mode_ = 0 ;
            double target_ = 10.0; // 目标，用于测试pid参数效果
            // 底盘运动学定义
            double vx , vy, omega; // 线速度和角速度
            // 存储四个驱动轮速度和转向轮转向角度
            std::array<double, 4> wheel_speed= {0.0, 0.0, 0.0, 0.0}; 
            std::array<double, 4> steering_angle= {0.0, 0.0, 0.0, 0.0}; 
            // 定义轮子名字数组,方便传参
            const std::array<std::string, 4> wheel_names = {
                "front_left", "front_right", "back_left", "back_right"};
            // PID控制器对象
            std::array<control_toolbox::Pid, 4> pivot_pids_;
            std::array<control_toolbox::Pid, 4> wheel_pids_;
            // PID发布数据对象
            std::array<ros::Publisher, 4> wheel_target_pub;
            std::array<ros::Publisher, 4> wheel_actual_pub;
            std::array<ros::Publisher, 4> pivot_target_pub;
            std::array<ros::Publisher, 4> pivot_actual_pub;
            //dynamic_reconfigure 服务器对象
            std::unique_ptr<dynamic_reconfigure::Server<sentry_chassis_controller::SentryChassisControllerConfig>> dynamic_server;
            // 接收cmd_vel话题回调对象
            ros::Subscriber cmd_vel_sub;
            // 里程计发布器
            std::unique_ptr<Odometry> odometry_;
            // tf监听器指针
            std::unique_ptr<tf::TransformListener> tf_listener_ ;
            // 从yaml文件加载参数函数
            void controller_param_load(ros::NodeHandle &controller_nh);
            void testmode_callback(const std_msgs::Int32::ConstPtr& msg);
            void dynamicReconfigureCallback(sentry_chassis_controller::SentryChassisControllerConfig &config, uint32_t level);
            void vel_callback(const geometry_msgs::Twist::ConstPtr& msg);
            bool tf_global_to_local(const geometry_msgs::Twist& global_vel, geometry_msgs::Twist& local_vel);
    };
}// namespace sentry_chassis_controller

#endif // SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H