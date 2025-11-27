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

/*自定义头文件依赖*/
#include "sentry_chassis_controller/kinematics.h"
#include "sentry_chassis_controller/test_pid.h"


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
            double wheel_base_, wheel_track_;// 车轮间距和轴距
            double max_wheel_speed_, max_pivot_speed_;// 最大车轮速度和最大转向速度     
            double pivot_cmd_[4][4];//关节角度转向数组
            double wheel_cmd_[4][4];//车轮速度数组
            //测试模式选择，1为测试pid，0为测试逆运动学，3为测试正运动学，等等...    
            int test_mode_ = 0 ;
            double target_ = 10.0; // 目标参数，用于测试pid参数效果

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

            // 从yaml文件加载参数函数
            void controller_param_load(ros::NodeHandle &controller_nh);
            void testmode_callback(const std_msgs::Int32::ConstPtr& msg);
            void dynamicReconfigureCallback(sentry_chassis_controller::SentryChassisControllerConfig &config, uint32_t level);    
            

    };
}// namespace sentry_chassis_controller

#endif // SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H