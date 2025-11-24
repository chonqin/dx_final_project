#ifndef SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
#define SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
/*ros官方头文件依赖*/
#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <ros/ros.h>
#include <string>
#include <array>
#include <std_msgs/Int32.h>
/*自定义头文件依赖*/
#include "sentry_chassis_controller/kinematics.h"
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
            hardware_interface::JointHandle front_left_pivot_joint_, front_right_pivot_joint_, back_left_pivot_joint_, back_right_pivot_joint_;
            hardware_interface::JointHandle front_left_wheel_joint_, front_right_wheel_joint_,
                                            back_left_wheel_joint_, back_right_wheel_joint_;
        private:
            double wheel_base_, wheel_track_;// 车轮间距和轴距
            double max_wheel_speed_, max_pivot_speed_;// 最大车轮速度和最大转向速度     
            double pivot_cmd_[4][4];//关节角度转向数组
            double wheel_cmd_[4][4];//车轮速度数组
            //测试模式选择，1为测试pid，0为测试逆运动学，3为测试正运动学，等等...    
            int test_mode_ = 0 ;
            // 初始化PID控制器对象
            std::array<control_toolbox::Pid, 4> pivot_pids_;
            std::array<control_toolbox::Pid, 4> wheel_pids_;
            // 从yaml文件加载参数函数
            void controller_param_load(ros::NodeHandle &controller_nh);
            void testmode_callback(const std_msgs::Int32::ConstPtr& msg);

            

    };
}// namespace sentry_chassis_controller

#endif // SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H