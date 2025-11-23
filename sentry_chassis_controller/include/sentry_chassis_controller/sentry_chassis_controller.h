#ifndef SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H
#define SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H

#include <control_toolbox/pid.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_interface/controller.h>
#include <ros/ros.h>
#include <string>
namespace sentry_chassis_controller {
    // 定义SentryChassisController类,继承自controller_interface::Controller模板类
    class SentryChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
        public:
            SentryChassisController() = default;
            ~SentryChassisController() override = default;

            bool init(hardware_interface::EffortJointInterface* sentry_chassis_controller, 
                ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) override;

            void update(const ros::Time& time, const ros::Duration& period) override;

            hardware_interface::JointHandle front_left_pivot_joint_, front_right_pivot_joint_, back_left_pivot_joint_, back_right_pivot_joint_;
            hardware_interface::JointHandle front_left_wheel_joint_, front_right_wheel_joint_,
                                            back_left_wheel_joint_, back_right_wheel_joint_;
        private:
            double wheel_base_, wheel_track_;// 车轮间距和轴距
            double max_wheel_speed_, max_pivot_speed_;// 最大车轮速度和最大转向速度     
            double pivot_cmd_[4][4];//关节角度转向数组
            double wheel_cmd_[4][4];//车轮速度数组
            // 初始化PID控制器对象
            control_toolbox::Pid pid_lf_, pid_rf_, pid_lb_, pid_rb_;
            control_toolbox::Pid pid_lf_wheel_, pid_rf_wheel_, pid_lb_wheel_, pid_rb_wheel_;
            // 
            void controller_param_load(ros::NodeHandle &controller_nh);

    };
}// namespace sentry_chassis_controller

#endif // SENTRY_CHASSIS_CONTROLLER_SENTRY_CHASSIS_CONTROLLER_H