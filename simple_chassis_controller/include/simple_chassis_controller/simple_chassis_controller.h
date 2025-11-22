//
// Created by qiayuan on 2/6/21.
//

#ifndef SIMPLE_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H
#define SIMPLE_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>
// 定义命名空间simple_chassis_controller
namespace simple_chassis_controller {
// 定义SimpleChassisController类，继承自controller_interface::Controller类
class SimpleChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  SimpleChassisController() = default;//默认构造函数,用于创建SimpleChassisController对象
  ~SimpleChassisController() override = default;//默认析构函数,用于销毁SimpleChassisController对象
  //初始化函数,用于初始化控制器,重写controller_interface::Controller类的init函数
  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;//override关键字表示该函数重写了基类中的虚函数

  void update(const ros::Time &time, const ros::Duration &period) override;

  hardware_interface::JointHandle front_left_pivot_joint_, front_right_pivot_joint_, back_left_pivot_joint_, back_right_pivot_joint_;
  hardware_interface::JointHandle front_left_wheel_joint_, front_right_wheel_joint_,
      back_left_wheel_joint_, back_right_wheel_joint_;
 private:
  //运动状态变量,用于记录当前的运动状态
  int state_{};
  ros::Time last_change_;
  double wheel_track_;
  double wheel_base_;
  double pivot_cmd_[4][4];
  double wheel_cmd_[4][4];
  //初始化PID控制器对象
  control_toolbox::Pid pid_lf_, pid_rf_, pid_lb_, pid_rb_;
  control_toolbox::Pid pid_lf_wheel_, pid_rf_wheel_, pid_lb_wheel_, pid_rb_wheel_;
};
}// namespace simple_chassis_controller

#endif //SIMPLE_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H
