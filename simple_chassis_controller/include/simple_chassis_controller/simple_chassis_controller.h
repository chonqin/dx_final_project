//
// Created by qiayuan on 2/6/21.
//

#ifndef SIMPLE_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H
#define SIMPLE_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H

#include <ros/ros.h>
#include <controller_interface/controller.h>
#include <hardware_interface/joint_command_interface.h>
#include <control_toolbox/pid.h>

namespace simple_chassis_controller {

class SimpleChassisController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
 public:
  SimpleChassisController() = default;
  ~SimpleChassisController() override = default;

  bool init(hardware_interface::EffortJointInterface *effort_joint_interface,
            ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) override;

  void update(const ros::Time &time, const ros::Duration &period) override;

  hardware_interface::JointHandle front_left_pivot_joint_, front_right_pivot_joint_, back_left_pivot_joint_, back_right_pivot_joint_;
  hardware_interface::JointHandle front_left_wheel_joint_, front_right_wheel_joint_,
      back_left_wheel_joint_, back_right_wheel_joint_;
 private:
  int state_{};
  ros::Time last_change_;
  double wheel_track_;
  double wheel_base_;
  double pivot_cmd_[4][4];
  double wheel_cmd_[4][4];
  control_toolbox::Pid pid_lf_, pid_rf_, pid_lb_, pid_rb_;
  control_toolbox::Pid pid_lf_wheel_, pid_rf_wheel_, pid_lb_wheel_, pid_rb_wheel_;
};
}// namespace simple_chassis_controller

#endif //SIMPLE_CHASSIS_CONTROLLER_SIMPLE_CHASSIS_CONTROLLER_H
