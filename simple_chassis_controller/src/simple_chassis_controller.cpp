//
// Created by qiayuan on 2/6/21.
//

#include "simple_chassis_controller/simple_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>

namespace simple_chassis_controller {
bool SimpleChassisController::init(hardware_interface::EffortJointInterface *effort_joint_interface,
                                   ros::NodeHandle &root_nh, ros::NodeHandle &controller_nh) {
  front_left_wheel_joint_ =
      effort_joint_interface->getHandle("left_front_wheel_joint");
  front_right_wheel_joint_ =
      effort_joint_interface->getHandle("right_front_wheel_joint");
  back_left_wheel_joint_ = 
      effort_joint_interface->getHandle("left_back_wheel_joint");
  back_right_wheel_joint_ =
      effort_joint_interface->getHandle("right_back_wheel_joint");


  front_left_pivot_joint_ =
      effort_joint_interface->getHandle("left_front_pivot_joint");
  front_right_pivot_joint_ =
      effort_joint_interface->getHandle("right_front_pivot_joint");
  back_left_pivot_joint_ = 
      effort_joint_interface->getHandle("left_back_pivot_joint");
  back_right_pivot_joint_ =
      effort_joint_interface->getHandle("right_back_pivot_joint");

  wheel_track_ = controller_nh.param("wheel_track", 0.362);
  wheel_base_ = controller_nh.param("wheel_base", 0.362);

  pid_lf_.initPid(1.0, 0.0, 0.0, 0.0, 0.0);
  pid_rf_.initPid(1.0, 0.0, 0.0, 0.0, 0.0);
  pid_lb_.initPid(1.0, 0.0, 0.0, 0.0, 0.0);
  pid_rb_.initPid(1.0, 0.0, 0.0, 0.0, 0.0);

  // PID for wheel velocity control
  pid_lf_wheel_.initPid(2.0, 0.1, 0.0, 0.0, 0.0);
  pid_rf_wheel_.initPid(2.0, 0.1, 0.0, 0.0, 0.0);
  pid_lb_wheel_.initPid(2.0, 0.1, 0.0, 0.0, 0.0);
  pid_rb_wheel_.initPid(2.0, 0.1, 0.0, 0.0, 0.0);

  double speed = 8.0;


  // forward
  double theta_fwd[4] = {0.0, 0.0, 0.0, 0.0};
  double v_fwd[4] = {speed, speed, speed, speed};

  // backward
  double theta_bwd[4] = {0.0, 0.0, 0.0, 0.0};
  double v_bwd[4] = {-speed, -speed, -speed, -speed};

  //  left strafe
  double theta_left[4] = {M_PI/2, M_PI/2, M_PI/2, M_PI/2};
  double v_left[4] = {speed, speed, speed, speed};

  // right strafe
  double theta_right[4] = {-M_PI/2, -M_PI/2, -M_PI/2, -M_PI/2};
  double v_right[4] = {speed, speed, speed, speed};

  for (int i = 0; i < 4; ++i) {
    pivot_cmd_[0][i] = theta_fwd[i];
    wheel_cmd_[0][i] = v_fwd[i];
    pivot_cmd_[1][i] = theta_bwd[i];
    wheel_cmd_[1][i] = v_bwd[i];
    pivot_cmd_[2][i] = theta_left[i];
    wheel_cmd_[2][i] = v_left[i];
    pivot_cmd_[3][i] = theta_right[i];
    wheel_cmd_[3][i] = v_right[i];
  }

  return true;
}

void SimpleChassisController::update(const ros::Time &time, const ros::Duration &period) {


  if ((time - last_change_).toSec() > 8) {
    state_ = (state_ + 1) % 4;
    last_change_ = time;
  }
  front_left_wheel_joint_.setCommand(pid_lf_wheel_.computeCommand(wheel_cmd_[state_][0] - front_left_wheel_joint_.getVelocity(), period));
  front_right_wheel_joint_.setCommand(pid_rf_wheel_.computeCommand(wheel_cmd_[state_][1] - front_right_wheel_joint_.getVelocity(), period));
  back_left_wheel_joint_.setCommand(pid_lb_wheel_.computeCommand(wheel_cmd_[state_][2] - back_left_wheel_joint_.getVelocity(), period));
  back_right_wheel_joint_.setCommand(pid_rb_wheel_.computeCommand(wheel_cmd_[state_][3] - back_right_wheel_joint_.getVelocity(), period));

  front_left_pivot_joint_.setCommand(pid_lf_.computeCommand(pivot_cmd_[state_][0] - front_left_pivot_joint_.getPosition(), period));
  front_right_pivot_joint_.setCommand(pid_rf_.computeCommand(pivot_cmd_[state_][1] - front_right_pivot_joint_.getPosition(), period));
  back_left_pivot_joint_.setCommand(pid_lb_.computeCommand(pivot_cmd_[state_][2] - back_left_pivot_joint_.getPosition(), period));
  back_right_pivot_joint_.setCommand(pid_rb_.computeCommand(pivot_cmd_[state_][3] - back_right_pivot_joint_.getPosition(), period));
}

PLUGINLIB_EXPORT_CLASS(simple_chassis_controller::SimpleChassisController, controller_interface::ControllerBase)
}
