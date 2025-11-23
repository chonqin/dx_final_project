//
// Created by qiayuan on 2/6/21.
//

#include "simple_chassis_controller/simple_chassis_controller.h"
#include <pluginlib/class_list_macros.hpp>
/*
初始化函数：
1. 获取四个车轮和四个转向关节的句柄。
2. 从参数服务器获取车轮间距和轴距参数。
3. 初始化四个转向关节的PID控制器和四个车轮速度的PID控制器。
4. 设定四种运动模式（前进、后退、左平移、右平移）的参数。
*/

//设定命名空间为simple_chassis_controller
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
  //定义了基础速度                                  
  double speed = 8.0;

  /*麦轮底盘：四个全向轮，既可以前进后退，也可以360度转动角度；
    theta表示每个关节的目标角度；v表示每个车轮的目标速度;
    也就是说，我们设定四个轮子共八个参数，就可以控制麦轮底盘的运动
  */

  /*定义了四种运动模式（前进、后退、左平移、右平移）的参数。
    当然你也可以根据需要，自定义更多的运动模式，比如斜向移动、原地旋转等。
    例如：
    斜向运动：

    原地旋转：

  */

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

  /*将各个模式的参数存储到二维数组中，便于后续调用。
  二维数组前面的参数代表模式，后面的参数代表每个车轮，所以设置为i，：
  0->左前轮，1->右前轮，2->左后轮，3->右后轮
  pivot_cmd_ 数组（转向角度）：
        左前  右前  左后  右后
模式0  [0.0] [0.0] [0.0] [0.0]  - 前进
模式1  [0.0] [0.0] [0.0] [0.0]  - 后退
模式2  [π/2] [π/2] [π/2] [π/2]  - 左平移
模式3  [-π/2][-π/2][-π/2][-π/2] - 右平移

wheel_cmd_ 数组（车轮速度）：
        左前  右前  左后  右后
模式0  [8.0] [8.0] [8.0] [8.0]  - 前进
模式1  [-8.0][-8.0][-8.0][-8.0] - 后退
模式2  [8.0] [8.0] [8.0] [8.0]  - 左平移
模式3  [8.0] [8.0] [8.0] [8.0]  - 右平移
  */
 /*
    使用这个二维数组的好处是：
    1. 结构清晰：每种运动模式的参数一目了然
    2. 易于扩展：可以方便地添加更多运动模式，只需在数组中往后添加一行模式5，6，7……
    3. 如果使用传统的if-else语句，需要写4个if语句，每个if语句对应一个运动模式，代码会非常冗长。
 */
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
  state_ = 0;
  last_change_ = ros::Time::now();
  return true;
}
/*
更新函数：
1. 检查是否需要切换运动模式（每8秒切换一次）。
2. 根据当前状态更新PID控制器的目标值。
3. 计算并设置每个车轮和转向关节的命令值。
*/
void SimpleChassisController::update(const ros::Time &time, const ros::Duration &period) {
    //state_表示当前的运动模式，初始值为0（前进）
    //(state_ + 1) % 4表示下一个运动模式，循环切换
  if ((time - last_change_).toSec() > 8) {
    state_ = (state_ + 1) % 4;
    last_change_ = time;
  }
  //setCommand()函数发送指令到对应的关节，函数传入output
  //computeCommand()函数pid计算输出, 函数传入当前error（目标值-当前值）和周期
  front_left_wheel_joint_.setCommand(pid_lf_wheel_.computeCommand(wheel_cmd_[state_][0] - front_left_wheel_joint_.getVelocity(), period));
  front_right_wheel_joint_.setCommand(pid_rf_wheel_.computeCommand(wheel_cmd_[state_][1] - front_right_wheel_joint_.getVelocity(), period));
  back_left_wheel_joint_.setCommand(pid_lb_wheel_.computeCommand(wheel_cmd_[state_][2] - back_left_wheel_joint_.getVelocity(), period));
  back_right_wheel_joint_.setCommand(pid_rb_wheel_.computeCommand(wheel_cmd_[state_][3] - back_right_wheel_joint_.getVelocity(), period));

  front_left_pivot_joint_.setCommand(pid_lf_.computeCommand(pivot_cmd_[state_][0] - front_left_pivot_joint_.getPosition(), period));
  front_right_pivot_joint_.setCommand(pid_rf_.computeCommand(pivot_cmd_[state_][1] - front_right_pivot_joint_.getPosition(), period));
  back_left_pivot_joint_.setCommand(pid_lb_.computeCommand(pivot_cmd_[state_][2] - back_left_pivot_joint_.getPosition(), period));
  back_right_pivot_joint_.setCommand(pid_rb_.computeCommand(pivot_cmd_[state_][3] - back_right_pivot_joint_.getPosition(), period));
}
//注册pluginlib插件,使得控制器可以被ROS控制器管理器加载和使用
PLUGINLIB_EXPORT_CLASS(simple_chassis_controller::SimpleChassisController, controller_interface::ControllerBase)
}
