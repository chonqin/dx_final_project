#include "sentry_chassis_controller/sentry_chassis_controller.h"




namespace sentry_chassis_controller {
  /*ros_control init函数*/
  bool SentryChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                   ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    //获取四个轮子句柄
    front_left_wheel_joint_ =
      effort_joint_interface->getHandle("left_front_wheel_joint");
    front_right_wheel_joint_ =
      effort_joint_interface->getHandle("right_front_wheel_joint");
    back_left_wheel_joint_ = 
      effort_joint_interface->getHandle("left_back_wheel_joint");
    back_right_wheel_joint_ =
      effort_joint_interface->getHandle("right_back_wheel_joint");
    //获取四个转向关节句柄
    front_left_pivot_joint_ =
      effort_joint_interface->getHandle("left_front_pivot_joint");
    front_right_pivot_joint_ =
      effort_joint_interface->getHandle("right_front_pivot_joint");
    back_left_pivot_joint_ = 
      effort_joint_interface->getHandle("left_back_pivot_joint");
    back_right_pivot_joint_ =
      effort_joint_interface->getHandle("right_back_pivot_joint"); 
      
    ROS_INFO("关节句柄获取成功！");  
    //从yaml文件加载参数
    controller_param_load(controller_nh);                                
    
     test_mode_sub_ = controller_nh.subscribe<std_msgs::Int32>(
        "/test_mode", 1, &SentryChassisController::testmode_callback, this);
    ROS_INFO("参数加载成功！等待键盘输入测试模式...");
    return true;

  }
  
  /*ros_control update函数*/
  void SentryChassisController::update(const ros::Time& time, const ros::Duration& period) {
        // 控制逻辑实现
  }
       
  void SentryChassisController::testmode_callback(const std_msgs::Int32::ConstPtr& msg){
    test_mode_ = msg->data;
    ROS_INFO("测试模式已切换为: %d", test_mode_);
  }
  /*参数加载函数，从yaml文件获取参数*/
  void SentryChassisController::controller_param_load(ros::NodeHandle &controller_nh) {
    //从参数服务器获取车轮间距和轴距参数
    wheel_track_ = controller_nh.param("wheel_track", 0.362);
    wheel_base_ = controller_nh.param("wheel_base", 0.362);
    //从参数服务器获取最大车轮速度和最大转向速度参数
    max_wheel_speed_ = controller_nh.param("max_wheel_speed", 10.0);
    max_pivot_speed_ = controller_nh.param("max_pivot_speed", 5.0);
    /*从参数服务器获取八组PID参数*/ 
    //定义轮子名字数组，后续使用
    std::array<std::string, 4> wheel_names = {"front_left", "front_right", 
      "back_left", "back_right"}; 
    //加载轮速pid参数 
    for (size_t j = 0; j < 4; j++){
      //临时参数
      double p , i , d , i_max , i_min;
      //路径字符前后缀
      std::string wheel_pid_prefix = "pid/" + wheel_names[j] + "_wheel/";
      //从参数服务器获取对应的PID参数,
      controller_nh.param(wheel_pid_prefix + "p", p, 2.0);
      controller_nh.param(wheel_pid_prefix + "i", i, 0.1);
      controller_nh.param(wheel_pid_prefix + "d", d, 0.0);
      controller_nh.param(wheel_pid_prefix + "i_max", i_max, 0.0);
      controller_nh.param(wheel_pid_prefix + "i_min", i_min, 0.0);
      //初始化对应的pid对象
      wheel_pids_[j].initPid(p, i, d, i_max, i_min);
    }
    //加载转向pid参数
    for (size_t j = 0; j < 4; j++){
      //临时参数
      double p , i , d , i_max , i_min;
      //路径字符前后缀
      std::string wheel_pid_prefix = "pid/" + wheel_names[j] + "_pivot/";
      //从参数服务器获取对应的PID参数
      controller_nh.param(wheel_pid_prefix + "p", p, 2.0);
      controller_nh.param(wheel_pid_prefix + "i", i, 0.1);
      controller_nh.param(wheel_pid_prefix + "d", d, 0.0);
      controller_nh.param(wheel_pid_prefix + "i_max", i_max, 0.0);
      controller_nh.param(wheel_pid_prefix + "i_min", i_min, 0.0);
      //初始化对应的pid对象
      pivot_pids_[j].initPid(p, i, d, i_max, i_min);
    }
      
  }
  
}
PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, 
          controller_interface::ControllerBase)