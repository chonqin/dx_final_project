#include "sentry_chassis_controller/sentry_chassis_controller.h"

namespace sentry_chassis_controller {
  /*ros_control init函数*/
  bool SentryChassisController::init(hardware_interface::EffortJointInterface* effort_joint_interface,
                                   ros::NodeHandle& root_nh, ros::NodeHandle& controller_nh) {
    //由于urdf中定义了四个转向关节和四个车轮关节，这里定义对应的名字数组
    const std::array<std::string, 4> pivot_joint_names = {
      "left_front_pivot_joint", "right_front_pivot_joint", 
      "left_back_pivot_joint", "right_back_pivot_joint"};
    const std::array<std::string, 4> wheel_joint_names = {
      "left_front_wheel_joint", "right_front_wheel_joint", 
      "left_back_wheel_joint", "right_back_wheel_joint"};
    //获取四个轮子句柄，四个转向关节句柄，索引0-3分别对应左前，右前，左后，右后,索引顺序全项目统一  
    for(size_t i = 0; i < 4; ++i) {
      pivot_joints_[i] = effort_joint_interface->getHandle(pivot_joint_names[i]);
      wheel_joints_[i] = effort_joint_interface->getHandle(wheel_joint_names[i]);
    }
    //设置本地化环境，支持中文输出
    setlocale(LC_ALL, ""); 
    ROS_INFO("关节句柄获取成功！");  
    //从yaml文件加载参数
    controller_param_load(controller_nh);                                
    // 初始化pid参数发布器
    for (size_t i = 0; i < 4; ++i) {
        wheel_target_pub[i] = controller_nh.advertise<std_msgs::Float64>(wheel_names[i] + "_wheel/target", 1);
        wheel_actual_pub[i] = controller_nh.advertise<std_msgs::Float64>(wheel_names[i] + "_wheel/actual", 1);
        pivot_target_pub[i] = controller_nh.advertise<std_msgs::Float64>(wheel_names[i] + "_pivot/target", 1);
        pivot_actual_pub[i] = controller_nh.advertise<std_msgs::Float64>(wheel_names[i] + "_pivot/actual", 1);
    }
    //初始化 dynamic_reconfigure 服务器，并设置回调函数
    dynamic_server.reset(new dynamic_reconfigure::Server<sentry_chassis_controller::SentryChassisControllerConfig>(controller_nh));
    dynamic_reconfigure::Server<sentry_chassis_controller::SentryChassisControllerConfig>::CallbackType f =
    boost::bind(&SentryChassisController::dynamicReconfigureCallback, this, _1, _2);
    dynamic_server->setCallback(f);
    
    // 订阅测试模式话题  
    test_mode_sub_ = controller_nh.subscribe<std_msgs::Int32>(
        "/test_mode", 1, &SentryChassisController::testmode_callback, this);
    // 订阅cmd_vel话题
    cmd_vel_sub = controller_nh.subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 1, &SentryChassisController::vel_callback, this);
    // 发布里程计话题 
    odometry_ = std::make_unique<Odometry>(controller_nh, wheel_base_, wheel_track_, wheel_radius_);
    ROS_INFO("参数加载成功！等待键盘输入测试模式...");
    return true;

  }
  
  /*ros_control update函数*/
  void SentryChassisController::update(const ros::Time& time, const ros::Duration& period) {
      
      odometry_->update(time, period, pivot_joints_, wheel_joints_);
      
      switch (test_mode_){
      case 1:// 测试转向轮pid
        test_pivots_pid(pivot_joints_,pivot_pids_, pivot_target_pub, pivot_actual_pub,target_, period);
        break;
      case 2:// 测试轮速pid
        test_wheels_pid(wheel_joints_,wheel_pids_, wheel_target_pub, wheel_actual_pub, target_, period);
        break;
      case 3:
        // 测试逆运动学
        test_inverse(vx, vy, omega, wheel_base_, wheel_track_, wheel_radius_,
                                    wheel_speed, steering_angle);
      case 4 :
        // 测试正运动学
        wheel_speed = {1.0, 1.0, 1.0, 1.0};
        steering_angle = {0.0, 0.0, 0.0, 0.0};
        forward_solution(wheel_speed, steering_angle, 
                        wheel_radius_, wheel_base_, 
                        wheel_track_, vx, vy, omega);
        ROS_INFO("正运动学解算结果: vx=%.2f, vy=%.2f, omega=%.2f ", vx, vy, omega);
        break;                              
      }
  }
  
  /*接收cmd_vel话题回调函数*/
  void SentryChassisController::vel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    // 回调函数只负责接收cmd_vel话题消息并提取
    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double omega = msg->angular.z;
    ROS_INFO("接收到cmd_vel指令:vx=%.2f, vy=%.2f, omega=%.2f ", vx, vy, omega);
  }

  /*测试模式回调函数*/
  void SentryChassisController::testmode_callback(const std_msgs::Int32::ConstPtr& msg){
    test_mode_ = msg->data;
    ROS_INFO("测试模式已切换为: %d", test_mode_);
  }
  void SentryChassisController::dynamicReconfigureCallback(sentry_chassis_controller::SentryChassisControllerConfig &config, uint32_t level) {
    ROS_INFO("Dynamic reconfigure 更新PID参数");
    
    // 更新驱动 PID 参数 ，索引 0-3 分别对应左前，右前，左后，右后
    wheel_pids_[0].setGains(config.front_left_wheel_p, config.front_left_wheel_i, 
      config.front_left_wheel_d, config.front_left_wheel_i_max, config.front_left_wheel_i_min);
    
    wheel_pids_[1].setGains(config.front_right_wheel_p, config.front_right_wheel_i, 
      config.front_right_wheel_d, config.front_right_wheel_i_max, config.front_right_wheel_i_min);

    wheel_pids_[2].setGains(config.back_left_wheel_p, config.back_left_wheel_i,
      config.back_left_wheel_d, config.back_left_wheel_i_max, config.back_left_wheel_i_min);

    wheel_pids_[3].setGains(config.back_right_wheel_p, config.back_right_wheel_i, 
      config.back_right_wheel_d, config.back_right_wheel_i_max, config.back_right_wheel_i_min);
    // 更新转向 PID 参数 ，索引 0-3 分别对应左前，右前，左后，右后
    pivot_pids_[0].setGains(config.front_left_pivot_p, config.front_left_pivot_i,
      config.front_left_pivot_d, config.front_left_pivot_i_max, config.front_left_pivot_i_min);

    pivot_pids_[1].setGains(config.front_right_pivot_p, config.front_right_pivot_i, 
      config.front_right_pivot_d, config.front_right_pivot_i_max, config.front_right_pivot_i_min);
    
    pivot_pids_[2].setGains(config.back_left_pivot_p, config.back_left_pivot_i,
      config.back_left_pivot_d, config.back_left_pivot_i_max, config.back_left_pivot_i_min);
    
    pivot_pids_[3].setGains(config.back_right_pivot_p, config.back_right_pivot_i, 
      config.back_right_pivot_d, config.back_right_pivot_i_max, config.back_right_pivot_i_min);
    
    target_ = config.target;
    ROS_WARN("PID Updated - P:%.2f I:%.2f D:%.4f target:%.2f", 
             config.front_left_wheel_p,
             config.front_left_wheel_i,
             config.front_left_wheel_d,
             config.target);    
  }

  

  /*参数加载函数，从yaml文件获取参数*/
  void SentryChassisController::controller_param_load(ros::NodeHandle &controller_nh) {
    //从参数服务器获取车轮间距、轴距参数、轮子半径参数
    wheel_track_ = controller_nh.param("wheel_track", 0.362);
    wheel_base_ = controller_nh.param("wheel_base", 0.362);
    wheel_radius_ = controller_nh.param("wheel_radius", 0.055);

    /*从参数服务器获取八组PID参数*/  
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
      wheel_pids_[j].initPid(p, i, d, i_max, i_min);
    }  //初始化对应的pid对象
    for (size_t j = 0; j < 4; j++){
      //临时参数
      double p , i , d , i_max , i_min;
      //从参数服务器获取对应的PID参数
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
    for (size_t i = 0; i < 4; ++i) {
      wheel_pids_[i].reset();
      pivot_pids_[i].reset();
    }  
  }
  
}
PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, 
          controller_interface::ControllerBase)