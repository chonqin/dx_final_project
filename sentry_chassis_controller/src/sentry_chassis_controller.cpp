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
    // 初始化tf监听器
    tf_listener_ = std::make_unique<tf::TransformListener>();
    // 订阅测试模式话题  
    test_mode_sub_ = controller_nh.subscribe<std_msgs::Int32>(
        "/test_mode", 1, &SentryChassisController::testmode_callback, this);
    // 订阅cmd_vel话题
    cmd_vel_sub = controller_nh.subscribe<geometry_msgs::Twist>(
      "/cmd_vel", 1, &SentryChassisController::cmdvel_callback, this);
    // 初始化速度命令时间戳
    last_cmd_vel_time_ = ros::Time::now();
    // 发布里程计话题 
    odometry_ = std::make_unique<Odometry>(controller_nh, wheel_base_, wheel_track_, wheel_radius_);
    ROS_INFO("初始化成功!默认模式为0...等待键盘输入测试模式...");
    return true;

  }
  
  /*ros_control update函数*/
  void SentryChassisController::update(const ros::Time& time, const ros::Duration& period) {
      odometry_->update(time, period, pivot_joints_, wheel_joints_);
      
      // 从RealtimeBuffer读取最新的cmd_vel消息
      geometry_msgs::Twist* cmd_vel_ptr = cmd_vel_buffer_.readFromRT();
      
      // 只有在有新的速度命令时才进行处理
      if (cmd_vel_ptr != nullptr && (time - last_cmd_vel_time_).toSec() < cmd_vel_timeout_) {
        //  接收到新的速度命令
        geometry_msgs::Twist received_vel = *cmd_vel_ptr;
        ROS_INFO("收到原始cmd_vel: 线速度(%.2f, %.2f), 角速度(%.2f)", 
                 received_vel.linear.x, received_vel.linear.y, received_vel.angular.z);
              
        // 步骤3: 根据坐标系选择进行坐标变换或直接使用
        if(!coordinate_system) {
          // 如果选择全局坐标系，则进行坐标变换
          geometry_msgs::Twist local_vel;
          tf_global_to_local(received_vel, local_vel);
          vx = local_vel.linear.x;
          vy = local_vel.linear.y;
          omega = local_vel.angular.z;
          ROS_INFO("转换到底盘坐标系cmd_vel: 线速度(%.2f, %.2f), 角速度(%.2f)", 
                   vx, vy, omega);
        } 
        else if(coordinate_system) {
          // 否则直接使用接收到的速度（底盘坐标系）
          vx = received_vel.linear.x;
          vy = received_vel.linear.y;
          omega = received_vel.angular.z;
          ROS_INFO("使用底盘坐标系cmd_vel: 线速度(%.2f, %.2f), 角速度(%.2f)", 
                   vx, vy, omega);
        }
      }
      else {
        // 速度命令超时或缓冲区为空：将速度置0
        vx = 0.0;
        vy = 0.0;
        omega = 0.0;
        ROS_DEBUG("速度命令超时,底盘速度为0");
      }         
      switch (test_mode_){
      case 0:{// 正常模式,没有接受速度指令时车子自锁
        ROS_INFO_ONCE("正常模式");
        break;
      }
      case 1:{// 测试转向轮pid
        ROS_INFO_ONCE("测试转向轮pid");
        test_pivots_pid(pivot_joints_,pivot_pids_, pivot_target_pub, pivot_actual_pub,target_, period);
        break;
      }
      case 2:{// 测试轮速pid
        ROS_INFO_ONCE("测试驱动轮pid");
        test_wheels_pid(wheel_joints_,wheel_pids_, wheel_target_pub, wheel_actual_pub, target_, period);
        break;
      }
      case 3:{// 测试逆运动学,先在终端给出速度指令，然后解算轮速和转向角度，最后进行pid控制
        ROS_INFO_ONCE("测试逆运动学函数，计算轮速和转向角度");
        std::array<double, 4> current_angles;
        for(size_t i = 0; i < 4; i++) {
          current_angles[i] = pivot_joints_[i].getPosition();
        }
        Inverse_solution(vx, vy, omega, wheel_base_, wheel_track_, wheel_radius_,
                                    wheel_speed, steering_angle, current_angles);
        pid_control(wheel_joints_,pivot_joints_, wheel_speed, steering_angle, wheel_pids_, 
          pivot_pids_, wheel_target_pub, wheel_actual_pub, pivot_target_pub, pivot_actual_pub, period);
        break;
      }                            
      case 4:{// 测试正运动学，下面给出一段轮速和转向角度，计算得到底盘速度为 x=0.0,y=0.0,omega=0.5
        ROS_INFO_ONCE("测试正运动学函数,计算底盘速度vx, vy, omega");
        wheel_speed = {2.33, 2.33, 2.33, 2.33};
        steering_angle = {2.36, 0.79, -2.36, -0.79};
        forward_solution(wheel_speed, steering_angle, 
                        wheel_base_, wheel_track_, 
                        wheel_radius_, vx, vy, omega);
        ROS_INFO("正运动学解算结果: vx=%.2f, vy=%.2f, omega=%.2f ", vx, vy, omega);
        break;
      }
      case 5: {// 实现小陀螺模式,车子原地旋转
        ROS_INFO_ONCE("小陀螺模式启动，车子原地旋转");
        wheel_speed = {8.33, 8.33, 8.33, 8.33};// 轮速随意设定
        steering_angle = {2.36, 0.79, -2.36, -0.79};// 转向角度设定为逆时针旋转
        pid_control(wheel_joints_,pivot_joints_, wheel_speed, steering_angle, wheel_pids_, 
          pivot_pids_, wheel_target_pub, wheel_actual_pub, pivot_target_pub, pivot_actual_pub, period);
        break;
      }
      case 6: {// 实现自锁模式,车子无法移动
        ROS_INFO_ONCE("自锁模式启动，车子无法移动");
        wheel_speed = {0.0, 0.0, 0.0, 0.0};
        steering_angle = {-M_PI / 4.0,M_PI / 4.0, M_PI / 4.0, -M_PI / 4.0};
        pid_control(wheel_joints_,pivot_joints_, wheel_speed, steering_angle, wheel_pids_, 
          pivot_pids_, wheel_target_pub, wheel_actual_pub, pivot_target_pub, pivot_actual_pub, period);
        break;
      }
      case 7: {// 键盘控制模式
        ROS_INFO_ONCE("开启键盘控制模式，请使用键盘控制底盘运动");
        std::array<double, 4> current_angles;
        for(size_t i = 0; i < 4; i++) {
          current_angles[i] = pivot_joints_[i].getPosition();
        }
        Inverse_solution(vx, vy, omega, wheel_base_, wheel_track_, wheel_radius_, wheel_speed, steering_angle, current_angles);
        pid_control(wheel_joints_,pivot_joints_, wheel_speed, steering_angle, wheel_pids_, 
          pivot_pids_, wheel_target_pub, wheel_actual_pub, pivot_target_pub, pivot_actual_pub, period);
        break;
      }
      case 8: {
        break;
      }
      // powerlimit(wheel_joints_,pivot_joints_);
    }
  }
  /*接收cmd_vel话题回调函数：只用作写入缓冲区*/
  void SentryChassisController::cmdvel_callback(const geometry_msgs::Twist::ConstPtr& msg){
    // 将收到的Twist消息写入RealtimeBuffer
    geometry_msgs::Twist cmd_vel = *msg;
    cmd_vel_buffer_.writeFromNonRT(cmd_vel);
    
    // 更新最后一次收到命令的时间戳
    last_cmd_vel_time_ = ros::Time::now();
  }
  /*测试模式回调函数*/
  void SentryChassisController::testmode_callback(const std_msgs::Int32::ConstPtr& msg){
    test_mode_ = msg->data;
    ROS_INFO("测试模式已切换为: %d", test_mode_);
  }
  /*动态参数更改回调函数*/
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

  /*tf坐标变化函数：将全局坐标系速度变换到底盘坐标系 */
  bool SentryChassisController::tf_global_to_local(const geometry_msgs::Twist& global_vel, 
                                                  geometry_msgs::Twist& local_vel){
    
    tf_listener_->waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(0.1));
      
    tf::StampedTransform transform;
    tf_listener_->lookupTransform("odom", "base_link", ros::Time(0), transform);
      
    // 获取旋转矩阵：R_odom_to_base（从odom到base_link的旋转）
    tf::Matrix3x3 R_odom_to_base(transform.getRotation());
      
    // 提取全局速度向量
    tf::Vector3 global_linear(global_vel.linear.x, global_vel.linear.y, 0);
      
    // 将全局线速度变换到底盘坐标系
    // 公式：v_local = R_odom_to_base^T * v_global （使用转置，将全局坐标变换到局部坐标）
    tf::Vector3 local_linear = R_odom_to_base.transpose() * global_linear;
      
    // 角速度在两坐标系中相同（绕z轴）
    double local_omega = global_vel.angular.z;
      
    // 填充输出的局部速度
    local_vel.linear.x = local_linear.getX();
    local_vel.linear.y = local_linear.getY();
    local_vel.linear.z = 0;
    local_vel.angular.x = 0;
    local_vel.angular.y = 0;
    local_vel.angular.z = local_omega;
    return true;
  }
  /*参数加载函数，从yaml文件获取参数*/
  void SentryChassisController::controller_param_load(ros::NodeHandle &controller_nh) {
    //从参数服务器获取车轮间距、轴距参数、轮子半径参数、功率控制参数
    wheel_track_ = controller_nh.param("wheel_track", 0.362);
    wheel_base_ = controller_nh.param("wheel_base", 0.362);
    wheel_radius_ = controller_nh.param("wheel_radius", 0.055);
    coordinate_system = controller_nh.param("coordinate_system", 0);
    vel_coeff = controller_nh.param("vel_coeff", 0.6);
    effort_coeff = controller_nh.param("effort_coeff", 0.4);
    ROS_INFO("坐标系模式: %d", coordinate_system);
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
  
  /*功率限制*/
  void SentryChassisController::powerlimit(std::array<hardware_interface::JointHandle, 4>& wheel_joints,
                                          std::array<hardware_interface::JointHandle, 4>& pivot_joints){
    double power_limit = 80; // 功率限制，单位瓦特,数值假设为80W
    double a = 0, b = 0, c = 0; // 用于计算功率的中间变量                   
    for(size_t i = 0; i < 4; i++) {
      double vel = wheel_joints[i].getVelocity();
      double effort = wheel_joints[i].getEffort();
      // 计算功率
      a += std::pow(effort,2);  // τ²
      b += std::abs(vel * effort ); // |ω·τ|
      c += std::pow(vel,2); // ω²
    }
    // P = effort_coeff * Σ(τ²) + vel_coeff * Σ(ω²) + power_offset_
    // power_offset_ 为基础功率损耗，可以根据实际测定,这里设置为0 
    a *= effort_coeff;
    c  = c * vel_coeff - power_limit - power_offset_;

    double k = 1.0; // 功率限制系数，初始为1.0（不限制）
    if(b!=0){ // 避免除零错误
      k = (-b + std::sqrt(b*b - 4*a*c) )/ (2*a);
    }
    // 应用功率限制系数
    if(k < 1.0) {
      for(size_t i = 0; i < 4; i++) {
        double limited_effort = wheel_joints[i].getEffort() * k;
        wheel_joints[i].setCommand(limited_effort);
      }
    }
    else{
      // 不需要限制，保持原命令
      for(size_t i = 0; i < 4; i++) {
        double original_effort = wheel_joints[i].getEffort();
        wheel_joints[i].setCommand(original_effort);
      }
    }
  }
}
PLUGINLIB_EXPORT_CLASS(sentry_chassis_controller::SentryChassisController, 
          controller_interface::ControllerBase)