# sentry_chassis_controller内容说明



## 简介



**此pkg包含以下内容：**

- PID控制轮子速度
- 使用逆运动学计算各个轮子的期望速度
- 使用正运动学实现里程计
- 使用tf计算实现世界坐标下的速度控制
- 使用键盘操控底盘
- 实现小陀螺
- 实现功率控制
- 实现自锁
- ...

### 目录结构与文件说明



**目录结构如下**：

- src/sentry_chassis_controller
    - config
    - doc
    - include
    - src
    - CMakeLists.txt
    - package.xml

**具体文件说明**：

- config : 存放参数文件包括dynamic_reconfigure参数文件和yaml参数文件
- doc : 存放说明文档
- include : 存放头文件
- src : 存放源文件
    - function.cpp : pid功能实现
    - kinematics.cpp/h : 运动学计算
    - odometry.cpp/h : 里程计实现
    - sentry_chassis_controller.cpp/h : 主控器实现
    - choose_testmode.cpp : 选择测试模式节点
    - keyboard_control.cpp : 键盘控制节点
- CMakeLists.txt : 编译配置文件
- package.xml : 包配置文件

## 主体框架说明

使用自定义Controller 插件，先进行init初始化，随后不断执行update函数内容。

**Init**：

- 从EffortJoint获取四个转向电机和四个驱动电机的句柄，储存在数组里面
- 加载参数文件的参数，包括pid参数、底盘参数、功率控制参数
- 为八个电机初始化ROS发布器，用于发布调参所用的数据
- 初始化dynamic_recontfigure服务器，创建tf变化监听器
- 订阅全局/test_mode话题，创建里程计对象

**Update**：

- 里程计更新
- 获取速度消息，进行速度消息的变化
- 选择测试模式
- 进行功率控制

额外的两个节点： 

- **choose_testmode node** : 用于读取键盘的数字输入，用于选择测试模式 

- **keyboard_control node** ：用于读取键盘的控制按键，实现了前进、后退、平移、转向的全向控制，能实现加减速度，不输入任何按键的时候也不发送速度消息。

## 主要功能说明

### PID控制轮子速度



- 在`sentry_chassis_controller.h`中定义了四个`hardware_interface::JointHandle`类型的成员变量，分别对应四个轮子的轮速电机。
- 在`sentry_chassis_controller.cpp`的`init`函数中，加载了每个轮子的速度PID参数，并初始化了四个PID控制器。
- `pid_control`函数，根据期望速度和当前速度计算控制输出，并将其应用于八个电机。
- 加入`dynamicx_reconfigure`，能实现方便调参

### 运动学计算



- 在`kinematics.h`中定义了运动学相关的函数声明。
- 在`kinematics.cpp`中实现了逆运动学和正运动学的计算函数。
- 逆运动学函数：根据底盘的线速度和角速度计算各个轮子的期望速度。
- 正运动学函数：根据各个轮子的实际速度计算底盘的线速度和角速度。

### 里程计



- 获取四个舵向电机的角度和驱动电机的速度，利用正运动学计算出底盘整体速度
- 使用简单的积分更新求得里程计所需数据
- 发布数据到/odom话题

### TF变化



- 将odom系和base_link系进行关联
- 发布tf变换

### 键盘操控底盘



- 通过keyboard_control节点读取键盘输入
- 将键盘输入转换为速度命令，发布到/cmd_vel话题

### 小陀螺



- 在上面的基础上，只需要发布一个cmd_vel消息即可
- 底盘会自动调整各个轮子的速度和方向，实现小陀螺功能

### 功率控制



- 计算当前底盘的功率消耗
- 如果功率超过设定的最大值，则计算一个缩放因子，按比例降低各个轮子的速度
- 确保底盘在功率限制内运行

### 自锁功能



- 将轮子方向调整为特定角度，实现四个轮子速度相互抵消，从而达到自锁效果