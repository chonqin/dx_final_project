#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

// 全局变量
double linear_vel = 0.5;  // 默认线速度
double angular_vel = 0.5; // 默认角速度

// 函数：获取键盘按键（阻塞式）
int getch() {
    struct termios oldt, newt;
    int ch;
    
    // 获取当前终端属性
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    
    // 禁用规范模式和回显
    newt.c_lflag &= ~(ICANON | ECHO);
    
    // 应用新属性
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    
    // 读取一个字符
    ch = getchar();
    
    // 恢复终端属性
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    
    return ch;
}

// 函数：显示帮助信息
void print_help() {
    printf("       键盘控制节点\n");
    printf("移动控制:\n");
    printf("   w - 前进\n");
    printf("   s - 后退\n");
    printf("   a - 左移\n");
    printf("   d - 右移\n");
    printf("   q - 左转\n");
    printf("   e - 右转\n");
    printf("\n速度调整:\n");
    printf("   k - 增加线速度\n");
    printf("   l - 减小线速度\n");
    printf("   o - 增加角速度\n");
    printf("   p - 减小角速度\n");
    printf("\n其他:\n");
    printf("   h - 显示帮助\n");
    printf("当前速度: 线速度=%.2f m/s, 角速度=%.2f rad/s\n", linear_vel, angular_vel);
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "keyboard_control_node");
    ros::NodeHandle nh;

    // 创建发布器
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    // 创建Twist消息
    geometry_msgs::Twist twist;

    print_help();
    printf("等待键盘输入...\n");

    while (ros::ok()) {
        // 阻塞式读取键盘输入
        int key = getch();

        // 重置速度
        twist.linear.x = 0;
        twist.linear.y = 0;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = 0;

        bool should_publish = true;
        switch (key) {
            case 'w':
                twist.linear.x = linear_vel;
                break;
            case 's':
                twist.linear.x = -linear_vel;
                break;
            case 'a':
                twist.linear.y = linear_vel;
                break;
            case 'd':
                twist.linear.y = -linear_vel;
                break;
            case 'q':
                twist.angular.z = angular_vel;
                break;
            case 'e':
                twist.angular.z = -angular_vel;
                break;
            case 'k': // 增加线速度
                linear_vel += 0.1;
                should_publish = false;
                print_help();
                break;
            case 'l': // 减小线速度
                linear_vel -= 0.1;
                if (linear_vel < 0) linear_vel = 0;
                should_publish = false;
                print_help();
                break;
            case 'o': // 增加角速度
                angular_vel += 0.1;
                should_publish = false;
                print_help();
                break;
            case 'p': // 减小角速度
                angular_vel -= 0.1;
                if (angular_vel < 0) angular_vel = 0;
                should_publish = false;
                print_help();
                break;
            case 'h': // 帮助
                print_help();
                should_publish = false;
                break;
            default:
                should_publish = false;
                break;
        }

        // 发布速度命令（移动命令发布速度，其他命令不发布）
        if (should_publish) {
            cmd_vel_pub.publish(twist);
        }

        ros::spinOnce();
    }

    return 0;
}
