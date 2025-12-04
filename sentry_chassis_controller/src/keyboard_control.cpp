#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h> //
#include <sys/select.h>
// 全局变量
double linear_vel = 0.5;  // 默认线速度
double angular_vel = 0.5; // 默认角速度
double current_linear_x = 0;
double current_linear_y = 0;
double current_angular_z = 0;

// 函数：获取键盘按键，非阻塞
int getch() {
    // 定义termios结构体变量，用于保存和修改终端属性
    struct termios oldt, newt;
    int ch;
    int oldf;
    // 获取当前终端属性并保存到oldt, 用于后续恢复
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    // 修改终端属性，使其不等待换行且不回显输入字符
    newt.c_lflag &= ~(ICANON | ECHO);
    // 应用修改后的属性
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    // 使用select进行非阻塞检查：如果没有可读字符则立即返回-1
    fd_set readfds;
    struct timeval tv;
    FD_ZERO(&readfds);
    FD_SET(STDIN_FILENO, &readfds);
    tv.tv_sec = 0;
    tv.tv_usec = 0; // 不等待
    int rv = select(STDIN_FILENO + 1, &readfds, NULL, NULL, &tv);
    if (rv > 0 && FD_ISSET(STDIN_FILENO, &readfds)) {
        ch = getchar();
    } else {
        ch = -1;
    }
    // 恢复终端属性和文件描述符状态
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    // fcntl(STDIN_FILENO, F_SETFL, oldf);
    // 如果读取到字符则返回该字符，否则返回-1
    if(ch != EOF)
    {
        return ch;
    }

    return -1;
}

// 函数：显示帮助信息
void print_help() {
    printf("Reading from keyboard\n");
    printf("Use 'wasd' keys to move the robot.\n");
    printf("Use 'q/e' to rotate the robot.\n");
    printf("Press 'space' to stop the robot.\n");
    printf("Press 'k/l' to increase/decrease linear speed.\n");
    printf("Press 'o/p' to increase/decrease angular speed.\n");
    printf("Press Ctrl-C to quit.\n");
    printf("Current speeds: linear %.2f, angular %.2f\n", linear_vel, angular_vel);
}

int main(int argc, char** argv) {
    // 初始化ROS节点
    ros::init(argc, argv, "keyboard_control_node");
    ros::NodeHandle nh;

    // 创建一个发布器，发布到 /cmd_vel 话题
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    // 创建Twist消息
    geometry_msgs::Twist twist;

    print_help();
    // 按键活动标志：用于检测按键松开并只发送一次零速度
    bool active = false;
    bool zero_published = false;

    while (ros::ok()) {
        // 获取键盘输入（非阻塞）
        int key = getch();

        if (key != -1) {
            // 有按键输入，标记为活动并允许再次发送速度
            active = true;
            zero_published = false;

            switch (key) {
                case 'w':
                    current_linear_x = linear_vel;
                    current_linear_y = 0;
                    current_angular_z = 0;
                    break;
                case 's':
                    current_linear_x = -linear_vel;
                    current_linear_y = 0;
                    current_angular_z = 0;
                    break;
                case 'a':
                    current_linear_y = linear_vel;
                    current_linear_x = 0;
                    current_angular_z = 0;
                    break;
                case 'd':
                    current_linear_y = -linear_vel;
                    current_linear_x = 0;
                    current_angular_z = 0;
                    break;
                case 'q':
                    current_angular_z = angular_vel;
                    current_linear_x = 0;
                    current_linear_y = 0;
                    break;
                case 'e':
                    current_angular_z = -angular_vel;
                    current_linear_x = 0;
                    current_linear_y = 0;
                    break;
                case ' ': // 空格键，急停
                    current_linear_x = 0;
                    current_linear_y = 0;
                    current_angular_z = 0;
                    break;
                case 'k': // 增加线速度
                    linear_vel += 0.1;
                    print_help();
                    break;
                case 'l': // 减小线速度
                    linear_vel -= 0.1;
                    if (linear_vel < 0) linear_vel = 0;
                    print_help();
                    break;
                case 'o': // 增加角速度
                    angular_vel += 0.1;
                    print_help();
                    break;
                case 'p': // 减小角速度
                    angular_vel -= 0.1;
                    if (angular_vel < 0) angular_vel = 0;
                    print_help();
                    break;
                default:
                    break;
            }

            // 填充并发布速度（仅在有按键输入时持续发布）
            twist.linear.x = current_linear_x;
            twist.linear.y = current_linear_y;
            twist.linear.z = 0;
            twist.angular.x = 0;
            twist.angular.y = 0;
            twist.angular.z = current_angular_z;
            cmd_vel_pub.publish(twist);
        } else {
            // 没有按键输入
            if (active && !zero_published) {
                // 刚刚从活动状态变为无按键：发送一次零速度以立即停车
                current_linear_x = 0;
                current_linear_y = 0;
                current_angular_z = 0;
                twist.linear.x = 0;
                twist.linear.y = 0;
                twist.linear.z = 0;
                twist.angular.x = 0;
                twist.angular.y = 0;
                twist.angular.z = 0;
                cmd_vel_pub.publish(twist);
                zero_published = true;
                active = false;
            }
            // 如果已经空闲并且零速度已发送，则不再发布任何消息
        }

        ros::spinOnce();
        ros::Duration(0.05).sleep(); // 20Hz loop，提高响应
    }

    return 0;
}
