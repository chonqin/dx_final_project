#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h> //
// 全局变量
double linear_vel = 0.5;  // 默认线速度
double angular_vel = 0.5; // 默认角速度
double current_linear_x = 0;
double current_linear_y = 0;
double current_angular_z = 0;

// 函数：获取键盘按键，非阻塞
int getch() {
    struct termios oldt, newt;
    int ch;
    int oldf;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);

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

    while (ros::ok()) {
        // 获取键盘输入
        int key = getch();

        // 根据按键更新速度指令
        if (key != -1) {
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
                    // 其他按键，可以不处理
                    break;
            }
        }

        // 填充Twist消息
        twist.linear.x = current_linear_x;
        twist.linear.y = current_linear_y;
        twist.linear.z = 0;
        twist.angular.x = 0;
        twist.angular.y = 0;
        twist.angular.z = current_angular_z;

        // 发布消息
        cmd_vel_pub.publish(twist);

        // 减速逻辑：如果不持续按键，则速度逐渐归零
        current_linear_x *= 0.8;
        current_linear_y *= 0.8;
        current_angular_z *= 0.8;

        ros::spinOnce();
        ros::Duration(0.1).sleep(); // 10Hz
    }

    return 0;
}
