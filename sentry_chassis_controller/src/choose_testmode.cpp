#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <iostream>
#include <limits>

int main(int argc, char** argv) {
    setlocale(LC_ALL, "");
    ros::init(argc, argv, "choose_testmode_node");
    ros::NodeHandle nh;

    ros::Publisher mode_pub = nh.advertise<std_msgs::Int32>("/test_mode", 1);

    std::cout << "键盘监听已启动，请输入数字选择测试模式并按回车：" << std::endl;
    std::cout << "  0: 停止测试" << std::endl;
    std::cout << "  1: 测试转向PID" << std::endl;
    std::cout << "  2: 测试驱动轮PID" << std::endl;

    ros::Rate rate(10); // 10 Hz
    while (ros::ok()) {
        //读取用户输入
        std::cout << "> ";
        int mode;
        std::cin >> mode;

        if (std::cin.fail()) { // 处理无效输入
            std::cout << "无效输入,请输入一个数字0-10。" << std::endl;
            std::cin.clear();
            std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');
            continue;
        }

        std_msgs::Int32 msg;
        msg.data = mode;
        mode_pub.publish(msg);
        ROS_INFO("读取到输入: %d ", mode);
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}