#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
//#include "example_interfaces/msg/string.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// #include "ros/ros.h"
#include <std_msgs/msg/int64.hpp>
#include <cmath>

using std::placeholders::_1;
using namespace std;

class D_Calc : public rclcpp::Node
{
public:
    D_Calc() : Node("rpm") // Create a node with name stated
    {

        publish_rpm_A = this->create_publisher<std_msgs::msg::Int64>("right_wheel_rpm", 100);
        publish_rpm_B = this->create_publisher<std_msgs::msg::Int64>("left_wheel_rpm", 100);

        RCLCPP_INFO(this->get_logger(), "looping......");
        wheel_odom_sub_ = this->create_subscription<std_msgs::msg::String>("distance_finder", 100, std::bind(&D_Calc::receive_Msg, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::microseconds(500), std::bind(&D_Calc::timer_callback, this));
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr wheel_odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publish_rpm_A;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr publish_rpm_B;

    std_msgs::msg::Int64 rpm_A_2;
    std_msgs::msg::Int64 rpm_B_2;
    int64_t rpm_A;
    int64_t rpm_B;

    void receive_Msg(const std_msgs::msg::String::SharedPtr msg)
    {

        std::string message = msg->data;
        // RCLCPP_INFO(this->get_logger(), "Data Received: %s", message.c_str());

        std::vector<std::string> text = split(message, ',');
        // RCLCPP_INFO(this->get_logger(), "Data Received: %s", text[1].c_str());

        rpm_A = stoi(text[0]);
        rpm_B = stoi(text[1]);

        // RCLCPP_INFO(this->get_logger(), "Rot A= %d , Rot B= %d", rot_A, rot_B);
    }
    void timer_callback()
    {
        rpm_A_2.data = rpm_A;
        rpm_B_2.data = rpm_B;
        publish_rpm_A->publish(rpm_A_2);
        publish_rpm_B->publish(rpm_B_2);
        //RCLCPP_INFO(this->get_logger(), "RPM_A %d....RMP_B= %d....", rpm_A_2.data, rpm_B_2.data);
    }

    std::vector<std::string> split(const std::string &s, char delimiter)
    {
        std::vector<std::string> result;
        std::stringstream ss(s);
        std::string item;

        while (getline(ss, item, delimiter))
        {
            result.push_back(item);
        }
        return result;
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);               // initialize ROS 2 communication
    auto node = std::make_shared<D_Calc>(); // creating a shared ponter to the node
    rclcpp::spin(node);                     // loop as node is executed
    rclcpp::shutdown();                     // shutdown the communication, stop spinning
    return 0;
}
