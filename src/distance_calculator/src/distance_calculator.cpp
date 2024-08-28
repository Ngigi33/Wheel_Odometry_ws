#include "rclcpp/rclcpp.hpp"
#include <std_msgs/msg/string.hpp>
#include "example_interfaces/msg/string.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// #include "ros/ros.h"
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/float64.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

using std::placeholders::_1;
using namespace std;

class Distance_calc_Class : public rclcpp::Node
{
public:
    Distance_calc_Class() : Node("Distance_Calculator")
    {
        RCLCPP_INFO(this->get_logger(), "Looping.......");
        wheel_odom_sub_ = this->create_subscription<std_msgs::msg::String>("/distance_finder", 1, std::bind(&Distance_calc_Class::receive_Msg, this, std::placeholders::_1));
        initial_2D_wheel = this->create_publisher<geometry_msgs::msg::PoseStamped>("initial_2D_wheel", 100);
        displacement_A_pub_ = this->create_publisher<std_msgs::msg::Float64>("displacement_A", 100);
        displacement_B_pub_ = this->create_publisher<std_msgs::msg::Float64>("displacement_B", 100);
        timer_ = this->create_wall_timer(std::chrono::microseconds(500), std::bind(&Distance_calc_Class::timer_callback, this));
    }

private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr wheel_odom_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr initial_2D_wheel;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr displacement_A_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr displacement_B_pub_;
    geometry_msgs::msg::PoseStamped wheel_data;
    std_msgs::msg::Float64 displacement_A;
    std_msgs::msg::Float64 displacement_B;

    // Declare member variables for persistent state
    float old_rot_A = 0.0;
    float old_rot_B = 0.0;
    float new_rot_A = 0.0;
    float new_rot_B = 0.0;
    float delta_rot_A = 0.0;
    float delta_rot_B = 0.0;

    float distance_A = 0.0;
    float distance_B = 0.0;
    float distance_Avg = 0.0;
    float current_distance = 0.0; // Initialize current_distance
    float old_distance = 0.0;     // Initialize old_distance

    float distance_btwn_wheels = 0.235;
    float wheel_diameter = 0.085;

    float delta_Orientation = 0.0;
    float new_Orientation = 0.0;
    float old_Orientation = 0.00000000001;

    float delta_X = 0.0;
    float delta_Y = 0.0;
    float new_X = 0.0;
    float new_Y = 0.0;
    float old_X = 0.0;
    float old_Y = 0.0;

    // float data_store_array[]

    void receive_Msg(const std_msgs::msg::String::SharedPtr msg)
    {

        std::string message = msg->data;
        // Split the message by commas
        std::vector<std::string> text = split(message, ',');

        // Check if the split message has the expected number of elements
        if (text.size() < 4)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid message received: %s. Expected at least 4 parts.", message.c_str());
            return; // Exit the function early if the message is invalid
        }

        try
        {
            // Convert the strings to floats
            new_rot_A = std::stof(text[2]);
            new_rot_B = std::stof(text[3]);
            // RCLCPP_INFO(this->get_logger(), "Rot A= %f , Rot B= %f", new_rot_A, new_rot_B);
        }
        catch (const std::invalid_argument &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Invalid number format in message: %s", message.c_str());
        }
        catch (const std::out_of_range &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Number out of range in message: %s", message.c_str());
        }
    }
    void timer_callback()
    {
        odom();
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
    float Distance_Calculation(const float rotation)
    {
        float distance = ((rotation * 0.1) * M_PI * wheel_diameter);
        return distance;
    }
    void odom()
    {

        delta_rot_A = new_rot_A - old_rot_A;
        delta_rot_B = new_rot_B - old_rot_B;

        distance_A = Distance_Calculation(delta_rot_A);
        distance_B = Distance_Calculation(delta_rot_B);

        old_rot_A = new_rot_A;
        old_rot_B = new_rot_B;

        distance_Avg = ((distance_A + distance_B) / 2.0);

        if (distance_Avg > 0)
        {
            current_distance = current_distance + distance_Avg;
        }
        else if (distance_Avg == 0)
        {
            current_distance = current_distance;
        }

        delta_Orientation = ((distance_A - distance_B) / distance_btwn_wheels);
        new_Orientation = delta_Orientation + old_Orientation;

        delta_X = distance_Avg * cos(delta_Orientation);
        delta_Y = distance_Avg * sin(delta_Orientation);

        if (distance_Avg > 0)
        {

            new_X = old_X + delta_X;
            new_Y = old_Y + delta_Y;
        }
        else if (distance_Avg == 0)
        {
            new_X = old_X;
            new_Y = old_Y;
        }

        if (isnan(new_X) || isnan(new_Y) || isnan(new_Orientation))
        {
            new_X = old_X;
            new_Y = old_Y;
            new_Orientation = old_Orientation;
        }

        // Make sure theta stays in the correct range
        if (new_Orientation > M_PI)
        {
            new_Orientation -= 2 * M_PI;
        }
        else if (new_Orientation < -M_PI)
        {
            new_Orientation += 2 * M_PI;
        }
        else
        {
        }

        old_X = new_X;
        old_Y = new_Y;
        old_Orientation = new_Orientation;

        wheel_data.pose.position.x = old_X;
        wheel_data.pose.position.y = old_Y;
        wheel_data.pose.orientation.z = new_Orientation;

        displacement_A.data = distance_A;
        displacement_B.data = distance_B;

        initial_2D_wheel->publish(wheel_data);
        displacement_A_pub_->publish(displacement_A);
        displacement_B_pub_->publish(displacement_B);

        // RCLCPP_INFO(this->get_logger(), "Distance_A=%f....Distance_B=%f..... Current_Distance=%f", distance_A, distance_B, current_distance);
        RCLCPP_INFO(this->get_logger(), "Distance moved=%f", current_distance);
        // old_distance = current_distance;
    }
};

int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);                            // initialize ROS 2 communication
    auto node = std::make_shared<Distance_calc_Class>(); // creating a shared ponter to the node
    rclcpp::spin(node);                                  // loop as node is executed
    rclcpp::shutdown();                                  // shutdown the communication, stop spinning
    return 0;
}
