#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float64.hpp>
#include "example_interfaces/msg/string.hpp"
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// #include "ros/ros.h"
#include <std_msgs/msg/int16.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>

using std::placeholders::_1;
using namespace std;

class odom_class : public rclcpp::Node
{
public:
    odom_class() : Node("Wheel_Odom_node")
    {
        // Set the data fields of the odometry message
        odomNew.header.frame_id = "odom";
        odomNew.pose.pose.position.z = 0;
        odomNew.pose.pose.orientation.x = 0;
        odomNew.pose.pose.orientation.y = 0;
        odomNew.twist.twist.linear.x = 0;
        odomNew.twist.twist.linear.y = 0;
        odomNew.twist.twist.linear.z = 0;
        odomNew.twist.twist.angular.x = 0;
        odomNew.twist.twist.angular.y = 0;
        odomNew.twist.twist.angular.z = 0;
        odomOld.pose.pose.position.x = initialX;
        odomOld.pose.pose.position.y = initialY;
        odomOld.pose.pose.orientation.z = initialTheta;

        odom_data_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom_data_euler", 100);
        odom_data_pub_quat = this->create_publisher<nav_msgs::msg::Odometry>("odom_data_quat", 100);

        RCLCPP_INFO(this->get_logger(), "looping......");

        wheel_odom_data = this->create_subscription<geometry_msgs::msg::PoseStamped>("/initial_2D_wheel", 1, std::bind(&odom_class::set_initial_2D, this, std::placeholders::_1));
        displacement_A_sub_ = this->create_subscription<std_msgs::msg::Float64>("/displacement_A", 1, std::bind(&odom_class::funct_disp_A, this, std::placeholders::_1));
        displacement_B_sub_ = this->create_subscription<std_msgs::msg::Float64>("/displacement_B", 1, std::bind(&odom_class::funct_disp_B, this, std::placeholders::_1));
        timer_ = this->create_wall_timer(std::chrono::microseconds(200), std::bind(&odom_class::timer_callback, this));
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr wheel_odom_data;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr displacement_A_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr displacement_B_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_quat;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub;

    nav_msgs::msg::Odometry odomOld;
    nav_msgs::msg::Odometry odomNew;

    const double initialX = 0.0;
    const double initialY = 0.0;
    const double initialTheta = 0.00000000001;

    bool initialPoseRecieved = false;

    double distanceRight;
    double distanceLeft;
    double distance_btwn_wheels = 235.00;
    // float PI = 3.141592653589793238462643383279502884;

    void set_initial_2D(const geometry_msgs::msg::PoseStamped &data)
    {
        odomOld.pose.pose.position.x = data.pose.position.x;
        odomOld.pose.pose.position.y = data.pose.position.y;
        odomOld.pose.pose.orientation.z = data.pose.orientation.z;

        initialPoseRecieved = true;
    }
    void funct_disp_A(const std_msgs::msg::Float64 &gotten_distance_A)
    {
        distanceRight = gotten_distance_A.data;
    }
    void funct_disp_B(const std_msgs::msg::Float64 &gotten_distance_B)
    {
        distanceLeft = gotten_distance_B.data;
    }

    void publish_Quaternion()
    {
        tf2::Quaternion q;

        q.setRPY(0, 0, odomNew.pose.pose.orientation.z);

        nav_msgs::msg::Odometry quatOdom;
        quatOdom.header.stamp = odomNew.header.stamp;
        quatOdom.header.frame_id = "odom";
        quatOdom.child_frame_id = "base_link";
        quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
        quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
        quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
        quatOdom.pose.pose.orientation.x = q.x();
        quatOdom.pose.pose.orientation.y = q.y();
        quatOdom.pose.pose.orientation.z = q.z();
        quatOdom.pose.pose.orientation.w = q.w();
        quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
        quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
        quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
        quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
        quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
        quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;

        for (int i = 0; i < 36; i++)
        {
            if (i == 0 || i == 7 || i == 14)
            {
                quatOdom.pose.covariance[i] = .01;
            }
            else if (i == 21 || i == 28 || i == 35)
            {
                quatOdom.pose.covariance[i] += 0.1;
            }
            else
            {
                quatOdom.pose.covariance[i] = 0;
            }
        }

        odom_data_pub_quat->publish(quatOdom);
    }
    void update_odom()
    {

        // Calculate the average distance
        double cycleDistance = (distanceRight + distanceLeft) / 2;

        // Calculate the number of radians the robot has turned since the last cycle
        double cycleAngle = asin((distanceRight - distanceLeft) / distance_btwn_wheels);

        // Average angle during the last cycle
        double avgAngle = cycleAngle / 2 + odomOld.pose.pose.orientation.z;

        if (avgAngle > M_PI)
        {
            avgAngle -= 2 * M_PI;
        }
        else if (avgAngle < -M_PI)
        {
            avgAngle += 2 * M_PI;
        }

        // Calculate the new pose (x, y, and theta)
        odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle) * cycleDistance;
        odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle) * cycleDistance;
        odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;

        // Prevent lockup from a single bad cycle
        if (std::isnan(odomNew.pose.pose.position.x) || std::isnan(odomNew.pose.pose.position.y) ||
            std::isnan(odomNew.pose.pose.position.z))
        {
            odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
            odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
            odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
        }

        // Make sure theta stays in the correct range
        if (odomNew.pose.pose.orientation.z > M_PI)
        {
            odomNew.pose.pose.orientation.z -= 2 * M_PI;
        }
        else if (odomNew.pose.pose.orientation.z < -M_PI)
        {
            odomNew.pose.pose.orientation.z += 2 * M_PI;
        }
        else
        {
        }

        // Compute the velocity
        odomNew.header.stamp = this->get_clock()->now();
        odomNew.twist.twist.linear.x = cycleDistance / (odomNew.header.stamp.sec - odomOld.header.stamp.sec);
        odomNew.twist.twist.angular.z = cycleAngle / (odomNew.header.stamp.sec - odomOld.header.stamp.sec);
        // Save the pose data for the next cycle
        odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
        odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
        odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
        odomOld.header.stamp = odomNew.header.stamp;

        // Publish the odometry message
        odom_data_pub->publish(odomNew);
    }

    void timer_callback()
    {
        if (initialPoseRecieved)
        {
            update_odom();
            publish_Quaternion();
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);                   // initialize ROS 2 communication
    auto node = std::make_shared<odom_class>(); // creating a shared ponter to the node
    rclcpp::spin(node);                         // loop as node is executed
    rclcpp::shutdown();                         // shutdown the communication, stop spinning
    return 0;
}