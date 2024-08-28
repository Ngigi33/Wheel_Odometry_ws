// #include "rclcpp/rclcpp.hpp"
// #include <std_msgs/msg/string.hpp>
// #include "example_interfaces/msg/string.hpp"
// #include <iostream>
// #include <sstream>
// #include <string>
// #include <vector>

// // #include "ros/ros.h"
// #include <std_msgs/msg/int16.hpp>
// #include <nav_msgs/msg/odometry.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include <cmath>

// using std::placeholders::_1;
// using namespace std;

// class D_Calc : public rclcpp::Node
// {
// public:
//     D_Calc() : Node("Distance_Calculator_Node_1") // Create a node with name stated
//     {

//         // Set the data fields of the odometry message
//         odomNew.header.frame_id = "odom";
//         odomNew.pose.pose.position.z = 0;
//         odomNew.pose.pose.orientation.x = 0;
//         odomNew.pose.pose.orientation.y = 0;
//         odomNew.twist.twist.linear.x = 0;
//         odomNew.twist.twist.linear.y = 0;
//         odomNew.twist.twist.linear.z = 0;
//         odomNew.twist.twist.angular.x = 0;
//         odomNew.twist.twist.angular.y = 0;
//         odomNew.twist.twist.angular.z = 0;
//         odomOld.pose.pose.position.x = 0;
//         odomOld.pose.pose.position.y = 0;
//         odomOld.pose.pose.orientation.z = 0;

//         // set_initial_2D();

//         // Publishers
//         odom_data_pub = this->create_publisher<nav_msgs::msg::Odometry>("odom_data_euler", 100);
//         odom_data_pub_quat = this->create_publisher<nav_msgs::msg::Odometry>("odom_data_quat", 100);

//         RCLCPP_INFO(this->get_logger(), "looping......");

//         wheel_odom_sub_ = this->create_subscription<std_msgs::msg::String>("/distance_finder", 1, std::bind(&D_Calc::receive_Msg, this, std::placeholders::_1));
//         timer_ = this->create_wall_timer(std::chrono::microseconds(500), std::bind(&D_Calc::timer_callback, this));
//     }

// private:
//     rclcpp::Subscription<std_msgs::msg::String>::SharedPtr wheel_odom_sub_;
//     rclcpp::TimerBase::SharedPtr timer_;
//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub;
//     rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_data_pub_quat;
//     nav_msgs::msg::Odometry odomNew;
//     nav_msgs::msg::Odometry odomOld;

//     int rot_A;
//     int rot_B;

//     double distance_A = 0.0;
//     double distance_B = 0.0;
//     double distance_Avg;

//     float distance_btwn_wheels = 235.00;
//     float wheel_diameter = 85.00;

//     double delta_Orientation;
//     double new_Orientation;
//     double old_Orientation = 0.00000000001;
//     double PI = 3.141592;

//     // Flag to see if initial pose has been received
//     bool initialPoseRecieved = false;

//     double delta_X;
//     double delta_Y;
//     double new_X;
//     double new_Y;
//     double old_X = 0.0;
//     double old_Y = 0.0;

//     void set_initial_2D()
//     {
//         odomOld.pose.pose.position.x = delta_X;
//         odomOld.pose.pose.position.y = delta_Y;
//         odomOld.pose.pose.orientation.z = delta_Orientation;
//         initialPoseRecieved = true;
//     }

//     void receive_Msg(const std_msgs::msg::String::SharedPtr msg)
//     {

//         std::string message = msg->data;
//         // RCLCPP_INFO(this->get_logger(), "Data Received: %s", message.c_str());

//         std::vector<std::string> text = split(message, ',');
//         // RCLCPP_INFO(this->get_logger(), "Data Received: %s", text[1].c_str());

//         rot_A = stoi(text[2]);
//         rot_B = stoi(text[3]);
//         // RCLCPP_INFO(this->get_logger(), "Rot A= %d , Rot B= %d", rot_A, rot_B);
//     }

//     double Distance_Calculation(const int rotation)
//     {
//         double distance = (rotation * 2 * 3.14159 * (wheel_diameter / 2));
//         return distance;
//     }

//     void publish_Quaternion()
//     {
//         tf2::Quaternion q;

//         q.setRPY(0, 0, odomNew.pose.pose.orientation.z);

//         nav_msgs::msg::Odometry quatOdom;
//         quatOdom.header.stamp = odomNew.header.stamp;
//         quatOdom.header.frame_id = "odom";
//         quatOdom.child_frame_id = "base_link";
//         quatOdom.pose.pose.position.x = odomNew.pose.pose.position.x;
//         quatOdom.pose.pose.position.y = odomNew.pose.pose.position.y;
//         quatOdom.pose.pose.position.z = odomNew.pose.pose.position.z;
//         quatOdom.pose.pose.orientation.x = q.x();
//         quatOdom.pose.pose.orientation.y = q.y();
//         quatOdom.pose.pose.orientation.z = q.z();
//         quatOdom.pose.pose.orientation.w = q.w();
//         quatOdom.twist.twist.linear.x = odomNew.twist.twist.linear.x;
//         quatOdom.twist.twist.linear.y = odomNew.twist.twist.linear.y;
//         quatOdom.twist.twist.linear.z = odomNew.twist.twist.linear.z;
//         quatOdom.twist.twist.angular.x = odomNew.twist.twist.angular.x;
//         quatOdom.twist.twist.angular.y = odomNew.twist.twist.angular.y;
//         quatOdom.twist.twist.angular.z = odomNew.twist.twist.angular.z;

//         for (int i = 0; i < 36; i++)
//         {
//             if (i == 0 || i == 7 || i == 14)
//             {
//                 quatOdom.pose.covariance[i] = .01;
//             }
//             else if (i == 21 || i == 28 || i == 35)
//             {
//                 quatOdom.pose.covariance[i] += 0.1;
//             }
//             else
//             {
//                 quatOdom.pose.covariance[i] = 0;
//             }
//         }

//         odom_data_pub_quat->publish(quatOdom);
//     }

//     void update_odom()
//     {
//         distance_A = Distance_Calculation(rot_A);
//         distance_B = Distance_Calculation(rot_B);
//         distance_Avg = ((distance_A + distance_B) / 2.0);

//         // Calculate the number of radians the robot has turned since the last cycle
//         double cycleAngle = asin((distance_A - distance_B) / distance_btwn_wheels);

//         // Average angle during the last cycle
//         double avgAngle = cycleAngle / 2 + odomOld.pose.pose.orientation.z;

//         if (avgAngle > PI)
//         {
//             avgAngle -= 2 * PI;
//         }
//         else if (avgAngle < -PI)
//         {
//             avgAngle += 2 * PI;
//         }

//         // Calculate the new pose (x, y, and theta)
//         odomNew.pose.pose.position.x = odomOld.pose.pose.position.x + cos(avgAngle) * distance_Avg;
//         odomNew.pose.pose.position.y = odomOld.pose.pose.position.y + sin(avgAngle) * distance_Avg;
//         odomNew.pose.pose.orientation.z = cycleAngle + odomOld.pose.pose.orientation.z;

//         // Prevent lockup from a single bad cycle
//         if (std::isnan(odomNew.pose.pose.position.x) || std::isnan(odomNew.pose.pose.position.y) ||
//             std::isnan(odomNew.pose.pose.position.z))
//         {
//             odomNew.pose.pose.position.x = odomOld.pose.pose.position.x;
//             odomNew.pose.pose.position.y = odomOld.pose.pose.position.y;
//             odomNew.pose.pose.orientation.z = odomOld.pose.pose.orientation.z;
//         }

//         // Make sure theta stays in the correct range
//         if (odomNew.pose.pose.orientation.z > PI)
//         {
//             odomNew.pose.pose.orientation.z -= 2 * PI;
//         }
//         else if (odomNew.pose.pose.orientation.z < -PI)
//         {
//             odomNew.pose.pose.orientation.z += 2 * PI;
//         }
//         else
//         {
//         }

//         // Compute the velocity
//         odomNew.header.stamp = this->get_clock()->now();
//         odomNew.twist.twist.linear.x = distance_Avg / (odomNew.header.stamp.sec - odomOld.header.stamp.sec);
//         odomNew.twist.twist.angular.z = cycleAngle / (odomNew.header.stamp.sec - odomOld.header.stamp.sec);
//         // Save the pose data for the next cycle
//         odomOld.pose.pose.position.x = odomNew.pose.pose.position.x;
//         odomOld.pose.pose.position.y = odomNew.pose.pose.position.y;
//         odomOld.pose.pose.orientation.z = odomNew.pose.pose.orientation.z;
//         odomOld.header.stamp = odomNew.header.stamp;

//         // Publish the odometry message
//         odom_data_pub->publish(odomNew);
//     }

//     void timer_callback()
//     {
//         odom();
//         set_initial_2D();

//         if (initialPoseRecieved)
//         {
//             update_odom();
//             publish_Quaternion();
//         }
//     }

//     void odom()
//     {
//         distance_A = Distance_Calculation(rot_A);
//         distance_B = Distance_Calculation(rot_B);

//         distance_Avg = ((abs(distance_A) + abs(distance_B)) / 2.0);

//         delta_Orientation = ((distance_B - distance_A) / distance_btwn_wheels);
//         // RCLCPP_INFO(this->get_logger(), "Distance A= %f....Distance B= %f....Distance Avg=%f....Orientation =%f", distance_A, distance_B, distance_Avg, delta_Orientation);

//         // Average angle during last cycle double cycleAngle = asin((distance_A - distance_B) / distance_btwn_wheels);

//         // Average angle during the last cycle double avgAngle = cycleAngle / 2 + odomOld.pose.pose.orientation.z;

//         new_Orientation = old_Orientation + delta_Orientation;

//         delta_X = distance_Avg * cos(delta_Orientation);
//         delta_Y = distance_Avg * sin(delta_Orientation);

//         new_X = old_X + delta_X;
//         new_Y = old_Y + delta_Y;

//         old_X = new_X;
//         old_Y = new_Y;
//         old_Orientation = new_Orientation;

//         RCLCPP_INFO(this->get_logger(), "X= %f....Y= %f....Distance Avg=%f....Orientation =%f", delta_X, delta_Y, distance_Avg, delta_Orientation);
//     }

//     std::vector<std::string> split(const std::string &s, char delimiter)
//     {
//         std::vector<std::string> result;
//         std::stringstream ss(s);
//         std::string item;

//         while (getline(ss, item, delimiter))
//         {
//             result.push_back(item);
//         }
//         return result;
//     }
// };

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);               // initialize ROS 2 communication
//     auto node = std::make_shared<D_Calc>(); // creating a shared ponter to the node
//     rclcpp::spin(node);                     // loop as node is executed
//     rclcpp::shutdown();                     // shutdown the communication, stop spinning
//     return 0;
// }
