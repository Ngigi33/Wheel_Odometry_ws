#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <libserial/SerialPort.h>

using std::placeholders::_1;

class Serial_Class : public rclcpp::Node
{
public:
  Serial_Class() : Node("SerialNode1")
  {
    declare_parameter<std::string>("port", "/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_A50285BI-if00-port0");

    port_ = get_parameter("port").as_string();

    stm_.Open(port_);

    if (stm_.IsOpen())
    {
      RCLCPP_INFO(this->get_logger(), "PORT IS OPEN");
    }

    subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 1, std::bind(&Serial_Class::send_msg_callback, this, std::placeholders::_1));

    stm_.SetBaudRate(LibSerial::BaudRate::BAUD_115200);
    wheel_odom_publisher_ = this->create_publisher<std_msgs::msg::String>("/distance_finder", 1);
    timer_ = this->create_wall_timer(std::chrono::microseconds(1000), std::bind(&Serial_Class::msgCallback, this));

    // subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
    //     "/cmd_vel", 1, std::bind(&Serial_Class::msgCallback, this, std::placeholders::_1));

    // msgCallback();
  }

  ~Serial_Class()
  {
    stm_.Close();
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscriber_;
  std::string port_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr wheel_odom_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  LibSerial::SerialPort stm_;

  void send_msg_callback(const geometry_msgs::msg::Twist::SharedPtr data)
  {
    // RCLCPP_INFO_STREAM(this->get_logger(), "Sending Message ");
    RCLCPP_INFO(this->get_logger(), "Callback triggered.");
    RCLCPP_INFO(this->get_logger(), "Linear:[%f,%f,%f]",
                data->linear.x, data->linear.y, data->linear.z);

    if ((data->linear.x) > 0.0) // Moving Forward
    {
      /* 1st 3 ->  speed for Motor A
         2nd 3 ->  speed for Motor B
         7th bit  -> Motor A direction
         8th bit  -> Motor B direction
         */
      int speed = 150150;
      // std::string vel = "120120";
      std::string vel = std::to_string(speed);
      std::string data_to_send = vel;
      RCLCPP_INFO(this->get_logger(), "Speed Sent to STM: %s Moving Forward", data_to_send.c_str());
      stm_.Write(data_to_send.c_str());
    }

    else if ((data->linear.x) < 0.0) // Moving Backward
    {
      int speed = 250250;
      // std::string vel = "12012011";
      std::string vel = std::to_string(speed);
      std::string data_to_send = vel;
      RCLCPP_INFO(this->get_logger(), "Speed Sent to STM: %s Moving Backward ", data_to_send.c_str());
      stm_.Write(data_to_send.c_str());
    }
    else if ((data->angular.z) < 0.0) // Moving to Right
    {
      int speed = 150250;
      // std::string vel = "12012001";
      std::string vel = std::to_string(speed);
      std::string data_to_send = vel;
      RCLCPP_INFO(this->get_logger(), "Speed Sent to STM: %s Moving Right ", data_to_send.c_str());
      stm_.Write(data_to_send.c_str());
    }
    else if ((data->angular.z) > 0.0) // Moving to Left
    {
      int speed = 250150;
      // std::string vel = "12012010";
      std::string vel = std::to_string(speed);
      std::string data_to_send = vel;
      RCLCPP_INFO(this->get_logger(), "Speed Sent to STM: %s Moving Left", data_to_send.c_str());
      stm_.Write(data_to_send.c_str());
    }

    else if ((data->linear.x) == 0.0 && (data->angular.z) == 0.0)
    {
      // int speed = 000000;
      std::string vel = "000000";
      std::string data_to_send = vel;
      RCLCPP_INFO(this->get_logger(), "Speed Sent to STM: %s", data_to_send.c_str());
      stm_.Write(data_to_send);
    }
  }

  void publish_Odom(const std_msgs::msg::String Data_)
  {
    wheel_odom_publisher_->publish(Data_);
  }

  void msgCallback()
  {

    auto message = std_msgs::msg::String();
    if (rclcpp::ok() && stm_.IsDataAvailable())
    {
      stm_.ReadLine(message.data);
      // RCLCPP_INFO_STREAM(this->get_logger(), "MSG received from Stm:" << message.data);
      publish_Odom(message);
    }
  }
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Serial_Class>());
  rclcpp::shutdown();
  return 0;
}