#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "/home/orl-orin/repository/testing/install/odrive_can/include/odrive_can/odrive_can/msg/control_message.hpp"
#include "/home/orl-orin/repository/testing/install/odrive_can/include/odrive_can/odrive_can/msg/o_drive_status.hpp"
#include "/home/orl-orin/repository/testing/install/odrive_can/include/odrive_can/odrive_can/msg/controller_status.hpp"

using std::placeholders::_1;

class ForwardingSubscriber : public rclcpp::Node
{
public:
  ForwardingSubscriber(const std::string& subscribe_topic1, const std::string& subscribe_topic2, const std::string& subscribe_topic3, const std::string& publish_topic1, const std::string& publish_topic2)
  : Node("forwarding_subscriber"),
  message1_from_topic_1_(0.0), // Initialize with a default value
  message2_from_topic_1_(0.0),
  message_from_topic_2_(0.0),
  message_from_topic_3_(0.0)
  {
    // joystick data
    subscription1_ = this->create_subscription<sensor_msgs::msg::Joy>(
      subscribe_topic1, 10, std::bind(&ForwardingSubscriber::topic_callback1, this, _1));
    // axis1 - shin - controller data - position and so on
    subscription2_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
      subscribe_topic2, 10, std::bind(&ForwardingSubscriber::topic_callback2, this, _1));
    // axis0 - thigh - controller data - position and so on
    subscription3_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
      subscribe_topic3, 10, std::bind(&ForwardingSubscriber::topic_callback3, this, _1));
    // publish to axis1 - shin - control message
    publisher1_ = this->create_publisher<odrive_can::msg::ControlMessage>(publish_topic1, 10);
    // publish to axis0 - thigh - control message
    publisher2_ = this->create_publisher<odrive_can::msg::ControlMessage>(publish_topic2, 10);
  }  

private:
  void topic_callback2(const odrive_can::msg::ControllerStatus::SharedPtr msg)
  {
    message_from_topic_2_ = msg->pos_estimate;    
    processAndPublish();
  }
  void topic_callback3(const odrive_can::msg::ControllerStatus::SharedPtr msg)
  {
    message_from_topic_3_ = msg->pos_estimate;    
    processAndPublish();
  }
  void topic_callback1(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    message1_from_topic_1_ = msg->axes[0];    
    message2_from_topic_1_ = msg->axes[3];    
    processAndPublish();
  }  

  void processAndPublish()
  {
    if (!std::isnan(message1_from_topic_1_) && !std::isnan(message2_from_topic_1_) &&
        !std::isnan(message_from_topic_2_) && !std::isnan(message_from_topic_3_)) {
      // Shin message 
      auto message = odrive_can::msg::ControlMessage();
      message.input_vel = message1_from_topic_1_;
      message.control_mode = 2;
      message.input_mode = 1;
      message.input_pos = 0.0;
      message.input_torque = 0.0;    
      publisher1_->publish(message);

      // Thigh message 
      auto message2 = odrive_can::msg::ControlMessage();
      // if (message2_from_topic_1_>0.1){
      //   message2.input_pos = message_from_topic_3_ + 0.2;
      // }else if (message2_from_topic_1_<-0.1){
      //   message2.input_pos = message_from_topic_3_ - 0.2;
      // }
      // else {
      //   message2.input_pos = message_from_topic_3_;
      // }
      // message2.control_mode = 3;
      // message2.input_mode = 5;
      // message2.input_vel = 0.0;
      // message2.input_torque = 0.0;    
      // publisher2_->publish(message2);
      message2.input_vel = message2_from_topic_1_ * 3.0;
      message2.control_mode = 2;
      message2.input_mode = 2;
      message2.input_pos = 0.0;
      message2.input_torque = 0.0;    
      publisher2_->publish(message2);

    }    
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr subscription1_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr subscription2_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr subscription3_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr publisher1_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr publisher2_;
  std::string publish_topic_name_;
  float message1_from_topic_1_;
  float message2_from_topic_1_;
  float message_from_topic_2_;
  float message_from_topic_3_;  
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Specify the subscribing and publishing topic names
  std::string subscribe_topic1 = "joy";
  std::string subscribe_topic2 = "/axis1/controller_status";
  std::string subscribe_topic3 = "/axis0/controller_status";
  std::string publish_topic1 = "/axis1/control_message";
  std::string publish_topic2 = "/axis0/control_message";
  rclcpp::spin(std::make_shared<ForwardingSubscriber>(subscribe_topic1, subscribe_topic2, subscribe_topic3, publish_topic1, publish_topic2));
  rclcpp::shutdown();
  return 0;
}