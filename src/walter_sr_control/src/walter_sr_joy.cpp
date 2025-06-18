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
  ForwardingSubscriber(const std::string& joy_sub, 
    const std::string& head_left_thigh_sub, const std::string& head_left_thigh_pub, 
    const std::string& head_left_shin_sub, const std::string& head_left_shin_pub, 
    const std::string& head_right_thigh_sub, const std::string& head_right_thigh_pub, 
    const std::string& head_right_shin_sub, const std::string& head_right_shin_pub, 
    const std::string& torso_left_thigh_sub, const std::string& torso_left_thigh_pub, 
    const std::string& torso_left_shin_sub, const std::string& torso_left_shin_pub, 
    const std::string& torso_right_thigh_sub, const std::string& torso_right_thigh_pub, 
    const std::string& torso_right_shin_sub, const std::string& torso_right_shin_pub)
  : Node("walter_sr_joy_node"),  
  // joy_shin(0.0), // Initialize with a default value
  // joy_thigh(0.0),
  hl_thigh_pos_(0.0), 
  hr_thigh_pos_(0.0),
  tl_thigh_pos_(0.0),
  tr_thigh_pos_(0.0)
  {
    // joystick data
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_sub, 10, std::bind(&ForwardingSubscriber::joy_callback, this, _1));

    
    // head left
    head_left_thigh_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
      head_left_thigh_sub, 10, std::bind(&ForwardingSubscriber::head_left_thigh_callback, this, _1));
    head_left_thigh_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(head_left_thigh_pub, 10);

    // head_left_shin_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
    //   head_left_shin_sub, 10, std::bind(&ForwardingSubscriber::head_left_shin_callback, this, _1));
    head_left_shin_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(head_left_shin_pub, 10);

    // head right
    head_right_thigh_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
      head_right_thigh_sub, 10, std::bind(&ForwardingSubscriber::head_right_thigh_callback, this, _1));
    head_right_thigh_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(head_right_thigh_pub, 10);

    // head_right_shin_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
    //   head_right_shin_sub, 10, std::bind(&ForwardingSubscriber::head_right_shin_callback, this, _1));
    head_right_shin_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(head_right_shin_pub, 10);    


    // torso left
    torso_left_thigh_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
      torso_left_thigh_sub, 10, std::bind(&ForwardingSubscriber::torso_left_thigh_callback, this, _1));
    torso_left_thigh_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(torso_left_thigh_pub, 10);

    // torso_left_shin_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
    //   torso_left_shin_sub, 10, std::bind(&ForwardingSubscriber::torso_left_shin_callback, this, _1));
    torso_left_shin_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(torso_left_shin_pub, 10);

    // torso right
    torso_right_thigh_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
      torso_right_thigh_sub, 10, std::bind(&ForwardingSubscriber::torso_right_thigh_callback, this, _1));
    torso_right_thigh_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(torso_right_thigh_pub, 10);

    // torso_right_shin_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
    //   torso_right_shin_sub, 10, std::bind(&ForwardingSubscriber::torso_right_shin_callback, this, _1));
    torso_right_shin_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(torso_right_shin_pub, 10);    
  }  

private:
  void head_left_thigh_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg)
  {
    hl_thigh_pos_ = msg->pos_estimate;    
    processAndPublish();
  }
  void head_right_thigh_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg)
  {
    hr_thigh_pos_ = msg->pos_estimate;    
    processAndPublish();
  }
  void torso_left_thigh_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg)
  {
    tl_thigh_pos_ = msg->pos_estimate;    
    processAndPublish();
  }
  void torso_right_thigh_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg)
  {
    tr_thigh_pos_ = msg->pos_estimate;    
    processAndPublish();
  }
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // shins
    left_analog_ = msg->axes[0];    
    // thighs
    right_analog_ = msg->axes[3];    
    processAndPublish();
  }  

  void processAndPublish()
  {
    if (!std::isnan(hl_thigh_pos_) && !std::isnan(hr_thigh_pos_) &&
        !std::isnan(tl_thigh_pos_) && !std::isnan(tr_thigh_pos_) && !std::isnan(left_analog_) && !std::isnan(right_analog_)) {
          float max_thigh_angle = 0.4;
          if ((hl_thigh_pos_ < max_thigh_angle) && (hl_thigh_pos_ > -max_thigh_angle) && 
          (hr_thigh_pos_ < max_thigh_angle) && (hr_thigh_pos_ > -max_thigh_angle) && 
          (tl_thigh_pos_ < max_thigh_angle) && (tl_thigh_pos_ > -max_thigh_angle) && 
          (tr_thigh_pos_ < max_thigh_angle) && (tr_thigh_pos_ > -max_thigh_angle)){
            // Shin message - positive - cw 
            auto message = odrive_can::msg::ControlMessage();
            auto message_rev = odrive_can::msg::ControlMessage();
            message.input_vel = left_analog_*0.5;
            message.control_mode = 2;
            message.input_mode = 2;
            message.input_pos = 0.0;
            message.input_torque = 0.0;    

            message_rev.input_vel = -left_analog_*0.5;
            message_rev.control_mode = 2;
            message_rev.input_mode = 2;
            message_rev.input_pos = 0.0;
            message_rev.input_torque = 0.0;    
            
            head_left_shin_pub_->publish(message_rev);
            head_right_shin_pub_->publish(message);
            torso_left_shin_pub_->publish(message_rev);
            torso_right_shin_pub_->publish(message);

            // Thigh message - positive - ccw
            auto message2 = odrive_can::msg::ControlMessage();
            auto message2_rev = odrive_can::msg::ControlMessage();
            // message2.input_vel = right_analog_ * 3.0;
            message2.input_vel = right_analog_*0.5;
            message2.control_mode = 2;
            message2.input_mode = 2;
            message2.input_pos = 0.0;
            message2.input_torque = 0.0;

            message2_rev.input_vel = -right_analog_*0.5;
            message2_rev.control_mode = 2;
            message2_rev.input_mode = 2;
            message2_rev.input_pos = 0.0;
            message2_rev.input_torque = 0.0;

            head_left_thigh_pub_->publish(message2);
            head_right_thigh_pub_->publish(message2_rev);
            torso_left_thigh_pub_->publish(message2);
            torso_right_thigh_pub_->publish(message2_rev);
          }
    }    
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr head_left_thigh_sub_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr head_right_thigh_sub_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr torso_left_thigh_sub_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr torso_right_thigh_sub_;

  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr head_left_thigh_pub_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr head_left_shin_pub_;

  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr head_right_thigh_pub_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr head_right_shin_pub_;

  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr torso_left_thigh_pub_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr torso_left_shin_pub_;

  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr torso_right_thigh_pub_;
  rclcpp::Publisher<odrive_can::msg::ControlMessage>::SharedPtr torso_right_shin_pub_;

  float left_analog_;
  float right_analog_;
  float hl_thigh_pos_;
  float hr_thigh_pos_;  
  float tl_thigh_pos_;
  float tr_thigh_pos_;   
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  // Specify the subscribing and publishing topic names
  // joystick
  std::string joy_sub = "joy";

  //      all 4 legs 

  // head left
  std::string head_left_thigh_sub = "/axis3/controller_status";
  std::string head_left_thigh_pub = "/axis3/control_message";

  std::string head_left_shin_sub = "/axis2/controller_status";
  std::string head_left_shin_pub = "/axis2/control_message";
  
  // head right
  std::string head_right_thigh_sub = "/axis1/controller_status";
  std::string head_right_thigh_pub = "/axis1/control_message";

  std::string head_right_shin_sub = "/axis0/controller_status";
  std::string head_right_shin_pub = "/axis0/control_message";


  // torso left
  std::string torso_left_thigh_sub = "/axis5/controller_status";
  std::string torso_left_thigh_pub = "/axis5/control_message";

  std::string torso_left_shin_sub = "/axis4/controller_status";
  std::string torso_left_shin_pub = "/axis4/control_message";
  
  // torso right
  std::string torso_right_thigh_sub = "/axis7/controller_status";
  std::string torso_right_thigh_pub = "/axis7/control_message";

  std::string torso_right_shin_sub = "/axis6/controller_status";
  std::string torso_right_shin_pub = "/axis6/control_message";  
  
  
  rclcpp::spin(std::make_shared<ForwardingSubscriber>(joy_sub, 
    head_left_thigh_sub, head_left_thigh_pub,
    head_left_shin_sub, head_left_shin_pub,
    head_right_thigh_sub, head_right_thigh_pub,
    head_right_shin_sub, head_right_shin_pub,
    torso_left_thigh_sub, torso_left_thigh_pub,
    torso_left_shin_sub, torso_left_shin_pub,
    torso_right_thigh_sub, torso_right_thigh_pub,
    torso_right_shin_sub, torso_right_shin_pub));
  rclcpp::shutdown();
  return 0;
}