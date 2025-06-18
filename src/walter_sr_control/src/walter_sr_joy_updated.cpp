#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "/home/orl-orin/repository/testing/install/odrive_can/include/odrive_can/odrive_can/msg/control_message.hpp"
#include "/home/orl-orin/repository/testing/install/odrive_can/include/odrive_can/odrive_can/msg/o_drive_status.hpp"
#include "/home/orl-orin/repository/testing/install/odrive_can/include/odrive_can/odrive_can/msg/controller_status.hpp"

#include <cmath>   // For round, floor, ceil, trunc
#include <limits>    // For std::numeric_limits
#include <iomanip>   // For std::fixed, std::setprecision
#include <sstream>   // For std::stringstream


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif



using std::placeholders::_1;


double round_to_nearest_pi(double value) {
    // Handle potential edge cases for very small or NaN/Inf values
    if (std::isnan(value)) {
        return std::numeric_limits<double>::quiet_NaN();
    }
    if (std::isinf(value)) {
        return value; // Infinity remains infinity
    }
    // No need for explicit 0.0 check, as 0.0 / M_PI is 0.0, std::round(0.0) is 0.0, 0.0 * M_PI is 0.0.

    // Step 1: Divide the number by PI
    double ratio = value / 4.74;

    // Step 2: Round the ratio to the nearest integer
    double rounded_ratio_int = std::round(ratio);

    // Step 3: Multiply the rounded integer ratio by PI
    return rounded_ratio_int * 4.74;
}


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
  : Node("walter_sr_joy_updated_node"),  
  // joy_shin(0.0), // Initialize with a default value
  // joy_thigh(0.0),
  hl_thigh_pos_(0.0), 
  hr_thigh_pos_(0.0),
  tl_thigh_pos_(0.0),
  tr_thigh_pos_(0.0),
  hl_shin_pos_(0.0), 
  hr_shin_pos_(0.0),
  tl_shin_pos_(0.0),
  tr_shin_pos_(0.0)  
  {
    // joystick data
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
      joy_sub, 10, std::bind(&ForwardingSubscriber::joy_callback, this, _1));
        
      
    // head left
    head_left_thigh_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
      head_left_thigh_sub, 10, std::bind(&ForwardingSubscriber::head_left_thigh_callback, this, _1));
    head_left_thigh_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(head_left_thigh_pub, 10);
    
    head_left_shin_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
      head_left_shin_sub, 10, std::bind(&ForwardingSubscriber::head_left_shin_callback, this, _1));
    head_left_shin_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(head_left_shin_pub, 10);
    
    // head right
    head_right_thigh_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
      head_right_thigh_sub, 10, std::bind(&ForwardingSubscriber::head_right_thigh_callback, this, _1));
    head_right_thigh_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(head_right_thigh_pub, 10);

    head_right_shin_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
      head_right_shin_sub, 10, std::bind(&ForwardingSubscriber::head_right_shin_callback, this, _1));
    head_right_shin_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(head_right_shin_pub, 10);    


    // torso left
    torso_left_thigh_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
      torso_left_thigh_sub, 10, std::bind(&ForwardingSubscriber::torso_left_thigh_callback, this, _1));
    torso_left_thigh_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(torso_left_thigh_pub, 10);

    torso_left_shin_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
      torso_left_shin_sub, 10, std::bind(&ForwardingSubscriber::torso_left_shin_callback, this, _1));
    torso_left_shin_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(torso_left_shin_pub, 10);

    // torso right
    torso_right_thigh_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
      torso_right_thigh_sub, 10, std::bind(&ForwardingSubscriber::torso_right_thigh_callback, this, _1));
    torso_right_thigh_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(torso_right_thigh_pub, 10);

    torso_right_shin_sub_ = this->create_subscription<odrive_can::msg::ControllerStatus>(
      torso_right_shin_sub, 10, std::bind(&ForwardingSubscriber::torso_right_shin_callback, this, _1));
    torso_right_shin_pub_ = this->create_publisher<odrive_can::msg::ControlMessage>(torso_right_shin_pub, 10);    
  }  

private:
// thigh callbacks
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
// shin callbacks
  void head_left_shin_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg)
  {
    hl_shin_pos_ = msg->pos_estimate;    
    processAndPublish();
  }
  void head_right_shin_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg)
  {
    hr_shin_pos_ = msg->pos_estimate;    
    processAndPublish();
  }
  void torso_left_shin_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg)
  {
    tl_shin_pos_ = msg->pos_estimate;    
    processAndPublish();
  }
  void torso_right_shin_callback(const odrive_can::msg::ControllerStatus::SharedPtr msg)
  {
    tr_shin_pos_ = msg->pos_estimate;    
    processAndPublish();
  }  
// joy callback
  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    // shins
    left_analog_ = msg->axes[0];    
    // thighs
    right_analog_ = msg->axes[3];

    // modes
    y_button_ = msg->buttons[6];
    x_button_ = msg->buttons[7];
    b_button_ = msg->buttons[8];
    a_button_ = msg->buttons[9];
    processAndPublish();
  }  

  void processAndPublish()
  {
    if (!std::isnan(hl_thigh_pos_) && !std::isnan(hr_thigh_pos_) &&
        !std::isnan(tl_thigh_pos_) && !std::isnan(tr_thigh_pos_) && 
        !std::isnan(hl_shin_pos_) && !std::isnan(hr_shin_pos_) &&
        !std::isnan(tl_shin_pos_) && !std::isnan(tr_shin_pos_) && 
        !std::isnan(left_analog_) && !std::isnan(right_analog_) &&
        !std::isnan(y_button_) && !std::isnan(x_button_) && !std::isnan(b_button_) && !std::isnan(a_button_)){
          
          if (a_button_ == 1){
            head_left_thigh_des = 0.0;
            head_right_thigh_des = 0.0;
            torso_left_thigh_des = 0.0;
            torso_right_thigh_des = 0.0;
            head_left_shin_des = round_to_nearest_pi(hl_shin_pos_);
            head_right_shin_des = round_to_nearest_pi(hr_shin_pos_);
            torso_left_shin_des = round_to_nearest_pi(tl_shin_pos_);
            torso_right_shin_des = round_to_nearest_pi(tr_shin_pos_);
          } else if (y_button_ == 1){
            head_left_thigh_des = 1.0;
            head_right_thigh_des = -1.0;
            torso_left_thigh_des = -1.0;
            torso_right_thigh_des = 1.0;
          } else {
            head_left_thigh_des = hl_thigh_pos_ + (left_analog_ * 0.2);
            head_right_thigh_des = hr_thigh_pos_ - (left_analog_ * 0.2);
            torso_left_thigh_des = tl_thigh_pos_ + (left_analog_ * 0.2);
            torso_right_thigh_des = tr_thigh_pos_ - (left_analog_ * 0.2);
            head_left_shin_des = hl_shin_pos_ + (right_analog_ * 0.2);
            head_right_shin_des = hr_shin_pos_ - (right_analog_ * 0.2);
            torso_left_shin_des = tl_shin_pos_ + (right_analog_ * 0.2);
            torso_right_shin_des = tr_shin_pos_ - (right_analog_ * 0.2);
          }

          // Shin messages 
          auto head_right_shin_message = odrive_can::msg::ControlMessage();
          head_right_shin_message.input_vel = 0.0;
          head_right_shin_message.control_mode = 3;
          head_right_shin_message.input_mode = 5;
          head_right_shin_message.input_pos = head_right_shin_des;
          head_right_shin_message.input_torque = 0.0;

          auto torso_right_shin_message = odrive_can::msg::ControlMessage();
          torso_right_shin_message.input_vel = 0.0;
          torso_right_shin_message.control_mode = 3;
          torso_right_shin_message.input_mode = 5;
          torso_right_shin_message.input_pos = torso_right_shin_des;
          torso_right_shin_message.input_torque = 0.0;    

          auto head_left_shin_message = odrive_can::msg::ControlMessage();
          head_left_shin_message.input_vel = 0.0;
          head_left_shin_message.control_mode = 3;
          head_left_shin_message.input_mode = 5;
          head_left_shin_message.input_pos = head_left_shin_des;
          head_left_shin_message.input_torque = 0.0;

          auto torso_left_shin_message = odrive_can::msg::ControlMessage();
          torso_left_shin_message.input_vel = 0.0;
          torso_left_shin_message.control_mode = 3;
          torso_left_shin_message.input_mode = 5;
          torso_left_shin_message.input_pos = torso_left_shin_des;
          torso_left_shin_message.input_torque = 0.0;    
          
          head_left_shin_pub_->publish(head_left_shin_message);
          head_right_shin_pub_->publish(head_right_shin_message);
          torso_left_shin_pub_->publish(torso_left_shin_message);
          torso_right_shin_pub_->publish(torso_right_shin_message);

          // Thigh messages 
          auto head_right_thigh_message = odrive_can::msg::ControlMessage();
          head_right_thigh_message.input_vel = 0.0;
          head_right_thigh_message.control_mode = 3;
          head_right_thigh_message.input_mode = 5;
          head_right_thigh_message.input_pos = head_right_thigh_des;
          head_right_thigh_message.input_torque = 0.0;

          auto torso_right_thigh_message = odrive_can::msg::ControlMessage();
          torso_right_thigh_message.input_vel = 0.0;
          torso_right_thigh_message.control_mode = 3;
          torso_right_thigh_message.input_mode = 5;
          torso_right_thigh_message.input_pos = torso_right_thigh_des;
          torso_right_thigh_message.input_torque = 0.0;    

          auto head_left_thigh_message = odrive_can::msg::ControlMessage();
          head_left_thigh_message.input_vel = 0.0;
          head_left_thigh_message.control_mode = 3;
          head_left_thigh_message.input_mode = 5;
          head_left_thigh_message.input_pos = head_left_thigh_des;
          head_left_thigh_message.input_torque = 0.0;

          auto torso_left_thigh_message = odrive_can::msg::ControlMessage();
          torso_left_thigh_message.input_vel = 0.0;
          torso_left_thigh_message.control_mode = 3;
          torso_left_thigh_message.input_mode = 5;
          torso_left_thigh_message.input_pos = torso_left_thigh_des;
          torso_left_thigh_message.input_torque = 0.0;    
          
          head_left_thigh_pub_->publish(head_left_thigh_message);
          head_right_thigh_pub_->publish(head_right_thigh_message);
          torso_left_thigh_pub_->publish(torso_left_thigh_message);
          torso_right_thigh_pub_->publish(torso_right_thigh_message);          
    }    
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr head_left_thigh_sub_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr head_right_thigh_sub_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr torso_left_thigh_sub_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr torso_right_thigh_sub_;

  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr head_left_shin_sub_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr head_right_shin_sub_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr torso_left_shin_sub_;
  rclcpp::Subscription<odrive_can::msg::ControllerStatus>::SharedPtr torso_right_shin_sub_;

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
  bool y_button_;
  bool x_button_;
  bool b_button_;
  bool a_button_;
  float hl_thigh_pos_;
  float hr_thigh_pos_;  
  float tl_thigh_pos_;
  float tr_thigh_pos_;
  float hl_shin_pos_;
  float hr_shin_pos_;
  float tl_shin_pos_;
  float tr_shin_pos_;

  float head_left_thigh_des;
  float head_right_thigh_des;
  float torso_left_thigh_des;
  float torso_right_thigh_des;
  float head_left_shin_des;
  float head_right_shin_des;
  float torso_left_shin_des;
  float torso_right_shin_des;

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