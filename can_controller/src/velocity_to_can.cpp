#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <tuple>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/Twist.h"

// TODO: Install the following libraries:
#include "ctre/Phoenix.h"
#include "ctre/phoenix/platform/Platform.h"
#include "ctre/phoenix/unmanaged/Unmanaged.h"
#include "ctre/phoenix/cci/Unmanaged_CCI.h"

using namespace std::chrono_literals;
using namespace ctre::phoenix;
using namespace ctre::phoenix::platform;
using namespace ctre::phoenix::motorcontrol;
using namespace ctre::phoenix::motorcontrol::can;

// TODO: Update these port numbers!
TalonSRX talLeft(1);
TalonSRX talRght(0);

talLeft.ConfigSelectedFeedbackSensor(1); // TODO: This is the number for the IntegratedEncoder, might be incorrect?
talRight.ConfigSelectedFeedbackSensor(1);

int kPIDLoopIdx = 0;
int kTimeoutMs = 30;

void configPIDF(TalonSRX talon, double P, double I, double D, double F){
  talon.Config_kF(kPIDLoopIdx, P, kTimeoutMs); // TODO: Tune PIDF values according to https://phoenix-documentation.readthedocs.io/en/latest/ch16_ClosedLoop.html.
  talon.Config_kP(kPIDLoopIdx, I, kTimeoutMs); // CHECK WHILE TESTING: this might have to be moved to main, alongside the ConfigSelectedFeedbackSensor()
  talon.Config_kI(kPIDLoopIdx, D, kTimeoutMs);
  talon.Config_kD(kPIDLoopIdx, F, kTimeoutMs);
}

configPIDF(talLeft, 0, 0, 0, 0);
configPIDF(talRight, 0, 0, 0, 0);

void initDrive()
{
	talRght.SetInverted(false); /* both talons should blink green when driving forward */
}

void drive(double left, double right)
{
	talLeft.Set(ControlMode::Velocity, left);
	talRght.Set(ControlMode::Velocity, right);
}

class TalonNode : public rclcpp::Node
{
  public:
    TalonNode()
    : Node("talon_interface")
    {
      motion_subscriber_ = this->create_subscription<tuple>("PLACEHOLDER_VELOCITY_TOPIC", 10, std::bind(&PathfinderSubscriber::topic_callback, this, _1));
      left_encoder_publisher_ = this->create_publisher<geometry_msgs::Twist>("cmd_vel/left_wheel_encoder_position", 10)
      right_encoder_publisher_ = this->create_publisher<geometry_msgs::Twist>("cmd_vel/right_wheel_encoder_position", 10)
      timer_ = this->create_wall_timer(20ms, std::bind(&TalonNode::timer_callback, this)); //TODO: Change timer time.
    }
  //TODO: WARNING—CURRENTLY, THIS AUTOMATICALLY RUNS THE MOTORS WHENEVER THIS NODE IS RUNNING... JUST FYI.
  private:
    void topic_callback(const tuple::SharedPtr msg) const
    {
      double left_vel = get<0>msg;
      double right_vel = get<1>msg;
      drive(left_vel, right_vel);
    }
    
    void timer_callback()
    {
      geometry_msgs::Twist left_encoder_out;
      geometry_msgs::Twist right_encoder_out;
      left_encoder_out.linear.x = talLeft.getSelectedSensorVelocity();
      right_encoder_out.linear.x = talRight.getSelectedSensorVelocity();
      left_encoder_publisher_->publish(left_encoder_out);
      right_encoder_publisher_->publish(right_encoder_out);
    }
    rclcpp::Subscription<tuple>::SharedPtr motion_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::Twist>::SharedPtr left_encoder_publisher_;
    rclcpp::Publisher<geometry_msgs::Twist>::SharedPtr right_encoder_publisher_;
    size_t count_;

};

int main(int argc, char ** argv)
{
  std::string interface = "can0";
  ctre::phoenix::platform::can::SetCANInterface(interface.c_str()); 
  initDrive();
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
