#include <cstdio>
#include <functional>
#include <memory>
#include <cmath>
#include <algorithm>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/u_int8_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

double getAbsMaxElement(const double* arr, const unsigned int length){
  double maxAbs = 0;
  double currAbs = 0;
  for(unsigned int i = 0; i < length; i++){
    currAbs = std::abs(arr[i]);
    if(currAbs > maxAbs)
      maxAbs = currAbs;
  }

  return maxAbs;
}

class Twist2BigOmniControl : public rclcpp::Node
{
  private:
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription;
	rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr publisher;
	double r_wheel = 1;// for future use
	double r_robot = 1;// for future use

	void onMessage(const geometry_msgs::msg::Twist::SharedPtr msgIn){
	  double linX = msgIn->linear.x;  // x is the forward direction
	  double linY = msgIn->linear.y;  // y is the left direction
	  double angZ = msgIn->angular.z; // z points up

	  // calculate wheel velocities
	  double u[3];
	  u[0] = ( std::sin(M_PI/3) * linX + 0.5 * linY + angZ * r_robot) / r_wheel; //front right wheel
	  u[1] = (-std::sin(M_PI/3) * linX + 0.5 * linY + angZ * r_robot) / r_wheel; //front left wheel
	  u[2] = (	  			 -linY + angZ * r_robot) / r_wheel; //rear wheel

	  // limit max velocities
	  double absMax = getAbsMaxElement(u,3);
	  if(absMax > 1){
	    for(uint8_t i = 0; i < 3; i++){
	      u[i] /= 3;
	    }
	  }

	  // construct message
	  uint8_t data[] = {0,0,0,0,0,0};
	  for(uint8_t i = 0; i < 3; i++){
	    if(u[i] >= 0)
	      data[i*2] = (uint8_t)(u[i] * 255);
	    else
	      data[i*2+1] = (uint8_t)(std::abs(u[i]) * 255);
	  }
	  
	  // compose message
	  std_msgs::msg::UInt8MultiArray msgOut = std_msgs::msg::UInt8MultiArray();
	  msgOut.layout.data_offset = 0;
	  msgOut.layout.dim.push_back(std_msgs::msg::MultiArrayDimension());
	  msgOut.layout.dim[0].size = 6;
	  msgOut.layout.dim[0].stride = 1;
	  msgOut.layout.dim[0].label = "length";
	  for(uint8_t i = 0; i < 6; i++){
	    msgOut.data.push_back(data[i]);
	  }
	  this->publisher->publish(msgOut);
	}
  public:
	Twist2BigOmniControl() : Node("twist2bigomni"){
	  this->subscription = this->create_subscription<geometry_msgs::msg::Twist>(
	  	"cmd_vel", 10, std::bind(&Twist2BigOmniControl::onMessage, this, _1));
	  this->publisher = this->create_publisher<std_msgs::msg::UInt8MultiArray>(
		"control", rclcpp::SensorDataQoS());
  	  RCLCPP_INFO(this->get_logger(), "Twist to BigOmni converter started");
	}
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Twist2BigOmniControl>());
  rclcpp::shutdown();
  return 0;
}
