// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <memory>
#include <iostream>

#include "rclcpp/rclcpp.hpp"
#include <rclcpp_action/rclcpp_action.hpp>
#include "std_msgs/msg/string.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <turtlesim/action/rotate_absolute.hpp>

#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include <string.h>
#include <chrono>


using namespace std::chrono_literals;
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node
{ 
public:

  MinimalSubscriber()
  : Node("teleop_keyboard_node")
  {

    _x = 0.0;
    _x_factor = 0.1;

    _th = 0.0;
    _th_factor = 0.1;

    twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    
    action_client_ = rclcpp_action::create_client<nav2_msgs::action::NavigateToPose>(this, "navigate_to_pose");

    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    
    timer_=this->create_wall_timer(
      100ms, std::bind(&MinimalSubscriber::publishTwist, this));
    
  }

private:

  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::string mensj(msg->data.c_str());
    
    if(mensj == "UP" || mensj == "DOWN" || mensj == "RIGHT" || mensj == "STOP" || mensj=="LEFT"){
      double x = twist.linear.x;
      double x_factor = _x_factor;
      double th = twist.angular.z;
      double th_factor = _th_factor;

      navigating=true;

      if(mensj == "UP"){
        x = x + x_factor;
      } else if(mensj == "DOWN"){
        x = x - x_factor;
      } else if(mensj == "LEFT"){
        th = th + th_factor;
      } else if(mensj == "RIGHT"){
        th = th - th_factor;
      } else if(mensj == "STOP"){
        x = 0.0;
        th = 0.0;
      }

      if(th > 1.0){
        th = 1.0;
      } else if(th < -1.0){
        th = -1.0;
      }

      if(x > 1.0){
        x = 1.0;
      } else if(x < -1.0){
        x = -1.0;
      }

      RCLCPP_INFO(this->get_logger(), "Currently:\tspeed = '%f'\tturn = '%f'",x, th);
      
      //this->_x = x;
      //this->_th = th;

      twist.linear.x = x;
      twist.linear.y = 0.0;
      twist.linear.z = 0.0;
      twist.angular.x = 0.0;
      twist.angular.y = 0.0;
      twist.angular.z = th;
      //set(twist);
      
    } else if (mensj == "COCINA" || mensj == "SALON" || mensj == "HABITACION") {

      //geometry_msgs::msg::Pose pose;
      auto goal = nav2_msgs::action::NavigateToPose::Goal();
      goal.pose.header.frame_id = "map";
      navigating = false;
        
      if(mensj == "COCINA"){
        goal.pose.pose.position.x = 3.79;
        goal.pose.pose.position.y = 6.77;
        //goal.pose.pose.position.z = -1.9821582495221923e-06;
        goal.pose.pose.orientation.x = 0.0;
        goal.pose.pose.orientation.y = 0.00;
        goal.pose.pose.orientation.z = 0.99;
        goal.pose.pose.orientation.w = 0.12;
      } else if(mensj == "SALON"){
        goal.pose.pose.position.x = 1.55;
        goal.pose.pose.position.y = 4.03;
        //goal.pose.pose.position.z = 0.00010146076678398813;
        goal.pose.pose.orientation.x = 0.0;
        goal.pose.pose.orientation.y = 0.0;
        goal.pose.pose.orientation.z = -0.69;
        goal.pose.pose.orientation.w = 0.72;
      } else if(mensj == "HABITACION"){
        goal.pose.pose.position.x = 7.5;
        goal.pose.pose.position.y = 4.89;
        //goal.pose.pose.position.z = 1.610925595582971e-05;
        goal.pose.pose.orientation.x = 0.0;
        goal.pose.pose.orientation.y = 0.0;
        goal.pose.pose.orientation.z = 0.76;
        goal.pose.pose.orientation.w = 0.65;
      }

      //RCLCPP_INFO(this->get_logger(), "%s", mensj);
      
      //goal.pose = pose;

      action_client_->async_send_goal(goal);
     
    }
  }

  void publishTwist(){
    if(navigating){
      twist_pub_->publish(twist);
    }
  }
    
    //Speed
    double _x;
    double _x_factor;

    //Turn
    double _th;
    double _th_factor;

    bool navigating = true;

    rclcpp::TimerBase::SharedPtr timer_;
    geometry_msgs::msg::Twist twist;

    rclcpp_action::Client<nav2_msgs::action::NavigateToPose>::SharedPtr action_client_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_pub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}
