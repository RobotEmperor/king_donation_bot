/*
 * mobile_manager.h
 *
 *      Author: robotemperor
 */



#include <ros/ros.h>
#include <stdio.h>
#include <math.h>

//ros_communication_message type
#include <std_msgs/String.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

//custom header
#include <mobile_manager/motor_cmd.h>


//ros communication
ros::Publisher  motor1_pub;
ros::Publisher  motor2_pub;

ros::Publisher  motor3_pub;
ros::Publisher  motor4_pub;


//ros::Subscriber motor_theta_dist_sub;
ros::Subscriber joy_sub;


//ros msg
mobile_manager::motor_cmd motor_cmd_msg_1;
mobile_manager::motor_cmd motor_cmd_msg_2;
mobile_manager::motor_cmd motor_cmd_msg_3;
mobile_manager::motor_cmd motor_cmd_msg_4;


//variables
double move_x, move_y;
bool rotation_left, rotation_right;
double max_speed;
double speed_ratio_rad;


//function
void initialize();
void wheel_move_function(double x, double y);
void wheel_direction_group(bool motor1, bool motor2, bool motor3, bool motor4);
void wheel_rotation(bool rotation_left, bool rotation_right);
//callback
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);






