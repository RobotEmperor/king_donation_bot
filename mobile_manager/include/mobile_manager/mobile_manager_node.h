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
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Joy.h>

//custom header
#include <mobile_manager/motor_cmd.h>
#include <erica_arm_module_msgs/ArmCmd.h>
#include <robotis_math/robotis_math.h>


//ros communication
ros::Publisher  motor1_pub;
ros::Publisher  motor2_pub;

ros::Publisher  motor3_pub;
ros::Publisher  motor4_pub;


ros::Publisher arm_displacement_pub;
ros::Publisher script_number_pub;

ros::Publisher enable_module_pub;


//ros::Subscriber motor_theta_dist_sub;
ros::Subscriber joy_sub;


//ros msg
mobile_manager::motor_cmd motor_cmd_msg_1;
mobile_manager::motor_cmd motor_cmd_msg_2;
mobile_manager::motor_cmd motor_cmd_msg_3;
mobile_manager::motor_cmd motor_cmd_msg_4;

erica_arm_module_msgs::ArmCmd arm_displacement_msg;

std_msgs::Int32 script_number_msg;

std_msgs::String enable_module_msg;

//variables
double move_x, move_y;
bool rotation_left, rotation_right;
double max_speed;
double speed_ratio_rad;
Eigen::Quaterniond rqyToQ;

ros::Time count;


//function
void initialize();
void wheel_move_function(double x, double y);
void wheel_direction_group(int8_t motor1, int8_t motor2, int8_t motor3, int8_t motor4);
void wheel_rotation(bool rotation_left, bool rotation_right);
//callback
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg);







