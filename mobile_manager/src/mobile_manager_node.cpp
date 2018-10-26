/*
 * motor_algorithm.cpp
 *
 *      Author: robotemperor
 */
#include <mobile_manager/mobile_manager_node.h>
#include <mobile_manager/motor_cmd.h>

void initialize()
{
  move_x = 0.0;
  move_y = 0.0;
  speed_ratio_rad = 0.0;
  max_speed = 300; // pwm
}
//callback
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  move_x = msg->axes[0];
  move_y = msg->axes[1];

  rotation_left  = msg->buttons[4];
  rotation_right = msg->buttons[5];
}
//
void wheel_direction_group(bool motor1, bool motor2, bool motor3, bool motor4)
{
  motor_cmd_msg_1.motor_desired_direction = motor1;
  motor_cmd_msg_2.motor_desired_direction = motor2;
  motor_cmd_msg_3.motor_desired_direction = motor3;
  motor_cmd_msg_4.motor_desired_direction = motor4;
}
void wheel_move_function(double x, double y)
{
  //direction classification
  if(x > 0 && y > 0)      //2사분면
  {
    if(x < y) // 3번
    {
      speed_ratio_rad = acos(fabs(y)/fabs(sqrt(pow(x,2)+pow(y,2))));
      wheel_direction_group(1,1,1,1);
    }
    if(x >= y) //  4번
    {
      speed_ratio_rad = acos(fabs(x)/fabs(sqrt(pow(x,2)+pow(y,2))));
      wheel_direction_group(1,1,1,1);
    }
  }
  else if(x > 0 && y < 0) //3사분면
  {
    if(x > -y) // 5번
    {
      speed_ratio_rad = acos(fabs(x)/fabs(sqrt(pow(x,2)+pow(y,2))));
      wheel_direction_group(1,1,1,1);
    }
    if(x <= -y) // 6번
    {
      speed_ratio_rad = acos(fabs(y)/fabs(sqrt(pow(x,2)+pow(y,2))));
      wheel_direction_group(1,1,1,1);
    }
  }
  else if(x < 0 && y < 0) //4사분면
  {
    if(-x > -y) // 8번
    {
      speed_ratio_rad = acos(fabs(x)/fabs(sqrt(pow(x,2)+pow(y,2))));
      wheel_direction_group(1,1,1,1);
    }
    if(-x <= -y)// 7번
    {
      speed_ratio_rad = acos(fabs(y)/fabs(sqrt(pow(x,2)+pow(y,2))));
      wheel_direction_group(1,1,1,1);
    }
  }
  else if(x < 0 && y > 0) //1사분면
  {
    if(-x > y) // 1번
    {
      speed_ratio_rad = acos(fabs(x)/fabs(sqrt(pow(x,2)+pow(y,2))));
      wheel_direction_group(1,1,1,1);
    }
    if(-x <= y) // 2번
    {
      speed_ratio_rad = acos(fabs(y)/fabs(sqrt(pow(x,2)+pow(y,2))));
      wheel_direction_group(1,1,1,1);
    }
  }
  else if(x == 0 && y > 0) // forward
  {
    speed_ratio_rad = 0;
    wheel_direction_group(1,1,1,1);
  }
  else if(x == 0 && y < 0) // backward
  {
    speed_ratio_rad = 0;
    wheel_direction_group(1,1,1,1);
  }
  else if(x > 0 && y == 0) // left horizontal
  {
    speed_ratio_rad = 0;
    wheel_direction_group(1,1,1,1);
  }
  else if(x < 0 && y == 0) // right horizontal
  {
    speed_ratio_rad = 0;
    wheel_direction_group(1,1,1,1);
  }
  else // (x=0, y=0) foward initialize // stop
  {
    motor_cmd_msg_1.motor_desired_speed = 0;
    motor_cmd_msg_2.motor_desired_speed = 0;
    motor_cmd_msg_3.motor_desired_speed = 0;
    motor_cmd_msg_4.motor_desired_speed = 0;
  }
  // speed decision
  motor_cmd_msg_1.motor_desired_speed = max_speed - (max_speed/(45*M_PI/180))*speed_ratio_rad;
  motor_cmd_msg_2.motor_desired_speed = max_speed - (max_speed/(45*M_PI/180))*speed_ratio_rad;
  motor_cmd_msg_3.motor_desired_speed = max_speed - (max_speed/(45*M_PI/180))*speed_ratio_rad;
  motor_cmd_msg_4.motor_desired_speed = max_speed - (max_speed/(45*M_PI/180))*speed_ratio_rad;
}
void wheel_rotation(bool rotation_left, bool rotation_right)
{
  if(rotation_left || rotation_right)
  {
    if(rotation_left && rotation_right)//error
    {
      motor_cmd_msg_1.motor_desired_speed = 0;
      motor_cmd_msg_2.motor_desired_speed = 0;
      motor_cmd_msg_3.motor_desired_speed = 0;
      motor_cmd_msg_4.motor_desired_speed = 0;
      return;
    }

    if(rotation_left == 1)
      wheel_direction_group(1,1,1,1);
    if(rotation_right == 1)
      wheel_direction_group(1,1,1,1);

    motor_cmd_msg_1.motor_desired_speed = max_speed;
    motor_cmd_msg_2.motor_desired_speed = max_speed;
    motor_cmd_msg_3.motor_desired_speed = max_speed;
    motor_cmd_msg_4.motor_desired_speed = max_speed;
  }
  else
    return;
}
//////////////////////////////////////////////////////////////////////////////
int main (int argc, char **argv)
{
  ros::init(argc, argv, "mobile_manager_node");
  ros::NodeHandle nh;

  motor1_pub = nh.advertise<mobile_manager::motor_cmd>("/motor_1",10);
  motor2_pub = nh.advertise<mobile_manager::motor_cmd>("/motor_2",10);
  motor3_pub = nh.advertise<mobile_manager::motor_cmd>("/motor_3",10);
  motor4_pub = nh.advertise<mobile_manager::motor_cmd>("/motor_4",10);

  joy_sub   = nh.subscribe("/joy", 1, joy_callback);


  while(ros::ok())
  {
    wheel_move_function(move_x, move_y);
    wheel_rotation(rotation_left, rotation_right);
    usleep(100);
    motor1_pub.publish(motor_cmd_msg_1);
    motor2_pub.publish(motor_cmd_msg_2);
    motor3_pub.publish(motor_cmd_msg_3);
    motor4_pub.publish(motor_cmd_msg_4);


    printf("DIR Motor1 :: %d \n", motor_cmd_msg_1.motor_desired_direction);
    printf("DIR Motor2 :: %d \n", motor_cmd_msg_2.motor_desired_direction);
    printf("DIR Motor3 :: %d \n", motor_cmd_msg_3.motor_desired_direction);
    printf("DIR Motor4 :: %d \n", motor_cmd_msg_4.motor_desired_direction);

    printf("SPD Motor1 :: %d \n", motor_cmd_msg_1.motor_desired_speed);
    printf("SPD Motor2 :: %d \n", motor_cmd_msg_2.motor_desired_speed);
    printf("SPD Motor3 :: %d \n", motor_cmd_msg_3.motor_desired_speed);
    printf("SPD Motor4 :: %d \n", motor_cmd_msg_4.motor_desired_speed);

    ros::spinOnce();
  }



  return 0;
}


