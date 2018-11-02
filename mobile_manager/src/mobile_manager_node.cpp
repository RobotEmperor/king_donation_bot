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
  max_speed = 200; // pwm
  rqyToQ = robotis_framework::convertRPYToQuaternion(0,0,0);

  arm_displacement_msg.name = "left";
}
//callback
void joy_callback(const sensor_msgs::Joy::ConstPtr& msg)
{
  if(pow(msg->axes[0],2)+pow(msg->axes[1],2) > 0.01)
  {
    move_x = msg->axes[0];
    move_y = msg->axes[1];
  }
  else
  {
    move_x = 0;
    move_y = 0;
  }
  rotation_left  = msg->buttons[4];
  rotation_right = msg->buttons[5];

  if(fabs(msg->axes[4])>0.1)
  {
    arm_displacement_msg.pose.position.x = msg->axes[4]*0.001;
  }
  else
  {
    arm_displacement_msg.pose.position.x = 0;
  }
  if(fabs(msg->axes[3])>0.1)
  {
    arm_displacement_msg.pose.position.y = msg->axes[3]*0.001;
  }
  else
  {
    arm_displacement_msg.pose.position.y = 0;
  }

  arm_displacement_msg.pose.position.z = msg->axes[6]*0.001;

  rqyToQ = robotis_framework::convertRPYToQuaternion(0,0.01*msg->axes[7],0);
  arm_displacement_msg.pose.orientation.x = rqyToQ.x();
  arm_displacement_msg.pose.orientation.y = rqyToQ.y();
  arm_displacement_msg.pose.orientation.z = rqyToQ.z();
  arm_displacement_msg.pose.orientation.w = rqyToQ.w();

  arm_displacement_pub.publish(arm_displacement_msg);

  for(int idx = 0; idx <4; idx++)
  {
    if(msg->buttons[idx] == 1)
      script_number_msg.data = idx+1;
  }

  script_number_pub.publish(script_number_msg);
}
void wheel_direction_group(int8_t motor1, int8_t motor2, int8_t motor3, int8_t motor4)
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
      wheel_direction_group(1,-1,1,-1);
    }
    if(x >= y) //  4번
    {
      speed_ratio_rad = acos(fabs(x)/fabs(sqrt(pow(x,2)+pow(y,2))));
      wheel_direction_group(1,1,-1,-1);
    }

    motor_cmd_msg_1.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_1.motor_desired_direction;
    motor_cmd_msg_2.motor_desired_speed = ((max_speed -(max_speed/(45*M_PI/180))*speed_ratio_rad)*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_2.motor_desired_direction;
    motor_cmd_msg_3.motor_desired_speed = ((max_speed -(max_speed/(45*M_PI/180))*speed_ratio_rad)*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_3.motor_desired_direction;
    motor_cmd_msg_4.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_4.motor_desired_direction;
  }
  else if(x > 0 && y < 0) //3사분면
  {
    if(x > -y) // 5번
    {
      speed_ratio_rad = acos(fabs(x)/fabs(sqrt(pow(x,2)+pow(y,2))));
      wheel_direction_group(1,1,-1,-1);
    }
    if(x <= -y) // 6번
    {
      speed_ratio_rad = acos(fabs(y)/fabs(sqrt(pow(x,2)+pow(y,2))));
      wheel_direction_group(-1,1,-1,1);
    }

    motor_cmd_msg_1.motor_desired_speed = ((max_speed -(max_speed/(45*M_PI/180))*speed_ratio_rad)*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_1.motor_desired_direction;
    motor_cmd_msg_2.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_2.motor_desired_direction;
    motor_cmd_msg_3.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_3.motor_desired_direction;
    motor_cmd_msg_4.motor_desired_speed = ((max_speed -(max_speed/(45*M_PI/180))*speed_ratio_rad)*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_4.motor_desired_direction;
  }
  else if(x < 0 && y < 0) //4사분면
  {
    if(-x > -y) // 8번
    {
      speed_ratio_rad = acos(fabs(x)/fabs(sqrt(pow(x,2)+pow(y,2))));
      wheel_direction_group(-1,-1,1,1);
    }
    if(-x <= -y)// 7번
    {
      speed_ratio_rad = acos(fabs(y)/fabs(sqrt(pow(x,2)+pow(y,2))));
      wheel_direction_group(-1,1,-1,1);
    }
    motor_cmd_msg_1.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_1.motor_desired_direction;
    motor_cmd_msg_2.motor_desired_speed = ((max_speed -(max_speed/(45*M_PI/180))*speed_ratio_rad)*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_2.motor_desired_direction;
    motor_cmd_msg_3.motor_desired_speed = ((max_speed -(max_speed/(45*M_PI/180))*speed_ratio_rad)*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_3.motor_desired_direction;
    motor_cmd_msg_4.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_4.motor_desired_direction;
  }
  else if(x < 0 && y > 0) //1사분면
  {
    if(-x > y) // 1번
    {
      speed_ratio_rad = acos(fabs(x)/fabs(sqrt(pow(x,2)+pow(y,2))));
      wheel_direction_group(-1,-1,1,1);
    }
    if(-x <= y) // 2번
    {
      speed_ratio_rad = acos(fabs(y)/fabs(sqrt(pow(x,2)+pow(y,2))));
      wheel_direction_group(1,-1,1,-1);
    }
    motor_cmd_msg_1.motor_desired_speed = ((max_speed -(max_speed/(45*M_PI/180))*speed_ratio_rad)*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_1.motor_desired_direction;
    motor_cmd_msg_2.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_2.motor_desired_direction;
    motor_cmd_msg_3.motor_desired_speed = (max_speed*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_3.motor_desired_direction;
    motor_cmd_msg_4.motor_desired_speed = ((max_speed -(max_speed/(45*M_PI/180))*speed_ratio_rad)*fabs(sqrt(pow(x,2)+pow(y,2))))*motor_cmd_msg_4.motor_desired_direction;
  }
  else if(x == 0 && y > 0) // forward
  {
    speed_ratio_rad = 0;
    wheel_direction_group(1,-1,1,-1);
  }
  else if(x == 0 && y < 0) // backward
  {
    speed_ratio_rad = 0;
    wheel_direction_group(-1,1,-1,1);
  }
  else if(x > 0 && y == 0) // left horizontal
  {
    speed_ratio_rad = 0;
    wheel_direction_group(1,1,-1,-1);
  }
  else if(x < 0 && y == 0) // right horizontal
  {
    speed_ratio_rad = 0;
    wheel_direction_group(-1,-1,1,1);
  }
  else // (x=0, y=0) foward initialize // stop
  {
    // motor_cmd_msg_1.motor_desired_speed = 0;
    // motor_cmd_msg_2.motor_desired_speed = 0;
    // motor_cmd_msg_3.motor_desired_speed = 0;
    // motor_cmd_msg_4.motor_desired_speed = 0;
  }
  //speed decision
  if(x == 0 || y==0)
  {
    motor_cmd_msg_1.motor_desired_speed = max_speed*fabs(sqrt(pow(x,2)+pow(y,2)))*motor_cmd_msg_1.motor_desired_direction;
    motor_cmd_msg_2.motor_desired_speed = max_speed*fabs(sqrt(pow(x,2)+pow(y,2)))*motor_cmd_msg_2.motor_desired_direction;
    motor_cmd_msg_3.motor_desired_speed = max_speed*fabs(sqrt(pow(x,2)+pow(y,2)))*motor_cmd_msg_3.motor_desired_direction;
    motor_cmd_msg_4.motor_desired_speed = max_speed*fabs(sqrt(pow(x,2)+pow(y,2)))*motor_cmd_msg_4.motor_desired_direction;
  }

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
      wheel_direction_group(-1,-1,-1,-1);

    motor_cmd_msg_1.motor_desired_speed = max_speed*motor_cmd_msg_1.motor_desired_direction;
    motor_cmd_msg_2.motor_desired_speed = max_speed*motor_cmd_msg_2.motor_desired_direction;
    motor_cmd_msg_3.motor_desired_speed = max_speed*motor_cmd_msg_3.motor_desired_direction;
    motor_cmd_msg_4.motor_desired_speed = max_speed*motor_cmd_msg_4.motor_desired_direction;
  }
  else
    return;
}
//////////////////////////////////////////////////////////////////////////////
int main (int argc, char **argv)
{
  ros::init(argc, argv, "mobile_manager_node");
  ros::NodeHandle nh;
  initialize();

  motor1_pub = nh.advertise<mobile_manager::motor_cmd>("/motor_1",10);
  motor2_pub = nh.advertise<mobile_manager::motor_cmd>("/motor_2",10);
  motor3_pub = nh.advertise<mobile_manager::motor_cmd>("/motor_3",10);
  motor4_pub = nh.advertise<mobile_manager::motor_cmd>("/motor_4",10);

  arm_displacement_pub = nh.advertise<erica_arm_module_msgs::ArmCmd>("/heroehs/arm/displacement",1);
  script_number_pub = nh.advertise<std_msgs::Int32>("/heroehs/script_number",1);

  joy_sub   = nh.subscribe("/joy", 1, joy_callback);

  while(ros::ok())
  {
    wheel_move_function(move_x, move_y);
    wheel_rotation(rotation_left, rotation_right);

    motor1_pub.publish(motor_cmd_msg_1);
    motor2_pub.publish(motor_cmd_msg_2);
    motor3_pub.publish(motor_cmd_msg_3);
    motor4_pub.publish(motor_cmd_msg_4);
    usleep(100);

/*  printf("---------------------------------------\n");
    printf("DIR Motor1 :: %d \n", motor_cmd_msg_1.motor_desired_direction);
    printf("DIR Motor2 :: %d \n", motor_cmd_msg_2.motor_desired_direction);
    printf("DIR Motor3 :: %d \n", motor_cmd_msg_3.motor_desired_direction);
    printf("DIR Motor4 :: %d \n", motor_cmd_msg_4.motor_desired_direction);
    printf("---------------------------------------\n");
    printf("2    %f    ", motor_cmd_msg_2.motor_desired_speed);
    printf("1    %f \n" , motor_cmd_msg_1.motor_desired_speed);
    printf("4    %f    ", motor_cmd_msg_4.motor_desired_speed);
    printf("3    %f \n" , motor_cmd_msg_3.motor_desired_speed);*/

    ros::spinOnce();
  }
  return 0;
}


