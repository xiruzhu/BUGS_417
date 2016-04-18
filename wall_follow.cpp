#include "wall_follow.h"
//#include "keyop.hpp"

/*
Note it first runs in a straight line until it finds a wall and hugs it on the right side for eternity
*/
ros::NodeHandle * n;
ros::Publisher velocity_publisher_;
ros::Publisher motors_publisher_;
//ros::NodeHandle n;
//ros::Publisher disable_motors_publisher_;
geometry_msgs::Twist cmd;
kobuki_msgs::MotorPower msg_motor;
ros::Timer timer;
int running = 0;
int time_count = 0;
float ranges[1000];
int length;
bool init = true;
bool turning = false;

void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan){
     length =  (int)(scan->angle_max - scan->angle_min) / scan->angle_increment;
     if(running == 0){
       for(int i = 0; i < length; i++){
         ranges[i] = scan->ranges[i];
         //printf("ranges %f\n", ranges[i]);
       }
     }
     //path_planner();
}
void stop(){
  cmd.linear.x = 0;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = 0;
  velocity_publisher_.publish(cmd);
}

void forward(float dist){
  cmd.linear.x = dist;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = 0;
  velocity_publisher_.publish(cmd);
}

void turn(float rad){
  cmd.linear.x = 0;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = rad;
  velocity_publisher_.publish(cmd);
}

void turn_move(float rad, float vel){
  cmd.linear.x = vel;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = rad;
  velocity_publisher_.publish(cmd);
}

bool front_is_blocked(){
  running = 1;
  int front_blocked = 0;
  int right_blocked = 0;
  int left_blocked = 0;

  for(int i = 0; i < length/3; i++){
    if(ranges[i] < .85){
      left_blocked++;
    }
  //  printf("ranges %f\n", ranges[i]);
  }

  for(int i = length/3; i < 2 * length/3; i++){
    if(ranges[i] < .7){
      front_blocked++;
    }
    //printf("ranges %f\n", ranges[i]);
  }

  for(int i = 2 * length/3; i < length; i++){
    if(ranges[i] < .85){
      right_blocked++;
    }
    //printf("ranges %f\n", ranges[i]);
  }

  running = 0;
  //printf("Val %d\n", (left_blocked > 1)||(front_blocked > 1)||(right_blocked > 1));
  return (left_blocked > 0)||(front_blocked > 0)||(right_blocked > 0);
}

void find_wall_planner(){
  printf("Action[%d] ", time_count);

  if(front_is_blocked() == 0){
    printf("move wall find\n");
    forward(.3);
  }
  else{
    printf("Wall found!\n");
    init = false;
    turning = true;
    turn(.5);
  }
}

void timed_test(const ros::TimerEvent&){
  time_count++;
  if(init == true){
    find_wall_planner();
  }
  else if(turning == true){
    if(front_is_blocked() == 0){
      printf("Test\n");
      time_count = 0;
      turning = false;
      turn(.3);
    }
    else{
      turn(.3);
    }
  }
  else{
  if(time_count%3 == 0){
    if( front_is_blocked() == 1){
          turn_move(.8, .1);
    }
    else{
      forward(.3);
    }
  }
  else if(time_count%3 == 1){
    turn(-2.4);
  }
  else{
    if( front_is_blocked() == 1)
          turn(2.4);
    else
          forward(.3);

  }
}

}

int main(int argc, char **argv)
{
    if(time_count > 2147483645)
      time_count = 0;
    ros::init(argc, argv, "Waller");
    ros::NodeHandle nh("Waller");  //Ok so now we subscribe
    ros::Duration(.1).sleep();
    n = &nh;
    motors_publisher_ = nh.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1);
    velocity_publisher_ = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
    msg_motor.state = 1;
    motors_publisher_.publish(msg_motor); //enable engine!
    timer = nh.createTimer(ros::Duration(1), timed_test); //So now we run by the second instead of the crazy rate of msg
    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan",1000,processLaserScan);
    ros::spin();
}
