#define _USE_MATH_DEFINES

#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/MotorPower.h>
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/Pose.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include "tf/transform_datatypes.h"
#include <iostream>
#include <queue>

typedef enum{
  move,
  find_move,
  turn_left_slow,
  turn_right_slow,
  scan_pause_right,
  scan_pause_front,
  turn_left_90,
  turn_right_90,
  find_wall,

  find_wall_plan,
  follow_wall_plan,
  turn_wall_plan,
  bug2_plan,
}action;

class turtle_robot{
  private:
    ros::NodeHandle * n;
    ros::Publisher * velocity_publisher_;
    ros::Publisher * motors_publisher_;
    geometry_msgs::Twist cmd;
    geometry_msgs::Pose current_state;
    float laser_ranges[1000];
    int laser_range_length;
    int current_algorithm;
    std::queue<int> next_action;
    int mutex;
    int time_count;
    float x_target;
    float y_target;
    float s_slope;
    float s_intercept;
    float s_angle;
    int line_initialized;
    int turn_right;

  public:
    kobuki_msgs::MotorPower msg_motor;
    void stop();
    void turn(float rad);
    void forward(float rad);
    void overall_planner();
    void find_wall_planner();
    void follow_wall_planner();
    void turn_wall_planner();
    bool front_is_blocked();
    void init_motor();
    void calculate_bug_line(float * slope, float * intercept);
    int close_to_line();
    float calc_turn_rad();
    void process_laser_scan(const sensor_msgs::LaserScan::ConstPtr& scan);
    void process_state(const gazebo_msgs::ModelStates::ConstPtr& state);
    bool need_right_turn();
    void turn_move(float rad, float vel);
    bool turn_front();

    turtle_robot(ros::NodeHandle * nh, ros::Publisher * vp, ros::Publisher * motor, float x, float y){
      mutex = 0;
      time_count = 0;
      msg_motor.state = 1;
      current_algorithm = -1;
      //Set cmd to all zeros
      cmd.linear.x = 0;
      cmd.linear.y = 0;
      cmd.linear.z = 0;
      cmd.angular.x = 0;
      cmd.angular.y = 0;
      cmd.angular.z = 0;
      laser_range_length = 0;
      n = nh;
      velocity_publisher_ = vp;
      motors_publisher_ = motor;

      x_target = x;
      y_target = y;

      line_initialized = 0;
      init_motor();
      turn_right = 0;
    }
};

float convert_to_rad(float degree){
  if(degree <= 180)
    return degree * M_PI/180;
  else
    return -((degree - 180) * M_PI/180);
}

float convert_to_degree(float rad){
  if(rad > 0){
      return rad * 180/M_PI;
  }
  else{
      return 360 + rad * 180/M_PI;
  }
}

float turtle_robot::calc_turn_rad(){
  tf::Quaternion quat;
  tf::quaternionMsgToTF(current_state.orientation, quat);
  double roll, pitch, yaw, ret;
  tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
  printf("x: %f, y:%f, z: %f\n", roll, pitch, yaw);
  //So the z is our actual angle turn
  ret = convert_to_degree(yaw) - s_angle;
  if(ret > 360)
    ret -= 360;
  else if(ret < -360)
    ret += 360;
  printf("%f\n", ret);
  printf("%f\n", convert_to_rad(ret));
  return -convert_to_rad(ret);
}

int turtle_robot::close_to_line(){
  double distance = std::abs(s_slope * current_state.position.x - current_state.position.y + s_intercept)/std::sqrt(s_slope * s_slope + 1);
  printf("Distance : %f\n", distance);
  if(distance < 0.3)
    return 1;
  else
    return 0;
}

void turtle_robot::calculate_bug_line(float * s, float * i){
  mutex = 1;
  *s = (y_target - current_state.position.y)/(x_target - current_state.position.x);
  *i = current_state.position.y - *s * current_state.position.x;
  printf("Slope: %f, Intercept %f\n", s_slope, s_intercept);

  s_angle = convert_to_degree(std::atan(s_slope));
  printf("Debug %f\n", std::atan(s_slope));
  printf("Debug %f %f\n", current_state.position.y - y_target, current_state.position.x - x_target);
  if(current_state.position.y - y_target  > 0 && current_state.position.x - x_target > 0){
    s_angle += 180;
  }
  else if(current_state.position.y - y_target < 0 && current_state.position.x - x_target > 0){
    s_angle -= 180;
  }

  printf("OMG %f\n", s_angle);
  mutex = 0;
}

void turtle_robot::process_state(const gazebo_msgs::ModelStates::ConstPtr& state){
  if(state->name[2].compare("mobile_base") == 0){
    if(mutex == 0)
      current_state = state->pose[2];
  }
  else{
    //
    printf("mobile_base cannot be found... exiting\n");
    exit(1);
  }

  if(line_initialized == 0){
    line_initialized = 1;
    calculate_bug_line(&s_slope, &s_intercept);
    close_to_line();
  }

}

void turtle_robot::overall_planner(){
  time_count++;
  float turn_angle;
  if(std::sqrt(std::pow(current_state.position.y-y_target, 2) + std::pow(current_state.position.x-x_target, 2)) < .3 ){
    printf("Target found !\n");
    exit(0);
  }

  if(close_to_line() == 1){
    turn_angle = calc_turn_rad();
    printf("%f %d\n", turn_angle, current_algorithm);
    if(std::abs(turn_angle) > .05 && current_algorithm != follow_wall_plan && current_algorithm != turn_wall_plan){
      turn(turn_angle);
      printf("Aligning to line, Angle %f\n", turn_angle);
      current_algorithm = -1;
    }
    else if(std::abs(turn_angle) < .05 && current_algorithm != turn_wall_plan){
      current_algorithm = find_wall_plan;
    }
    else if(front_is_blocked() == 1 || current_algorithm == turn_wall_plan)
      current_algorithm = turn_wall_plan;
    else
      current_algorithm = -1;
  }

  if(current_algorithm == find_wall_plan){
    find_wall_planner();
  }
  else if(current_algorithm == follow_wall_plan){
    follow_wall_planner();
  }
  else if(current_algorithm == turn_wall_plan)
    turn_wall_planner();
}

  /*
  time_count++;
  float turn_angle;

  if(std::sqrt(std::pow(current_state.position.y-y_target, 2) + std::pow(current_state.position.x-x_target, 2)) < .3 ){
    printf("Target found !\n");
    exit(0);
  }
  if(current_algorithm == turn_wall_plan)
    turn_wall_planner();
  else if(close_to_line() == 1){
      turn_angle = calc_turn_rad();
    if(front_is_blocked() == 0 &&  std::abs(turn_angle) > .05 ){
      printf("S Aligning\n");
      turn_angle = calc_turn_rad();
      turn(turn_angle);
      printf("Turn angle %f\n", turn_angle);
      current_algorithm = find_wall_plan;
    }
    else{
      if(current_algorithm == follow_wall_plan)
        follow_wall_planner();
      else if(current_algorithm == find_wall_plan)
        find_wall_planner();
    }
  }
  else{
  if(current_algorithm == follow_wall_plan){
    printf("Follow Wall Plan\n");
    follow_wall_planner();
    printf("%d %d\n", current_algorithm, follow_wall_plan);
  }
  else if(current_algorithm == find_wall_plan)
    printf("Find Wall Plan\n");
    find_wall_planner();
  }
}
*/
void turtle_robot::turn_wall_planner(){
  if(front_is_blocked() == 0){
    printf("Charge\n");
    time_count = 0;
    current_algorithm = follow_wall_plan;
    forward(.3);
  }
  else{
    turn_move(.3, 0);
  }
}
void turtle_robot::turn_move(float rad, float vel){
  cmd.linear.x = vel;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = rad;
  velocity_publisher_->publish(cmd);
}

void turtle_robot::find_wall_planner(){
  printf("Action[%d] ", time_count);

  if(front_is_blocked() == 0){
    printf("move wall find\n");
    forward(.2);
  }
  else{
    printf("Wall found!\n");
    current_algorithm = turn_wall_plan;
    stop();
  }
}

void turtle_robot::follow_wall_planner(){

  printf("[%d]\n", time_count);

  //if(time_count %2 == 0){
  if(time_count%3 == 0){
    if( front_is_blocked() == 1){
          //turn(.5);
          turn_move(.8, .1);
    }
    else{
      forward(.3);
    }
  }
  else if(time_count%3 == 1){
    turn(-2.5);
  }
  else{
    if( front_is_blocked() == 1)
          turn(2.5);
    else
          forward(.3);
  }

  /*
  printf("[%d]\n", time_count);
  if(time_count % 2 == 1){
  if(need_right_turn() == 0){
    turn(-.5);
    turn_right = 1;
  }
  else if(front_is_blocked() == 1){
    printf("turn left\n");
    turn_move(.5, 0);
      //turn_move(.2, .2);
  }
}
else{
  printf("PLS\n");

    turn_move(.3, .3);
}

  printf("3 %f %f \n", cmd.linear.x, cmd.angular.z);
  /*
  if(time_count%3 == 0){
    if( front_is_blocked() == 1){
          //turn(.5);
          turn_move(.8, .1);
    }
    else{
      forward(.3);
    }
  }
  else if(time_count%3 == 1){
    turn(-2.5);
  }
  else{
    if( front_is_blocked() == 1)
          turn(2.5);
    else
          forward(.3);
  }
  */
}
bool turtle_robot::turn_front(){
  mutex = 1;
  int right_blocked = 0;

  for(int i = laser_range_length/3; i < 2*laser_range_length/3; i++){
    if(laser_ranges[i] < .7){
      right_blocked++;
    }
    //printf("ranges %f\n", ranges[i]);
  }

  mutex = 0;
  //printf("Val %d\n", (left_blocked > 1)||(front_blocked > 1)||(right_blocked > 1));
  printf("front blocked %d\n", right_blocked);
  return right_blocked > 0;
}

bool turtle_robot::need_right_turn(){
  mutex = 1;
  int right_blocked = 0;

  for(int i = laser_range_length/3; i < laser_range_length; i++){
    if(laser_ranges[i] < 3){
      right_blocked++;
    }
    //printf("ranges %f\n", ranges[i]);
  }

  mutex = 0;
  //printf("Val %d\n", (left_blocked > 1)||(front_blocked > 1)||(right_blocked > 1));
  printf("right blocked %d\n", right_blocked);
  return right_blocked > 0;
}


void turtle_robot::init_motor(){
  motors_publisher_->publish(msg_motor); //enable engine!
}


bool turtle_robot::front_is_blocked(){
  mutex = 1;
  int front_blocked = 0;
  int right_blocked = 0;
  int left_blocked = 0;

  for(int i = 0; i < laser_range_length/3; i++){
    if(laser_ranges[i] < .85){
      left_blocked++;
    }
  //  printf("ranges %f\n", ranges[i]);
  }

  for(int i = laser_range_length/3; i < 2 * laser_range_length/3; i++){
    if(laser_ranges[i] < .75){
      front_blocked++;
    }
    //printf("ranges %f\n", ranges[i]);
  }

  for(int i = 2 * laser_range_length/3; i < laser_range_length; i++){
    if(laser_ranges[i] < .85){
      right_blocked++;
    }
    //printf("ranges %f\n", ranges[i]);
  }

  mutex = 0;
  //printf("Val %d\n", (left_blocked > 1)||(front_blocked > 1)||(right_blocked > 1));
  return (left_blocked > 0)||(front_blocked > 0)||(right_blocked > 0);
}

void turtle_robot::stop(){
  cmd.linear.x = 0;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = 0;
  velocity_publisher_->publish(cmd);
}

void turtle_robot::forward(float dist){
  cmd.linear.x = dist;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = 0;
  velocity_publisher_->publish(cmd);
}

void turtle_robot::turn(float rad){
  cmd.linear.x = 0;
  cmd.linear.y = 0;
  cmd.linear.z = 0;
  cmd.angular.x = 0;
  cmd.angular.y = 0;
  cmd.angular.z = rad;
  printf("Turn degree %f\n", cmd.angular.z);
  printf("1 %f %f \n", cmd.linear.x, cmd.angular.z);
  velocity_publisher_->publish(cmd);
  printf("2 %f %f \n", cmd.linear.x, cmd.angular.z);
}

void turtle_robot::process_laser_scan(const sensor_msgs::LaserScan::ConstPtr& scan){
     laser_range_length =  (int)(scan->angle_max - scan->angle_min) / scan->angle_increment;
     if(mutex == 0){
       for(int i = 0; i < laser_range_length; i++){
         laser_ranges[i] = scan->ranges[i];
         //printf("ranges %f\n", ranges[i]);
       }
     }
}
