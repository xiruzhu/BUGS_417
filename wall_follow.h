#define _USE_MATH_DEFINES

#include <cmath>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <kobuki_msgs/MotorPower.h>
#include <nav_msgs/Odometry.h>
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
  plan,
}action;

void stop();
void turn(float rad);
  //Moves the robot to wall in front
void parallel_wall(double turn_degree);
  //adjust robot Parallel to the wall
void forward(float dist);
void turn_left(double degree);
void turn_right(double degree);
void path_planner();
bool is_parallel_to_wall(const sensor_msgs::LaserScan::ConstPtr& scan);
bool front_is_blocked();
void processLaserScan(const sensor_msgs::LaserScan::ConstPtr& scan);
