#include "bug2.h"

turtle_robot * bot;

void timer_routine(const ros::TimerEvent&){
  bot->overall_planner();
}

void laser_routine(const sensor_msgs::LaserScan::ConstPtr& scan){
   bot->process_laser_scan(scan);
}

void gps_routine(const gazebo_msgs::ModelStates::ConstPtr& state){
  bot->process_state(state);
}

int main(int argc, char **argv){
  ros::init(argc, argv, "Bug2");

  double x,y;
  ros::NodeHandle nh;  //Ok so now we subscribe
  ros::Duration(.1).sleep();
  nh.getParam("Bug2/goal_x", x);
  nh.getParam("Bug2/goal_y", y);
  ros::Publisher motors_publisher_ = nh.advertise<kobuki_msgs::MotorPower>("/mobile_base/commands/motor_power", 1);
  ros::Publisher velocity_publisher_ = nh.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1);
  printf("%f %f \n", x, y);
  bot = new turtle_robot(&nh, &velocity_publisher_, &motors_publisher_, x, y);

  ros::Timer timer = nh.createTimer(ros::Duration(.5), timer_routine); //So now we run by the second instead of the crazy rate of msg
  ros::Subscriber sub1 = nh.subscribe<sensor_msgs::LaserScan>("/scan",1000, laser_routine);
  ros::Subscriber sub2 = nh.subscribe<gazebo_msgs::ModelStates>("/gazebo/model_states",1000, gps_routine);
  ros::spin();
}
