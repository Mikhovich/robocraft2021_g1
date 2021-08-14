// "reactive_navigation" node: subscribes laser data and publishes velocity commands

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "geometry_msgs/Twist.h"

double TOUCH_THRESHHOLD = 1.05, THRESHHOLD = 0.65;

double g_right_distance, g_front_distance, g_left_distance;
geometry_msgs::Twist g_robot_velocity;

int findAnyWall();
int turnLeft();
int findLostRightWall();
int followWall();

void goForward();
void goRight();
void goLeft();

void laserCallback(const sensor_msgs::LaserScan::ConstPtr &laser_msg)
{
  g_right_distance = *std::min_element(laser_msg->ranges.begin(), laser_msg->ranges.begin() + 39);
  g_front_distance = *std::min_element(laser_msg->ranges.begin() + 40, laser_msg->ranges.begin() + 79);
  g_left_distance = *std::min_element(laser_msg->ranges.begin() + 80, laser_msg->ranges.begin() + 119);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "reactive_navigation");
  ros::NodeHandle n;
  ros::Rate loop_rate(20);

  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);
  ros::Subscriber laser_sub = n.subscribe("base_scan", 100, laserCallback);

  int state = 0;

  while (ros::ok())
  {
    ros::spinOnce();
    switch (state)
    { // right wall following algorythm
    case 0:
    {
      state = findAnyWall(); //finding any wall in front of the robot at the beginning of the maze
      break;
    }
    case 1:
    {
      state = turnLeft(); //turning left in case the robot is too close to the right wall and might soon crash into it
      break;
    }
    case 2:
    {
      state = findLostRightWall(); //turning right to find the wall(that made a right turn) that robot was just following
      break;
    }
    case 3:
    {
      state = followWall(); //following right wall, that goes straight
      break;
    }
    }
    velocity_pub.publish(g_robot_velocity);
    loop_rate.sleep();
  }

  return 0;
}

void goForward()
{
  g_robot_velocity.linear.x = 0.5;
  g_robot_velocity.angular.z = 0;
}

void goRight()
{
  g_robot_velocity.linear.x = 0.65;
  g_robot_velocity.angular.z = -2.7;
}

void goLeft()
{
  g_robot_velocity.linear.x = 0.55;
  g_robot_velocity.angular.z = 0.8;
}

int findAnyWall() //0
{
  if (g_right_distance > TOUCH_THRESHHOLD && g_front_distance > TOUCH_THRESHHOLD)
  {
    goForward();
    return 0;
  }
  if (g_right_distance < TOUCH_THRESHHOLD)
    return followWall();
  if (g_front_distance < TOUCH_THRESHHOLD)
    return turnLeft();
}

int turnLeft() //1
{
  if (g_right_distance < THRESHHOLD || g_front_distance < TOUCH_THRESHHOLD)
  {
    goLeft();
    return 1;
  }
  if (g_right_distance < TOUCH_THRESHHOLD)
    return followWall();
  return findLostRightWall();
}

int findLostRightWall() //2
{
  if (g_right_distance > TOUCH_THRESHHOLD && g_front_distance > TOUCH_THRESHHOLD)
  {
    goRight();
    return 2;
  }
  if (g_right_distance < TOUCH_THRESHHOLD)
    return followWall();
  if (g_front_distance < TOUCH_THRESHHOLD)
    return turnLeft();
}

int followWall() //3
{
  if (g_right_distance < TOUCH_THRESHHOLD && g_right_distance > THRESHHOLD && g_front_distance > TOUCH_THRESHHOLD)
  {
    goForward();
    return 3;
  }
  if (g_right_distance > TOUCH_THRESHHOLD)
    return findLostRightWall();
  if (g_right_distance < THRESHHOLD || g_front_distance < TOUCH_THRESHHOLD)
    return turnLeft();
}