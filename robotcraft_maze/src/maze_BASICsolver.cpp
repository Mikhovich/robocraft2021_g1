#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/MapMetaData.h"
#include "geometry_msgs/Twist.h"

#include <fstream>

double TOUCH_THRESHHOLD = 0.6, THRESHHOLD = 0.4;

double g_right_distance, g_front_distance, g_left_distance;
geometry_msgs::Twist g_robot_velocity;

int findAnyWall();
int turnLeft();
int findLostRightWall();
int followWall();

void goForward();
void goRight();
void goLeft();

void rightIrCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    g_right_distance = msg->ranges[0];
}

void frontIrCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    g_front_distance = msg->ranges[0];
}

void leftIrCallback(const sensor_msgs::LaserScan::ConstPtr &msg)
{

    g_left_distance = msg->ranges[0];
}

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    double resolution = msg->info.resolution;
    int width = msg->info.width, height = msg->info.height;

    std::ofstream map_file("/home/tatsiana/catkin_wc/src/robotcraft_maze/map.txt"); //saving 2d map to file
    std::ofstream map_index_file("/home/tatsiana/catkin_wc/src/robotcraft_maze/map_index.txt");
    map_file << resolution << " " << width << " " << height << std::endl;

    int k = 0;
    for (int iRowIndex = 0; iRowIndex < height; ++iRowIndex)
    {
        for (int iColIndex = 0; iColIndex < width; ++iColIndex)
        {
            map_file << static_cast<int>(msg->data[k]) << "\t";
            map_index_file << static_cast<int>(msg->data[k]) << "(" << iRowIndex << "," << iColIndex << ")" << "\t";
            k++;
        }
        map_file << std::endl;
        map_index_file << std::endl;
    }
    map_file.close();
    map_index_file.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reactive_navigation");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    ros::Subscriber right_ir_sub = n.subscribe("base_scan_3", 10, rightIrCallback);
    ros::Subscriber front_ir_sub = n.subscribe("base_scan_1", 10, frontIrCallback);
    ros::Subscriber left_ir_sub = n.subscribe("base_scan_2", 10, leftIrCallback);

    ros::Subscriber map_sub = n.subscribe("map", 10, mapCallback);

    int state = 0;

    while (ros::ok())
    {
        ros::spinOnce();

        switch (state)
        { // right wall following algorithm
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
    g_robot_velocity.linear.x = 1.0;
    g_robot_velocity.angular.z = 0;
}

void goRight()
{
    g_robot_velocity.linear.x = 0.25;
    g_robot_velocity.angular.z = -25.0;
}

void goLeft()
{
    g_robot_velocity.linear.x = 0.25;
    g_robot_velocity.angular.z = 25.0;
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