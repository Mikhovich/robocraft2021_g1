#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include "rosserial_arduino/Test.h"
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <string>

geometry_msgs::Pose2D initial_pose, current_pose;
int prev_color_id = 0, current_color_id = 0; //Ninja Extra Task
rosserial_arduino::Test color_id;            //Ninja Extra Task
geometry_msgs::Twist velocity;
bool initial_pose_set = false;
double covered_distance, rotation_angle;

void odomCallback(const nav_msgs::Odometry::ConstPtr &msg)
{
    current_pose.x = msg->pose.pose.position.x;
    current_pose.y = msg->pose.pose.position.y;
    tf::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    current_pose.theta = yaw;

    if (!initial_pose_set)
    {
        initial_pose = current_pose;
        initial_pose_set = true;
    }
}

void rightIrCallback(const sensor_msgs::Range::ConstPtr &msg)
{
    ROS_WARN_COND(msg->range < 0.15, "Collision risk! The robot is %f meters of an obsctacle, on the right side", msg->range);
}

void frontIrCallback(const sensor_msgs::Range::ConstPtr &msg)
{
    ROS_WARN_COND(msg->range < 0.15, "Collision risk! The robot is %f meters of an obsctacle in the front", msg->range);
}

void leftIrCallback(const sensor_msgs::Range::ConstPtr &msg)
{
    ROS_WARN_COND(msg->range < 0.15, "Collision risk! The robot is %f meters of an obsctacle, on the left side", msg->range);
}

int goStraight();
int turnRight();
double getDistance(geometry_msgs::Pose2D pose_1, geometry_msgs::Pose2D pose_2);
double getAngle(geometry_msgs::Pose2D pose_1, geometry_msgs::Pose2D pose_2);

int goStraight() //0
{
    covered_distance = getDistance(initial_pose, current_pose);
    if (covered_distance >= 0.5)
        return turnRight();

    velocity.linear.x = 2.5;
    velocity.angular.z = 0.0;
    current_color_id = 1; //Ninja Extra Task
    return 0;
}

int turnRight() //1
{
    rotation_angle = getAngle(initial_pose, current_pose);
    if (rotation_angle >= M_PI/2)
    {
        initial_pose = current_pose;
        return goStraight();
    }
    
    velocity.linear.x = 0.0;
    velocity.angular.z = -5.0;
    current_color_id = 2; //Ninja Extra Task
    return 1;
}

double getDistance(geometry_msgs::Pose2D pose_1, geometry_msgs::Pose2D pose_2)
{
    double x_diff = pose_1.x - pose_2.x;
    double y_diff = pose_1.y - pose_2.y;
    return sqrt(x_diff * x_diff + y_diff * y_diff);
}

double getAngle(geometry_msgs::Pose2D pose_1, geometry_msgs::Pose2D pose_2)
{
    return abs(pose_1.theta - pose_2.theta);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "square_test");
    ros::NodeHandle n;

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ros::Subscriber right_ir_sub = n.subscribe("ir_right_sensor", 10, rightIrCallback);
    ros::Subscriber front_ir_sub = n.subscribe("ir_front_sensor", 10, frontIrCallback);
    ros::Subscriber left_ir_sub = n.subscribe("ir_left_sensor", 10, leftIrCallback);
    ros::Subscriber odom_sub = n.subscribe("odom", 10, odomCallback);

    ros::ServiceClient rgb_leds_client = n.serviceClient<rosserial_arduino::Test>("set_rgb_leds");

    ros::Rate loop_rate(10);
    int state = 0;

    rgb_leds_client.waitForExistence();

    while (ros::ok())
    {
        switch (state)
        {
            case 0:
                state = goStraight();
                break;

            case 1:
                state = turnRight();
                break;
        }
        cmd_vel_pub.publish(velocity);

        if (prev_color_id != current_color_id) //Ninja Extra Task
        {
            color_id.request.input = std::to_string(current_color_id);
            prev_color_id = current_color_id;
            rgb_leds_client.call(color_id);
            ROS_INFO("set_rgb_leds answer %s", color_id.response.output);
        }
        

        ros::spinOnce();
        loop_rate.sleep();
    }
}