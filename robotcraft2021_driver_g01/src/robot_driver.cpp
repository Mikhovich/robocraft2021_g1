#include "ros/ros.h"
#include "geometry_msgs/Pose2D.h"
#include "std_msgs/Float32.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Range.h"
#include <tf/transform_broadcaster.h>
#include <string>

ros::Publisher odom_pub;
nav_msgs::Odometry odom_info;
double front_dist, right_dist, left_dist;

void poseCallback(const geometry_msgs::Pose2D::ConstPtr &msg)
{
    static tf::TransformBroadcaster br;
    tf::Transform transform;
    transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
    tf::Quaternion q;
    q.setRPY(0, 0, msg->theta);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "odom", "base_link"));

    odom_info.header.stamp = ros::Time::now();
    odom_info.header.frame_id = "odom";
    odom_info.child_frame_id = "base_link";

    odom_info.pose.pose.position.x = msg->x;
    odom_info.pose.pose.position.y = msg->y;
    odom_info.pose.pose.position.z = 0;

    odom_info.pose.pose.orientation.x = q[0];
    odom_info.pose.pose.orientation.y = q[1];
    odom_info.pose.pose.orientation.z = q[2];
    odom_info.pose.pose.orientation.w = q[3];

    odom_pub.publish(odom_info);
}

sensor_msgs::Range getRange(double distance, std::string frame_id)
{
    sensor_msgs::Range rangeMsg;
    rangeMsg.header.stamp = ros::Time::now();
    rangeMsg.header.frame_id = frame_id;
    rangeMsg.radiation_type = 1;
    rangeMsg.field_of_view = 0.034906585;
    rangeMsg.min_range = 0.1;
    rangeMsg.max_range = 0.8;
    rangeMsg.range = distance / 100;

    return rangeMsg;
}

void frontDistanceCallback(const std_msgs::Float32::ConstPtr &msg)
{
    front_dist = msg->data;
}
void rightDistanceCallback(const std_msgs::Float32::ConstPtr &msg)
{
    right_dist = msg->data;
}
void leftDistanceCallback(const std_msgs::Float32::ConstPtr &msg)
{
    left_dist = msg->data;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "robot_driver");
    ros::NodeHandle n;

    ros::Publisher right_sensor_pub = n.advertise<sensor_msgs::Range>("ir_right_sensor", 10);
    ros::Publisher front_sensor_pub = n.advertise<sensor_msgs::Range>("ir_front_sensor", 10);
    ros::Publisher left_sensor_pub = n.advertise<sensor_msgs::Range>("ir_left_sensor", 10);
    odom_pub = n.advertise<nav_msgs::Odometry>("odom", 10);

    ros::Subscriber right_distance_sub = n.subscribe("right_distance", 10, rightDistanceCallback);
    ros::Subscriber front_distance_sub = n.subscribe("front_distance", 10, frontDistanceCallback);
    ros::Subscriber left_distance_sub = n.subscribe("left_distance", 10, leftDistanceCallback);
    ros::Subscriber pose_sub = n.subscribe("pose", 10, poseCallback);

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        right_sensor_pub.publish(getRange(right_dist, "right_ir"));
        front_sensor_pub.publish(getRange(front_dist, "front_ir"));
        left_sensor_pub.publish(getRange(left_dist, "left_ir"));

        ros::spinOnce();
        loop_rate.sleep();
    }
}