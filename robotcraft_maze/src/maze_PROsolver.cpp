#include "ros/ros.h"
#include "ros/console.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <fstream>

void loadMap();
void fillInWavefront();
void generatePath();
bool followPath(int i);
void writeToFile();
void computeNeighbors(int x, int y);
bool checkCoordinate(int x, int y);
void enQueue(int x, int y);
void deQueue();
bool move1Square();
bool moveBack1Square();
bool turn90Clockwise();
bool turn180Clockwise();
bool turn90AntiClockwise();

int **world_map;

double resolution;
int width, height;

int *directions;
int *queue;
int rear = 0, front = 0;

int start_x = 100, start_y = 100,
    goal_x = 114, goal_y = 52;

int northNeigh_x, northNeigh_y,
    southNeigh_x, southNeigh_y,
    eastNeigh_x, eastNeigh_y,
    westNeigh_x, westNeigh_y;

#define NORTH 1
#define SOUTH 2
#define EAST 3
#define WEST 4

int current_orientation = EAST;

geometry_msgs::Pose2D initial_pose, current_pose;
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "reactive_navigation");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Publisher cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 100);

    ros::Subscriber odom_sub = n.subscribe("odom", 10, odomCallback);

    ROS_INFO("Loading map from the file");
    loadMap();

    ROS_INFO("Filling in the wavefront");
    fillInWavefront();

    ROS_INFO("Writing wavefront to the file");
    writeToFile();

    ROS_INFO("Generating path");
    generatePath();

    ROS_INFO("Following path");

    int i = 0;
    while (ros::ok())
    {
        if( followPath(i) ) i++;

        cmd_vel_pub.publish(velocity);
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void loadMap()
{
    std::ifstream map_file("/home/tatsiana/catkin_wc/src/robotcraft_maze/map.txt");
    map_file >> resolution;
    map_file >> width;
    map_file >> height;

    world_map = new int *[height];

    for (int row = 0; row < height; ++row)
    {
        world_map[row] = new int[width];
        for (int column = 0; column < width; ++column)
        {
            int num;
            map_file >> num;
            world_map[row][column] = num == 100 ? 1 : num == 0 ? 0
                                                               : num;
        }
    }
    map_file.close();

}

void fillInWavefront()
{
    queue = new int[height * width];
    world_map[goal_y][goal_x] = 2;
    enQueue(goal_x, goal_y);

    while (rear != front)
    {
        int next_to_examine_x = queue[front];
        int next_to_examine_y = queue[front + 1];

        int new_empty_cell_value = world_map[next_to_examine_y][next_to_examine_x] + 1;
        computeNeighbors(next_to_examine_x, next_to_examine_y);

        if (checkCoordinate(northNeigh_x, northNeigh_y) && world_map[northNeigh_y][northNeigh_x] == 0)
        {
            world_map[northNeigh_y][northNeigh_x] = new_empty_cell_value;
            enQueue(northNeigh_x, northNeigh_y);
        }

        if (checkCoordinate(southNeigh_x, southNeigh_y) && world_map[southNeigh_y][southNeigh_x] == 0)
        {
            world_map[southNeigh_y][southNeigh_x] = new_empty_cell_value;
            enQueue(southNeigh_x, southNeigh_y);
        }

        if (checkCoordinate(eastNeigh_x, eastNeigh_y) && world_map[eastNeigh_y][eastNeigh_x] == 0)
        {
            world_map[eastNeigh_y][eastNeigh_x] = new_empty_cell_value;
            enQueue(eastNeigh_x, eastNeigh_y);
        }

        if (checkCoordinate(westNeigh_x, westNeigh_y) && world_map[westNeigh_y][westNeigh_x] == 0)
        {
            world_map[westNeigh_y][westNeigh_x] = new_empty_cell_value;
            enQueue(westNeigh_x, westNeigh_y);
        }

        deQueue();
    }
}

void computeNeighbors(int x, int y)
{
    northNeigh_x = x;
    northNeigh_y = y - 1;

    southNeigh_x = x;
    southNeigh_y = y + 1;

    eastNeigh_x = x + 1;
    eastNeigh_y = y;

    westNeigh_x = x - 1;
    westNeigh_y = y;
}

void generatePath()
{
    int current_x = start_x;
    int current_y = start_y;
    directions = new int[world_map[start_y][start_x]];

    int smallestCell = world_map[start_y][start_x];

    int i = 0;
    while (smallestCell != world_map[goal_y][goal_x])
    {
        computeNeighbors(current_x, current_y);

        if (checkCoordinate(northNeigh_x, northNeigh_y) && world_map[northNeigh_y][northNeigh_x] < smallestCell)
        {
            smallestCell = world_map[northNeigh_y][northNeigh_x];
            current_x = northNeigh_x;
            current_y = northNeigh_y;
            directions[i] = 1;
        }

        else if (checkCoordinate(southNeigh_x, southNeigh_y) && world_map[southNeigh_y][southNeigh_x] < smallestCell)
        {
            smallestCell = world_map[southNeigh_y][southNeigh_x];
            current_x = southNeigh_x;
            current_y = southNeigh_y;
            directions[i] = 2;
        }
        else if (checkCoordinate(eastNeigh_x, eastNeigh_y) && world_map[eastNeigh_y][eastNeigh_x] < smallestCell)
        {
            smallestCell = world_map[eastNeigh_y][eastNeigh_x];
            current_x = eastNeigh_x;
            current_y = eastNeigh_y;
            directions[i] = 3;
        }
        else if (checkCoordinate(westNeigh_x, westNeigh_y) && world_map[westNeigh_y][westNeigh_x] < smallestCell)
        {
            smallestCell = world_map[westNeigh_y][westNeigh_x];
            current_x = westNeigh_x;
            current_y = westNeigh_y;
            directions[i] = 4;
        }
        i++;
    }
}

bool checkCoordinate(int x, int y)
{
    if (x == -1 || y == -1 || x >= width || y >= height)
        return false;

    if (world_map[y][x] == 1 || world_map[y][x] == -1)
        return false;

    return true;
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

bool followPath(int i)
{
    switch (directions[i])
    {
    case NORTH:
        if(current_orientation == NORTH){
            if(move1Square())
                return true;
        }
        else if (current_orientation == SOUTH)
        {
            if (turn180Clockwise())
                current_orientation = NORTH;
        }
        else if (current_orientation == EAST)
        {
            if(turn90AntiClockwise())
                current_orientation = NORTH;
        }
        else if (current_orientation == WEST)
        {
            if(turn90Clockwise())
                current_orientation = NORTH;
        }
        break;
    case SOUTH:
        if (current_orientation == SOUTH)
        {
            if (move1Square())
                return true;
        }
        if (current_orientation == NORTH)
        {
            if (turn180Clockwise())
                current_orientation = SOUTH;
        }
        else if (current_orientation == EAST)
        {
            if (turn90Clockwise())
                current_orientation = SOUTH;
        }
        else if (current_orientation == WEST)
        {
            if (turn90AntiClockwise())
                current_orientation = SOUTH;
        }
        break;
    case EAST:
        if (current_orientation == EAST)
        {
            if (move1Square())
                return true;
        }
        if (current_orientation == NORTH)
        {
            if (turn90Clockwise())
                current_orientation = EAST;
        }
        else if (current_orientation == SOUTH)
        {
            if(turn90AntiClockwise())
                current_orientation = EAST;
        }
        else if (current_orientation == WEST)
        {
            if(turn180Clockwise())
                current_orientation = EAST;
        }
        break;
    case WEST:
        if (current_orientation == WEST)
        {
            if (move1Square())
                return true;
        }
        if (current_orientation == NORTH)
        {
            if(turn90AntiClockwise())
                current_orientation = WEST;
        }
        else if (current_orientation == SOUTH)
        {
            if(turn90Clockwise())
                current_orientation = WEST;
        }
        else if (current_orientation == EAST)
        {
            if(turn180Clockwise())
                current_orientation = WEST;
        }
        break;
    }
    return false;
}

void writeToFile()
{
    std::ofstream map_file("/home/tatsiana/catkin_wc/src/robotcraft_maze/map_wave.txt");
    int k = 0;
    for (int row = 0; row < height; ++row)
    {
        for (int column = 0; column < width; ++column)
        {
            map_file << world_map[row][column] << "(" << row << "," << column << ")" << "\t";
            k++;
        }
        map_file << std::endl;
    }
    map_file.close();
}

void enQueue(int x, int y)
{
    queue[rear] = x;
    queue[rear + 1] = y;
    rear += 2;
}

void deQueue()
{
    queue[front] = 0;
    queue[front + 1] = 0;
    front += 2;
}

bool move1Square()
{
    covered_distance = getDistance(initial_pose, current_pose);
    if (covered_distance >= resolution )
    {
        initial_pose = current_pose;
        return true;
    }

    velocity.linear.x = 0.5;
    velocity.angular.z = 0.0;
    return false;
}

bool turn90Clockwise()
{
    rotation_angle = getAngle(initial_pose, current_pose);
    if (rotation_angle >= M_PI * 0.9 / 2)
    {
        initial_pose = current_pose;
        return true;
    }

    velocity.linear.x = 0.0;
    velocity.angular.z = 0.5;
    return false;
}

bool turn180Clockwise()
{
    rotation_angle = getAngle(initial_pose, current_pose);
    if (rotation_angle >= M_PI * 0.9)
    {
        initial_pose = current_pose;
        return true;
    }

    velocity.linear.x = 0.0;
    velocity.angular.z = 0.5;
    return false;
}

bool turn90AntiClockwise()
{

    rotation_angle = getAngle(initial_pose, current_pose);
    if (rotation_angle >= M_PI * 0.9 / 2)
    {
        initial_pose = current_pose;
        return true;
    }

    velocity.linear.x = 0.0;
    velocity.angular.z = -0.5;
    return false;
}