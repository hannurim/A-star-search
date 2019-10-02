#ifndef ASTAR_H
#define ASTAR_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <math.h>
#include <time.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <move_base_msgs/MoveBaseActionGoal.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

#define INF 100                                         // it means there is obstacle
#define DIFF 4                                          // DIFF * resolution = distance(m) of robot's one step

using namespace std;

int     **init_map;                                     // map information
float   resolution;                                     // map resolution
int     center;                                         // half of map width
int     width;                                          // map width
int     height;                                         // map height

geometry_msgs::PoseWithCovarianceStamped init_pose;
int     grid_pose[2];                                   // index 0 means x and 1 means y at map data
move_base_msgs::MoveBaseGoal goal;
int     grid_goal[2];

void GoalPublisher(class Path p);
void IntrinsicCost(const int x, const int y);

class Pose {
    friend class Path;
private:
    float x;
    float y;
    float z;
    float w;
    class Pose *link;
public:
    Pose(float xx=0, float yy=0, float zz=0, float ww=1)
    { x = xx; y = yy; z = zz; w = ww; link=0; }
};

class Path{
private:
    class Pose *first;
    class Pose *last;
    class Pose *point;
    float       distance[4];
    void Pathplan();
    void Push(const float &xx, const float &yy, const float &zz, const float &ww);
    int ShortestPath(const int x, const int y, const int pre);
public:
    Path() { first=0; last=0; point=0; Pathplan(); }
    float returnval(const int num) {
        float temp;
        if (num == 0) { temp = point->x; }
        else if (num == 1) { temp = point->y; }
        else if (num == 2) { temp = point->z; }
        else if (num == 3) { temp = point->w; }
        return temp;
    }
    bool Pluspoint() {
        if (point->link) {
            point = point->link;
            return true;
        }
        else return false;
    }
};

void Path::Push(const float &xx, const float &yy, const float &zz, const float &ww)
// x, y, z, w data saved to linked list
{
    if (first) {
        last->link = new Pose(xx, yy, zz, ww);
        last = last->link;
    }
    else {
      first = new Pose(xx, yy, zz, ww);
      last = first;
      point = first;
    }
}

int Path::ShortestPath(const int x, const int y, const int pre=-1)
// calculate cost = euclidean distance + intrinsic cost + previous direction
{
    distance[0] = distance[1] = 0;
    distance[2] = distance[3] = 0;
    for (int i = y - 1; i >= y - DIFF; i--) {
        distance[0] += init_map[x][i];
    }
    distance[0] += pow(x - grid_goal[0], 2) + pow(y - DIFF - grid_goal[1], 2);
    for (int i = x - 1; i >= x - DIFF; i--) {
        distance[1] += init_map[i][y];
    }
    distance[1] += pow(x - DIFF - grid_goal[0], 2) + pow(y - grid_goal[1], 2);
    for (int i = y + 1; i <= y + DIFF; i++) {
        distance[2] += init_map[x][i];
    }
    distance[2] += pow(x - grid_goal[0], 2) + pow(y + DIFF - grid_goal[1], 2);
    for (int i = x + 1; i <= x + DIFF; i++) {
        distance[3] += init_map[i][y];
    }
    distance[3] += pow(x + DIFF - grid_goal[0], 2) + pow(y - grid_goal[1], 2);
    if (pre != -1) distance[pre] += INF;
    float min = 99999;
    int index = 4;
    for (int i = 0; i < 4; i++) {
        if (min > distance[i]) {
            min = distance[i];
            index = i;
        }
    }
    return index;
}

void Path::Pathplan()
// path planning using A* search algorithm
{
    Push(init_pose.pose.pose.position.x, init_pose.pose.pose.position.y, init_pose.pose.pose.orientation.z, init_pose.pose.pose.orientation.w);
    int direct = ShortestPath(grid_pose[0], grid_pose[1]);
    int pre_direct = (direct + 2) % 4;
    while(ros::ok()) {
        switch (direct) {
        case 0:
            init_pose.pose.pose.position.y = init_pose.pose.pose.position.y - DIFF * resolution;
            Push(init_pose.pose.pose.position.x, init_pose.pose.pose.position.y, init_pose.pose.pose.orientation.z, init_pose.pose.pose.orientation.w);
            grid_pose[1] = grid_pose[1] - DIFF;
            break;
        case 1:
            init_pose.pose.pose.position.x = init_pose.pose.pose.position.x - DIFF * resolution;
            Push(init_pose.pose.pose.position.x, init_pose.pose.pose.position.y, init_pose.pose.pose.orientation.z, init_pose.pose.pose.orientation.w);
            grid_pose[0] = grid_pose[0] - DIFF;
            break;
        case 2:
            init_pose.pose.pose.position.y = init_pose.pose.pose.position.y + DIFF * resolution;
            Push(init_pose.pose.pose.position.x, init_pose.pose.pose.position.y, init_pose.pose.pose.orientation.z, init_pose.pose.pose.orientation.w);
            grid_pose[1] = grid_pose[1] + DIFF;
            break;
        case 3:
            init_pose.pose.pose.position.x = init_pose.pose.pose.position.x + DIFF * resolution;
            Push(init_pose.pose.pose.position.x, init_pose.pose.pose.position.y, init_pose.pose.pose.orientation.z, init_pose.pose.pose.orientation.w);
            grid_pose[0] = grid_pose[0] + DIFF;
            break;
        default:
            ROS_ERROR("We can't get path!!");
            break;
        }
        direct = ShortestPath(grid_pose[0], grid_pose[1], pre_direct);
        if (direct != 4) pre_direct = (direct + 2) % 4;
        if (abs(grid_pose[0] - grid_goal[0]) <= DIFF && abs(grid_pose[1] - grid_goal[1]) <= DIFF) break;
    }
    Push(goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, init_pose.pose.pose.orientation.z, init_pose.pose.pose.orientation.w);
}

#endif // ASTAR_H
