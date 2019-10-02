/**************************************************************
 * Writer : HAN NU RIM
 * Date : 2019.10.02 WED
 * Input : 1) Map Data
 *         2) Initial Pose
 *         3) Final goal
 * Output : Path planning using A* search algorithm
 * ************************************************************/
#include "astar.h"

void mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
// subscribe map data and update init_map variable considered intrinsic cost
{
    width = msg->info.width;
    height = msg->info.height;
    resolution = msg->info.resolution;
    center = msg->info.width / 2;
    init_map = new int*[msg->info.height];
    for (int i = 0; i < msg->info.height; i++)
        init_map[i] = new int[msg->info.width];

    for (int i = 0; i < msg->info.height; i++)
        for (int j = 0; j < msg->info.width; j++) {
            if (msg->data[i*msg->info.width + j] == 100) {
                init_map[j][i] = INF;
                IntrinsicCost(j, i);
            }
        }
    ROS_INFO("map subscribe success!!");
}

void initialCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg)
// subscribe robot's initial pose and convert to grid pose that matched map data
{
    init_pose.pose.pose.position.x = msg->pose.pose.position.x;
    init_pose.pose.pose.position.y = msg->pose.pose.position.y;
    init_pose.pose.pose.orientation.z = msg->pose.pose.orientation.z;
    init_pose.pose.pose.orientation.w = msg->pose.pose.orientation.w;
    grid_pose[0] = init_pose.pose.pose.position.x / resolution + center;
    grid_pose[1] = init_pose.pose.pose.position.y / resolution + center;
    ROS_INFO("initial pose subscribe success!!");
}

void goalCallback(const move_base_msgs::MoveBaseGoal::ConstPtr& msg)
// subscribe final goal that published goal_publisher node and make class Path p and publish the path
{
    goal.target_pose.pose.position.x = msg->target_pose.pose.position.x;
    goal.target_pose.pose.position.y = msg->target_pose.pose.position.y;
    grid_goal[0] = goal.target_pose.pose.position.x / resolution + center;
    grid_goal[1] = goal.target_pose.pose.position.y / resolution + center;
    ROS_INFO("goal subscribe success!!");
    class Path p;
    GoalPublisher(p);
}

void GoalPublisher(class Path p)
// goal publish
{
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0)))
            ROS_INFO("Waiting for the move_base action server to come up");

    move_base_msgs::MoveBaseGoal goalOutput;

    goalOutput.target_pose.header.frame_id = "map";

    while(ros::ok()) {
        goalOutput.target_pose.header.stamp = ros::Time::now();
        goalOutput.target_pose.pose.position.x = p.returnval(0);     // input x position that you want
        goalOutput.target_pose.pose.position.y = p.returnval(1);     // input y position that you want
        goalOutput.target_pose.pose.orientation.z = p.returnval(2);  // input z orientation that you want
        goalOutput.target_pose.pose.orientation.w = p.returnval(3);  // input w orientation that you want

        ac.sendGoal(goalOutput);
        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("success!!");
            if (!p.Pluspoint()) break;
        }
    }
    ROS_INFO("the path is done.");
}

void IntrinsicCost(const int x, const int y)
// intrinsic cost only have 100, 50, 25, 12
{
    if (init_map[x][y] / 2 < 10) return;
    if (x < 0 || x > width) return;
    if (y < 0 || y > height) return;
    for (int i = y-1; i <= y+1; i++)
        for (int j = x-1; j <= x+1; j++) {
            if (i != x && j != y && init_map[j][i] < init_map[x][y] / 2 && init_map[j][i] != -1) {
                init_map[j][i] = init_map[x][y] / 2;
                IntrinsicCost(j, i);
            }
        }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "astar");
  ros::NodeHandle nh;

  ROS_INFO("astar start");

  ros::Subscriber map_sup = nh.subscribe("map", 10, mapCallback);
  ros::Subscriber initialpose_sub = nh.subscribe("initialpose", 10, initialCallback);
  ros::Subscriber goal_sup = nh.subscribe("goal", 10, goalCallback);

  ros::spin();

  return 0;
}
