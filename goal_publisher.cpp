#include <ros/ros.h>
#include <iostream>
#include <move_base_msgs/MoveBaseActionGoal.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "publisher_node");
  ros::NodeHandle nh;

  ros::Publisher goal_pub = nh.advertise<move_base_msgs::MoveBaseGoal>("goal", 1000);

  move_base_msgs::MoveBaseGoal msg;

  cout << "Enter goal position x : ";
  cin >> msg.target_pose.pose.position.x;
  cout << "Enter goal position y : ";
  cin >> msg.target_pose.pose.position.y;

  goal_pub.publish(msg);

  return 0;
}
