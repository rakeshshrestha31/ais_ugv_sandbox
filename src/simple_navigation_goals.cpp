#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include </home/ugv/catkin_ws/src/ais_ugv2_pick_drop/include/ais_pickup_dropoff/utils.h>
#include <iostream>
#include <boost/numeric/ublas/vector.hpp>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

// ros::Publisher stop_base;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  ROS_INFO("Stimple navigation goals opened");
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);
  // stop_base = n.advertise<std_msgs::Float32>("au_stop", 10);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // checkpoints (x, y, qx, qy, qz, qw)
  std::vector< std::array<float, 6> > checkpoints = {{
    // 2m square 
    {1.0, 0.0, 0.0, 0.0, 0.0, 1.0},
    {2.0, 0.0, 0.0, 0.0, 0.0, 1.0},
    {2.0, 0.0, 0.0, 0.0, -0.707, 0.707}, 
    {2.0, -1.0, 0.0, 0.0, -0.707, 0.707}, 
    {2.0, -2.0, 0.0, 0.0, -0.707, 0.707}, 
    {2.0, -2.0, 0.0, 0.0, 1.0, 0.0},
    {1.0, -2.0, 0.0, 0.0, 1.0, 0.0},
    {0.0, -2.0, 0.0, 0.0, 1.0, 0.0},
    {0.0, -2.0, 0.0, 0.0, -0.707, -0.707},
    {0.0, -1.0, 0.0, 0.0, -0.707, -0.707},
    {0.0, 0.0, 0.0, 0.0, -0.707, -0.707}, 
    {0.0, 0.0, 0.0, 0.0, 0.0f, 1.0f}
  }};

  std::vector<move_base_msgs::MoveBaseGoal> goal(checkpoints.size());
  for (uint i=0; i<checkpoints.size(); i++)
  {
    goal[i].target_pose.header.frame_id = "map";
    goal[i].target_pose.header.stamp = ros::Time::now();
    goal[i].target_pose.pose.position.x = checkpoints[i][0];
    goal[i].target_pose.pose.position.y = checkpoints[i][1];
    goal[i].target_pose.pose.orientation.x = checkpoints[i][2];
    goal[i].target_pose.pose.orientation.y = checkpoints[i][3];
    goal[i].target_pose.pose.orientation.z = checkpoints[i][4];
    goal[i].target_pose.pose.orientation.w = checkpoints[i][5];
  }

  while (true)
  {
    for (int i = 0;i<goal.size();i++){
      ROS_INFO("Sending goal");
      waitForSec(1);

      goal[i].target_pose.header.stamp = ros::Time::now();
      ac.sendGoal(goal[i]); //current goal
      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      {
        ROS_INFO("Reached Goal Successfully!");
        //waitForSec(5);
      }
      else
        ROS_INFO("The base failed to move!");
      // char dummy;
      // ROS_INFO("Press to continue");
      // std::cin >> dummy;
    }
  }

  return 0;

}
