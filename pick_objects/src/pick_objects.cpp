#include <std_msgs/String.h>
#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

// Define a client for to send goal requests to the move_base server through a SimpleActionClient
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

move_base_msgs::MoveBaseGoal create_goal(float x, float y, float w) {
  move_base_msgs::MoveBaseGoal goal;

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();

  goal.target_pose.pose.position.x = x;
  goal.target_pose.pose.position.y = y;
  goal.target_pose.pose.orientation.w = w;

  return goal;
}

std_msgs::String create_message(std::string data) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << data;
    msg.data = ss.str();
    return msg;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pick_objects");
  ros::NodeHandle n;
  ros::Publisher object_state_pub = n.advertise<std_msgs::String>("object_state", 1000);

  MoveBaseClient ac("move_base", true);

  while(!ac.waitForServer(ros::Duration(5.0))) {
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  // Pickup Goal
  ROS_INFO("Sending pickup goal");
  ac.sendGoal(create_goal(-5.2, 6.2, 0.5));
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    object_state_pub.publish(create_message("pickup"));
    ROS_INFO("Pickup goal reached!");
  } else {
    ROS_INFO("Failed to reach pickup goal :-(");
  }

  // Wait 5 seconds
  ROS_INFO("Waiting 5 seconds...");
  ros::Duration(5.0).sleep();

  // Dropoff Goal
  ROS_INFO("Sending dropoff goal");
  ac.sendGoal(create_goal(-3.3, -3.3, 0.5));
  ac.waitForResult();
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
    object_state_pub.publish(create_message("dropoff"));
    ROS_INFO("Dropoff goal reached!");
  } else {
    ROS_INFO("Failed to reach dropoff goal :-(");
  }

  ros::spin();

  return 0;
}
