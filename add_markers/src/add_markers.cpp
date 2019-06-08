#include <std_msgs/String.h>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

ros::Publisher marker_pub;

void delete_marker(std::string name, int id) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();
    marker.ns = name;
    marker.id = id;
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
}

void add_marker(std::string name, int id, float x, float y) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = name;
    marker.id = id;

    marker.type = visualization_msgs::Marker::CUBE;

    marker.action = visualization_msgs::Marker::ADD;

    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0;
    marker.pose.orientation.x = x;
    marker.pose.orientation.y = y;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 0.0;

    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;

    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_pub.publish(marker);
}

void object_state_callback(const std_msgs::String::ConstPtr& msg) {
  std::string state = msg->data.c_str();

  if (state == "pickup") {
      delete_marker("pickup", 0);
  } else if (state == "dropoff") {
      add_marker("dropoff", 1, -3.3, -3.3);
  }
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "basic_shapes");
  ros::NodeHandle n;
  ros::Rate r(1);
  marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1, true);
  ros::Subscriber object_state_pub = n.subscribe("object_state", 1000, object_state_callback);

  add_marker("pickup", 0, -5.2, 6.2);

  ros::spin();
  return 0;
}
