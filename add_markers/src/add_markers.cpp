#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

// Pickup location
double pickup_x = 2.4;
double pickup_y = -5.5;

// Dropoff location
double dropoff_x = 5.4;
double dropoff_y = -4.6;

// Current position of the robot
double current_pose_x = 0;
double current_pose_y = 0;

// Minimum distance to consider the robot has reached its goal position
double dist_threshold = 0.4;

// Flags to know if we reach one of the goal locations
bool reached_pickup_zone = false;
bool reached_dropoff_zone = false;

double distance(double x1, double x2, double y1, double y2)
{
  return sqrt(pow((x1 - x2), 2.0) + pow((y1 - y2), 2.0));
}

// Get the current position of the robot and check if at pickup or dropoff location
void odometry_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  current_pose_x = msg->pose.pose.position.y;
  current_pose_y = msg->pose.pose.position.x * (-1);

  double dist;

  if (reached_pickup_zone == false)
  {
    dist = distance(current_pose_x, pickup_x, current_pose_y, pickup_y);
    //ROS_INFO("Distance = %lf", dist);
    if (dist <= dist_threshold)
    {
      reached_pickup_zone = true;
      ROS_INFO("Reached picking up zone");
    }
  }
  else
  {
    dist = distance(current_pose_x, dropoff_x, current_pose_y, dropoff_y);
    if (dist <= dist_threshold)
    {
      reached_dropoff_zone = true;
      ROS_INFO("Reached dropping off zone");
    }
  }
 
}

int main( int argc, char** argv )
{
  ros::init(argc, argv, "add_markers");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
  ros::Subscriber odometry_sub = n.subscribe("odom", 10, odometry_callback);

  // Set our initial shape type to be a cube
  uint32_t shape = visualization_msgs::Marker::CUBE;

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "virtual_object";
    marker.id = 0;

    // Set the marker type.
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker at pick up zone.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = 3;
    marker.pose.position.y = -5.5;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    // Publish the marker
    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);

    // Spin until reaching pickup zone
    while (reached_pickup_zone == false)
    {
      ros::spinOnce();
    }

    // Simulate picking up
    ros::Duration(1.0).sleep(); 
    ROS_INFO("Picking up object...");
    // Delete virtual object then wait 5 seconds
    marker.action = visualization_msgs::Marker::DELETE;
    marker_pub.publish(marker);
    ros::Duration(5.0).sleep(); 

    // Spin until reaching dropoff zone
    while (reached_dropoff_zone == false)
    {
      ros::spinOnce();
    }

    // Re-add the object at drop off zone
    ros::Duration(1.0).sleep(); 
    ROS_INFO("Dropping the object...");
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = 6;
    marker.pose.position.y = -4.5;

    marker_pub.publish(marker);
    
    // Wait 10 seconds before exiting the code
    ros::Duration(10.0).sleep(); 

    return 0;
  }
}

