#include "smb_highlevel_controller/SmbHighlevelController.hpp"

// STD
#include <string>

namespace smb_highlevel_controller {
std::string scanTopic_;
std::string laserScanTopic_;
float controlGain_ = 1.2;
std::string serviceTopic_;
bool stop;

SmbHighlevelController::SmbHighlevelController(ros::NodeHandle &nodeHandle)
    : nodeHandle_(nodeHandle) {

  if (!readParameters()) {
    ROS_ERROR("Could not read parameters.");
    ros::requestShutdown();
  }

  // sub to scan topic
  subscriber_ = nodeHandle_.subscribe(
      "/scan", 1, &SmbHighlevelController::scanCallback, this);

  // sub to pointcloud_to_laserscan
  pointCloudSubscriber_ = nodeHandle_.subscribe(
      "/rslidar_points", 1, &SmbHighlevelController::pointCloudCallback, this);

  // publisher on the cmd_vel topic
  velPublisher_ = nodeHandle_.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

  // publishing a visualisation marker which shows estimated position of pillar
  visPublisher_ = nodeHandle_.advertise<visualization_msgs::Marker>("/visualization_marker",1);

  // service server from exercise 5
  service_ = nodeHandle_.advertiseService(
      "/start_stop_smb", &SmbHighlevelController::serviceCallback, this);

  // service client for the server
  // client_ = nodeHandle_.serviceClient<std_srvs::SetBool>(serviceTopic_);
   

  
  ROS_INFO("Successfully launched node.");
  // ros::Duration(10).sleep();
}

SmbHighlevelController::~SmbHighlevelController() {}

bool SmbHighlevelController::readParameters() {
  if (!nodeHandle_.getParam("scanTopic_", scanTopic_))
    return false;
  return true;
  if (!nodeHandle_.getParam("laserScanTopic_", laserScanTopic_))
    return false;
  return true;
  if (!nodeHandle_.getParam("controlGain_", controlGain_))
    return false;
  return true;
  if (!nodeHandle_.getParam("serviceTopic_", serviceTopic_))
    return false;
  return true;
}

void SmbHighlevelController::scanCallback(const sensor_msgs::LaserScan &msg) {
  int size = msg.ranges.size();
  float min = msg.range_max;
  int minIndex = 0;

  for (int i = 0; i < size; i++) {
    if (msg.ranges.at(i) < msg.range_max and msg.ranges.at(i) > msg.range_min) {
      min = msg.ranges.at(i);
      minIndex = i;
    }
  }

  float pillarAngle = msg.angle_min + (msg.angle_increment * minIndex);

  // p controller for the bot
  geometry_msgs::Twist vel;

  if (stop == false) {
    vel.linear.x = 0.75;
    vel.angular.z = controlGain_ * (pillarAngle);
  } else {
    vel.linear.x = 0;
    vel.angular.z = 0;
  }

  velPublisher_.publish(vel);

  // visualization marker for the pillar
  visualization_msgs::Marker marker;

  marker.header.frame_id = "base_link";
  marker.header.stamp = ros::Time();

  marker.ns = "pillar";
  marker.id = 0;

  marker.type = visualization_msgs::Marker::CYLINDER;
  marker.action = visualization_msgs::Marker::ADD;

  marker.pose.position.x = (min + 1) * cos(pillarAngle);
  marker.pose.position.y = -(min)*sin(pillarAngle);
  marker.pose.position.z = 0;

  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;

  marker.scale.x = 1;
  marker.scale.y = 1;
  marker.scale.z = 1;

  marker.color.a = 1;
  marker.color.r = 0;
  marker.color.g = 0;
  marker.color.b = 1;

  visPublisher_.publish(marker);  

  

  ROS_INFO_STREAM("Minimum index: " << min);
}
void SmbHighlevelController::pointCloudCallback(
    const sensor_msgs::PointCloud2 &m) {
  int s = m.height * m.row_step;
  ROS_INFO_STREAM("Number of points in 3D cloud: " << s);
}

bool SmbHighlevelController::serviceCallback(std_srvs::SetBool::Request& req, std_srvs::SetBool::Response& res) {
  if (req.data == true) {
    stop = true;
    res.message = "SMB stopped";
    ROS_INFO_STREAM("Robot stopped");
  } else {
    stop = false;
    res.message = "SMB started";
    ROS_INFO_STREAM("Robot started");
  }
  res.success = true;
  return true;
}

} // namespace smb_highlevel_controller