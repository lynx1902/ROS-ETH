#pragma once

#include <geometry_msgs/Twist.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <std_srvs/SetBool.h>

namespace smb_highlevel_controller {
/*!
 * Main class for the node to handle the ROS interfacing.
 */
class SmbHighlevelController {
public:
  /*!
   * Constructor.
   * @param nodeHandle the ROS node handle.
   */
  SmbHighlevelController(ros::NodeHandle &nodeHandle);

  /*!
   * Destructor.
   */
  virtual ~SmbHighlevelController();

private:
  ros::NodeHandle nodeHandle_;

  ros::Subscriber subscriber_;
  ros::Subscriber pointCloudSubscriber_;

  ros::Publisher velPublisher_;
  ros::Publisher visPublisher_;

  ros::ServiceServer service_;
  ros::ServiceClient client_;

  /*!
   * Reads and verifies the ROS parameters.
   * @return true if successful.
   */

  /*!
   * ROS topic callback method.
   * @param message the received message.
   */

  bool readParameters();

  void scanCallback(const sensor_msgs::LaserScan &msg);

  void pointCloudCallback(const sensor_msgs::PointCloud2 &m);

  bool serviceCallback(std_srvs::SetBool::Request& req,std_srvs::SetBool::Response& res);
};

} // namespace smb_highlevel_controller