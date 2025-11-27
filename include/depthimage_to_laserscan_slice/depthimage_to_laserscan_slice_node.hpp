#include <depthimage_to_laserscan_slice/depthimage_to_laserscan_slice.hpp>

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>

namespace depthimage_to_laserscan_slice {

class DepthImageToLaserScanSliceNode {
 public:
  DepthImageToLaserScanSliceNode(ros::NodeHandle& nh);

  void callback(const sensor_msgs::ImageConstPtr& image_msg,
                const sensor_msgs::CameraInfoConstPtr& info_msg);

 private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  image_transport::ImageTransport it_;
  image_transport::CameraSubscriber depth_sub_;
  ros::Publisher scan_pub_;

  std::string output_frame_id_;
  float scan_time_;
  float range_min_;
  float range_max_;
  std::unique_ptr<DepthimageToLaserscanSlice> depth_to_scan_;

  sensor_msgs::LaserScanPtr scan_msg_;

  void initialize_laserscan_msg();
};

}  // namespace depthimage_to_laserscan_slice