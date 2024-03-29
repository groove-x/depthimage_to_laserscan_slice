#include <depthimage_to_laserscan_slice/depthimage_to_laserscan_slice_node.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

namespace depthimage_to_laserscan_slice {

inline double DEG2RAD(double degrees) { return degrees * M_PI / 180.0; }
inline double RAD2DEG(double radians) { return radians * 180.0 / M_PI; }

DepthImageToLaserScanSliceNode::DepthImageToLaserScanSliceNode(ros::NodeHandle& nh)
    : nh_(nh), it_(nh) {
  private_nh_ = ros::NodeHandle("~");
  private_nh_.getParam("output_frame_id", output_frame_id_);

  // load camera_info from rosparam
  int width;
  nh_.getParam("/image_width", width);
  int height;
  nh_.getParam("/image_height", height);
  std::vector<double> param_list;
  nh_.getParam("/camera_matrix/data", param_list);
  double fx = param_list[0];
  double fy = param_list[4];
  double cx = param_list[2];
  double cy = param_list[5];

  depth_to_scan_ = std::make_unique<DepthimageToLaserscanSlice>(
      width, height, fx, fy, cx, cy, 1000.0);

  double height_min, height_max;
  private_nh_.getParam("height_min", height_min);
  private_nh_.getParam("height_max", height_max);
  depth_to_scan_->set_height(height_min, height_max);

  private_nh_.getParam("range_min", range_min_);
  private_nh_.getParam("range_max", range_max_);
  depth_to_scan_->set_range(range_min_, range_max_);

  double optical_axis_pitch;
  private_nh_.getParam("optical_axis_pitch", optical_axis_pitch);
  depth_to_scan_->set_optical_axis_pitch(optical_axis_pitch);

  auto angles = depth_to_scan_->angles();
  initialize_laserscan_msg();

  depth_sub_ = it_.subscribe(
      "/depth/image_raw", 1,
      boost::bind(&DepthImageToLaserScanSliceNode::callback, this, _1));
  scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/depth/scan", 1);
};

void DepthImageToLaserScanSliceNode::callback(
    const sensor_msgs::ImageConstPtr& depth_msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(depth_msg, sensor_msgs::image_encodings::TYPE_16UC1);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  // construct scan and publish
  scan_msg_->header = depth_msg->header;
  if(output_frame_id_.length() > 0){
    scan_msg_->header.frame_id = output_frame_id_;
  }
  scan_msg_->ranges = *depth_to_scan_->convert(cv_ptr->image);
  scan_pub_.publish(scan_msg_);
}

void DepthImageToLaserScanSliceNode::initialize_laserscan_msg() {
  float angle_min, angle_max;
  auto angles = depth_to_scan_->angles();
  if (angles->front() > angles->back()) {
    angle_max = angles->front();
    angle_min = angles->back();
  } else {
    angle_min = angles->front();
    angle_max = angles->back();
  }

  scan_msg_ = sensor_msgs::LaserScanPtr(new sensor_msgs::LaserScan());
  scan_msg_->angle_min = angle_min;
  scan_msg_->angle_max = angle_max;
  scan_msg_->angle_increment =
      (scan_msg_->angle_max - scan_msg_->angle_min) / (angles->size() - 1);
  scan_msg_->time_increment = 0.0;
  scan_msg_->scan_time = scan_time_;
  scan_msg_->range_min = range_min_;
  scan_msg_->range_max = range_max_;
}

}  // namespace depthimage_to_laserscan_slice

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "depthimage_to_laserscan_slice_node");
  ros::NodeHandle nh;
  depthimage_to_laserscan_slice::DepthImageToLaserScanSliceNode laser_scan_node(nh);

  ros::spin();
  return 0;
}
