#include <depthimage_to_laserscan_slice/depthimage_to_laserscan_slice_node.hpp>

#include <boost/make_shared.hpp>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/image_encodings.h>

namespace depthimage_to_laserscan_slice {

namespace {

bool is_host_big_endian() {
  const uint16_t one = 1;
  return *reinterpret_cast<const uint8_t*>(&one) == 0;
}

}  // namespace

DepthImageToLaserScanSliceNode::DepthImageToLaserScanSliceNode(
    ros::NodeHandle& nh, ros::NodeHandle& private_nh)
    : nh_(nh), private_nh_(private_nh), it_(nh) {
  private_nh_.getParam("output_frame_id", output_frame_id_);

  private_nh_.getParam("range_min", range_min_);
  private_nh_.getParam("range_max", range_max_);

  depth_sub_ = it_.subscribeCamera(
      "/depth/image_raw", 1, &DepthImageToLaserScanSliceNode::callback, this);
  scan_pub_ = nh_.advertise<sensor_msgs::LaserScan>("/depth/scan", 1);
};

void DepthImageToLaserScanSliceNode::callback(
    const sensor_msgs::ImageConstPtr& depth_msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // Initialize depth_to_scan_ on first callback using camera info
  if (!depth_to_scan_) {
    int width = info_msg->width;
    int height = info_msg->height;
    double fx = info_msg->K[0];
    double fy = info_msg->K[4];
    double cx = info_msg->K[2];
    double cy = info_msg->K[5];

    depth_to_scan_ = std::make_unique<DepthimageToLaserscanSlice>(
        width, height, fx, fy, cx, cy, 1000.0);

    double height_min, height_max;
    private_nh_.getParam("height_min", height_min);
    private_nh_.getParam("height_max", height_max);
    depth_to_scan_->set_height(height_min, height_max);

    depth_to_scan_->set_range(range_min_, range_max_);

    double optical_axis_pitch;
    private_nh_.getParam("optical_axis_pitch", optical_axis_pitch);
    depth_to_scan_->set_optical_axis_pitch(optical_axis_pitch);

    initialize_laserscan_msg();
  }

  // Read the message buffer in place instead of copying it with cv_bridge.
  namespace enc = sensor_msgs::image_encodings;
  if (depth_msg->encoding != enc::TYPE_16UC1 &&
      depth_msg->encoding != enc::MONO16) {
    ROS_ERROR_THROTTLE(10, "unsupported depth encoding: %s",
                       depth_msg->encoding.c_str());
    return;
  }
  if (bool(depth_msg->is_bigendian) != is_host_big_endian()) {
    ROS_ERROR_THROTTLE(10, "depth image byte order differs from the host");
    return;
  }
  if (static_cast<int>(depth_msg->width) != depth_to_scan_->width() ||
      static_cast<int>(depth_msg->height) != depth_to_scan_->height() ||
      depth_msg->step < depth_msg->width * sizeof(uint16_t) ||
      depth_msg->data.size() <
          static_cast<size_t>(depth_msg->step) * depth_msg->height) {
    ROS_ERROR_THROTTLE(10,
                       "depth image size %ux%u (step %u, %zu bytes) does not "
                       "match camera info %dx%d",
                       depth_msg->width, depth_msg->height, depth_msg->step,
                       depth_msg->data.size(), depth_to_scan_->width(),
                       depth_to_scan_->height());
    return;
  }

  // construct scan and publish
  // A new message per frame, so that subscribers in the same process
  // can keep the shared pointer without copying it.
  auto scan_msg = boost::make_shared<sensor_msgs::LaserScan>(scan_template_);
  scan_msg->header = depth_msg->header;
  if (output_frame_id_.length() > 0) {
    scan_msg->header.frame_id = output_frame_id_;
  }
  depth_to_scan_->convert(depth_msg->data.data(), depth_msg->step,
                          scan_msg->ranges);
  scan_pub_.publish(scan_msg);
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

  scan_template_ = sensor_msgs::LaserScan();
  scan_template_.angle_min = angle_min;
  scan_template_.angle_max = angle_max;
  scan_template_.angle_increment =
      (scan_template_.angle_max - scan_template_.angle_min) /
      (angles->size() - 1);
  scan_template_.time_increment = 0.0;
  scan_template_.scan_time = scan_time_;
  scan_template_.range_min = range_min_;
  scan_template_.range_max = range_max_;
}

}  // namespace depthimage_to_laserscan_slice
