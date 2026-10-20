#include <depthimage_to_laserscan_slice/depthimage_to_laserscan_slice_node.hpp>

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "depthimage_to_laserscan_slice_node");
  ros::NodeHandle nh;
  ros::NodeHandle private_nh("~");
  depthimage_to_laserscan_slice::DepthImageToLaserScanSliceNode laser_scan_node(
      nh, private_nh);

  ros::spin();
  return 0;
}
