#include <depthimage_to_laserscan_slice/depthimage_to_laserscan_slice_node.hpp>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace depthimage_to_laserscan_slice {

class DepthImageToLaserScanSliceNodelet : public nodelet::Nodelet {
 private:
  void onInit() override {
    node_ = std::make_unique<DepthImageToLaserScanSliceNode>(
        getNodeHandle(), getPrivateNodeHandle());
  }

  std::unique_ptr<DepthImageToLaserScanSliceNode> node_;
};

}  // namespace depthimage_to_laserscan_slice

PLUGINLIB_EXPORT_CLASS(depthimage_to_laserscan_slice::DepthImageToLaserScanSliceNodelet,
                       nodelet::Nodelet)
