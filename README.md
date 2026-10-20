# depthimage_to_laserscan_slice
Convert depth image to laser scan with slicing.

## Usage

- Node: `depthimage_to_laserscan_slice_node` (see `launch/sample.launch`).
- Nodelet: `depthimage_to_laserscan_slice/DepthImageToLaserScanSliceNodelet`.
  Load it into the same process as the depth image publisher,
  so that the image reaches the scan without serialization or a copy.

The input must be a `16UC1` (or `mono16`) depth image in millimeters.
The node reads the message buffer directly and does not depend on cv_bridge or OpenCV.
