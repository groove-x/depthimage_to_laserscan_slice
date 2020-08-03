#pragma once
#include <opencv2/opencv.hpp>

#include <memory>

namespace depthimage_to_laserscan_slice {

class DepthimageToLaserscanSlice {
 public:
  DepthimageToLaserscanSlice(int width, int height, double fx, double fy,
                             double cx, double cy, double factor);
  ~DepthimageToLaserscanSlice() {}

  void set_height(double lower, double upper) {
    lower_height_ = lower;
    upper_height_ = upper;
  }
  void set_range(double lower, double upper) {
    lower_range_ = lower;
    upper_range_ = upper;
  }
  void set_optical_axis_pitch(double optical_axis_pitch) {
    optical_axis_pitch_ = optical_axis_pitch;
  }

  std::shared_ptr<std::vector<float>> angles() { return angles_; }
  std::shared_ptr<std::vector<float>> convert(const cv::Mat &depth_image);

 private:
  int width_;
  int height_;
  double fx_;
  double fy_;
  double cx_;
  double cy_;
  double factor_;
  std::vector<double> coeff_;

  double lower_height_;
  double upper_height_;

  double lower_range_;
  double upper_range_;

  double optical_axis_pitch_;

  std::shared_ptr<std::vector<float>> angles_;

  void initialize_coefficient();
};

}  // namespace depthimage_to_laserscan_slice
