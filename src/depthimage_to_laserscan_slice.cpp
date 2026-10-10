#include "depthimage_to_laserscan_slice/depthimage_to_laserscan_slice.hpp"

namespace depthimage_to_laserscan_slice {

DepthimageToLaserscanSlice::DepthimageToLaserscanSlice(int width, int height,
                                                       double fx, double fy,
                                                       double cx, double cy,
                                                       double factor)
    : width_(width),
      height_(height),
      fx_(fx),
      fy_(fy),
      cx_(cx),
      cy_(cy),
      factor_(factor),
      lower_height_(-INFINITY),
      upper_height_(INFINITY),
      lower_range_(0),
      upper_range_(INFINITY),
      optical_axis_pitch_(0) {
  initialize_coefficient();
}

void DepthimageToLaserscanSlice::convert(const uint8_t *depth, size_t step,
                                         std::vector<float> &ranges) const {
  ranges.assign(width_, INFINITY);

  // The image is upside down: walk the pixels from the last one backwards.
  const double *pcoeff = coeff_.data();
  for (int v = 0; v < height_; ++v) {
    const auto *row = reinterpret_cast<const uint16_t *>(
        depth + (height_ - 1 - v) * step);
    for (int u = 0; u < width_; ++u) {
      const auto d = row[width_ - 1 - u];
      auto range = d * *(pcoeff++);
      auto y = d * *(pcoeff++);
      // store minimum range with valid conditions
      if (lower_height_ < y && y < upper_height_ && lower_range_ < range &&
          range < upper_range_ && range < ranges[u])
        ranges[u] = range;
    }
  }
}

void DepthimageToLaserscanSlice::initialize_coefficient() {
  angles_ = std::make_shared<std::vector<float>>(width_, 0);

  coeff_.resize(2 * width_ * height_);
  auto *pcoeff = coeff_.data();
  auto s_pitch = sin(optical_axis_pitch_);
  auto c_pitch = cos(optical_axis_pitch_);
  for (int v = 0; v < height_; ++v) {
    for (int u = 0; u < width_; ++u) {
      auto &range = *(pcoeff++);
      auto &z = *(pcoeff++);

      // camera coordinate
      // (x: right, y: down, z: optical)
      auto x_cam = (u - cx_) / fx_ / factor_;
      auto y_cam = (v - cy_) / fy_ / factor_;
      auto z_cam = 1.0 / factor_;

      // base coordinate
      // (x: front, y: left, z: up)
      auto x = y_cam * s_pitch + z_cam * c_pitch;
      auto y = -x_cam;
      z = -(y_cam * c_pitch - z_cam * s_pitch);
      range = sqrt(x * x + y * y);
      if (v == int(height_ / 2)) (*angles_)[u] = atan2(y, x);
    }
  }
}

}  // namespace depthimage_to_laserscan_slice
