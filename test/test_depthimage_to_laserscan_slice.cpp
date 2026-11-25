#include <gtest/gtest.h>

#include <cstring>
#include <random>

#include <depthimage_to_laserscan_slice/depthimage_to_laserscan_slice.hpp>

using depthimage_to_laserscan_slice::DepthimageToLaserscanSlice;

namespace {

constexpr int kWidth = 40;
constexpr int kHeight = 30;

DepthimageToLaserscanSlice make_converter(double pitch = 0.0) {
  DepthimageToLaserscanSlice c(kWidth, kHeight, 30.0, 30.0, 19.5, 14.5, 1000.0);
  c.set_height(-0.1, 0.1);
  c.set_range(0.2, 6.0);
  c.set_optical_axis_pitch(pitch);
  return c;
}

std::vector<uint16_t> random_depth(unsigned seed) {
  std::mt19937 rng(seed);
  std::uniform_int_distribution<int> dist(0, 7000);
  std::vector<uint16_t> depth(kWidth * kHeight);
  for (auto &d : depth) d = dist(rng);
  return depth;
}

// Ranges computed the slow way: project every pixel and keep the minimum
// range per column inside the height and range limits.
// The camera is rotated by `pitch` about its x axis.
std::vector<float> reference_ranges(const std::vector<uint16_t> &depth,
                                    double pitch = 0.0) {
  const double fx = 30.0, fy = 30.0, cx = 19.5, cy = 14.5, factor = 1000.0;
  std::vector<float> ranges(kWidth, INFINITY);
  for (int v = 0; v < kHeight; ++v) {
    for (int u = 0; u < kWidth; ++u) {
      // the image is upside down
      const double d = depth[(kHeight - 1 - v) * kWidth + (kWidth - 1 - u)];
      const double x_cam = (u - cx) / fx / factor;
      const double y_cam = (v - cy) / fy / factor;
      const double z_cam = 1.0 / factor;
      const double x = d * (y_cam * std::sin(pitch) + z_cam * std::cos(pitch));
      const double y = d * -x_cam;
      const double z = d * (z_cam * std::sin(pitch) - y_cam * std::cos(pitch));
      const double range = std::sqrt(x * x + y * y);
      if (-0.1 < z && z < 0.1 && 0.2 < range && range < 6.0 &&
          range < ranges[u])
        ranges[u] = range;
    }
  }
  return ranges;
}

}  // namespace

TEST(DepthimageToLaserscanSlice, MatchesReference) {
  const auto converter = make_converter();
  const auto depth = random_depth(1);
  std::vector<float> ranges;
  converter.convert(reinterpret_cast<const uint8_t *>(depth.data()),
                    kWidth * sizeof(uint16_t), ranges);
  const auto expected = reference_ranges(depth);
  ASSERT_EQ(ranges.size(), expected.size());
  int finite = 0;
  for (int u = 0; u < kWidth; ++u) {
    if (std::isinf(expected[u])) {
      EXPECT_TRUE(std::isinf(ranges[u])) << "column " << u;
    } else {
      EXPECT_NEAR(ranges[u], expected[u], 1e-5) << "column " << u;
      ++finite;
    }
  }
  EXPECT_GT(finite, kWidth / 2);
}

TEST(DepthimageToLaserscanSlice, PaddedRowsGiveTheSameRanges) {
  const auto converter = make_converter();
  const auto depth = random_depth(2);
  std::vector<float> contiguous_ranges;
  converter.convert(reinterpret_cast<const uint8_t *>(depth.data()),
                    kWidth * sizeof(uint16_t), contiguous_ranges);

  // An odd padding leaves every other row unaligned for uint16_t.
  for (const size_t padding : {6, 7}) {
    SCOPED_TRACE(padding);
    const size_t padded_step = kWidth * sizeof(uint16_t) + padding;
    std::vector<uint8_t> padded(padded_step * kHeight, 0xff);
    for (int v = 0; v < kHeight; ++v) {
      std::memcpy(padded.data() + v * padded_step, depth.data() + v * kWidth,
                  kWidth * sizeof(uint16_t));
    }
    std::vector<float> padded_ranges;
    converter.convert(padded.data(), padded_step, padded_ranges);
    EXPECT_EQ(contiguous_ranges, padded_ranges);
  }
}

TEST(DepthimageToLaserscanSlice, AppliesOpticalAxisPitch) {
  const double pitch = 0.1;
  const auto converter = make_converter(pitch);
  const auto depth = random_depth(3);
  std::vector<float> ranges;
  converter.convert(reinterpret_cast<const uint8_t *>(depth.data()),
                    kWidth * sizeof(uint16_t), ranges);
  const auto expected = reference_ranges(depth, pitch);
  // Make sure the pitch changes the result, so the check below means something.
  ASSERT_NE(expected, reference_ranges(depth));
  ASSERT_EQ(ranges.size(), expected.size());
  int finite = 0;
  for (int u = 0; u < kWidth; ++u) {
    if (std::isinf(expected[u])) {
      EXPECT_TRUE(std::isinf(ranges[u])) << "column " << u;
    } else {
      EXPECT_NEAR(ranges[u], expected[u], 1e-5) << "column " << u;
      ++finite;
    }
  }
  EXPECT_GT(finite, kWidth / 2);
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
