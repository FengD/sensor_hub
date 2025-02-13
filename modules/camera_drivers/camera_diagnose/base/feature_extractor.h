#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/persistence.hpp>

namespace sensor {
namespace hub {

class FeatureExtractor {
 public:
  FeatureExtractor() {}
  virtual ~FeatureExtractor() {}
};

}  // namespace hub
}  // namespace sensor
