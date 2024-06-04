#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/core/persistence.hpp>

namespace crdc {
namespace airi {

class FeatureExtractor {
 public:
  FeatureExtractor() {}
  virtual ~FeatureExtractor() {}
};

}  // namespace airi
}  // namespace crdc
