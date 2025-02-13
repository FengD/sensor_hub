// Copyright (C) 2021 FengD
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: for each

#pragma once

#include <type_traits>
#ifndef WITH_ROS2
#include "cyber/cyber.h"
#else
#include "common/macros.h"
#endif

namespace sensor {
namespace hub {
namespace common {
DEFINE_TYPE_TRAIT(HasLess, operator<)  // NOLINT

template <class Value, class End>
typename std::enable_if<HasLess<Value>::value && HasLess<End>::value,
                        bool>::type
LessThan(const Value& val, const End& end) {
  return val < end;
}

template <class Value, class End>
typename std::enable_if<!HasLess<Value>::value || !HasLess<End>::value,
                        bool>::type
LessThan(const Value& val, const End& end) {
  return val != end;
}

#define FOR_EACH(i, begin, end)           \
  for (auto i = (true ? (begin) : (end)); \
    sensor::hub::common::LessThan(i, (end)); ++i)
}  // namespace common
}  // namespace hub
}  // namespace sensor
