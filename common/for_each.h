// Copyright (C) 2021 Hirain Technologies
// License: Modified BSD Software License Agreement
// Author: Feng DING
// Description: for each

#pragma once

#include <type_traits>

#define DEFINE_TYPE_TRAIT(name, func)                            \
  template <typename T>                                          \
  class name {                                                   \
   private:                                                      \
    template <typename Class>                                    \
    static char Test(decltype(&Class::func)*);                   \
    template <typename>                                          \
    static int Test(...);                                        \
                                                                 \
   public:                                                       \
    static constexpr bool value = sizeof(Test<T>(nullptr)) == 1; \
  };                                                             \
                                                                 \
  template <typename T>                                          \
  constexpr bool name<T>::value;

namespace itd {
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
    itd::common::LessThan(i, (end)); ++i)

}  // namespace common
}  // namespace itd