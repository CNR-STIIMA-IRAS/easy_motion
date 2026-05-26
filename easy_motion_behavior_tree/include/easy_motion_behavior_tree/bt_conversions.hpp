#pragma once

#include <behaviortree_cpp/basic_types.h>

#include <vector>

namespace BT
{

template<>
inline std::vector<double> convertFromString(StringView str)
{
  std::vector<double> vec;
  auto parts = splitString(str, ';');
  vec.reserve(parts.size());
  for (auto & element : parts) {
    vec.push_back(convertFromString<double>(element));
  }
  return vec;
}

}  // namespace BT
