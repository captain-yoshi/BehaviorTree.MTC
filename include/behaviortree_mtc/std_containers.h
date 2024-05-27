#pragma once

#include <behaviortree_cpp/bt_factory.h>

// add this just in case, if it is necessary to register it with
// Groot2 publisher.
// You will need to add `RegisterJsonDefinition<Vector3D>(ToJson);` in you main
inline void ToJson(nlohmann::json& dest, const std::map<std::string, double>& map)
{
  for(auto const& [key, val] : map)
    dest[key] = val;
}

namespace BT
{
template <>
inline std::map<std::string, double> convertFromString(StringView key)
{
  const auto parts = BT::splitString(key, ',');

  std::map<std::string, double> map;
  for(const auto& part : parts)
  {
    const auto pairs = BT::splitString(part, ':');
    if(pairs.size() != 2)
    {
      throw BT::RuntimeError("invalid input)");
    }

    if(!map.try_emplace(convertFromString<std::string>(pairs[0]), convertFromString<double>(pairs[1])).second)
    {
      throw BT::RuntimeError("invalid input)");
    }
  }

  return map;
}

template <>
inline std::shared_ptr<std::map<std::string, double>> convertFromString(StringView key)
{
  auto map = std::make_shared<std::map<std::string, double>>(convertFromString<std::map<std::string, double>>(key));

  return map;
}

}  // namespace BT
