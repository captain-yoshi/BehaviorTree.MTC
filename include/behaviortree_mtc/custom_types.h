#pragma once

#include <behaviortree_cpp/bt_factory.h>

struct Vector3D
{
  double x;
  double y;
  double z;
};

struct Vector4D
{
  double w;
  double x;
  double y;
  double z;
};

// add this just in case, if it is necessary to register it with
// Groot2 publisher.
// You will need to add `RegisterJsonDefinition<Vector3D>(ToJson);` in you main
inline void ToJson(nlohmann::json& dest, const Vector3D& pose)
{
  dest["x"] = pose.x;
  dest["y"] = pose.y;
  dest["z"] = pose.z;
}

inline void ToJson(nlohmann::json& dest, const Vector4D& pose)
{
  dest["w"] = pose.w;
  dest["x"] = pose.x;
  dest["y"] = pose.y;
  dest["z"] = pose.z;
}

namespace BT
{
template <>
inline Vector3D convertFromString(StringView key)
{
  const auto parts = BT::splitString(key, ',');
  if(parts.size() != 3)
  {
    throw BT::RuntimeError("invalid input)");
  }

  Vector3D output;
  output.x = convertFromString<double>(parts[0]);
  output.y = convertFromString<double>(parts[1]);
  output.z = convertFromString<double>(parts[2]);
  return output;
}

template <>
inline Vector4D convertFromString(StringView key)
{
  const auto parts = BT::splitString(key, ',');
  if(parts.size() != 4)
  {
    throw BT::RuntimeError("invalid input)");
  }

  Vector4D output;
  output.w = convertFromString<double>(parts[0]);
  output.x = convertFromString<double>(parts[1]);
  output.y = convertFromString<double>(parts[2]);
  output.z = convertFromString<double>(parts[3]);
  return output;
}
}  // namespace BT
