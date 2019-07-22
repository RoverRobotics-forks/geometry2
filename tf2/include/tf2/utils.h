// Copyright 2014 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef TF2_UTILS_H
#define TF2_UTILS_H

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/impl/utils.h>
#include <tf2/visibility_control.h>

namespace tf2 {
/** Return the yaw, pitch, roll of anything that can be converted to a tf2::Quaternion
 * The conventions are the usual ROS ones defined in tf2/LineMath/Matrix3x3.h
 * \param a the object to get data from (it represents a rotation/quaternion)
 * \param yaw yaw
 * \param pitch pitch
 * \param roll roll
 */
template <class A>
  void getEulerYPR(const A& a, double& yaw, double& pitch, double& roll)
  {
    tf2::Quaternion q = impl::toQuaternion(a);
    impl::getEulerYPR(q, yaw, pitch, roll);
  }

/** Return the yaw of anything that can be converted to a tf2::Quaternion
 * The conventions are the usual ROS ones defined in tf2/LineMath/Matrix3x3.h
 * This function is a specialization of getEulerYPR and is useful for its
 * wide-spread use in navigation
 * \param a the object to get data from (it represents a rotation/quaternion)
 * \param yaw yaw
 */
template <class A>
  double getYaw(const A& a)
  {
    tf2::Quaternion q = impl::toQuaternion(a);
    return impl::getYaw(q);
  }

/** Return the identity for any type that can be converted to a tf2::Transform
 * \return an object of class A that is an identity transform
 */
template <class A>
  A getTransformIdentity()
  {
    tf2::Transform t;
    t.setIdentity();
    A a;
    convert(t, a);
    return a;
  }


// Tolerance for acceptable quaternion normalization
static double QUATERNION_NORMALIZATION_TOLERANCE = 10e-3;

/** \brief convert Transform msg to Transform */
void transformMsgToTF2(const geometry_msgs::msg::Transform& msg, tf2::Transform& tf2)
{tf2 = tf2::Transform(tf2::Quaternion(msg.rotation.x, msg.rotation.y, msg.rotation.z, msg.rotation.w), tf2::Vector3(msg.translation.x, msg.translation.y, msg.translation.z));}

/** \brief convert Transform to Transform msg*/
void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::msg::Transform& msg)
{
  msg.translation.x = tf2.getOrigin().x();
  msg.translation.y = tf2.getOrigin().y();
  msg.translation.z = tf2.getOrigin().z();
  msg.rotation.x = tf2.getRotation().x();
  msg.rotation.y = tf2.getRotation().y();
  msg.rotation.z = tf2.getRotation().z();
  msg.rotation.w = tf2.getRotation().w();
}

/** \brief convert Transform to Transform msg*/
void transformTF2ToMsg(const tf2::Transform& tf2, geometry_msgs::msg::TransformStamped& msg, builtin_interfaces::msg::Time stamp, const std::string& frame_id, const std::string& child_frame_id)
{
  transformTF2ToMsg(tf2, msg.transform);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
}

void transformTF2ToMsg(const tf2::Quaternion& orient, const tf2::Vector3& pos, geometry_msgs::msg::Transform& msg)
{
  msg.translation.x = pos.x();
  msg.translation.y = pos.y();
  msg.translation.z = pos.z();
  msg.rotation.x = orient.x();
  msg.rotation.y = orient.y();
  msg.rotation.z = orient.z();
  msg.rotation.w = orient.w();
}

void transformTF2ToMsg(const tf2::Quaternion& orient, const tf2::Vector3& pos, geometry_msgs::msg::TransformStamped& msg, builtin_interfaces::msg::Time stamp, const std::string& frame_id, const std::string& child_frame_id)
{
  transformTF2ToMsg(orient, pos, msg.transform);
  msg.header.stamp = stamp;
  msg.header.frame_id = frame_id;
  msg.child_frame_id = child_frame_id;
}

void setIdentity(geometry_msgs::msg::Transform& tx)
{
  tx.translation.x = 0;
  tx.translation.y = 0;
  tx.translation.z = 0;
  tx.rotation.x = 0;
  tx.rotation.y = 0;
  tx.rotation.z = 0;
  tx.rotation.w = 1;
}

std::string stripSlash(const std::string& in)
{
  auto c = in.begin();
  for (; c!= in.end() && *c == '/'; c++){}
  // c now points to the first non-slash character
  return std::string(c,in.end());
}


}

#endif //TF2_UTILS_H
