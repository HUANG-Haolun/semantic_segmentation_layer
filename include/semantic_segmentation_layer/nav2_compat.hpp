// Copyright (c) 2026 robot.com
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
//
// Jazzy / Kilted+ compatibility shim.
//
// Upstream targets Kilted+ and uses nav2_ros_common, which does not exist on
// Jazzy (/opt/ros/jazzy/share/nav2_ros_common is absent), so the whole package
// fails to configure. This header forks on __has_include: Kilted+ keeps the
// original path; Jazzy and earlier get the three things upstream uses provided
// inside namespace nav2, so all 33 call sites and the SegmentationBuffer node
// parameter type compile unchanged.
//
// Why LifecycleNode aliases rclcpp_lifecycle and not nav2_util: the installed
// nav2_costmap_2d 1.3.12 declares Layer::node_ as
// rclcpp_lifecycle::LifecycleNode::WeakPtr (layer.hpp:169), while
// nav2_util::LifecycleNode is a *derived* class (nav2_util/lifecycle_node.hpp:38).
// Passing shared_ptr<Base> where weak_ptr<Derived> is expected is a downcast and
// a hard compile error -- that is exactly why upstream's origin/jazzy branch does
// not build here, so do not copy it.
#ifndef SEMANTIC_SEGMENTATION_LAYER__NAV2_COMPAT_HPP_
#define SEMANTIC_SEGMENTATION_LAYER__NAV2_COMPAT_HPP_

#include "rclcpp/rclcpp.hpp"

#if __has_include("nav2_ros_common/lifecycle_node.hpp")   // Kilted+
#include "nav2_ros_common/lifecycle_node.hpp"
#include "nav2_ros_common/node_utils.hpp"
#include "nav2_ros_common/qos_profiles.hpp"
#else                                                     // Jazzy 及更早
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "nav2_util/node_utils.hpp"

namespace nav2
{
using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
using nav2_util::declare_parameter_if_not_declared;

namespace qos
{
// 上游把这两个当函数用（const auto q = nav2::qos::SensorDataQoS(depth)），
// 所以返回 rclcpp::QoS 的函数就能满足全部调用点。
inline rclcpp::QoS SensorDataQoS(int depth = 10)
{
  return rclcpp::SensorDataQoS(rclcpp::KeepLast(depth));
}

inline rclcpp::QoS LatchedSubscriptionQoS(int depth = 1)
{
  rclcpp::QoS q{rclcpp::KeepLast(depth)};   // 花括号：避免 most-vexing-parse
  q.transient_local().reliable();
  return q;
}
}  // namespace qos
}  // namespace nav2
#endif

#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

namespace semantic_segmentation_layer
{
// Jazzy: LifecycleNode::create_publisher() 返回 LifecyclePublisher（派生自
// rclcpp::Publisher），而上游把它存进 rclcpp::Publisher<T>::SharedPtr —— 向上转换后
// 就拿不到 on_activate() 了。而 LifecyclePublisher 默认【未激活】，publish() 会 no-op
// 并打 "publisher is not activated"，于是 visualize_tile_map / visualize_frustum_fov
// 静默无输出。这里转回派生类去激活。
// Kilted+ 的 nav2::Publisher 自激活，那边这个函数是空操作。
template<typename MessageT>
inline void activate_publisher(const typename rclcpp::Publisher<MessageT>::SharedPtr & pub)
{
#if !__has_include("nav2_ros_common/lifecycle_node.hpp")
  if (auto lp = std::dynamic_pointer_cast<rclcpp_lifecycle::LifecyclePublisher<MessageT>>(pub)) {
    lp->on_activate();
  }
#else
  (void)pub;
#endif
}
}  // namespace semantic_segmentation_layer

#endif  // SEMANTIC_SEGMENTATION_LAYER__NAV2_COMPAT_HPP_
