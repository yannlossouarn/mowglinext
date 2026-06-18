// Copyright 2026 Mowgli Project
//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <https://www.gnu.org/licenses/>.

#include "mowgli_behavior/collision_nodes.hpp"

#include <cmath>
#include <exception>
#include <mutex>

#include "geometry_msgs/msg/point32.hpp"

namespace mowgli_behavior
{

namespace
{
// drive_telemetry layout (std_msgs/Float32MultiArray):
//   [l_target, r_target, l_actual, r_actual, l_pwm, r_pwm, wheel_yaw, imu_yaw,
//    yaw_residual, accel_peak_g, left_load, right_load, slip_flags]
constexpr int kSlipFlagsIndex = 12;
constexpr int kImpactBit = 1 << 2;  // DRIVE_SLIP_FLAG_IMPACT
constexpr double kDebounceSec = 3.0;  // ignore repeat IMPACT within this window

double yaw_from_quat(const geometry_msgs::msg::Quaternion& q)
{
  return std::atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z));
}
}  // namespace

// ---------------------------------------------------------------------------
// DetectCollision
// ---------------------------------------------------------------------------

DetectCollision::DetectCollision(const std::string& name, const BT::NodeConfig& config)
    : BT::ConditionNode(name, config), last_latch_(0, 0, RCL_ROS_TIME)
{
  ctx_ = config.blackboard->get<std::shared_ptr<BTContext>>("context");
  // Match the publisher's reliable QoS (hardware_bridge publishes at depth 10)
  // so we never miss the (rare, brief) IMPACT edge.
  telem_sub_ = ctx_->node->create_subscription<std_msgs::msg::Float32MultiArray>(
      "/hardware_bridge/drive_telemetry",
      rclcpp::QoS(10),
      std::bind(&DetectCollision::onTelem, this, std::placeholders::_1));
  // Runtime inhibit (latched so a late-set state is honored). When disabled,
  // IMPACT edges are tracked but never latched, so no keepout is promoted —
  // used by the tuning tool to bump an obstacle repeatedly for test purposes.
  enable_sub_ = ctx_->node->create_subscription<std_msgs::msg::Bool>(
      "~/collision_keepout_enabled",
      rclcpp::QoS(1).transient_local().reliable(),
      std::bind(&DetectCollision::onEnable, this, std::placeholders::_1));
}

void DetectCollision::onEnable(std_msgs::msg::Bool::SharedPtr msg)
{
  enabled_ = msg->data;
  RCLCPP_INFO(ctx_->node->get_logger(),
              "Collision keepout creation %s",
              msg->data ? "ENABLED" : "INHIBITED");
}

void DetectCollision::onTelem(std_msgs::msg::Float32MultiArray::SharedPtr msg)
{
  if (static_cast<int>(msg->data.size()) <= kSlipFlagsIndex)
  {
    return;
  }
  const bool impact = (static_cast<int>(msg->data[kSlipFlagsIndex]) & kImpactBit) != 0;
  const bool rising = impact && !last_impact_;
  last_impact_ = impact;  // tracked even when inhibited, so re-arming is edge-clean
  if (!rising)
  {
    return;
  }
  if (!enabled_)
  {
    return;  // keepout creation inhibited (e.g. tuning-tool collision tests)
  }

  std::lock_guard<std::mutex> lk(ctx_->context_mutex);

  // React throughout the autonomous mission — undock, transit-to-strip,
  // inter-segment/inter-area transit, AND coverage — but never in manual mode
  // (COMMAND_MANUAL_MOW=7) or idle, where a deliberate bump must not stamp a
  // phantom keepout. current_command stays COMMAND_START(1) for the whole
  // mission (battery dock + resume included). The obstacle is attached to an
  // area downstream; if no area is active yet (early undock) the recovery still
  // stops/backs off but PromoteCollisionObstacle skips the (area-less) promote.
  if (ctx_->current_command != 1 /* COMMAND_START */)
  {
    return;
  }
  if (ctx_->collision_pending)
  {
    return;  // a promotion is already queued
  }
  const auto now = ctx_->node->now();
  if (last_latch_.nanoseconds() != 0 && (now - last_latch_).seconds() < kDebounceSec)
  {
    return;
  }

  geometry_msgs::msg::PoseStamped pose;
  try
  {
    const auto tf = ctx_->tf_buffer->lookupTransform("map", "base_footprint", tf2::TimePointZero);
    pose.header = tf.header;
    pose.pose.position.x = tf.transform.translation.x;
    pose.pose.position.y = tf.transform.translation.y;
    pose.pose.position.z = tf.transform.translation.z;
    pose.pose.orientation = tf.transform.rotation;
  }
  catch (const std::exception& ex)
  {
    RCLCPP_WARN(ctx_->node->get_logger(),
                "DetectCollision: TF map<-base_footprint failed, dropping impact: %s",
                ex.what());
    return;
  }

  ctx_->collision_pose = pose;
  ctx_->collision_pending = true;
  last_latch_ = now;
  RCLCPP_WARN(ctx_->node->get_logger(),
              "Firmware IMPACT during autonomous op (area %d) at (%.2f, %.2f) — will recover",
              ctx_->current_area,
              pose.pose.position.x,
              pose.pose.position.y);
}

BT::NodeStatus DetectCollision::tick()
{
  std::lock_guard<std::mutex> lk(ctx_->context_mutex);
  return ctx_->collision_pending ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

// ---------------------------------------------------------------------------
// PromoteCollisionObstacle
// ---------------------------------------------------------------------------

PromoteCollisionObstacle::PromoteCollisionObstacle(const std::string& name,
                                                   const BT::NodeConfig& config)
    : BT::SyncActionNode(name, config)
{
  ctx_ = config.blackboard->get<std::shared_ptr<BTContext>>("context");
  client_ = ctx_->node->create_client<PromoteObstacle>("/map_server_node/promote_obstacle");
}

BT::NodeStatus PromoteCollisionObstacle::tick()
{
  double width = 0.40;
  double length = 0.54;
  double center_x = 0.18;
  double depth = 0.10;
  bool probe = false;
  getInput("chassis_width", width);
  getInput("chassis_length", length);
  getInput("chassis_center_x", center_x);
  getInput("box_depth", depth);
  getInput("probe_obstacle", probe);

  geometry_msgs::msg::PoseStamped pose;
  int area_idx = -1;
  {
    std::lock_guard<std::mutex> lk(ctx_->context_mutex);
    if (!ctx_->collision_pending)
    {
      return BT::NodeStatus::FAILURE;
    }
    pose = ctx_->collision_pose;
    area_idx = ctx_->current_area;
  }

  if (area_idx < 0)
  {
    // No area to attach the obstacle to — clear the latch and bail.
    std::lock_guard<std::mutex> lk(ctx_->context_mutex);
    ctx_->collision_pending = false;
    return BT::NodeStatus::FAILURE;
  }

  if (probe)
  {
    // FUTURE: drive slowly along/around the contact to trace the obstacle's
    // real footprint before promoting. Not implemented yet — fall through to
    // the fixed box so the behavior stays safe.
    RCLCPP_WARN(ctx_->node->get_logger(),
                "PromoteCollisionObstacle: probe_obstacle not implemented — stamping fixed box");
  }

  // Anchor the box at the EXACT contact point: its near face is the real front
  // bumper (chassis_center_x + chassis_length/2) of the IMPACT-time pose
  // snapshot (`pose`), so it stays at the true obstacle position even though the
  // recovery sequence has already BackUp'd the robot away by the time this runs.
  // No forward margin — the prior BackUp (> the costmap footprint margin) is
  // what keeps the robot's current footprint clear of the keepout, so Smac
  // never sees "start in collision". The box extends `depth` forward into the
  // obstacle from that contact face.
  const double front = center_x + length / 2.0;
  const double offset = front + depth / 2.0;
  const double yaw = yaw_from_quat(pose.pose.orientation);
  const double cx = pose.pose.position.x + offset * std::cos(yaw);
  const double cy = pose.pose.position.y + offset * std::sin(yaw);
  const double hd = depth / 2.0;
  const double hw = width / 2.0;
  // Body-frame corners (along heading = +x, across = +y), CCW.
  const double corners[4][2] = {{+hd, +hw}, {+hd, -hw}, {-hd, -hw}, {-hd, +hw}};

  auto req = std::make_shared<PromoteObstacle::Request>();
  req->area_index = static_cast<uint32_t>(area_idx);
  req->obstacle_id = 0;  // 0 => use the polygon field directly
  for (const auto& c : corners)
  {
    geometry_msgs::msg::Point32 p;
    p.x = static_cast<float>(cx + c[0] * std::cos(yaw) - c[1] * std::sin(yaw));
    p.y = static_cast<float>(cy + c[0] * std::sin(yaw) + c[1] * std::cos(yaw));
    p.z = 0.0F;
    req->polygon.points.push_back(p);
  }

  // Fire-and-forget: wait_for_service is unreliable on ARM, and promote_obstacle
  // is idempotent enough that a lost request just gets re-stamped on the next
  // impact (the robot is already stopped/backed off at this point).
  client_->async_send_request(req);

  {
    std::lock_guard<std::mutex> lk(ctx_->context_mutex);
    ctx_->collision_pending = false;
  }
  RCLCPP_INFO(ctx_->node->get_logger(),
              "Promoted collision keepout (%.2f x %.2f m) to area %d at (%.2f, %.2f)",
              width,
              depth,
              area_idx,
              cx,
              cy);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace mowgli_behavior
