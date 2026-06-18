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

#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp/behavior_tree.h"
#include "mowgli_behavior/bt_context.hpp"
#include "mowgli_interfaces/srv/promote_obstacle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"

namespace mowgli_behavior
{

// ---------------------------------------------------------------------------
// DetectCollision  (condition)
// ---------------------------------------------------------------------------
//
// Watches /hardware_bridge/drive_telemetry for the firmware IMPACT slip-flag
// (bit 2 of the slip_flags element). On a rising edge — throughout the
// autonomous mission (current_command == COMMAND_START), so it fires during
// undock and every transit as well as coverage, but NOT in manual mode where a
// deliberate bump must not stamp a phantom keepout — it captures the map-frame
// robot pose and latches ctx->collision_pending. Returns SUCCESS while a
// collision is pending, FAILURE otherwise. On a sensor-less robot (no
// LiDAR/depth/radar) the firmware IMPACT detector is the ONLY obstacle source,
// so this is what feeds Option B.
class DetectCollision : public BT::ConditionNode
{
public:
  DetectCollision(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts() { return {}; }

  BT::NodeStatus tick() override;

private:
  void onTelem(std_msgs::msg::Float32MultiArray::SharedPtr msg);

  std::shared_ptr<BTContext> ctx_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr telem_sub_;
  bool last_impact_{false};
  rclcpp::Time last_latch_;
};

// ---------------------------------------------------------------------------
// PromoteCollisionObstacle  (action)
// ---------------------------------------------------------------------------
//
// Turns a pending collision into a PERSISTENT keepout. Builds a small box
// (chassis_width across x box_depth deep) at the contact point — forward of the
// impact pose along heading — and calls /map_server_node/promote_obstacle for
// the active area. map_server then appends it to the area's obstacle list (F2C
// coverage strips stop at it + bypass-arc detour), rebuilds /keepout_mask (the
// no-lidar global costmap's keepout_filter makes Smac treat it as lethal), and
// persists it to the area YAML (survives restart). Clears ctx->collision_pending.
//
// FUTURE (probe_obstacle=true): instead of a fixed box, slowly nudge the chassis
// around the contact to map the obstacle's real footprint, then promote that.
class PromoteCollisionObstacle : public BT::SyncActionNode
{
public:
  using PromoteObstacle = mowgli_interfaces::srv::PromoteObstacle;

  PromoteCollisionObstacle(const std::string& name, const BT::NodeConfig& config);

  static BT::PortsList providedPorts()
  {
    // The box mirrors the REAL chassis form factor from mowgli_robot.yaml (the
    // same dims that build the URDF collision box and the Nav2 footprint),
    // passed in from the blackboard as {chassis_*}. It spans the full
    // chassis_width and its near face sits at the EXACT contact point — the real
    // front bumper (chassis_center_x + chassis_length/2) of the impact-time pose
    // snapshot, with NO forward margin. The recovery sequence BackUps the robot
    // BEFORE this node runs, so the robot's current footprint is already clear
    // of the keepout (no "start in collision") even though the obstacle is at
    // the true contact position. Defaults match the YardForce500; the XML should
    // bind {chassis_*} so the box follows the operator-configured shape.
    return {
        BT::InputPort<double>("chassis_width", 0.40, "Robot width across heading (m)"),
        BT::InputPort<double>("chassis_length", 0.54, "Robot length along heading (m)"),
        BT::InputPort<double>("chassis_center_x", 0.18,
                              "base_footprint->chassis-center distance along heading (m)"),
        BT::InputPort<double>("box_depth", 0.10, "Obstacle box depth along heading (m)"),
        BT::InputPort<bool>("probe_obstacle", false,
                            "FUTURE: slowly probe the obstacle to map its real footprint")};
  }

  BT::NodeStatus tick() override;

private:
  std::shared_ptr<BTContext> ctx_;
  rclcpp::Client<PromoteObstacle>::SharedPtr client_;
};

}  // namespace mowgli_behavior
