// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// pulse_modulate_wz — sub-deadband angular-velocity pulse-width (duty-cycle)
// modulator for the cmd_vel → firmware path.
//
// WHY this exists:
//   The chassis has a PWM static-friction deadband on pure rotation
//   (~0.5 rad/s on PWM 40). Any commanded |wz| below it produces motor
//   buzz but no rotation, so the dock graceful-controller's fine heading
//   corrections (0.05-0.3 rad/s) never moved the robot and DockRobot
//   never settled.
//
//   The previous mitigation FLOORED every sub-deadband |wz| up to the
//   deadband amplitude. That over-rotates: measured on-robot 2026-05-27,
//   a commanded 0.3 rad/s produced 0.38-0.49 rad/s of actual yaw (127-164%)
//   because the firmware ran at the full deadband amplitude for the whole
//   command duration. The dock controller's fine corrections all jumped to
//   0.5 rad/s → overshoot → reverse → ping-pong, so DockRobot oscillated.
//
//   This modulator fixes the over-rotation by holding the AMPLITUDE at the
//   deadband (enough to break stiction) but cutting the DUTY CYCLE so the
//   time-average rate equals the commanded wz. duty = |wz_cmd| / deadband.
//   A sigma-delta / Bresenham accumulator integrates `duty` each tick and
//   fires a full-deadband pulse whenever it crosses 1.0, then subtracts 1.0.
//   Over many ticks the long-run average is exactly wz_cmd, and the pulses
//   are naturally spaced.
//
//   The gyro sees the pulses while the wheel encoders barely do; pre-
//   fusion_graph that wheel/IMU mismatch corrupted the localizer (which is
//   why an earlier pulse attempt, PR #221 / commit 00952173, was reverted in
//   09abe1ac). fusion_graph now slip-vetoes the mismatch in both the graph
//   between-factors and the dead-reckoning, so pulsing is safe again.
//
// This is a free function with no ROS dependency so it can be unit-tested
// in isolation. State (the accumulator) is owned by the caller.

#ifndef MOWGLI_HARDWARE__WZ_PULSE_MODULATOR_HPP_
#define MOWGLI_HARDWARE__WZ_PULSE_MODULATOR_HPP_

#include <cmath>

namespace mowgli_hardware
{

// Smallest commanded |wz| we treat as a real command; anything below is
// floating-point dust / rotate-to-heading-reached and maps to exactly 0.
inline constexpr double kWzMinCmdToConsider = 1.0e-3;

/// Pulse-width-modulate a sub-deadband angular-velocity command.
///
/// \param wz_cmd   commanded angular velocity (rad/s, signed).
/// \param deadband chassis pivot deadband / pulse amplitude (rad/s, > 0).
/// \param accum    sigma-delta accumulator, owned by the caller; carries
///                 phase between ticks. Reset to 0 on sign flip / return to 0.
/// \return the wz to actually send this tick:
///   - |wz_cmd| <= kWzMinCmdToConsider          → 0 (and accum reset to 0)
///   - |wz_cmd| >= deadband                      → wz_cmd unchanged (passthrough)
///   - otherwise                                 → ±deadband on the fraction of
///     ticks given by duty = |wz_cmd|/deadband, else 0; long-run average == wz_cmd.
inline double pulse_modulate_wz(double wz_cmd, double deadband, double& accum)
{
  const double mag = std::abs(wz_cmd);

  // Treat dust / rotate-to-heading-reached as a clean stop and drop phase
  // so the next command starts fresh.
  if (mag <= kWzMinCmdToConsider)
  {
    accum = 0.0;
    return 0.0;
  }

  // At or above the deadband the firmware can rotate on its own — pass
  // through unchanged. Drop phase so re-entering the sub-deadband regime
  // doesn't fire a stale pulse.
  if (deadband <= 0.0 || mag >= deadband)
  {
    accum = 0.0;
    return wz_cmd;
  }

  // Sub-deadband: integrate duty and fire when the accumulator crosses 1.0.
  // A direction change must not carry stale phase, so reset when the sign of
  // the command disagrees with the sign held in the accumulator.
  if (accum != 0.0 && std::signbit(accum) != std::signbit(wz_cmd))
  {
    accum = 0.0;
  }

  const double duty = mag / deadband;  // in (0, 1)
  // Keep the accumulator signed so it doubles as the "last sign" memory.
  accum += std::copysign(duty, wz_cmd);

  if (std::abs(accum) >= 1.0)
  {
    accum -= std::copysign(1.0, wz_cmd);
    return std::copysign(deadband, wz_cmd);
  }
  return 0.0;
}

}  // namespace mowgli_hardware

#endif  // MOWGLI_HARDWARE__WZ_PULSE_MODULATOR_HPP_
