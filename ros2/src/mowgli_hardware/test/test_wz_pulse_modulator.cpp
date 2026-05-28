// Copyright 2026 Mowgli Project
// SPDX-License-Identifier: GPL-3.0-or-later
//
// Unit tests for pulse_modulate_wz. Pure math, no ROS plumbing.

#include <cmath>

#include "mowgli_hardware/wz_pulse_modulator.hpp"
#include <gtest/gtest.h>

namespace mh = mowgli_hardware;

namespace
{

// Run a sustained command for `n` ticks and return the time-average output.
double average_over(double wz_cmd, double deadband, double& accum, int n)
{
  double sum = 0.0;
  for (int i = 0; i < n; ++i)
  {
    sum += mh::pulse_modulate_wz(wz_cmd, deadband, accum);
  }
  return sum / static_cast<double>(n);
}

}  // namespace

// (a) Commands at or above the deadband pass through unchanged, and never
//     mutate the accumulator phase.
TEST(WzPulseModulator, AtOrAboveDeadbandPassesThrough)
{
  double accum = 0.0;
  EXPECT_DOUBLE_EQ(mh::pulse_modulate_wz(0.5, 0.5, accum), 0.5);
  EXPECT_DOUBLE_EQ(accum, 0.0);
  EXPECT_DOUBLE_EQ(mh::pulse_modulate_wz(0.8, 0.5, accum), 0.8);
  EXPECT_DOUBLE_EQ(accum, 0.0);
  EXPECT_DOUBLE_EQ(mh::pulse_modulate_wz(-1.2, 0.5, accum), -1.2);
  EXPECT_DOUBLE_EQ(accum, 0.0);
}

// (b) A sustained sub-deadband command averages to the commanded rate over
//     many ticks (duty cycle ~ |wz|/deadband), and every emitted pulse is
//     exactly the deadband amplitude (not the smaller commanded value).
TEST(WzPulseModulator, SubDeadbandAveragesToCommand)
{
  const double deadband = 0.5;
  const int n = 10000;

  // 0.25 / 0.5 → duty ~50%.
  double accum = 0.0;
  EXPECT_NEAR(average_over(0.25, deadband, accum, n), 0.25, 1.0e-3);

  // 0.30 / 0.5 → duty 60% (the over-rotation case from 2026-05-27 should now
  // average to the command, not 0.38-0.49).
  accum = 0.0;
  EXPECT_NEAR(average_over(0.30, deadband, accum, n), 0.30, 1.0e-3);

  // 0.10 / 0.5 → duty 20%, negative sign preserved.
  accum = 0.0;
  EXPECT_NEAR(average_over(-0.10, deadband, accum, n), -0.10, 1.0e-3);
}

// Every nonzero output of a sub-deadband command is the full deadband
// amplitude with the command's sign (it is the amplitude clamp's job we are
// preserving — break stiction — but only on a fraction of ticks).
TEST(WzPulseModulator, PulsesAtDeadbandAmplitude)
{
  const double deadband = 0.5;
  double accum = 0.0;
  for (int i = 0; i < 1000; ++i)
  {
    const double out = mh::pulse_modulate_wz(0.3, deadband, accum);
    EXPECT_TRUE(out == 0.0 || out == deadband) << "unexpected pulse amplitude " << out;
  }
}

// (c) Near-zero commands stay exactly zero and clear phase.
TEST(WzPulseModulator, NearZeroStaysZero)
{
  double accum = 0.4;  // leftover phase
  EXPECT_DOUBLE_EQ(mh::pulse_modulate_wz(0.0, 0.5, accum), 0.0);
  EXPECT_DOUBLE_EQ(accum, 0.0);

  accum = 0.4;
  EXPECT_DOUBLE_EQ(mh::pulse_modulate_wz(5.0e-4, 0.5, accum), 0.0);  // below kWzMinCmdToConsider
  EXPECT_DOUBLE_EQ(accum, 0.0);
}

// (d) A sign flip resets the accumulator phase so a direction change does not
//     carry stale phase into the new direction.
TEST(WzPulseModulator, SignFlipResetsPhase)
{
  const double deadband = 0.5;
  double accum = 0.0;

  // Build up some positive phase without firing (0.2/0.5 = 0.4 per tick).
  EXPECT_DOUBLE_EQ(mh::pulse_modulate_wz(0.2, deadband, accum), 0.0);
  EXPECT_NEAR(accum, 0.4, 1.0e-9);

  // Flip sign: the stored +0.4 phase must be dropped before integrating the
  // new (negative) duty, so this tick alone (-0.4) cannot fire a pulse.
  EXPECT_DOUBLE_EQ(mh::pulse_modulate_wz(-0.2, deadband, accum), 0.0);
  EXPECT_NEAR(accum, -0.4, 1.0e-9);

  // And the long-run average after the flip still tracks the new command.
  accum = 0.0;
  EXPECT_NEAR(average_over(-0.25, deadband, accum, 10000), -0.25, 1.0e-3);
}

// Degenerate deadband (<= 0) must not divide-by-zero or pulse — pass through.
TEST(WzPulseModulator, NonPositiveDeadbandPassesThrough)
{
  double accum = 0.3;
  EXPECT_DOUBLE_EQ(mh::pulse_modulate_wz(0.1, 0.0, accum), 0.1);
  EXPECT_DOUBLE_EQ(accum, 0.0);
}
