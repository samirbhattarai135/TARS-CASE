// Proves case_sim::PidV1 is arithmetically identical to the Arduino PID_v1 the
// firmware actually runs. Every sweep result rests on that, so any drift here
// is a failing test rather than a quietly wrong simulation.

#include <gtest/gtest.h>

#include <algorithm>
#include <random>

#include "pid_v1.hpp"

#include <PID_v1.h>

// The vendored library's clock, stepped by the test.
unsigned long g_fake_millis = 0;

namespace
{

constexpr double kKp = 60.0;
constexpr double kKi = 140.0;
constexpr double kKd = 0.9;
constexpr double kOutMin = -255.0;
constexpr double kOutMax = 255.0;
constexpr int kSampleTimeMs = 10;
constexpr double kSampleTimeS = 0.010;
constexpr double kSetpoint = 182.0;
constexpr double kTolerance = 1e-9;

/// The upstream library plus the three doubles it links to by pointer, wired in
/// the order BalanceControl::begin() uses. The order is load-bearing:
/// SetSampleTime(10) rescales gains the constructor already scaled for the
/// 100 ms default, and SetMode(AUTOMATIC) runs Initialize() while the limits are
/// still the default [0, 255].
struct UpstreamPid
{
  double input = 0.0;
  double output = 0.0;
  double setpoint = kSetpoint;
  PID pid;

  UpstreamPid()
  : pid(&input, &output, &setpoint, kKp, kKi, kKd, DIRECT)
  {
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(kSampleTimeMs);
    pid.SetOutputLimits(kOutMin, kOutMax);
  }

  bool step(double measurement, unsigned long now_ms)
  {
    input = measurement;
    g_fake_millis = now_ms;
    return pid.Compute();
  }
};

}  // namespace

TEST(PidV1Equivalence, MatchesUpstreamOverRandomTrajectory)
{
  // The upstream constructor reads the clock (lastTime = millis() - SampleTime),
  // so it has to see boot time, and g_fake_millis is shared between tests.
  g_fake_millis = 0;
  UpstreamPid upstream;

  case_sim::PidV1 port(kKp, kKi, kKd, kOutMin, kOutMax, kSampleTimeS);
  port.initialize(upstream.input, upstream.output);

  std::mt19937 rng(20260915u);
  std::uniform_real_distribution<double> jitter(-0.35, 0.35);

  double angle = 180.0;
  for (int i = 0; i < 5000; ++i) {
    // A mean-reverting walk around upright with periodic shoves: quiet tracking
    // where the output is well inside the limits, plus excursions large enough
    // to wind the integrator and drive both clamps. Uniform wide-band noise
    // would pin the output at +/-255 almost every step and prove nothing.
    angle += 0.03 * (kSetpoint - angle) + jitter(rng);
    if (i % 617 == 0) {
      angle += (i % 2 == 0) ? 9.0 : -9.0;
    }
    angle = std::clamp(angle, 150.0, 214.0);

    const unsigned long now_ms = static_cast<unsigned long>(i) * kSampleTimeMs;
    const double now_s = static_cast<double>(now_ms) / 1000.0;

    const bool upstream_ran = upstream.step(angle, now_ms);
    const bool port_ran = port.compute(angle, kSetpoint, now_s);

    ASSERT_TRUE(upstream_ran) << "step " << i;
    ASSERT_EQ(upstream_ran, port_ran) << "step " << i;
    ASSERT_NEAR(upstream.output, port.output(), kTolerance) << "step " << i;
  }
}

TEST(PidV1Equivalence, ReproducesStartupTransient)
{
  g_fake_millis = 0;
  UpstreamPid upstream;

  case_sim::PidV1 port(kKp, kKi, kKd, kOutMin, kOutMax, kSampleTimeS);
  port.initialize(upstream.input, upstream.output);

  // begin() calls SetMode(AUTOMATIC) before any DMP packet has arrived, so
  // Initialize() captures lastInput = input = 0 against setpoint = 182, and the
  // constructor's lastTime back-date makes the first Compute() run immediately.
  //
  // Steps 0-1: blind. error = 182 each call, so outputSum gains ki * 182 = 254.8
  //            and hits its +255 clamp on the second call, before any reading.
  // Step 2:    first real reading at 180. dInput = 180, so the derivative term
  //            is -90 * 180 = -16200 and the output clamps to -255.
  // Step 3:    182 with dInput = 2, so output = 0 + 255 - 90 * 2 = 75. Only a
  //            wound-up outputSum of exactly 255 produces that.
  // Step 4:    182 again. error and dInput are both 0, so output == outputSum,
  //            showing the integrator is still pinned at +255. Nothing since
  //            step 1 could have raised it -- the only contributions were
  //            ki * 2 and ki * 0, both clamped straight back to 255 -- so 255
  //            is also the value it held before the first reading arrived.
  struct Step
  {
    double input;
    double expected_output;
  };
  const Step sequence[] = {
    {0.0, 255.0},
    {0.0, 255.0},
    {180.0, -255.0},
    {182.0, 75.0},
    {182.0, 255.0},
  };

  int i = 0;
  for (const Step & s : sequence) {
    const unsigned long now_ms = static_cast<unsigned long>(i) * kSampleTimeMs;
    const double now_s = static_cast<double>(now_ms) / 1000.0;

    ASSERT_TRUE(upstream.step(s.input, now_ms)) << "step " << i;
    ASSERT_TRUE(port.compute(s.input, kSetpoint, now_s)) << "step " << i;

    EXPECT_NEAR(s.expected_output, upstream.output, kTolerance) << "upstream step " << i;
    EXPECT_NEAR(s.expected_output, port.output(), kTolerance) << "port step " << i;
    EXPECT_NEAR(upstream.output, port.output(), kTolerance) << "step " << i;
    ++i;
  }
}

TEST(PidV1Equivalence, PreScalesGainsBySamplePeriod)
{
  // Kp is used as written, Ki is multiplied by the sample period and Kd divided
  // by it. Observed through the public surface: with the integrator and the
  // previous input both zeroed, one step isolates each term.
  g_fake_millis = 0;
  case_sim::PidV1 port(kKp, kKi, kKd, -1.0e9, 1.0e9, kSampleTimeS);
  port.initialize(0.0, 0.0);

  // error = 1, dInput = 0  ->  kp * 1 + ki * 1
  ASSERT_TRUE(port.compute(0.0, 1.0, 0.0));
  EXPECT_NEAR(60.0 + 1.4, port.output(), kTolerance);

  // error = 0, dInput = 1  ->  outputSum (1.4) - kd * 1
  ASSERT_TRUE(port.compute(1.0, 1.0, 0.010));
  EXPECT_NEAR(1.4 - 90.0, port.output(), kTolerance);
}

TEST(PidV1Equivalence, GatesOnTheSamplePeriod)
{
  g_fake_millis = 0;
  UpstreamPid upstream;

  case_sim::PidV1 port(kKp, kKi, kKd, kOutMin, kOutMax, kSampleTimeS);
  port.initialize(upstream.input, upstream.output);

  // 1 ms ticks: both implementations must run on every tenth one, and the port
  // must agree with upstream about which. A double-precision gate comparing
  // now_s - last_time_s against 0.010 fails this, because the subtraction lands
  // fractionally short for most values on a millisecond grid.
  for (unsigned long ms = 0; ms <= 200; ++ms) {
    const bool upstream_ran = upstream.step(181.0, ms);
    const bool port_ran = port.compute(181.0, kSetpoint, static_cast<double>(ms) / 1000.0);
    ASSERT_EQ(upstream_ran, port_ran) << "ms " << ms;
    ASSERT_EQ(ms % 10 == 0, port_ran) << "ms " << ms;
    ASSERT_NEAR(upstream.output, port.output(), kTolerance) << "ms " << ms;
  }
}
