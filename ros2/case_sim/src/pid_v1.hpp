#ifndef CASE_SIM__PID_V1_HPP_
#define CASE_SIM__PID_V1_HPP_

#include <cmath>

namespace case_sim
{

/// Verbatim port of Arduino PID_v1 1.1.1 in its P_ON_E / DIRECT configuration,
/// with no Arduino and no ROS dependency.
///
/// The quirks below are reproduced deliberately. Kp=60 / Ki=140 / Kd=0.9 only
/// mean what the firmware makes them mean under this exact arithmetic, so a
/// textbook PID here would silently invalidate every simulation result.
class PidV1
{
public:
  /// kp/ki/kd are the unscaled user-facing gains, exactly as passed to the
  /// PID constructor in case_voice/balance_control.cpp.
  PidV1(
    double kp, double ki, double kd,
    double out_min, double out_max, double sample_time_s)
  : sample_time_ms_(static_cast<unsigned long>(std::lround(sample_time_s * 1000.0))),
    out_min_(out_min),
    out_max_(out_max),
    // Upstream's constructor sets lastTime = millis() - SampleTime, relying on
    // unsigned wraparound at boot so the first Compute() fires immediately
    // instead of one period later. That is not a bug to be smoothed over: the
    // firmware really does run a control cycle before the first sensor packet,
    // and the transient it produces is part of what is being simulated.
    last_time_ms_(0UL - sample_time_ms_)
  {
    // SetTunings folds the sample period into the gains once, here, so compute()
    // neither multiplies the integral by dt nor divides the derivative by it.
    const double sample_time_seconds = static_cast<double>(sample_time_ms_) / 1000.0;
    kp_ = kp;
    ki_ = ki * sample_time_seconds;
    kd_ = kd / sample_time_seconds;
  }

  /// Mirrors PID::Initialize(), which upstream reaches via SetMode(AUTOMATIC).
  /// `output` is the caller's current output variable, which upstream reads
  /// through myOutput to make the manual-to-automatic transfer bumpless.
  void initialize(double input, double output)
  {
    output_sum_ = output;
    last_input_ = input;
    if (output_sum_ > out_max_) {
      output_sum_ = out_max_;
    } else if (output_sum_ < out_min_) {
      output_sum_ = out_min_;
    }
    output_ = output;
  }

  /// Mirrors PID::Compute(). Returns true when a computation occurred.
  bool compute(double input, double setpoint, double now_s)
  {
    // The gate runs on integer milliseconds because upstream's clock is
    // millis(). Rounding to that grid is not a convenience: a literal
    // double-precision gate drops cycles, since 0.03 - 0.02 lands a few ULP
    // below 0.010 and fails a >= 0.010 test. On a 10 ms grid that silently
    // skips roughly 44% of control cycles, and Ki=140 is far too
    // timestep-sensitive to survive that.
    const unsigned long now_ms = static_cast<unsigned long>(std::lround(now_s * 1000.0));
    if (now_ms - last_time_ms_ < sample_time_ms_) {
      return false;
    }

    const double error = setpoint - input;
    // Derivative on measurement, not on error: a setpoint step produces no kick.
    const double d_input = input - last_input_;

    output_sum_ += ki_ * error;
    // Anti-windup: the accumulator is bounded at the point of accumulation,
    // before the proportional term joins it. Clamping the finished sum instead
    // would change how the controller recovers from a large tilt.
    if (output_sum_ > out_max_) {
      output_sum_ = out_max_;
    } else if (output_sum_ < out_min_) {
      output_sum_ = out_min_;
    }

    // Grouped as upstream groups it: P on its own, then (I - D) added to it.
    // Collapsing this into one expression would reassociate the rounding.
    double output = kp_ * error;
    output += output_sum_ - kd_ * d_input;

    if (output > out_max_) {
      output = out_max_;
    } else if (output < out_min_) {
      output = out_min_;
    }
    output_ = output;

    last_input_ = input;
    last_time_ms_ = now_ms;
    return true;
  }

  double output() const {return output_;}

private:
  unsigned long sample_time_ms_;
  double kp_ = 0.0;
  double ki_ = 0.0;
  double kd_ = 0.0;
  double out_min_;
  double out_max_;
  unsigned long last_time_ms_;
  double output_sum_ = 0.0;
  double last_input_ = 0.0;
  double output_ = 0.0;
};

}  // namespace case_sim

#endif  // CASE_SIM__PID_V1_HPP_
