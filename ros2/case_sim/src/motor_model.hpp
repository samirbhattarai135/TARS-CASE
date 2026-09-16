#pragma once

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace case_sim
{

struct MotorParams
{
  double torque_constant_kt;    // N*m per amp
  double back_emf_constant_kv;  // V per rad/s
  double resistance_ohm;
  double supply_voltage_v;
};

/// The firmware's MotorMode (case_voice/balance_control.h:9), selecting which
/// branch of applyMotorControl() drives the wheels.
enum class MotorMode
{
  BalanceOnly,
  ForwardAssist,
  BackwardAssist,
  TurnLeft,
  TurnRight,
  Stopped,
};

/// What the two motors were told to do this cycle.
///
/// `coasting` is not "pwm happens to be zero". stop() pulls both direction pins
/// LOW, which is high impedance: no applied torque at all. A pwm of zero with a
/// direction still selected is short brake, and torque_from_pwm returns the
/// braking torque for it. The two are different physics and the caller has to
/// be able to tell them apart.
struct WheelDrive
{
  int left_pwm;
  int right_pwm;
  bool coasting;
};

/// The fixed PWM the firmware adds in the assist modes
/// (case_voice/balance_control.cpp:178). A constant there, so a constant here.
constexpr int kFirmwareAssistPwm = 80;

/// skipDeadzone(): map a non-zero request into [deadzone_pwm, 255] so the
/// smallest correction the PID asks for still turns the wheels. Zero stays
/// zero, so "no correction" still means "no drive".
///
/// Extracted because the firmware does NOT apply it uniformly: forward() and
/// reverse() call it, the TURN_LEFT / TURN_RIGHT branches write analogWrite()
/// directly and skip it. Naming it makes that asymmetry visible instead of
/// buried.
inline int skip_deadzone(int pwm, int deadzone_pwm)
{
  return pwm <= 0 ? 0 : deadzone_pwm + (pwm * (255 - deadzone_pwm)) / 255;
}

/// Signed PWM command from a PID output, reproducing the BALANCE_ONLY actuator
/// chain of case_voice/balance_control.cpp.
///
/// The arithmetic is integer on purpose. `int basePWM = output` truncates the
/// double toward zero, `abs(basePWM) / 2` is integer division, and skipDeadzone()
/// is integer too. A floating-point port moves the 0 -> deadzone step, and that
/// step is the limit-cycle generator the sweep exists to measure, so moving it
/// would report a stability the hardware does not have.
///
/// Consequence: |pid_output| below 2 commands nothing, and 2 or above commands
/// deadzone_pwm immediately. There is no value in between.
///
/// The halving happens before the deadzone remap, so with the PID clamped to
/// +/-255 upstream the largest magnitude reachable is 137 -- roughly 54% duty at
/// deadzone_pwm = 20. The firmware cannot request full torque at any tilt. That
/// bounds the recoverable angle and is the behaviour under test, not a defect to
/// correct here.
// output_divisor is the firmware's `output / 2`. It is a parameter, not a
// constant, because that single division caps the robot at ~54% of its motors'
// speed once the deadzone remap is applied -- and the 2026-09-16 sweep found
// wheel top speed (low back-EMF constant) to be the strongest discriminator
// between balancing and falling, ahead of torque. Setting it to 1 answers
// whether removing one character from the firmware materially changes
// stability. Default 2 is the firmware as written.
inline int pwm_from_pid_output(double pid_output, int deadzone_pwm, int output_divisor = 2)
{
  const int base_pwm = static_cast<int>(pid_output);

  // forward()/reverse() constrain the already-divided value rather than the PID
  // output, which is why the return range is [-255, 255] and not [-137, 137].
  const int divisor = output_divisor > 0 ? output_divisor : 1;
  const int requested = std::clamp(std::abs(base_pwm) / divisor, 0, 255);

  // deadzone_pwm == 0 collapses the remap to the identity and reproduces
  // self_balance/self_balance.ino, which has no deadzone compensation and is the
  // only hardware-validated actuator path available.
  const int magnitude = skip_deadzone(requested, deadzone_pwm);

  if (pid_output > 0.0) {
    return magnitude;
  }
  if (pid_output < 0.0) {
    return -magnitude;
  }
  return 0;
}

/// Port of applyMotorControl()'s mode switch, arithmetic unchanged.
///
/// The three non-balance branches are not the same kind of code as BALANCE_ONLY
/// and the differences are deliberate, not tidied:
///
///  * The assist modes add a flat kFirmwareAssistPwm on top of the balance
///    correction rather than leaning the setpoint, so the chassis is driven
///    without being tilted into the motion and the PID ends up opposing its own
///    throttle. Whether that topples the robot is a question this simulation can
///    now answer rather than guess.
///  * The turn branches bypass skipDeadzone() and constrain(): below the
///    deadzone they command a duty the motors cannot turn at, where
///    BALANCE_ONLY would have commanded the floor.
///  * The turn branches divide the DOUBLE `output` and truncate at the
///    analogWrite() call, where BALANCE_ONLY divides the already-truncated int.
///    Different results in the last count.
///
/// Motor A is the left wheel and Motor B the right (docs/wiring_diagram.md:68).
inline WheelDrive firmware_wheel_drive(
  MotorMode mode, double pid_output, int deadzone_pwm, int output_divisor = 2)
{
  const int divisor = output_divisor > 0 ? output_divisor : 1;
  const int halved = std::abs(static_cast<int>(pid_output)) / divisor;

  switch (mode) {
    case MotorMode::BalanceOnly: {
      const int pwm = pwm_from_pid_output(pid_output, deadzone_pwm, output_divisor);
      return {pwm, pwm, false};
    }

    // The assist is added whichever way the balance loop is correcting: when
    // the PID asks for the opposite direction the firmware still drives
    // forward, at the bare assist value. forward() commands both motors
    // identically, so an assist cannot steer.
    case MotorMode::ForwardAssist: {
      const int requested = pid_output > 0.0 ? halved + kFirmwareAssistPwm : kFirmwareAssistPwm;
      const int pwm = skip_deadzone(std::clamp(requested, 0, 255), deadzone_pwm);
      return {pwm, pwm, false};
    }

    case MotorMode::BackwardAssist: {
      const int requested = pid_output < 0.0 ? halved + kFirmwareAssistPwm : kFirmwareAssistPwm;
      const int pwm = skip_deadzone(std::clamp(requested, 0, 255), deadzone_pwm);
      return {-pwm, -pwm, false};
    }

    case MotorMode::TurnLeft:
    case MotorMode::TurnRight: {
      const double magnitude = std::abs(pid_output);
      const int slow = static_cast<int>(magnitude / 4.0);
      const int fast = static_cast<int>(magnitude / 2.0);

      // Both wheels take the sign of the PID output, so a turn drives the pair
      // the same way and yaws only because one side is slower. It cannot spin
      // in place, and it cannot turn at all while the balance correction is
      // near zero -- which is exactly when the robot is upright and a turn was
      // most likely to be asked for.
      const int sign = pid_output > 0.0 ? 1 : -1;
      const int left = (mode == MotorMode::TurnLeft) ? slow : fast;
      const int right = (mode == MotorMode::TurnLeft) ? fast : slow;
      return {sign * left, sign * right, false};
    }

    case MotorMode::Stopped:
      return {0, 0, true};
  }

  return {0, 0, true};
}

/// Threshold a continuous velocity command into one of the firmware's modes.
///
/// The firmware's modes are latched and mutually exclusive: there is no state
/// that drives and turns at once, so a Twist carrying both has to lose one.
/// Turn wins, because a robot that ignores the steering is harder to recover
/// from than one that ignores the throttle.
///
/// Zero command maps to BalanceOnly, not Stopped: releasing the throttle means
/// "stop driving", never "cut the motors while balancing". Stopped is reachable
/// on hardware from the voice command, which has no velocity to threshold.
inline MotorMode firmware_mode_from_command(
  double forward_command, double turn_command, double deadband = 1e-3)
{
  if (std::abs(turn_command) > deadband) {
    return turn_command > 0.0 ? MotorMode::TurnLeft : MotorMode::TurnRight;
  }
  if (forward_command > deadband) {
    return MotorMode::ForwardAssist;
  }
  if (forward_command < -deadband) {
    return MotorMode::BackwardAssist;
  }
  return MotorMode::BalanceOnly;
}

/// Steering split for the lean-offset drive. NOT firmware behaviour.
///
/// Forward motion is deliberately absent here: it is a shift of the PID
/// setpoint applied upstream, so the balance loop tilts the chassis into the
/// direction of travel and then drives to hold that tilt. That is what lets a
/// balancer go somewhere without fighting itself, and it is the one thing the
/// firmware's assist modes do not do.
///
/// What is left is the yaw term, which is differential: it adds to one wheel
/// exactly what it subtracts from the other, so the net forward command the
/// pitch loop sees is unchanged and steering cannot destabilise balance.
///
/// ponytail: the split is applied AFTER the deadzone remap, so a small steer
/// while the balance output is near zero lands both wheels inside the deadzone.
/// torque_from_pwm is linear and models no stiction, so the simulation will not
/// show it -- but as a firmware proposal the split belongs before
/// skip_deadzone. Move it there if this is ever ported to the hardware.
inline WheelDrive lean_offset_wheel_drive(int balance_pwm, double steer_pwm)
{
  const int steer = static_cast<int>(steer_pwm);
  return {
    std::clamp(balance_pwm - steer, -255, 255),
    std::clamp(balance_pwm + steer, -255, 255),
    false};
}

/// tau = Kt * (V_supply * pwm/255 - Kv * omega) / R
///
/// At pwm = 0 while inside the gate this returns -Kt*Kv*omega/R, which is
/// correct rather than an artifact: forward(0)/reverse(0) still drives IN1/IN2,
/// and a TB6612 with a direction selected and PWM low is short brake. Only
/// stop() -- IN1 and IN2 both LOW, high impedance -- truly coasts, which is what
/// is_coasting() and WheelDrive::coasting report.
///
/// ponytail: linear back-EMF with no saturation, gearbox friction, or
/// commutation loss. Upgrade path is a measured torque/speed curve from the
/// bench once the motors exist.
inline double torque_from_pwm(int pwm, double wheel_omega_rad_s, const MotorParams & p)
{
  // Integer division here would zero the duty cycle for every |pwm| < 255.
  const double duty = static_cast<double>(pwm) / 255.0;

  return p.torque_constant_kt *
         (p.supply_voltage_v * duty - p.back_emf_constant_kv * wheel_omega_rad_s) /
         p.resistance_ohm;
}

/// True when applyMotorControl() falls through to stop() and the wheels carry no
/// applied torque at all.
///
/// The firmware's gate is `input > 150 && input < 200`, strict on both sides, so
/// a pitch sitting exactly on either bound coasts.
inline bool is_coasting(double pitch_deg, double gate_lower_deg, double gate_upper_deg)
{
  return !(pitch_deg > gate_lower_deg && pitch_deg < gate_upper_deg);
}

}  // namespace case_sim
