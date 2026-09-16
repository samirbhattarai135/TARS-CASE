#pragma once

#include <algorithm>
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

  // skipDeadzone(): zero stays zero, so "no correction" still means "no drive".
  // deadzone_pwm == 0 collapses the remap to the identity and reproduces
  // self_balance/self_balance.ino, which has no deadzone compensation and is the
  // only hardware-validated actuator path available.
  const int magnitude =
    requested <= 0 ? 0 : deadzone_pwm + (requested * (255 - deadzone_pwm)) / 255;

  if (pid_output > 0.0) {
    return magnitude;
  }
  if (pid_output < 0.0) {
    return -magnitude;
  }
  return 0;
}

/// tau = Kt * (V_supply * pwm/255 - Kv * omega) / R
///
/// At pwm = 0 while inside the gate this returns -Kt*Kv*omega/R, which is
/// correct rather than an artifact: forward(0)/reverse(0) still drives IN1/IN2,
/// and a TB6612 with a direction selected and PWM low is short brake. Only
/// stop() -- IN1 and IN2 both LOW, high impedance -- truly coasts, which is what
/// is_coasting() reports.
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
