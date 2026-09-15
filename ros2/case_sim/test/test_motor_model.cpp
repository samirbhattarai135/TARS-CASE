#include <gtest/gtest.h>

#include "motor_model.hpp"

namespace
{

constexpr int kFirmwareDeadzone = 20;   // MOTOR_DEADZONE_PWM in balance_control.cpp
constexpr int kNoDeadzone = 0;          // self_balance.ino's actuator path

// Nominal column of config/sweep_ranges.yaml.
case_sim::MotorParams nominal_motor()
{
  return case_sim::MotorParams{0.0090, 0.0090, 4.5, 7.4};
}

}  // namespace

// The integer chain has no value between 0 and the deadzone floor. This step is
// what generates the hunt, so its position is asserted exactly rather than
// approximately.
TEST(PwmFromPidOutput, StepsStraightFromZeroToDeadzone)
{
  EXPECT_EQ(case_sim::pwm_from_pid_output(1.9, kFirmwareDeadzone), 0);
  EXPECT_EQ(case_sim::pwm_from_pid_output(1.999, kFirmwareDeadzone), 0);
  EXPECT_EQ(case_sim::pwm_from_pid_output(2.0, kFirmwareDeadzone), 20);
  EXPECT_EQ(case_sim::pwm_from_pid_output(-1.9, kFirmwareDeadzone), 0);
  EXPECT_EQ(case_sim::pwm_from_pid_output(-2.0, kFirmwareDeadzone), -20);
}

// Integer division in the remap holds the output flat until the numerator
// crosses the next multiple of 255.
TEST(PwmFromPidOutput, RemapAdvancesInIntegerSteps)
{
  EXPECT_EQ(case_sim::pwm_from_pid_output(3.9, kFirmwareDeadzone), 20);
  EXPECT_EQ(case_sim::pwm_from_pid_output(4.0, kFirmwareDeadzone), 21);
  EXPECT_EQ(case_sim::pwm_from_pid_output(-4.0, kFirmwareDeadzone), -21);
}

TEST(PwmFromPidOutput, SignFollowsPidOutputAcrossZero)
{
  EXPECT_EQ(case_sim::pwm_from_pid_output(0.0, kFirmwareDeadzone), 0);
  EXPECT_EQ(case_sim::pwm_from_pid_output(0.5, kFirmwareDeadzone), 0);
  EXPECT_EQ(case_sim::pwm_from_pid_output(-0.5, kFirmwareDeadzone), 0);
  EXPECT_EQ(case_sim::pwm_from_pid_output(100.0, kFirmwareDeadzone), 66);
  EXPECT_EQ(case_sim::pwm_from_pid_output(-100.0, kFirmwareDeadzone), -66);
  EXPECT_EQ(
    case_sim::pwm_from_pid_output(255.0, kFirmwareDeadzone),
    -case_sim::pwm_from_pid_output(-255.0, kFirmwareDeadzone));
}

// The halving happens before the remap, so the PID's full +/-255 authority
// reaches the motor as 137 -- about 54% duty. Not a bug to fix; a ceiling the
// simulation has to respect.
TEST(PwmFromPidOutput, CeilingIs137AtFullPidAuthority)
{
  EXPECT_EQ(case_sim::pwm_from_pid_output(255.0, kFirmwareDeadzone), 137);
  EXPECT_EQ(case_sim::pwm_from_pid_output(-255.0, kFirmwareDeadzone), -137);
  EXPECT_EQ(case_sim::pwm_from_pid_output(254.0, kFirmwareDeadzone), 137);
}

// deadzone_pwm = 0 must leave the halved value untouched: that is the only
// hardware-validated actuator path, and it is the sweep's anchor to reality.
TEST(PwmFromPidOutput, ZeroDeadzoneReproducesUncompensatedPath)
{
  EXPECT_EQ(case_sim::pwm_from_pid_output(2.0, kNoDeadzone), 1);
  EXPECT_EQ(case_sim::pwm_from_pid_output(1.9, kNoDeadzone), 0);
  EXPECT_EQ(case_sim::pwm_from_pid_output(100.0, kNoDeadzone), 50);
  EXPECT_EQ(case_sim::pwm_from_pid_output(255.0, kNoDeadzone), 127);
  EXPECT_EQ(case_sim::pwm_from_pid_output(-255.0, kNoDeadzone), -127);
}

TEST(TorqueFromPwm, IsNotQuantisedByIntegerDivision)
{
  const case_sim::MotorParams p = nominal_motor();
  EXPECT_GT(case_sim::torque_from_pwm(137, 0.0, p), 0.0);
  EXPECT_LT(case_sim::torque_from_pwm(-137, 0.0, p), 0.0);
  EXPECT_DOUBLE_EQ(case_sim::torque_from_pwm(0, 0.0, p), 0.0);
}

TEST(TorqueFromPwm, BackEmfReducesTorqueAsWheelSpeedRises)
{
  const case_sim::MotorParams p = nominal_motor();
  const double stall = case_sim::torque_from_pwm(137, 0.0, p);
  const double moving = case_sim::torque_from_pwm(137, 100.0, p);
  const double faster = case_sim::torque_from_pwm(137, 300.0, p);

  EXPECT_LT(moving, stall);
  EXPECT_LT(faster, moving);
}

// Past no-load speed the motor is being back-driven and opposes the wheel. A
// model without the Kv term would keep promising positive torque here, which is
// exactly the regime a falling robot enters.
TEST(TorqueFromPwm, GoesNegativeBeyondNoLoadSpeed)
{
  const case_sim::MotorParams p = nominal_motor();
  const double no_load_omega =
    p.supply_voltage_v * (137.0 / 255.0) / p.back_emf_constant_kv;

  EXPECT_GT(case_sim::torque_from_pwm(137, no_load_omega * 0.99, p), 0.0);
  EXPECT_LT(case_sim::torque_from_pwm(137, no_load_omega * 1.01, p), 0.0);
}

TEST(IsCoasting, GateBoundsCoastAndTheInteriorDrives)
{
  EXPECT_TRUE(case_sim::is_coasting(150.0, 150.0, 200.0));
  EXPECT_TRUE(case_sim::is_coasting(200.0, 150.0, 200.0));
  EXPECT_TRUE(case_sim::is_coasting(149.9, 150.0, 200.0));
  EXPECT_TRUE(case_sim::is_coasting(200.1, 150.0, 200.0));

  EXPECT_FALSE(case_sim::is_coasting(150.1, 150.0, 200.0));
  EXPECT_FALSE(case_sim::is_coasting(199.9, 150.0, 200.0));
  EXPECT_FALSE(case_sim::is_coasting(180.0, 150.0, 200.0));
  EXPECT_FALSE(case_sim::is_coasting(182.0, 150.0, 200.0));
}
