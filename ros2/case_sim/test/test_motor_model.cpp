#include <gtest/gtest.h>

#include "motor_model.hpp"

namespace
{

constexpr int kFirmwareDeadzone = 20;   // MOTOR_DEADZONE_PWM in balance_control.cpp
constexpr int kNoDeadzone = 0;          // self_balance.ino's actuator path

// Nominal drive column of config/sweep_ranges.yaml.
double drive_lean(double target_m_s, double measured_m_s)
{
  return case_sim::drive_lean_from_velocity_error(target_m_s, measured_m_s, 6.0, 4.0);
}

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


// The firmware's halving is now a parameter; default behaviour must not move,
// and removing it must genuinely raise the achievable PWM ceiling.
TEST(MotorModel, OutputDivisorDefaultsToFirmwareBehaviour)
{
  for (double out = -255.0; out <= 255.0; out += 0.5) {
    EXPECT_EQ(case_sim::pwm_from_pid_output(out, 20),
              case_sim::pwm_from_pid_output(out, 20, 2))
      << "default divisor must reproduce the firmware at output " << out;
  }
}

TEST(MotorModel, RemovingTheHalvingRaisesTheCeiling)
{
  const int halved = case_sim::pwm_from_pid_output(255.0, 20, 2);
  const int full = case_sim::pwm_from_pid_output(255.0, 20, 1);
  EXPECT_EQ(halved, 137) << "firmware ceiling is 137 of 255, about 54% duty";
  EXPECT_EQ(full, 255) << "without the halving the full range is reachable";
  EXPECT_GT(full, halved);
}

TEST(MotorModel, OutputDivisorRejectsZero)
{
  // A zero divisor would be a division by zero on the control path.
  EXPECT_EQ(case_sim::pwm_from_pid_output(100.0, 0, 0),
            case_sim::pwm_from_pid_output(100.0, 0, 1));
}


// ---------------------------------------------------------------------------
// Drive modes
// ---------------------------------------------------------------------------

// The whole sweep history was recorded through pwm_from_pid_output. The drive
// layer is only trustworthy if routing BALANCE_ONLY through it changes nothing.
TEST(FirmwareWheelDrive, BalanceOnlyIsUnchangedAndNeverSteers)
{
  for (double out = -255.0; out <= 255.0; out += 0.5) {
    const case_sim::WheelDrive drive =
      case_sim::firmware_wheel_drive(case_sim::MotorMode::BalanceOnly, out, kFirmwareDeadzone);
    const int expected = case_sim::pwm_from_pid_output(out, kFirmwareDeadzone);

    EXPECT_EQ(drive.left_pwm, expected) << "at output " << out;
    EXPECT_EQ(drive.right_pwm, expected) << "at output " << out;
    EXPECT_FALSE(drive.coasting) << "at output " << out;
  }
}

// forward(abs(basePWM) / 2 + 80), then skipDeadzone. At output 100 that is
// 50 + 80 = 130, remapped to 20 + (130 * 235) / 255 = 139.
TEST(FirmwareWheelDrive, ForwardAssistAddsTheFlatEightyBeforeTheRemap)
{
  const case_sim::WheelDrive drive =
    case_sim::firmware_wheel_drive(case_sim::MotorMode::ForwardAssist, 100.0, kFirmwareDeadzone);

  EXPECT_EQ(drive.left_pwm, 139);
  EXPECT_EQ(drive.right_pwm, drive.left_pwm) << "forward() commands both motors alike";
}

// The `else` branch: the assist drives forward at the bare 80 even while the
// PID is asking for the opposite correction. This is the behaviour worth
// measuring -- the robot is thrown forward with no balance authority spent.
TEST(FirmwareWheelDrive, ForwardAssistDrivesForwardAgainstANegativeCorrection)
{
  const case_sim::WheelDrive against =
    case_sim::firmware_wheel_drive(case_sim::MotorMode::ForwardAssist, -200.0, kFirmwareDeadzone);

  EXPECT_EQ(against.left_pwm, case_sim::skip_deadzone(80, kFirmwareDeadzone));
  EXPECT_GT(against.left_pwm, 0) << "forward, though the PID asked to go back";
}

TEST(FirmwareWheelDrive, BackwardAssistMirrorsForwardAssist)
{
  const case_sim::WheelDrive back =
    case_sim::firmware_wheel_drive(case_sim::MotorMode::BackwardAssist, -100.0, kFirmwareDeadzone);
  const case_sim::WheelDrive forward =
    case_sim::firmware_wheel_drive(case_sim::MotorMode::ForwardAssist, 100.0, kFirmwareDeadzone);

  EXPECT_EQ(back.left_pwm, -forward.left_pwm);
  EXPECT_EQ(back.right_pwm, -forward.right_pwm);
}

// The turn branches call analogWrite() directly, so they never pass through
// skipDeadzone(). At output 100 that is abs(100)/4 = 25 and abs(100)/2 = 50
// raw -- where BALANCE_ONLY would have remapped both onto the deadzone floor.
TEST(FirmwareWheelDrive, TurnsBypassTheDeadzoneRemap)
{
  const case_sim::WheelDrive left =
    case_sim::firmware_wheel_drive(case_sim::MotorMode::TurnLeft, 100.0, kFirmwareDeadzone);

  EXPECT_EQ(left.left_pwm, 25) << "Motor A, the left wheel, is the slow one";
  EXPECT_EQ(left.right_pwm, 50);
  EXPECT_NE(left.left_pwm, case_sim::skip_deadzone(25, kFirmwareDeadzone));
}

TEST(FirmwareWheelDrive, TurnRightSlowsTheOtherWheel)
{
  const case_sim::WheelDrive right =
    case_sim::firmware_wheel_drive(case_sim::MotorMode::TurnRight, 100.0, kFirmwareDeadzone);

  EXPECT_EQ(right.left_pwm, 50);
  EXPECT_EQ(right.right_pwm, 25);
}

// Both wheels take the sign of the PID output, so a turn cannot spin in place;
// it yaws only while the balance loop is already driving. Near upright, where
// a turn is most likely to be commanded, it barely turns at all.
TEST(FirmwareWheelDrive, TurnsCannotSpinInPlaceAndStallNearUpright)
{
  const case_sim::WheelDrive driving =
    case_sim::firmware_wheel_drive(case_sim::MotorMode::TurnLeft, 100.0, kFirmwareDeadzone);
  EXPECT_GT(driving.left_pwm, 0);
  EXPECT_GT(driving.right_pwm, 0) << "same direction; only the speeds differ";

  const case_sim::WheelDrive reversing =
    case_sim::firmware_wheel_drive(case_sim::MotorMode::TurnLeft, -100.0, kFirmwareDeadzone);
  EXPECT_LT(reversing.left_pwm, 0);
  EXPECT_LT(reversing.right_pwm, 0);

  const case_sim::WheelDrive upright =
    case_sim::firmware_wheel_drive(case_sim::MotorMode::TurnLeft, 3.0, kFirmwareDeadzone);
  EXPECT_EQ(upright.left_pwm, 0) << "abs(3.0)/4 truncates to nothing";
  EXPECT_EQ(upright.right_pwm, 1);
}

// STOPPED is stop(): both direction pins low, high impedance. It must be
// distinguishable from a pwm that happens to be zero, which is short brake.
TEST(FirmwareWheelDrive, StoppedCoastsRatherThanCommandingZero)
{
  const case_sim::WheelDrive stopped =
    case_sim::firmware_wheel_drive(case_sim::MotorMode::Stopped, 100.0, kFirmwareDeadzone);

  EXPECT_TRUE(stopped.coasting);
  EXPECT_EQ(stopped.left_pwm, 0);
  EXPECT_EQ(stopped.right_pwm, 0);
}

TEST(FirmwareModeFromCommand, TurnWinsOverDriveBecauseTheFirmwareCannotDoBoth)
{
  EXPECT_EQ(
    case_sim::firmware_mode_from_command(1.0, 1.0), case_sim::MotorMode::TurnLeft);
  EXPECT_EQ(
    case_sim::firmware_mode_from_command(-1.0, -1.0), case_sim::MotorMode::TurnRight);
}

TEST(FirmwareModeFromCommand, ZeroCommandKeepsBalancingRatherThanStopping)
{
  EXPECT_EQ(case_sim::firmware_mode_from_command(0.0, 0.0), case_sim::MotorMode::BalanceOnly);
  EXPECT_EQ(case_sim::firmware_mode_from_command(1e-9, 1e-9), case_sim::MotorMode::BalanceOnly);
  EXPECT_EQ(case_sim::firmware_mode_from_command(1.0, 0.0), case_sim::MotorMode::ForwardAssist);
  EXPECT_EQ(case_sim::firmware_mode_from_command(-1.0, 0.0), case_sim::MotorMode::BackwardAssist);
}

// The steer term is differential, so it adds nothing to the net forward command
// the pitch loop sees. Steering cannot destabilise balance by construction.
TEST(LeanOffsetWheelDrive, SteeringIsPurelyDifferential)
{
  const case_sim::WheelDrive straight = case_sim::lean_offset_wheel_drive(100, 0.0);
  EXPECT_EQ(straight.left_pwm, 100);
  EXPECT_EQ(straight.right_pwm, 100);

  const case_sim::WheelDrive turning = case_sim::lean_offset_wheel_drive(100, 40.0);
  EXPECT_EQ(turning.left_pwm, 60) << "positive yaw slows the left wheel";
  EXPECT_EQ(turning.right_pwm, 140);
  EXPECT_EQ(turning.left_pwm + turning.right_pwm, straight.left_pwm + straight.right_pwm);
}

TEST(LeanOffsetWheelDrive, ClampsToTheCommandableRange)
{
  const case_sim::WheelDrive saturated = case_sim::lean_offset_wheel_drive(250, 100.0);
  EXPECT_EQ(saturated.right_pwm, 255);
  EXPECT_EQ(saturated.left_pwm, 150);
  EXPECT_FALSE(saturated.coasting);
}

// At rest with a forward command the loop must ask for its full lean; once the
// robot is up to speed it must ask for none, because a robot already travelling
// at the commanded speed needs no acceleration and therefore no tilt.
TEST(DriveLeanFromVelocityError, LeansToAccelerateAndStandsUpOnceUpToSpeed)
{
  EXPECT_DOUBLE_EQ(drive_lean(0.5, 0.5), 0.0) << "at speed, upright";
  EXPECT_GT(drive_lean(0.5, 0.0), 0.0) << "below speed, lean forward";
  EXPECT_LT(drive_lean(0.0, 0.5), 0.0) << "above speed, lean back to slow down";
  EXPECT_GT(drive_lean(0.5, 0.25), 0.0);
  EXPECT_LT(drive_lean(0.5, 0.25), drive_lean(0.5, 0.0)) << "smaller error, smaller lean";
}

// The clamp is what keeps the commanded lean inside what the wheels can hold.
// Without it a large command asks for a tilt no torque can sustain, and the
// robot lurches once and falls -- measured on 2026-09-16 at 12 degrees.
TEST(DriveLeanFromVelocityError, ClampsToWhatTheWheelsCanSustain)
{
  EXPECT_DOUBLE_EQ(drive_lean(100.0, 0.0), 4.0);
  EXPECT_DOUBLE_EQ(drive_lean(-100.0, 0.0), -4.0);
  // A negative limit must clamp by magnitude, not invert the loop.
  EXPECT_DOUBLE_EQ(
    case_sim::drive_lean_from_velocity_error(100.0, 0.0, 6.0, -4.0), 4.0);
}

TEST(DriveLeanFromVelocityError, NegativeGainReversesTheDriveDirection)
{
  EXPECT_LT(case_sim::drive_lean_from_velocity_error(0.5, 0.0, -6.0, 4.0), 0.0);
}
