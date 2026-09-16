#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <exception>
#include <memory>
#include <optional>
#include <random>
#include <string>
#include <vector>

#include "controller_interface/controller_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float64.hpp"

// realtime_tools renamed its headers from .h to .hpp mid-distro. Probing keeps
// this file buildable on either side of that rename.
#if __has_include("realtime_tools/realtime_publisher.hpp")
#include "realtime_tools/realtime_publisher.hpp"
#else
#include "realtime_tools/realtime_publisher.h"
#endif

#if __has_include("realtime_tools/realtime_buffer.hpp")
#include "realtime_tools/realtime_buffer.hpp"
#else
#include "realtime_tools/realtime_buffer.h"
#endif

#include "motor_model.hpp"
#include "pid_v1.hpp"

namespace case_sim
{
namespace
{

constexpr double kRadiansToDegrees = 180.0 / M_PI;
constexpr double kDegreesToRadians = M_PI / 180.0;
constexpr double kGravity = 9.8;
constexpr double kUprightDeg = 180.0;

using Float64Publisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float64>;

// How a velocity command reaches the wheels. Parsed once from the
// `drive_mode` string at configuration so that update() can never be handed
// a mode the controller does not implement.
enum class DriveMode
{
  None,        // no drive input at all; what every sweep so far measured
  Firmware,    // applyMotorControl()'s assist and turn branches, as written
  LeanOffset,  // setpoint shift plus a differential steer; not firmware
};

// Reproduces dmpGetGravity() followed by dmpGetYawPitchRoll() from
// MPU6050_6Axis_MotionApps20, which is the formula the firmware's pitch comes
// out of. A generic quaternion-to-Euler conversion agrees with it near upright
// but not past vertical, and past vertical is where the pass criterion is
// decided, so the library's exact expression is used instead.
double dmp_pitch_rad(double qw, double qx, double qy, double qz)
{
  const double gravity_x = 2.0 * (qx * qz - qw * qy);
  const double gravity_y = 2.0 * (qw * qx + qy * qz);
  const double gravity_z = qw * qw - qx * qx - qy * qy + qz * qz;

  double pitch = std::atan2(gravity_x, std::sqrt(gravity_y * gravity_y + gravity_z * gravity_z));
  if (gravity_z < 0.0) {
    pitch = (pitch > 0.0) ? (M_PI - pitch) : (-M_PI - pitch);
  }
  return pitch;
}

// trylock() returns false rather than waiting, so a busy publisher costs a
// dropped sample instead of a missed control cycle.
void publish_without_blocking(Float64Publisher * publisher, double value)
{
  if (publisher != nullptr && publisher->trylock()) {
    publisher->msg_.data = value;
    publisher->unlockAndPublish();
  }
}

// Jazzy returns std::optional: an interface can legitimately have no value yet,
// most often before the simulation has produced its first sensor update.
// Substituting 0.0 for a missing orientation would read as a robot lying flat
// and provoke a full-scale correction against a measurement that does not exist.
bool read_state(const hardware_interface::LoanedStateInterface & interface, double & out)
{
  const std::optional<double> value = interface.get_optional();
  if (!value.has_value()) {
    return false;
  }
  out = value.value();
  return true;
}

}  // namespace

class BalanceController : public controller_interface::ControllerInterface
{
public:
  controller_interface::CallbackReturn on_init() override;
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  bool find_state_index(const std::string & full_name, std::size_t & index) const;
  bool find_command_index(const std::string & full_name, std::size_t & index) const;

  std::string left_wheel_joint_;
  std::string right_wheel_joint_;
  // Prefixes the IMU state interfaces; must match the <sensor name> that the
  // URDF's <ros2_control> block declares.
  std::string imu_sensor_name_;

  double lean_deg_{0.0};
  double gate_lower_deg_{0.0};
  double gate_upper_deg_{0.0};
  int deadzone_pwm_{0};

  int delay_samples_{0};
  double angle_noise_sigma_deg_{0.0};
  bool rotation_artifact_enabled_{false};
  double imu_lever_arm_m_{0.030};
  int noise_seed_{0};
  double motor_direction_sign_{1.0};

  DriveMode drive_mode_{DriveMode::None};
  double lean_gain_deg_{0.0};
  double steer_gain_pwm_{0.0};

  MotorParams motor_params_{};
  std::optional<PidV1> pid_;

  std::vector<double> pitch_delay_buffer_;
  std::size_t delay_write_index_{0};

  double previous_pitch_deg_{0.0};
  int pitch_history_depth_{0};

  std::mt19937 noise_rng_;
  std::normal_distribution<double> standard_normal_{0.0, 1.0};

  std::array<std::size_t, 4> orientation_index_{};  // x, y, z, w
  std::size_t left_velocity_index_{0};
  std::size_t right_velocity_index_{0};
  std::size_t left_effort_index_{0};
  std::size_t right_effort_index_{0};

  // set_value() is [[nodiscard]] because it can fail to take the lock. A
  // dropped effort command leaves the wheels on their previous torque for a
  // cycle, which on a balancing robot is a real disturbance rather than a
  // logging concern -- so failures are counted and reported, never discarded.
  bool command_effort(std::size_t index, double value);
  bool reproduce_startup_windup_{false};
  bool holding_{true};
  double hold_release_deg_{1.0};
  int output_divisor_{2};
  bool pid_initialised_{true};
  std::uint64_t command_write_failures_{0};
  std::uint64_t state_read_failures_{0};

  std::unique_ptr<Float64Publisher> pitch_publisher_;
  std::unique_ptr<Float64Publisher> pid_output_publisher_;
  std::unique_ptr<Float64Publisher> pwm_publisher_;

  // Written by the subscription thread, read by update(). The buffer is the
  // handoff: update() must not take a lock a publisher can hold.
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::Twist> velocity_command_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_subscription_;
};

controller_interface::CallbackReturn BalanceController::on_init()
{
  auto node = get_node();

  // Declared with a type but no default. config/sweep_ranges.yaml is the single
  // home for every one of these values; a default here would be a second home
  // that can silently drift out of step with it. A parameter left unset must
  // fail configuration, not quietly substitute a stale number.
  const auto declare_from_sweep_ranges =
    [&node](const std::string & name, rclcpp::ParameterType type) {
      if (!node->has_parameter(name)) {
        node->declare_parameter(name, type);
      }
    };

  const auto declare_with_default = [&node](const std::string & name, const auto & value) {
    if (!node->has_parameter(name)) {
      node->declare_parameter(name, value);
    }
  };

  try {
    declare_from_sweep_ranges("kp", rclcpp::PARAMETER_DOUBLE);
    declare_from_sweep_ranges("ki", rclcpp::PARAMETER_DOUBLE);
    declare_from_sweep_ranges("kd", rclcpp::PARAMETER_DOUBLE);
    declare_from_sweep_ranges("sample_time_ms", rclcpp::PARAMETER_INTEGER);
    declare_from_sweep_ranges("output_limit", rclcpp::PARAMETER_DOUBLE);
    declare_from_sweep_ranges("lean_deg", rclcpp::PARAMETER_DOUBLE);
    declare_from_sweep_ranges("gate_lower_deg", rclcpp::PARAMETER_DOUBLE);
    declare_from_sweep_ranges("gate_upper_deg", rclcpp::PARAMETER_DOUBLE);
    declare_from_sweep_ranges("deadzone_pwm", rclcpp::PARAMETER_INTEGER);

    declare_from_sweep_ranges("delay_samples", rclcpp::PARAMETER_INTEGER);
    declare_from_sweep_ranges("angle_noise_sigma_deg", rclcpp::PARAMETER_DOUBLE);
    declare_from_sweep_ranges("rotation_artifact_enabled", rclcpp::PARAMETER_BOOL);
    declare_from_sweep_ranges("reproduce_startup_windup", rclcpp::PARAMETER_BOOL);
    declare_from_sweep_ranges("hold_release_deg", rclcpp::PARAMETER_DOUBLE);
    declare_from_sweep_ranges("output_divisor", rclcpp::PARAMETER_INTEGER);
    declare_from_sweep_ranges("imu_lever_arm_m", rclcpp::PARAMETER_DOUBLE);

    declare_from_sweep_ranges("drive_mode", rclcpp::PARAMETER_STRING);
    declare_from_sweep_ranges("lean_gain_deg", rclcpp::PARAMETER_DOUBLE);
    declare_from_sweep_ranges("steer_gain_pwm", rclcpp::PARAMETER_DOUBLE);

    declare_from_sweep_ranges("torque_constant_kt", rclcpp::PARAMETER_DOUBLE);
    declare_from_sweep_ranges("back_emf_constant_kv", rclcpp::PARAMETER_DOUBLE);
    declare_from_sweep_ranges("resistance_ohm", rclcpp::PARAMETER_DOUBLE);
    declare_from_sweep_ranges("supply_voltage_v", rclcpp::PARAMETER_DOUBLE);

    // Not physical ranges, so not in sweep_ranges.yaml. Defaults are safe here.
    declare_with_default("left_wheel_joint", std::string("left_wheel_joint"));
    declare_with_default("right_wheel_joint", std::string("right_wheel_joint"));
    declare_with_default("imu_sensor_name", std::string("imu_sensor"));
    declare_with_default("noise_seed", 0);

    // Calibration knob, not a workaround. The feedback loop's sign depends on
    // the quaternion-to-pitch convention here and on the direction of the wheel
    // joint axes in the URDF, which are chosen independently. Inverted, the
    // controller drives the chassis over faster than gravity would and the
    // failure reads as "the gains do not work" rather than as a sign error.
    // Hardware carries the same ambiguity and resolves it by swapping motor
    // leads; this is that swap. Expected to be +1.0 or -1.0.
    declare_with_default("motor_direction_sign", 1.0);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_node()->get_logger(), "Parameter declaration failed: %s", error.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BalanceController::on_configure(
  const rclcpp_lifecycle::State & /*previous*/)
{
  auto node = get_node();

  double kp = 0.0;
  double ki = 0.0;
  double kd = 0.0;
  double output_limit = 0.0;
  double sample_time_s = 0.0;
  std::string drive_mode_name;

  try {
    kp = node->get_parameter("kp").as_double();
    ki = node->get_parameter("ki").as_double();
    kd = node->get_parameter("kd").as_double();
    sample_time_s = static_cast<double>(node->get_parameter("sample_time_ms").as_int()) / 1000.0;
    output_limit = node->get_parameter("output_limit").as_double();
    lean_deg_ = node->get_parameter("lean_deg").as_double();
    gate_lower_deg_ = node->get_parameter("gate_lower_deg").as_double();
    gate_upper_deg_ = node->get_parameter("gate_upper_deg").as_double();
    deadzone_pwm_ = static_cast<int>(node->get_parameter("deadzone_pwm").as_int());

    delay_samples_ = static_cast<int>(node->get_parameter("delay_samples").as_int());
    angle_noise_sigma_deg_ = node->get_parameter("angle_noise_sigma_deg").as_double();
    rotation_artifact_enabled_ = node->get_parameter("rotation_artifact_enabled").as_bool();
    reproduce_startup_windup_ = node->get_parameter("reproduce_startup_windup").as_bool();
    hold_release_deg_ = node->get_parameter("hold_release_deg").as_double();
    output_divisor_ = static_cast<int>(node->get_parameter("output_divisor").as_int());

    drive_mode_name = node->get_parameter("drive_mode").as_string();
    lean_gain_deg_ = node->get_parameter("lean_gain_deg").as_double();
    steer_gain_pwm_ = node->get_parameter("steer_gain_pwm").as_double();

    motor_params_.torque_constant_kt = node->get_parameter("torque_constant_kt").as_double();
    motor_params_.back_emf_constant_kv = node->get_parameter("back_emf_constant_kv").as_double();
    motor_params_.resistance_ohm = node->get_parameter("resistance_ohm").as_double();
    motor_params_.supply_voltage_v = node->get_parameter("supply_voltage_v").as_double();

    left_wheel_joint_ = node->get_parameter("left_wheel_joint").as_string();
    right_wheel_joint_ = node->get_parameter("right_wheel_joint").as_string();
    imu_sensor_name_ = node->get_parameter("imu_sensor_name").as_string();
    noise_seed_ = static_cast<int>(node->get_parameter("noise_seed").as_int());
    motor_direction_sign_ = node->get_parameter("motor_direction_sign").as_double();
  } catch (const std::exception & error) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Cannot read parameters (unset, or written with the wrong YAML type -- every value "
      "declared double must carry a decimal point): %s",
      error.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (sample_time_s <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "sample_time_ms must be positive");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (delay_samples_ < 0) {
    RCLCPP_ERROR(node->get_logger(), "delay_samples must not be negative");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (angle_noise_sigma_deg_ < 0.0 || imu_lever_arm_m_ < 0.0) {
    RCLCPP_ERROR(node->get_logger(), "noise sigma and IMU lever arm must not be negative");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (motor_params_.resistance_ohm <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "resistance_ohm must be positive");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Parsed here and nowhere else, so an unrecognised name fails configuration
  // instead of silently selecting a default and reporting a result for a
  // drive nobody asked for.
  if (drive_mode_name == "none") {
    drive_mode_ = DriveMode::None;
  } else if (drive_mode_name == "firmware") {
    drive_mode_ = DriveMode::Firmware;
  } else if (drive_mode_name == "lean_offset") {
    drive_mode_ = DriveMode::LeanOffset;
  } else {
    RCLCPP_ERROR(
      node->get_logger(),
      "drive_mode must be none, firmware or lean_offset; got '%s'",
      drive_mode_name.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  pid_.emplace(kp, ki, kd, -output_limit, output_limit, sample_time_s);

  // One slot beyond the delay so the slot about to be overwritten is the one
  // written delay_samples updates ago.
  pitch_delay_buffer_.assign(static_cast<std::size_t>(delay_samples_) + 1U, 0.0);

  auto pitch_topic = node->create_publisher<std_msgs::msg::Float64>(
    "/case/pitch", rclcpp::SystemDefaultsQoS());
  auto pid_output_topic = node->create_publisher<std_msgs::msg::Float64>(
    "/case/pid_output", rclcpp::SystemDefaultsQoS());
  auto pwm_topic = node->create_publisher<std_msgs::msg::Float64>(
    "/case/pwm", rclcpp::SystemDefaultsQoS());

  pitch_publisher_ = std::make_unique<Float64Publisher>(pitch_topic);
  pid_output_publisher_ = std::make_unique<Float64Publisher>(pid_output_topic);
  pwm_publisher_ = std::make_unique<Float64Publisher>(pwm_topic);

  // Seeded before the subscription exists so update() can never read an empty
  // buffer. Created even for DriveMode::None: no sweep run publishes to this
  // topic, and a subscription nobody writes to costs nothing.
  velocity_command_.writeFromNonRT(geometry_msgs::msg::Twist());
  velocity_subscription_ = node->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SystemDefaultsQoS(),
    [this](const geometry_msgs::msg::Twist::SharedPtr message) {
      velocity_command_.writeFromNonRT(*message);
    });

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::InterfaceConfiguration
BalanceController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
    left_wheel_joint_ + "/" + hardware_interface::HW_IF_EFFORT,
    right_wheel_joint_ + "/" + hardware_interface::HW_IF_EFFORT};
  return config;
}

controller_interface::InterfaceConfiguration
BalanceController::state_interface_configuration() const
{
  // Chassis pitch is the free world orientation of base_link, so no joint state
  // interface can supply it. It arrives instead through the IMU sensor that
  // GazeboSimSystem registers from the <sensor> block in the URDF.
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;
  config.names = {
    left_wheel_joint_ + "/" + hardware_interface::HW_IF_POSITION,
    left_wheel_joint_ + "/" + hardware_interface::HW_IF_VELOCITY,
    right_wheel_joint_ + "/" + hardware_interface::HW_IF_POSITION,
    right_wheel_joint_ + "/" + hardware_interface::HW_IF_VELOCITY,
    imu_sensor_name_ + "/orientation.x",
    imu_sensor_name_ + "/orientation.y",
    imu_sensor_name_ + "/orientation.z",
    imu_sensor_name_ + "/orientation.w"};
  return config;
}

bool BalanceController::find_state_index(const std::string & full_name, std::size_t & index) const
{
  for (std::size_t i = 0; i < state_interfaces_.size(); ++i) {
    if (state_interfaces_[i].get_prefix_name() + "/" +
      state_interfaces_[i].get_interface_name() == full_name)
    {
      index = i;
      return true;
    }
  }
  return false;
}

bool BalanceController::find_command_index(const std::string & full_name, std::size_t & index) const
{
  for (std::size_t i = 0; i < command_interfaces_.size(); ++i) {
    if (command_interfaces_[i].get_prefix_name() + "/" +
      command_interfaces_[i].get_interface_name() == full_name)
    {
      index = i;
      return true;
    }
  }
  return false;
}

controller_interface::CallbackReturn BalanceController::on_activate(
  const rclcpp_lifecycle::State & /*previous*/)
{
  const std::string velocity = hardware_interface::HW_IF_VELOCITY;
  const std::string effort = hardware_interface::HW_IF_EFFORT;

  // Resolved once here, by name, so that update() never searches and never
  // depends on the order the controller manager happened to loan them in.
  const bool resolved =
    find_state_index(imu_sensor_name_ + "/orientation.x", orientation_index_[0]) &&
    find_state_index(imu_sensor_name_ + "/orientation.y", orientation_index_[1]) &&
    find_state_index(imu_sensor_name_ + "/orientation.z", orientation_index_[2]) &&
    find_state_index(imu_sensor_name_ + "/orientation.w", orientation_index_[3]) &&
    find_state_index(left_wheel_joint_ + "/" + velocity, left_velocity_index_) &&
    find_state_index(right_wheel_joint_ + "/" + velocity, right_velocity_index_) &&
    find_command_index(left_wheel_joint_ + "/" + effort, left_effort_index_) &&
    find_command_index(right_wheel_joint_ + "/" + effort, right_effort_index_);

  if (!resolved) {
    RCLCPP_ERROR(
      get_node()->get_logger(),
      "Missing an interface: check that the URDF declares wheel joints '%s' and '%s' and an IMU "
      "sensor named '%s'",
      left_wheel_joint_.c_str(), right_wheel_joint_.c_str(), imu_sensor_name_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  // BalanceControl::begin() calls SetMode(AUTOMATIC) before any DMP packet has
  // arrived, so PID_v1 captures input = 0 against a setpoint near 180, winds its
  // integrator to the +255 clamp, and then takes a huge negative derivative kick
  // when the first real reading lands.
  //
  // Measured 2026-09-16: that transient is FATAL to a free-standing robot. It
  // leaves the give-up band 16 ms after activation, while gravity alone takes
  // 4.3 s -- and it does so for either sign of the feedback loop. Hardware
  // survives it only because a person is holding the robot while it burns off.
  //
  // So it is reproduced only when asked for. Left on, every run dies in the
  // lurch and the sweep reports 0% for every gain set, measuring the startup
  // sequence instead of the gains. `reproduce_startup_windup` turns it back on
  // to quantify what it costs, which is a question worth asking separately.
  // reproduce_startup_windup deliberately drives from power-on, so it opts out
  // of the hold: the wind-up transient IS the thing that experiment measures.
  holding_ = !reproduce_startup_windup_;
  if (reproduce_startup_windup_) {
    pid_->initialize(0.0, 0.0);
    // The firmware's `input` reads 0 until the first packet, so the coast gate
    // holds while the integrator winds.
    std::fill(pitch_delay_buffer_.begin(), pitch_delay_buffer_.end(), 0.0);
  } else {
    // Deferred: initialised from the first real reading in update(), which is
    // what SetMode(AUTOMATIC) would capture if it ran once DMP data were
    // flowing. The buffer is filled to match, so the first cycle sees no
    // derivative step either.
    pid_initialised_ = false;
  }
  delay_write_index_ = 0;

  previous_pitch_deg_ = 0.0;
  pitch_history_depth_ = 0;

  // A command left over from a previous activation would drive the robot the
  // instant the run arms, from a state the run protocol never defined.
  velocity_command_.writeFromNonRT(geometry_msgs::msg::Twist());

  noise_rng_.seed(static_cast<std::mt19937::result_type>(noise_seed_));
  standard_normal_.reset();

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn BalanceController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous*/)
{
  // Guarded because deactivation can follow a failed activation, in which case
  // no interface was ever loaned and the cached indices point at nothing.
  if (command_interfaces_.size() > std::max(left_effort_index_, right_effort_index_)) {
    command_effort(left_effort_index_, 0.0);
    command_effort(right_effort_index_, 0.0);
  }

  // Reported here rather than from update(), which must not log. A run with a
  // non-zero count was disturbed by the harness, and its metrics describe that
  // disturbance as well as the gains.
  if (command_write_failures_ > 0 || state_read_failures_ > 0) {
    RCLCPP_WARN(
      get_node()->get_logger(),
      "interface access failed during run: %lu command writes, %lu state reads",
      static_cast<unsigned long>(command_write_failures_),
      static_cast<unsigned long>(state_read_failures_));
  }
  return controller_interface::CallbackReturn::SUCCESS;
}

bool BalanceController::command_effort(std::size_t index, double value)
{
  if (command_interfaces_[index].set_value(value)) {
    return true;
  }
  ++command_write_failures_;
  return false;
}

controller_interface::return_type BalanceController::update(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  double qx = 0.0;
  double qy = 0.0;
  double qz = 0.0;
  double qw = 1.0;
  if (!read_state(state_interfaces_[orientation_index_[0]], qx) ||
      !read_state(state_interfaces_[orientation_index_[1]], qy) ||
      !read_state(state_interfaces_[orientation_index_[2]], qz) ||
      !read_state(state_interfaces_[orientation_index_[3]], qw))
  {
    // No orientation this cycle. Hold the previous command and, crucially,
    // leave the delay buffer and pitch history untouched: advancing them with
    // an invented sample would shift the modelled sensor latency, which is the
    // parameter this simulation is most sensitive to.
    ++state_read_failures_;
    return controller_interface::return_type::OK;
  }

  // The firmware's units: degrees offset by 180 so that upright reads 180.
  // Everything downstream of here -- gate, setpoint, PID -- works in them.
  const double pitch_deg = dmp_pitch_rad(qw, qx, qy, qz) * kRadiansToDegrees + kUprightDeg;

  // Differentiated from the true pitch rather than the degraded one. The
  // centripetal model needs the chassis's actual rate, so it is differentiated
  // from the true pitch rather than the delayed noisy one -- otherwise the
  // artifact would be manufactured out of the noise it is meant to describe.
  // First derivative only: the second one was what made this term explode.
  const double dt = period.seconds();
  double pitch_rate_deg_s = 0.0;
  if (dt > 0.0 && pitch_history_depth_ >= 1) {
    pitch_rate_deg_s = (pitch_deg - previous_pitch_deg_) / dt;
  }
  previous_pitch_deg_ = pitch_deg;
  if (pitch_history_depth_ < 1) {
    ++pitch_history_depth_;
  }

  // Hold zero effort until the run's initial condition actually exists.
  //
  // The controller must be active before physics can be paused, so between
  // activation and the teleport the robot stands upright for a variable amount
  // of wall time. Driving it during that window gave it momentum that set_pose
  // does not reset -- gz.msgs.Pose carries no velocity -- so every run began
  // with a different hidden initial state. Measured 2026-09-16: identical
  // parameters produced a pass in one run and a failure in another, and the
  // violent pre-tilt states aborted the physics engine on a quarter of runs.
  //
  // Upright with zero torque is a genuine equilibrium, so the robot simply
  // stands there until the supervisor tilts it. Releasing on the tilt itself
  // needs no signalling: the only thing that moves the robot is the teleport.
  //
  // This is harness scaffolding, not firmware behaviour. The firmware starts
  // driving the moment it powers on; the simulation needs a defined starting
  // state before it can measure anything, and reproducing power-on is a
  // separate experiment (see reproduce_startup_windup).
  if (holding_) {
    if (std::abs(pitch_deg - kUprightDeg) <= hold_release_deg_) {
      command_effort(left_effort_index_, 0.0);
      command_effort(right_effort_index_, 0.0);
      publish_without_blocking(pitch_publisher_.get(), pitch_deg);
      publish_without_blocking(pwm_publisher_.get(), 0.0);
      return controller_interface::return_type::OK;
    }
    holding_ = false;
  }

  // Deferred initialisation: seed the PID and the whole delay buffer from the
  // first real reading, so neither a wound integrator nor a phantom derivative
  // step exists on the first commanded cycle.
  if (!pid_initialised_) {
    pid_->initialize(pitch_deg, 0.0);
    std::fill(pitch_delay_buffer_.begin(), pitch_delay_buffer_.end(), pitch_deg);
    previous_pitch_deg_ = pitch_deg;
    pitch_history_depth_ = 0;
    pid_initialised_ = true;
  }

  // Transport delay first: the noise rides on the delayed sample, not the other
  // way round, because the DMP's own output is what arrives late.
  pitch_delay_buffer_[delay_write_index_] = pitch_deg;
  delay_write_index_ = (delay_write_index_ + 1U) % pitch_delay_buffer_.size();
  double measured_pitch_deg = pitch_delay_buffer_[delay_write_index_];

  // Scaled by a draw from a fixed unit normal rather than a per-call
  // distribution, so a sigma of zero needs no special case.
  measured_pitch_deg += angle_noise_sigma_deg_ * standard_normal_(noise_rng_);

  // Centripetal contamination of the gravity vector the DMP fuses against.
  //
  // The IMU sits a lever arm from the rotation centre, so rotating at omega it
  // measures a centripetal acceleration omega^2 * r on top of gravity. The
  // fused "down" therefore tilts by atan(omega^2 * r / g), a bias in a known
  // direction rather than noise -- a real sensor leans its estimate the same
  // way every time it swings the same way.
  //
  // The previous model multiplied a random draw by |angular ACCELERATION|,
  // which is pitch differentiated twice: a division by dt^2 that amplified any
  // wobble ten-thousand-fold at dt = 0.01. Measured 2026-09-16, it injected
  // swings of tens of degrees between adjacent cycles and one sample of 409
  // degrees -- outside the 0..360 the convention allows -- and the PID
  // saturated on that noise and threw the robot over. It was also
  // dimensionally wrong: this error depends on angular rate, not acceleration.
  //
  // atan bounds the result by construction, so no transient can make it large.
  if (rotation_artifact_enabled_) {
    const double omega_rad_s = pitch_rate_deg_s * kDegreesToRadians;
    measured_pitch_deg += kRadiansToDegrees * std::atan2(
      omega_rad_s * omega_rad_s * imu_lever_arm_m_, kGravity);
  }

  // Zero unless something is publishing /cmd_vel, and zero in DriveMode::None
  // whatever is published, so the sweep's runs are untouched by this path.
  const geometry_msgs::msg::Twist * command = velocity_command_.readFromRT();
  const double forward_command = (command != nullptr) ? command->linear.x : 0.0;
  const double turn_command = (command != nullptr) ? command->angular.z : 0.0;

  // Lean into the direction of travel, and let the balance loop do the driving:
  // holding a tilt IS moving forward on a balancer, so the wheels are never
  // commanded forward directly and the pitch loop is never fighting a throttle
  // it did not ask for. This is the one thing the firmware's assist modes do
  // not do, and the reason this mode exists to be compared against them.
  double setpoint_deg = kUprightDeg + lean_deg_;
  if (drive_mode_ == DriveMode::LeanOffset) {
    setpoint_deg += lean_gain_deg_ * forward_command;
  }

  // Simulation time, never the wall clock: PID_v1's integral term is scaled by
  // the sample period, so timing drift would change what Ki=140 means.
  pid_->compute(measured_pitch_deg, setpoint_deg, time.seconds());

  WheelDrive drive{0, 0, true};
  switch (drive_mode_) {
    case DriveMode::None:
      drive = firmware_wheel_drive(
        MotorMode::BalanceOnly, pid_->output(), deadzone_pwm_, output_divisor_);
      break;

    case DriveMode::Firmware:
      drive = firmware_wheel_drive(
        firmware_mode_from_command(forward_command, turn_command),
        pid_->output(), deadzone_pwm_, output_divisor_);
      break;

    case DriveMode::LeanOffset:
      drive = lean_offset_wheel_drive(
        pwm_from_pid_output(pid_->output(), deadzone_pwm_, output_divisor_),
        steer_gain_pwm_ * turn_command);
      break;
  }

  double commanded_pwm = 0.0;
  // Two separate ways to end up with no applied torque, and they mean different
  // things: the gate is the firmware refusing to drive a robot that is already
  // past saving, while drive.coasting is STOPPED, an explicit stop() request.
  if (is_coasting(measured_pitch_deg, gate_lower_deg_, gate_upper_deg_) || drive.coasting) {
    // TB6612 stop is IN1=LOW, IN2=LOW -- high impedance. The wheels coast; they
    // do not brake, whatever the comment in self_balance.ino says.
    command_effort(left_effort_index_, 0.0);
    command_effort(right_effort_index_, 0.0);
  } else {
    // The sign mirrors the whole motor frame, wheel speed included. Flipping
    // only the torque would leave back-EMF adding to the applied voltage
    // instead of opposing it, turning the motor model into positive feedback.
    double left_raw = 0.0;
    double right_raw = 0.0;
    if (!read_state(state_interfaces_[left_velocity_index_], left_raw) ||
        !read_state(state_interfaces_[right_velocity_index_], right_raw))
    {
      // Back-EMF needs wheel speed. Without it the torque would be overstated
      // at exactly the speeds where balance is won or lost, so hold instead.
      ++state_read_failures_;
      return controller_interface::return_type::OK;
    }
    const double left_velocity = motor_direction_sign_ * left_raw;
    const double right_velocity = motor_direction_sign_ * right_raw;
    command_effort(
      left_effort_index_,
      motor_direction_sign_ * torque_from_pwm(drive.left_pwm, left_velocity, motor_params_));
    command_effort(
      right_effort_index_,
      motor_direction_sign_ * torque_from_pwm(drive.right_pwm, right_velocity, motor_params_));

    // Mean of the two wheels, so this topic keeps meaning what it always meant:
    // the steering term is differential and cancels here, and in every mode
    // that does not steer the wheels are equal and the mean IS their value.
    commanded_pwm = 0.5 * static_cast<double>(drive.left_pwm + drive.right_pwm);
  }

  // The degraded pitch, not the true one: it is what the gate acts on and what
  // the firmware's getAngle() returns, so a pass computed from this topic means
  // what a pass on the bench means.
  publish_without_blocking(pitch_publisher_.get(), measured_pitch_deg);
  publish_without_blocking(pid_output_publisher_.get(), pid_->output());
  publish_without_blocking(pwm_publisher_.get(), commanded_pwm);

  return controller_interface::return_type::OK;
}

}  // namespace case_sim

PLUGINLIB_EXPORT_CLASS(case_sim::BalanceController, controller_interface::ControllerInterface)
