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
#include "std_msgs/msg/float64.hpp"

// realtime_tools renamed its headers from .h to .hpp mid-distro. Probing keeps
// this file buildable on either side of that rename.
#if __has_include("realtime_tools/realtime_publisher.hpp")
#include "realtime_tools/realtime_publisher.hpp"
#else
#include "realtime_tools/realtime_publisher.h"
#endif

#include "motor_model.hpp"
#include "pid_v1.hpp"

namespace case_sim
{
namespace
{

constexpr double kRadiansToDegrees = 180.0 / M_PI;
constexpr double kUprightDeg = 180.0;

using Float64Publisher = realtime_tools::RealtimePublisher<std_msgs::msg::Float64>;

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
  double rotation_artifact_gain_deg_s2_{0.0};
  int noise_seed_{0};
  double motor_direction_sign_{1.0};

  MotorParams motor_params_{};
  std::optional<PidV1> pid_;

  std::vector<double> pitch_delay_buffer_;
  std::size_t delay_write_index_{0};

  double previous_pitch_deg_{0.0};
  double previous_pitch_rate_deg_s_{0.0};
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
  std::uint64_t command_write_failures_{0};
  std::uint64_t state_read_failures_{0};

  std::unique_ptr<Float64Publisher> pitch_publisher_;
  std::unique_ptr<Float64Publisher> pid_output_publisher_;
  std::unique_ptr<Float64Publisher> pwm_publisher_;
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
    declare_from_sweep_ranges("rotation_artifact_gain_deg_s2", rclcpp::PARAMETER_DOUBLE);

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
    rotation_artifact_gain_deg_s2_ =
      node->get_parameter("rotation_artifact_gain_deg_s2").as_double();

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
  if (angle_noise_sigma_deg_ < 0.0 || rotation_artifact_gain_deg_s2_ < 0.0) {
    RCLCPP_ERROR(node->get_logger(), "noise sigma and artifact gain must not be negative");
    return controller_interface::CallbackReturn::ERROR;
  }
  if (motor_params_.resistance_ohm <= 0.0) {
    RCLCPP_ERROR(node->get_logger(), "resistance_ohm must be positive");
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

  // Deliberate reproduction of a probable firmware defect, not intended
  // behaviour. BalanceControl::begin() calls SetMode(AUTOMATIC) before any DMP
  // packet has arrived, so PID_v1 captures input = 0 against a setpoint near
  // 180 and winds its integrator to the +255 clamp before the first real
  // reading, which then lands as a large negative derivative kick. Initialising
  // after the first reading would remove the transient; the simulation exists to
  // measure the firmware as written, so it is reproduced rather than fixed.
  pid_->initialize(0.0, 0.0);

  // Zero-filled for the same reason: the firmware's `input` reads 0 until the
  // first packet, so the coast gate holds and the integrator keeps winding.
  // Hardware's pre-packet window is of unknown length; here it is delay_samples
  // updates long.
  std::fill(pitch_delay_buffer_.begin(), pitch_delay_buffer_.end(), 0.0);
  delay_write_index_ = 0;

  previous_pitch_deg_ = 0.0;
  previous_pitch_rate_deg_s_ = 0.0;
  pitch_history_depth_ = 0;

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
  // rotation artifact models contamination driven by the chassis's actual
  // motion; differentiating the noisy delayed signal twice instead would
  // manufacture an artifact out of the noise it is supposed to describe.
  const double dt = period.seconds();
  double pitch_rate_deg_s = 0.0;
  double pitch_accel_deg_s2 = 0.0;
  if (dt > 0.0) {
    if (pitch_history_depth_ >= 1) {
      pitch_rate_deg_s = (pitch_deg - previous_pitch_deg_) / dt;
    }
    if (pitch_history_depth_ >= 2) {
      pitch_accel_deg_s2 = (pitch_rate_deg_s - previous_pitch_rate_deg_s_) / dt;
    }
  }
  previous_pitch_deg_ = pitch_deg;
  previous_pitch_rate_deg_s_ = pitch_rate_deg_s;
  if (pitch_history_depth_ < 2) {
    ++pitch_history_depth_;
  }

  // Transport delay first: the noise rides on the delayed sample, not the other
  // way round, because the DMP's own output is what arrives late.
  pitch_delay_buffer_[delay_write_index_] = pitch_deg;
  delay_write_index_ = (delay_write_index_ + 1U) % pitch_delay_buffer_.size();
  double measured_pitch_deg = pitch_delay_buffer_[delay_write_index_];

  // Scaled by a draw from a fixed unit normal rather than a per-call
  // distribution, so a sigma of zero needs no special case.
  measured_pitch_deg += angle_noise_sigma_deg_ * standard_normal_(noise_rng_);

  // ponytail: crude stand-in for real accelerometer contamination -- the IMU
  // sits off the centre of mass, so fast rotation corrupts the gravity vector
  // the DMP fuses against. Upgrade to a modelled gravity-vector error if the
  // sweep turns out sensitive to it.
  if (rotation_artifact_enabled_) {
    measured_pitch_deg +=
      rotation_artifact_gain_deg_s2_ * std::abs(pitch_accel_deg_s2) * standard_normal_(noise_rng_);
  }

  const double setpoint_deg = kUprightDeg + lean_deg_;

  // Simulation time, never the wall clock: PID_v1's integral term is scaled by
  // the sample period, so timing drift would change what Ki=140 means.
  pid_->compute(measured_pitch_deg, setpoint_deg, time.seconds());

  int pwm = 0;
  if (is_coasting(measured_pitch_deg, gate_lower_deg_, gate_upper_deg_)) {
    // TB6612 stop is IN1=LOW, IN2=LOW -- high impedance. The wheels coast; they
    // do not brake, whatever the comment in self_balance.ino says.
    command_effort(left_effort_index_, 0.0);
    command_effort(right_effort_index_, 0.0);
  } else {
    pwm = pwm_from_pid_output(pid_->output(), deadzone_pwm_);

    // Both wheels take the same command: BALANCE_ONLY is the only mode
    // simulated, and it never steers.
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
      left_effort_index_, motor_direction_sign_ * torque_from_pwm(pwm, left_velocity, motor_params_));
    command_effort(
      right_effort_index_,
      motor_direction_sign_ * torque_from_pwm(pwm, right_velocity, motor_params_));
  }

  // The degraded pitch, not the true one: it is what the gate acts on and what
  // the firmware's getAngle() returns, so a pass computed from this topic means
  // what a pass on the bench means.
  publish_without_blocking(pitch_publisher_.get(), measured_pitch_deg);
  publish_without_blocking(pid_output_publisher_.get(), pid_->output());
  publish_without_blocking(pwm_publisher_.get(), static_cast<double>(pwm));

  return controller_interface::return_type::OK;
}

}  // namespace case_sim

PLUGINLIB_EXPORT_CLASS(case_sim::BalanceController, controller_interface::ControllerInterface)
