#include "jongky_hardware/jongky_system.hpp"

#include <cmath>
#include <tuple>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace jongky_hardware
{

namespace
{
constexpr const char * kLogger = "JongkySystemHardware";

/// 엔코더 채널 인덱스. 실측 확정값 — 야붐보드-레퍼런스.md 4절 참조.
constexpr size_t kRightEncoderIdx = 1;  // ch2
constexpr size_t kLeftEncoderIdx = 2;   // ch3

double get_param(
  const hardware_interface::HardwareInfo & info, const std::string & key, double fallback)
{
  const auto it = info.hardware_parameters.find(key);
  if (it == info.hardware_parameters.end()) {
    return fallback;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    RCLCPP_WARN(
      rclcpp::get_logger(kLogger), "파라미터 '%s' 를 숫자로 못 읽음 ('%s'). 기본값 %f 사용",
      key.c_str(), it->second.c_str(), fallback);
    return fallback;
  }
}

std::string get_param_str(
  const hardware_interface::HardwareInfo & info, const std::string & key,
  const std::string & fallback)
{
  const auto it = info.hardware_parameters.find(key);
  return (it == info.hardware_parameters.end()) ? fallback : it->second;
}

}  // namespace

hardware_interface::CallbackReturn JongkySystemHardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  serial_port_ = get_param_str(info_, "serial_port", serial_port_);
  baud_rate_ = static_cast<int>(get_param(info_, "baud_rate", baud_rate_));
  car_type_ = static_cast<uint8_t>(get_param(info_, "car_type", car_type_));
  counts_per_rev_ = get_param(info_, "counts_per_rev", counts_per_rev_);
  wheel_radius_ = get_param(info_, "wheel_radius", wheel_radius_);
  wheel_separation_ = get_param(info_, "wheel_separation", wheel_separation_);

  if (counts_per_rev_ <= 0.0 || wheel_radius_ <= 0.0 || wheel_separation_ <= 0.0) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogger),
      "counts_per_rev / wheel_radius / wheel_separation 은 모두 양수여야 함");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 관절은 정확히 둘이어야 한다. 이름은 URDF 순서대로 왼쪽·오른쪽으로 본다.
  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogger), "관절이 %zu 개다. 2개(좌·우 구동륜)여야 함",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }
  left_joint_ = info_.joints[0].name;
  right_joint_ = info_.joints[1].name;

  for (const auto & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1 ||
        joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLogger), "관절 '%s' 의 명령 인터페이스는 velocity 하나여야 함",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    bool has_pos = false;
    bool has_vel = false;
    for (const auto & si : joint.state_interfaces) {
      has_pos |= (si.name == hardware_interface::HW_IF_POSITION);
      has_vel |= (si.name == hardware_interface::HW_IF_VELOCITY);
    }
    if (!has_pos || !has_vel) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLogger), "관절 '%s' 는 position·velocity 상태가 모두 필요함",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLogger),
    "초기화 완료. port=%s baud=%d car_type=%u counts_per_rev=%.1f 좌='%s' 우='%s'",
    serial_port_.c_str(), baud_rate_, car_type_, counts_per_rev_, left_joint_.c_str(),
    right_joint_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JongkySystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  std::string error;
  if (!board_.open(serial_port_, baud_rate_, car_type_, error)) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "보드 연결 실패: %s", error.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 자동 보고가 흐르기 시작할 때까지 잠깐 기다린다. 보드는 약 25Hz 로 보고한다.
  for (int i = 0; i < 50 && !board_.has_data(); ++i) {
    rclcpp::sleep_for(std::chrono::milliseconds(20));
  }
  if (!board_.has_data()) {
    RCLCPP_ERROR(
      rclcpp::get_logger(kLogger), "포트는 열렸으나 보드에서 프레임이 오지 않음 (%s)",
      serial_port_.c_str());
    board_.close();
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto s = board_.snapshot();
  RCLCPP_INFO(
    rclcpp::get_logger(kLogger), "보드 연결됨. 배터리 %.1fV", s.battery_v);
  if (s.battery_v > 0.0 && s.battery_v < 10.5) {
    RCLCPP_WARN(
      rclcpp::get_logger(kLogger),
      "배터리 전압이 낮다 (%.1fV). 3S 기준 10.5V 아래는 충전 권장", s.battery_v);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JongkySystemHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // 인터페이스 핸들을 여기서 한 번만 잡는다. read()/write() 안에서
  // 이름으로 찾으면 매 주기 블로킹 호출이 되어 루프가 멈춘다.
  left_pos_handle_ = get_state_interface_handle(left_joint_ + "/position");
  right_pos_handle_ = get_state_interface_handle(right_joint_ + "/position");
  left_vel_handle_ = get_state_interface_handle(left_joint_ + "/velocity");
  right_vel_handle_ = get_state_interface_handle(right_joint_ + "/velocity");
  left_cmd_handle_ = get_command_interface_handle(left_joint_ + "/velocity");
  right_cmd_handle_ = get_command_interface_handle(right_joint_ + "/velocity");

  // 엔코더는 보드 부팅 이후 누적값이다. 활성화 시점을 원점으로 잡는다.
  const auto s = board_.snapshot();
  left_zero_ = s.encoder[kLeftEncoderIdx];
  right_zero_ = s.encoder[kRightEncoderIdx];
  zero_captured_ = true;
  left_pos_ = 0.0;
  right_pos_ = 0.0;

  set_state(left_pos_handle_, 0.0, false);
  set_state(right_pos_handle_, 0.0, false);
  set_state(left_vel_handle_, 0.0, false);
  set_state(right_vel_handle_, 0.0, false);
  set_command(left_cmd_handle_, 0.0, false);
  set_command(right_cmd_handle_, 0.0, false);

  board_.set_motion(0.0, 0.0);

  RCLCPP_INFO(
    rclcpp::get_logger(kLogger), "활성화. 엔코더 원점 좌=%d 우=%d", left_zero_, right_zero_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JongkySystemHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  board_.set_motion(0.0, 0.0);
  RCLCPP_INFO(rclcpp::get_logger(kLogger), "비활성화. 정지 명령 전송");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn JongkySystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  board_.close();
  zero_captured_ = false;
  return hardware_interface::CallbackReturn::SUCCESS;
}

double JongkySystemHardware::counts_to_rad(int32_t counts) const
{
  // [부호] 엔코더 증가는 물리적 후진이다. ROS 관절값은 전진이 양수이므로 뒤집는다.
  return -static_cast<double>(counts) / counts_per_rev_ * 2.0 * M_PI;
}

hardware_interface::return_type JongkySystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  if (!board_.is_alive()) {
    if (!comms_warned_) {
      RCLCPP_ERROR(
        rclcpp::get_logger(kLogger), "보드 통신 두절 (%.1f초). 마지막 상태를 유지한다",
        board_.seconds_since_last_rx());
      comms_warned_ = true;
    }
    return hardware_interface::return_type::ERROR;
  }
  comms_warned_ = false;

  const auto s = board_.snapshot();
  if (!zero_captured_) {
    return hardware_interface::return_type::OK;
  }

  const double prev_left = left_pos_;
  const double prev_right = right_pos_;

  left_pos_ = counts_to_rad(s.encoder[kLeftEncoderIdx] - left_zero_);
  right_pos_ = counts_to_rad(s.encoder[kRightEncoderIdx] - right_zero_);

  // 속도는 위치 차분으로 만든다. 보드가 주는 vx 는 차체 속도라
  // 좌우로 나누려면 어차피 트레드를 다시 써야 하므로 이쪽이 직접적이다.
  const double dt = period.seconds();
  double left_vel = 0.0;
  double right_vel = 0.0;
  if (dt > 1e-6) {
    left_vel = (left_pos_ - prev_left) / dt;
    right_vel = (right_pos_ - prev_right) / dt;
  }

  set_state(left_pos_handle_, left_pos_, false);
  set_state(right_pos_handle_, right_pos_, false);
  set_state(left_vel_handle_, left_vel, false);
  set_state(right_vel_handle_, right_vel, false);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type JongkySystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double wl = 0.0;  // rad/s
  double wr = 0.0;
  // 논블로킹으로 읽는다. 아직 값이 안 들어왔으면 0 을 유지한다.
  std::ignore = get_command<double>(left_cmd_handle_, wl, false);
  std::ignore = get_command<double>(right_cmd_handle_, wr, false);

  if (!std::isfinite(wl) || !std::isfinite(wr)) {
    board_.set_motion(0.0, 0.0);
    return hardware_interface::return_type::OK;
  }

  // 바퀴 각속도 -> 차체 속도. diff_drive_controller 가 쪼갠 것을 되돌리는 셈인데,
  // 2륜 차동구동에서는 정보 손실이 없다 (설계 결정 A안).
  const double vl = wl * wheel_radius_;
  const double vr = wr * wheel_radius_;
  const double vx = (vr + vl) / 2.0;
  const double vz = (vr - vl) / wheel_separation_;

  // [부호] 보드의 +vx 는 물리적 후진이므로 뒤집어 보낸다.
  //        +vz 는 반시계로 REP-103 과 일치하므로 그대로 보낸다.
  if (!board_.set_motion(-vx, vz)) {
    RCLCPP_WARN_THROTTLE(
      rclcpp::get_logger(kLogger), *rclcpp::Clock::make_shared(), 1000,
      "모션 명령 전송 실패");
    return hardware_interface::return_type::ERROR;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace jongky_hardware

PLUGINLIB_EXPORT_CLASS(
  jongky_hardware::JongkySystemHardware, hardware_interface::SystemInterface)
