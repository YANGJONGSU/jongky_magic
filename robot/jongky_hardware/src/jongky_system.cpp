#include "jongky_hardware/jongky_system.hpp"

#include <cmath>
#include <tuple>
#include <limits>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
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
  board_vel_scale_ = get_param(info_, "board_vel_scale", board_vel_scale_);
  // 음수면 "지정 안 함" 이고 보드의 현재 값을 그대로 쓴다.
  pid_kp_ = get_param(info_, "motor_pid_kp", -1.0);
  pid_ki_ = get_param(info_, "motor_pid_ki", -1.0);
  pid_kd_ = get_param(info_, "motor_pid_kd", -1.0);

  if (board_vel_scale_ <= 0.0) {
    RCLCPP_ERROR(rclcpp::get_logger(kLogger), "board_vel_scale 은 양수여야 함");
    return hardware_interface::CallbackReturn::ERROR;
  }
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

  if (!info_.sensors.empty()) {
    imu_name_ = info_.sensors[0].name;
  }

  RCLCPP_INFO(
    rclcpp::get_logger(kLogger),
    "초기화 완료. port=%s baud=%d car_type=%u counts_per_rev=%.1f "
    "board_vel_scale=%.3f 좌='%s' 우='%s'",
    serial_port_.c_str(), baud_rate_, car_type_, counts_per_rev_, board_vel_scale_,
    left_joint_.c_str(), right_joint_.c_str());

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

  // 건드리기 전에 보드의 원래 게인을 읽어 둔다. ki 를 0 으로 덮으면
  // 정상상태 오차를 못 없애 목표 속도에 도달하지 못한다.
  double bkp = -1.0, bki = -1.0, bkd = -1.0;
  if (board_.get_motor_pid(bkp, bki, bkd)) {
    RCLCPP_INFO(
      rclcpp::get_logger(kLogger), "보드 현재 PID kp=%.3f ki=%.3f kd=%.3f", bkp, bki, bkd);
    // URDF 에서 값을 안 준 항목만 보드 값을 따른다.
    if (pid_kp_ < 0.0) { pid_kp_ = bkp; }
    if (pid_ki_ < 0.0) { pid_ki_ = bki; }
    if (pid_kd_ < 0.0) { pid_kd_ = bkd; }
  } else {
    RCLCPP_WARN(rclcpp::get_logger(kLogger), "보드 PID 게인을 읽지 못했다");
  }

  // URDF 로 지정된 게인을 적용한다. 보드 설정은 휘발성이라
  // 전원을 껐다 켜면 공장값으로 돌아가므로 기동할 때마다 다시 쓴다.
  if (pid_kp_ >= 0.0 && pid_ki_ >= 0.0 && pid_kd_ >= 0.0 &&
      (std::abs(pid_kp_ - bkp) > 1e-6 || std::abs(pid_ki_ - bki) > 1e-6 ||
       std::abs(pid_kd_ - bkd) > 1e-6))
  {
    if (board_.set_motor_pid(pid_kp_, pid_ki_, pid_kd_, false)) {
      RCLCPP_INFO(
        rclcpp::get_logger(kLogger), "PID 적용 kp=%.3f ki=%.3f kd=%.3f",
        pid_kp_, pid_ki_, pid_kd_);
    } else {
      RCLCPP_WARN(rclcpp::get_logger(kLogger), "PID 적용 실패");
    }
  }

  // PID 게인을 ROS 파라미터로 노출한다. 음수면 보드 기본값을 건드리지 않는다.
  //   ros2 param set /controller_manager jongky.motor_pid_kp 0.6
  if (auto node = get_node()) {
    const auto declare = [&](const char * name, double & slot) {
      const std::string full = info_.name + "." + name;
      if (!node->has_parameter(full)) {
        slot = node->declare_parameter<double>(full, slot);
      }
    };
    declare("motor_pid_kp", pid_kp_);
    declare("motor_pid_ki", pid_ki_);
    declare("motor_pid_kd", pid_kd_);

    param_cb_ = node->add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & params) {
        rcl_interfaces::msg::SetParametersResult res;
        res.successful = true;
        for (const auto & p : params) {
          if (p.get_name() == info_.name + ".motor_pid_kp") { pid_kp_ = p.as_double(); }
          else if (p.get_name() == info_.name + ".motor_pid_ki") { pid_ki_ = p.as_double(); }
          else if (p.get_name() == info_.name + ".motor_pid_kd") { pid_kd_ = p.as_double(); }
          else { continue; }
        }
        if (pid_kp_ >= 0.0 && pid_ki_ >= 0.0 && pid_kd_ >= 0.0) {
          if (board_.set_motor_pid(pid_kp_, pid_ki_, pid_kd_, false)) {
            RCLCPP_INFO(
              rclcpp::get_logger(kLogger), "보드 PID 적용 kp=%.3f ki=%.3f kd=%.3f",
              pid_kp_, pid_ki_, pid_kd_);
          } else {
            res.successful = false;
            res.reason = "보드에 PID 전송 실패";
          }
        }
        return res;
      });
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

  imu_handles_.clear();
  if (!imu_name_.empty()) {
    for (const auto * n : {"orientation.x", "orientation.y", "orientation.z", "orientation.w",
                           "angular_velocity.x", "angular_velocity.y", "angular_velocity.z",
                           "linear_acceleration.x", "linear_acceleration.y",
                           "linear_acceleration.z"}) {
      imu_handles_.push_back(get_state_interface_handle(imu_name_ + "/" + n));
    }
  }

  // 엔코더는 보드 부팅 이후 누적값이다. 활성화 시점을 원점으로 잡는다.
  const auto s = board_.snapshot();
  left_zero_ = s.encoder[kLeftEncoderIdx];
  right_zero_ = s.encoder[kRightEncoderIdx];
  zero_captured_ = true;
  left_pos_ = 0.0;
  right_pos_ = 0.0;
  left_pos_at_seq_ = 0.0;
  right_pos_at_seq_ = 0.0;
  left_vel_ = 0.0;
  right_vel_ = 0.0;
  last_enc_seq_ = s.encoder_seq;
  last_enc_stamp_ns_ = s.encoder_stamp_ns;

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

  left_pos_ = counts_to_rad(s.encoder[kLeftEncoderIdx] - left_zero_);
  right_pos_ = counts_to_rad(s.encoder[kRightEncoderIdx] - right_zero_);

  // [중요] 속도는 엔코더 프레임이 갱신됐을 때만 다시 계산한다.
  // 보드 보고는 25Hz, read() 는 50Hz 다. 매 주기 차분하면 절반은 변화 0,
  // 절반은 2배가 되어 0 과 2배가 번갈아 나오는 사각파가 만들어진다.
  // 실제로 이 버그 때문에 관절 속도의 변동이 평균의 1.6배로 찍혔다.
  if (s.encoder_seq != last_enc_seq_) {
    // dt 는 프레임 도착 시각의 차이로 잰다. 컨트롤러 주기를 누적하면
    // 25Hz 프레임이 50Hz 격자에 반올림되어 속도가 ±50% 튄다.
    const double dt = static_cast<double>(s.encoder_stamp_ns - last_enc_stamp_ns_) * 1e-9;
    if (dt > 1e-4) {
      left_vel_ = (left_pos_ - left_pos_at_seq_) / dt;
      right_vel_ = (right_pos_ - right_pos_at_seq_) / dt;
    }
    left_pos_at_seq_ = left_pos_;
    right_pos_at_seq_ = right_pos_;
    last_enc_stamp_ns_ = s.encoder_stamp_ns;
    last_enc_seq_ = s.encoder_seq;
  }
  const double left_vel = left_vel_;
  const double right_vel = right_vel_;

  set_state(left_pos_handle_, left_pos_, false);
  set_state(right_pos_handle_, right_pos_, false);
  set_state(left_vel_handle_, left_vel, false);
  set_state(right_vel_handle_, right_vel, false);

  if (imu_handles_.size() == 10) {
    // [부호] 보드 자이로 z 는 시계가 양수. REP-103 은 반시계가 양수이므로
    // 뒤집는다. x·y 축 정렬은 아직 미검증이라 그대로 넘긴다.
    const double gz = -s.gyro[2];

    // 보드가 계산한 자세(roll/pitch/yaw)를 쿼터니언으로.
    // yaw 도 자이로와 같은 이유로 부호를 뒤집는다.
    const double roll = s.rpy[0], pitch = s.rpy[1], yaw = -s.rpy[2];
    const double cr = std::cos(roll * 0.5), sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5), sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5), sy = std::sin(yaw * 0.5);

    const double vals[10] = {
      sr * cp * cy - cr * sp * sy,   // orientation.x
      cr * sp * cy + sr * cp * sy,   // orientation.y
      cr * cp * sy - sr * sp * cy,   // orientation.z
      cr * cp * cy + sr * sp * sy,   // orientation.w
      s.gyro[0], s.gyro[1], gz,
      s.accel[0], s.accel[1], s.accel[2]};
    for (size_t i = 0; i < 10; ++i) {
      set_state(imu_handles_[i], vals[i], false);
    }
  }

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
  // [스케일] 보드는 자기 바퀴 크기로 vx·vz 를 각속도로 바꾼다. 그 가정이
  //          우리 바퀴보다 커서 같은 값에 느리게 돈다. 비율만큼 키워 보낸다.
  //          회전도 바퀴 속도로 만들어지므로 vz 에도 같이 적용한다.
  if (!board_.set_motion(-vx * board_vel_scale_, vz * board_vel_scale_)) {
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
