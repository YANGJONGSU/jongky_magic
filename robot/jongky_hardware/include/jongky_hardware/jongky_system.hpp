// 종키 AMR 의 ros2_control SystemInterface.
//
// diff_drive_controller 가 주는 좌우 바퀴 각속도를 야붐 보드가 이해하는
// 차체 속도로 바꿔 보내고, 엔코더를 관절 위치·속도로 되돌린다.
//
// [부호] 야붐 보드는 ROS 규약과 반대다. 반전은 전부 이 파일 안에서 처리한다.
// 근거는 작업 노트 "야붐보드-레퍼런스.md" 4절.
//
//   보드 +vx        = 물리적 후진   -> 보낼 때 반전
//   보드 +vz        = 반시계        -> 그대로
//   엔코더 증가(+)  = 후진          -> 읽을 때 반전
//   자이로 z(+)     = 시계          -> 읽을 때 반전
//   엔코더 ch2      = 오른쪽 바퀴
//   엔코더 ch3      = 왼쪽 바퀴

#ifndef JONGKY_HARDWARE__JONGKY_SYSTEM_HPP_
#define JONGKY_HARDWARE__JONGKY_SYSTEM_HPP_

#include <memory>
#include <string>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "jongky_hardware/yahboom_board.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace jongky_hardware
{

class JongkySystemHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(JongkySystemHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  /// 엔코더 카운트를 바퀴 회전각(rad)으로. 부호 반전 포함.
  double counts_to_rad(int32_t counts) const;

  // URDF <hardware><param> 에서 읽는 값들
  std::string serial_port_{"/dev/yahboom"};
  int baud_rate_{115200};
  uint8_t car_type_{1};
  double counts_per_rev_{3182.0};
  double wheel_radius_{0.0335};
  double wheel_separation_{0.11625};

  // 관절 이름. URDF 의 <joint> 순서로 채운다.
  std::string left_joint_;
  std::string right_joint_;

  YahboomBoard board_;

  // 인터페이스 핸들을 미리 잡아 둔다.
  // 이름으로 접근하는 set_state(name, v) / get_command(name) 은
  // wait_until_set=true 로 동작해 실시간 안전하지 않다 (헤더에 명시).
  // 50Hz 루프에서 쓰면 컨트롤러 매니저가 통째로 멈춘다.
  hardware_interface::StateInterface::SharedPtr left_pos_handle_;
  hardware_interface::StateInterface::SharedPtr right_pos_handle_;
  hardware_interface::StateInterface::SharedPtr left_vel_handle_;
  hardware_interface::StateInterface::SharedPtr right_vel_handle_;
  hardware_interface::CommandInterface::SharedPtr left_cmd_handle_;
  hardware_interface::CommandInterface::SharedPtr right_cmd_handle_;

  // 엔코더는 부팅 이후 누적값이라 활성화 시점을 0 으로 잡는다.
  int32_t left_zero_{0};
  int32_t right_zero_{0};
  bool zero_captured_{false};

  double left_pos_{0.0};
  double right_pos_{0.0};

  bool comms_warned_{false};
};

}  // namespace jongky_hardware

#endif  // JONGKY_HARDWARE__JONGKY_SYSTEM_HPP_
