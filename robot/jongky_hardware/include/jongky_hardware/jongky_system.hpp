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
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "jongky_hardware/yahboom_board.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "sensor_msgs/msg/battery_state.hpp"
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

  // 보드는 vx·vz 를 자기 바퀴 크기로 각속도로 바꾼다. 그 가정이 우리
  // 바퀴(반지름 33.5mm)와 달라서, 같은 vx 를 줘도 실제로는 느리게 돈다.
  // 실측: cmd 0.2 m/s -> 바퀴 4.29 rad/s (원하는 값 5.97 의 0.719배).
  // 보드 가정 반지름은 약 46.6mm 로 X3 메카넘 휠 크기다.
  // 그 비율의 역수를 곱해서 보낸다.
  double board_vel_scale_{1.391};

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

  // IMU. imu_sensor_broadcaster 가 읽어 sensor_msgs/Imu 로 발행한다.
  // 인터페이스 이름은 규약 고정 (orientation.x … linear_acceleration.z).
  std::vector<hardware_interface::StateInterface::SharedPtr> imu_handles_;
  std::string imu_name_;

  // 엔코더는 부팅 이후 누적값이라 활성화 시점을 0 으로 잡는다.
  int32_t left_zero_{0};
  int32_t right_zero_{0};
  bool zero_captured_{false};

  double left_pos_{0.0};
  double right_pos_{0.0};

  // 속도는 엔코더 프레임이 실제로 갱신됐을 때만 다시 계산한다.
  // 보드는 25Hz 로 보고하는데 read() 는 50Hz 로 돌기 때문에, 매 주기
  // 차분하면 0 과 2배가 번갈아 나오는 사각파가 된다.
  uint64_t last_enc_seq_{0};
  int64_t last_enc_stamp_ns_{0};
  double left_pos_at_seq_{0.0};
  double right_pos_at_seq_{0.0};
  double left_vel_{0.0};
  double right_vel_{0.0};

  bool comms_warned_{false};

  // 보드 속도 PID 게인. ROS 파라미터로 런타임에 바꿀 수 있다.
  // 게인이 높으면 목표 근처에서 사냥해 바퀴 속도가 진동한다.
  // 진단 도구가 이 파라미터를 바꿔가며 최적점을 찾는다.
  //   ros2 param set /controller_manager jongky.motor_pid_kp 0.6
  double pid_kp_{-1.0}, pid_ki_{-1.0}, pid_kd_{-1.0};
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_;

  // ── 배터리 발행 ─────────────────────────────────────────────────────────
  // 예전에는 기동할 때 로그 한 줄이 전부였다. 그래서 2026-08-21 새벽 촬영본에
  // 전압이 한 점도 없고, 주행 이상의 원인이 전압인지 제어인지 bag 으로는
  // 갈라낼 수가 없었다 — 시간에 따른 악화 추세로 간접 추정만 했다.
  //
  // 여기서 노드를 따로 만드는 이유: SystemInterface 는 자기 노드가 없고,
  // ros2_control 에는 배터리용 broadcaster 가 없다 (imu_sensor_broadcaster 는
  // 있다). 상태 인터페이스로만 내보내면 아무도 안 읽는다.
  std::shared_ptr<rclcpp::Node> batt_node_;
  rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr batt_pub_;
  double batt_last_pub_{0.0};
  bool batt_warned_{false};
};

}  // namespace jongky_hardware

#endif  // JONGKY_HARDWARE__JONGKY_SYSTEM_HPP_
