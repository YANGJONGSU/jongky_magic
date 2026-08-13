// 야붐 Rosmaster 보드 시리얼 드라이버.
//
// 프로토콜 근거: Rosmaster_Lib 3.3.9 소스 및 실물 계측.
// 상세는 작업 노트의 "야붐보드-레퍼런스.md" 참조.
//
// 이 클래스는 ROS 를 모른다. 부호 규약도 다루지 않는다 —
// 보드가 주는 값을 그대로 주고, 주는 값을 그대로 보낸다.
// ROS 규약으로의 변환은 JongkySystemHardware 가 한다.

#ifndef JONGKY_HARDWARE__YAHBOOM_BOARD_HPP_
#define JONGKY_HARDWARE__YAHBOOM_BOARD_HPP_

#include <atomic>
#include <array>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace jongky_hardware
{

/// 보드가 주기적으로 보고하는 값들. 수신 스레드가 채우고 read() 가 복사해 간다.
struct BoardState
{
  std::array<int32_t, 4> encoder{{0, 0, 0, 0}};  ///< 4채널. ch2=오른쪽, ch3=왼쪽
  double vx{0.0};                                ///< 보드 기준 m/s
  double vz{0.0};                                ///< 보드 기준 rad/s
  double battery_v{0.0};
  std::array<double, 3> gyro{{0.0, 0.0, 0.0}};   ///< 보드 기준 rad/s
  std::array<double, 3> accel{{0.0, 0.0, 0.0}};
  std::array<double, 3> rpy{{0.0, 0.0, 0.0}};    ///< 보드가 계산한 자세
  double pid_kp{-1.0}, pid_ki{-1.0}, pid_kd{-1.0};  ///< 보드가 보고한 게인
  uint64_t pid_seq{0};                              ///< PID 응답 수신 횟수
  uint64_t encoder_seq{0};                       ///< 엔코더 프레임 수신 횟수
  /// 엔코더 프레임이 실제로 도착한 시각(steady clock, ns).
  /// 속도를 컨트롤러 주기로 나누면 25Hz 프레임이 50Hz 격자에 반올림되어
  /// ±50% 오차가 생긴다. 도착 시각을 직접 써야 한다.
  int64_t encoder_stamp_ns{0};
};

class YahboomBoard
{
public:
  YahboomBoard() = default;
  ~YahboomBoard();

  YahboomBoard(const YahboomBoard &) = delete;
  YahboomBoard & operator=(const YahboomBoard &) = delete;

  /// 포트를 열고 수신 스레드를 띄운다. 자동 보고도 켠다.
  bool open(const std::string & port, int baud, uint8_t car_type, std::string & error);

  /// 정지 명령을 보내고 스레드를 접은 뒤 포트를 닫는다.
  void close();

  bool is_open() const { return fd_ >= 0; }

  /// 마지막으로 수신한 상태의 스냅샷.
  BoardState snapshot() const;

  /// 차체 속도 명령. 보드 부호 그대로 (반전은 호출자 책임).
  bool set_motion(double vx, double vz);

  /// 4채널 PWM 직접 구동. 진단용 — 보드 PID 를 우회한다.
  bool set_motor_pwm(int8_t m1, int8_t m2, int8_t m3, int8_t m4);

  /// 보드 속도 PID 게인. 각 [0, 10]. 게인이 높으면 목표 근처에서 사냥한다.
  /// persist=true 면 플래시에 쓴다 — 느리고 수명을 깎으므로 확정 후에만.
  ///
  /// [주의] ki 를 0 으로 두면 정상상태 오차를 못 없애 목표 속도에
  /// 영영 도달하지 못한다. 반드시 읽어서 확인한 뒤 조정할 것.
  bool set_motor_pid(double kp, double ki, double kd, bool persist = false);

  /// 보드에 현재 PID 게인을 물어본다. 응답까지 최대 timeout_ms 기다린다.
  /// 실패하면 false. 건드리기 전에 반드시 원래 값을 읽어둘 것.
  bool get_motor_pid(double & kp, double & ki, double & kd, int timeout_ms = 500);

  /// 프레임을 한 번이라도 받았는지. 연결 직후 보드 생존 확인용.
  bool has_data() const { return rx_count_.load() > 0; }

  /// 통신이 살아 있는지. 최초 수신 이력이 있고 최근에도 받았는지로 판단한다.
  bool is_alive() const;

  /// 마지막 수신 이후 경과 시간(초).
  double seconds_since_last_rx() const;

private:
  static constexpr uint8_t kHead = 0xFF;
  static constexpr uint8_t kDeviceId = 0xFC;
  static constexpr uint8_t kRxDeviceId = 0xFB;  // 0xFC - 1

  // 송신 기능 코드
  static constexpr uint8_t kFuncAutoReport = 0x01;
  static constexpr uint8_t kFuncMotor = 0x10;
  static constexpr uint8_t kFuncMotion = 0x12;
  static constexpr uint8_t kFuncSetMotorPid = 0x13;
  static constexpr uint8_t kFuncRequestData = 0x50;

  // 수신 프레임 종류
  static constexpr uint8_t kRptSpeed = 0x0A;
  static constexpr uint8_t kRptImuAtt = 0x0C;
  static constexpr uint8_t kRptEncoder = 0x0D;
  static constexpr uint8_t kRptIcmRaw = 0x0E;

  bool write_frame(uint8_t func, const std::vector<uint8_t> & payload);
  void rx_loop();
  void parse_frame(uint8_t type, const std::vector<uint8_t> & data);

  int fd_{-1};
  uint8_t car_type_{1};

  std::thread rx_thread_;
  std::atomic<bool> running_{false};

  mutable std::mutex state_mutex_;
  BoardState state_;

  std::atomic<int64_t> last_rx_ns_{0};
  std::atomic<uint64_t> rx_count_{0};

  // 송신 직렬화. 보드가 명령 사이 간격을 요구하므로 뮤텍스로 묶는다.
  std::mutex tx_mutex_;
};

}  // namespace jongky_hardware

#endif  // JONGKY_HARDWARE__YAHBOOM_BOARD_HPP_
