#include "jongky_hardware/yahboom_board.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <cerrno>
#include <cstring>
#include <thread>

namespace jongky_hardware
{

namespace
{

int64_t now_ns()
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(
           std::chrono::steady_clock::now().time_since_epoch())
    .count();
}

speed_t baud_constant(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    default: return B0;
  }
}

/// 리틀엔디언 int16 을 부호 있는 값으로.
int16_t le_i16(const std::vector<uint8_t> & d, size_t i)
{
  return static_cast<int16_t>(static_cast<uint16_t>(d[i]) | (static_cast<uint16_t>(d[i + 1]) << 8));
}

int32_t le_i32(const std::vector<uint8_t> & d, size_t i)
{
  return static_cast<int32_t>(
    static_cast<uint32_t>(d[i]) | (static_cast<uint32_t>(d[i + 1]) << 8) |
    (static_cast<uint32_t>(d[i + 2]) << 16) | (static_cast<uint32_t>(d[i + 3]) << 24));
}

}  // namespace

YahboomBoard::~YahboomBoard() { close(); }

bool YahboomBoard::open(
  const std::string & port, int baud, uint8_t car_type, std::string & error)
{
  const speed_t speed = baud_constant(baud);
  if (speed == B0) {
    error = "지원하지 않는 baud rate: " + std::to_string(baud);
    return false;
  }

  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    error = "포트를 열 수 없음 " + port + ": " + std::strerror(errno);
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    error = std::string("tcgetattr 실패: ") + std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  cfmakeraw(&tty);
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;   // 1 stop bit
  tty.c_cflag &= ~PARENB;   // no parity
  tty.c_cflag &= ~CRTSCTS;  // no flow control
  // 100ms 타임아웃으로 블로킹 읽기. 종료 시 스레드가 빨리 빠져나오게 한다.
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;

  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    error = std::string("tcsetattr 실패: ") + std::strerror(errno);
    ::close(fd_);
    fd_ = -1;
    return false;
  }

  tcflush(fd_, TCIOFLUSH);
  car_type_ = car_type;

  running_ = true;
  rx_count_ = 0;
  last_rx_ns_ = now_ns();
  rx_thread_ = std::thread(&YahboomBoard::rx_loop, this);

  // 자동 보고 켜기. state2=0 이면 임시 적용(플래시에 안 씀).
  if (!write_frame(kFuncAutoReport, {1, 0})) {
    error = "자동 보고 활성화 실패";
    close();
    return false;
  }

  return true;
}

void YahboomBoard::close()
{
  if (fd_ >= 0) {
    // 나가기 전에 반드시 세운다. 여러 번 보내는 건 패킷 유실 대비.
    for (int i = 0; i < 3; ++i) {
      set_motion(0.0, 0.0);
    }
  }

  running_ = false;
  if (rx_thread_.joinable()) {
    rx_thread_.join();
  }

  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool YahboomBoard::write_frame(uint8_t func, const std::vector<uint8_t> & payload)
{
  if (fd_ < 0) {
    return false;
  }

  // 송신 프레임: HEAD, DEVICE_ID, LEN, FUNC, payload..., CHECKSUM
  //   LEN      = 체크섬 붙이기 전 길이 - 1
  //   CHECKSUM = (전체 합 + (257 - DEVICE_ID)) & 0xFF
  std::vector<uint8_t> frame;
  frame.reserve(payload.size() + 5);
  frame.push_back(kHead);
  frame.push_back(kDeviceId);
  frame.push_back(0);  // LEN 자리, 아래에서 채움
  frame.push_back(func);
  frame.insert(frame.end(), payload.begin(), payload.end());
  frame[2] = static_cast<uint8_t>(frame.size() - 1);

  uint32_t sum = 257u - kDeviceId;
  for (uint8_t b : frame) {
    sum += b;
  }
  frame.push_back(static_cast<uint8_t>(sum & 0xFF));

  std::lock_guard<std::mutex> lock(tx_mutex_);
  const ssize_t written = ::write(fd_, frame.data(), frame.size());
  return written == static_cast<ssize_t>(frame.size());
}

bool YahboomBoard::set_motion(double vx, double vz)
{
  const auto to_le = [](double v, std::vector<uint8_t> & out) {
    const int16_t raw = static_cast<int16_t>(v * 1000.0);
    out.push_back(static_cast<uint8_t>(raw & 0xFF));
    out.push_back(static_cast<uint8_t>((raw >> 8) & 0xFF));
  };

  std::vector<uint8_t> payload;
  payload.push_back(car_type_);
  to_le(vx, payload);
  to_le(0.0, payload);  // vy — 2륜 차동구동이라 항상 0
  to_le(vz, payload);
  return write_frame(kFuncMotion, payload);
}

bool YahboomBoard::set_motor_pwm(int8_t m1, int8_t m2, int8_t m3, int8_t m4)
{
  return write_frame(
    kFuncMotor, {static_cast<uint8_t>(m1), static_cast<uint8_t>(m2),
                 static_cast<uint8_t>(m3), static_cast<uint8_t>(m4)});
}

bool YahboomBoard::set_motor_pid(double kp, double ki, double kd, bool persist)
{
  const auto clamp = [](double v) { return v < 0.0 ? 0.0 : (v > 10.0 ? 10.0 : v); };
  const auto put = [](double v, std::vector<uint8_t> & out) {
    // 보드는 게인을 1000배 정수로 받는다 (Rosmaster_Lib 기준).
    const uint16_t raw = static_cast<uint16_t>(v * 1000.0);
    out.push_back(static_cast<uint8_t>(raw & 0xFF));
    out.push_back(static_cast<uint8_t>((raw >> 8) & 0xFF));
  };
  std::vector<uint8_t> payload;
  put(clamp(kp), payload);
  put(clamp(ki), payload);
  put(clamp(kd), payload);
  payload.push_back(persist ? 0x5F : 0x00);
  return write_frame(kFuncSetMotorPid, payload);
}

bool YahboomBoard::get_motor_pid(double & kp, double & ki, double & kd, int timeout_ms)
{
  uint64_t before;
  {
    std::lock_guard<std::mutex> lock(state_mutex_);
    before = state_.pid_seq;
  }
  // FUNC_REQUEST_DATA 에 요청할 기능 코드와 파라미터(1)를 실어 보낸다.
  if (!write_frame(kFuncRequestData, {kFuncSetMotorPid, 1})) {
    return false;
  }
  for (int i = 0; i < timeout_ms; ++i) {
    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (state_.pid_seq != before) {
        kp = state_.pid_kp;
        ki = state_.pid_ki;
        kd = state_.pid_kd;
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
  return false;
}

BoardState YahboomBoard::snapshot() const
{
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

bool YahboomBoard::is_alive() const
{
  // 프레임을 한 번도 못 받았으면 살아 있다고 보지 않는다.
  // last_rx_ns_ 만 보면 open 시각 때문에 열자마자 참이 된다.
  return has_data() && seconds_since_last_rx() < 1.0;
}

double YahboomBoard::seconds_since_last_rx() const
{
  return static_cast<double>(now_ns() - last_rx_ns_.load()) * 1e-9;
}

void YahboomBoard::rx_loop()
{
  // 수신 프레임: HEAD, 0xFB, LEN, TYPE, data..., CHECKSUM
  //   CHECKSUM = (LEN + TYPE + data 합) % 256
  // 송신과 device id 도 체크섬 계산법도 다르다.
  const auto read_byte = [this](uint8_t & out) -> bool {
    while (running_) {
      const ssize_t n = ::read(fd_, &out, 1);
      if (n == 1) {
        return true;
      }
      if (n < 0 && errno != EAGAIN && errno != EINTR) {
        return false;
      }
      // n == 0 이면 VTIME 타임아웃. running_ 을 다시 확인하고 계속.
    }
    return false;
  };

  while (running_) {
    uint8_t b = 0;
    if (!read_byte(b) || b != kHead) {
      continue;
    }
    if (!read_byte(b) || b != kRxDeviceId) {
      continue;
    }

    uint8_t len = 0;
    uint8_t type = 0;
    if (!read_byte(len) || !read_byte(type)) {
      continue;
    }
    if (len < 2) {
      continue;
    }

    const size_t body_len = static_cast<size_t>(len) - 2;
    std::vector<uint8_t> body;
    body.reserve(body_len);
    bool ok = true;
    for (size_t i = 0; i < body_len; ++i) {
      if (!read_byte(b)) {
        ok = false;
        break;
      }
      body.push_back(b);
    }
    if (!ok || body.empty()) {
      continue;
    }

    const uint8_t rx_checksum = body.back();
    body.pop_back();

    uint32_t sum = static_cast<uint32_t>(len) + type;
    for (uint8_t v : body) {
      sum += v;
    }
    if ((sum % 256u) != rx_checksum) {
      continue;  // 체크섬 불일치 프레임은 조용히 버린다
    }

    last_rx_ns_ = now_ns();
    ++rx_count_;
    parse_frame(type, body);
  }
}

void YahboomBoard::parse_frame(uint8_t type, const std::vector<uint8_t> & d)
{
  std::lock_guard<std::mutex> lock(state_mutex_);

  switch (type) {
    case kRptSpeed:
      if (d.size() >= 7) {
        state_.vx = le_i16(d, 0) / 1000.0;
        state_.vz = le_i16(d, 4) / 1000.0;
        state_.battery_v = d[6] / 10.0;
      }
      break;

    case kRptEncoder:
      if (d.size() >= 16) {
        for (size_t i = 0; i < 4; ++i) {
          state_.encoder[i] = le_i32(d, i * 4);
        }
        ++state_.encoder_seq;
        state_.encoder_stamp_ns = now_ns();
      }
      break;

    case kRptImuAtt:
      if (d.size() >= 6) {
        for (size_t i = 0; i < 3; ++i) {
          state_.rpy[i] = le_i16(d, i * 2) / 10000.0;
        }
      }
      break;

    case kRptIcmRaw:
      // ICM20948. 자이로·가속도·지자기 전부 스케일 1/1000.
      // MPU9250(0x0B) 경로는 gy·gz 에 부호 반전이 들어가지만
      // 이 보드는 ICM 만 보고하므로 반전 없음.
      if (d.size() >= 12) {
        for (size_t i = 0; i < 3; ++i) {
          state_.gyro[i] = le_i16(d, i * 2) / 1000.0;
          state_.accel[i] = le_i16(d, 6 + i * 2) / 1000.0;
        }
      }
      break;

    case kFuncSetMotorPid:
      // 응답: [pid_index(1)] [kp(2)] [ki(2)] [kd(2)], 각 게인은 1000배 정수
      if (d.size() >= 7) {
        state_.pid_kp = le_i16(d, 1) / 1000.0;
        state_.pid_ki = le_i16(d, 3) / 1000.0;
        state_.pid_kd = le_i16(d, 5) / 1000.0;
        ++state_.pid_seq;
      }
      break;

    default:
      break;
  }
}

}  // namespace jongky_hardware
