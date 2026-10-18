#include "st3215_hardware/sts_bus.hpp"

#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

#include <chrono>

namespace st3215_hardware
{

namespace
{
constexpr uint8_t BROADCAST_ID = 0xFE;
constexpr uint8_t INST_SYNC_READ = 0x82;
constexpr uint8_t INST_SYNC_WRITE = 0x83;

speed_t to_speed(int baud)
{
  switch (baud) {
    case 1000000: return B1000000;
    case 500000: return B500000;
    case 115200: return B115200;
    default: return B1000000;
  }
}

int decode_signed(uint16_t raw)
{
  return (raw & 0x8000) ? -static_cast<int>(raw & 0x7FFF) : static_cast<int>(raw);
}
}

StsBus::~StsBus() {close();}

bool StsBus::open(const std::string & port, int baud, bool echo)
{
  close();
  echo_ = echo;
  fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY);
  if (fd_ < 0) {
    return false;
  }
  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    close();
    return false;
  }
  cfmakeraw(&tty);
  cfsetispeed(&tty, to_speed(baud));
  cfsetospeed(&tty, to_speed(baud));
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);
  port_ = port;
  return true;
}

void StsBus::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool StsBus::write_packet(uint8_t id, uint8_t instr, const std::vector<uint8_t> & params)
{
  std::vector<uint8_t> pkt;
  pkt.reserve(params.size() + 6);
  const uint8_t len = static_cast<uint8_t>(params.size() + 2);
  pkt.insert(pkt.end(), {0xFF, 0xFF, id, len, instr});
  uint32_t sum = id + len + instr;
  for (uint8_t p : params) {
    pkt.push_back(p);
    sum += p;
  }
  pkt.push_back(static_cast<uint8_t>(~sum & 0xFF));

  tcflush(fd_, TCIFLUSH);
  size_t written = 0;
  while (written < pkt.size()) {
    ssize_t n = ::write(fd_, pkt.data() + written, pkt.size() - written);
    if (n <= 0) {
      return false;
    }
    written += static_cast<size_t>(n);
  }
  pending_.clear();
  if (echo_) {
    drop_echo(pkt);
  }
  return true;
}

void StsBus::drop_echo(const std::vector<uint8_t> & pkt)
{
  std::vector<uint8_t> got(pkt.size());
  size_t n = 0;
  while (n < got.size() && read_exact(&got[n], 1, 3)) {
    ++n;
  }
  got.resize(n);
  for (size_t skip : {size_t{0}, size_t{1}}) {
    const size_t len = pkt.size() - skip;
    if (got.size() >= len && std::equal(pkt.begin() + skip, pkt.end(), got.begin())) {
      pending_.assign(got.begin() + len, got.end());
      return;
    }
  }
  pending_ = got;
}

bool StsBus::read_exact(uint8_t * buf, size_t n, int timeout_ms)
{
  using clock = std::chrono::steady_clock;
  const auto deadline = clock::now() + std::chrono::milliseconds(timeout_ms);
  size_t got = 0;
  while (got < n && !pending_.empty()) {
    buf[got++] = pending_.front();
    pending_.erase(pending_.begin());
  }
  while (got < n) {
    const int remaining = static_cast<int>(
      std::chrono::duration_cast<std::chrono::milliseconds>(deadline - clock::now()).count());
    if (remaining <= 0) {
      return false;
    }
    pollfd pfd{fd_, POLLIN, 0};
    if (poll(&pfd, 1, remaining) <= 0) {
      return false;
    }
    ssize_t r = ::read(fd_, buf + got, n - got);
    if (r < 0) {
      return false;
    }
    got += static_cast<size_t>(r);
  }
  return true;
}

bool StsBus::read_status(uint8_t & id, std::vector<uint8_t> & data, int timeout_ms)
{
  uint8_t b = 0;
  int ff = 0;
  for (int i = 0; i < 64 && ff < 2; ++i) {
    if (!read_exact(&b, 1, timeout_ms)) {
      return false;
    }
    ff = (b == 0xFF) ? ff + 1 : 0;
  }
  if (ff < 2) {
    return false;
  }
  uint8_t hdr[2];
  if (!read_exact(hdr, 2, timeout_ms)) {
    return false;
  }
  if (hdr[0] == 0xFF) {
    hdr[0] = hdr[1];
    if (!read_exact(&hdr[1], 1, timeout_ms)) {
      return false;
    }
  }
  const uint8_t len = hdr[1];
  if (len < 2) {
    return false;
  }
  std::vector<uint8_t> rest(len);
  if (!read_exact(rest.data(), len, timeout_ms)) {
    return false;
  }
  uint32_t sum = hdr[0] + len;
  for (size_t i = 0; i + 1 < rest.size(); ++i) {
    sum += rest[i];
  }
  if (static_cast<uint8_t>(~sum & 0xFF) != rest.back()) {
    return false;
  }
  id = hdr[0];
  data.assign(rest.begin() + 1, rest.end() - 1);
  return true;
}

bool StsBus::sync_write_position(
  const std::vector<uint8_t> & ids, const std::vector<int> & ticks, uint8_t acc, uint16_t speed)
{
  constexpr uint8_t data_len = 7;
  std::vector<uint8_t> params{reg::ACC, data_len};
  for (size_t i = 0; i < ids.size(); ++i) {
    const uint16_t p = static_cast<uint16_t>(ticks[i]);
    params.insert(
      params.end(),
      {ids[i], acc, static_cast<uint8_t>(p & 0xFF), static_cast<uint8_t>(p >> 8), 0, 0,
        static_cast<uint8_t>(speed & 0xFF), static_cast<uint8_t>(speed >> 8)});
  }
  return write_packet(BROADCAST_ID, INST_SYNC_WRITE, params);
}

bool StsBus::sync_write_byte(const std::vector<uint8_t> & ids, uint8_t addr, uint8_t value)
{
  std::vector<uint8_t> params{addr, 1};
  for (uint8_t id : ids) {
    params.push_back(id);
    params.push_back(value);
  }
  return write_packet(BROADCAST_ID, INST_SYNC_WRITE, params);
}

void StsBus::sync_read_position(
  const std::vector<uint8_t> & ids, std::vector<int> & ticks, std::vector<int> & speed,
  std::vector<bool> & ok, int timeout_ms)
{
  ticks.assign(ids.size(), 0);
  speed.assign(ids.size(), 0);
  ok.assign(ids.size(), false);

  std::vector<uint8_t> params{reg::PRESENT_POSITION, 4};
  params.insert(params.end(), ids.begin(), ids.end());
  if (!write_packet(BROADCAST_ID, INST_SYNC_READ, params)) {
    return;
  }
  for (size_t n = 0; n < ids.size(); ++n) {
    uint8_t id = 0;
    std::vector<uint8_t> data;
    if (!read_status(id, data, timeout_ms)) {
      return;
    }
    if (data.size() < 4) {
      continue;
    }
    for (size_t i = 0; i < ids.size(); ++i) {
      if (ids[i] == id) {
        ticks[i] = decode_signed(static_cast<uint16_t>(data[0] | (data[1] << 8)));
        speed[i] = decode_signed(static_cast<uint16_t>(data[2] | (data[3] << 8)));
        ok[i] = true;
      }
    }
  }
}

}
