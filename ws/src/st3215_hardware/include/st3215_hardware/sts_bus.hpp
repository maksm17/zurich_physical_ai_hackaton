// Minimal Feetech STS (ST3215) serial bus protocol: sync read / sync write.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace st3215_hardware
{

namespace reg
{
constexpr uint8_t ID = 5;
constexpr uint8_t TORQUE_ENABLE = 40;
constexpr uint8_t ACC = 41;
constexpr uint8_t GOAL_POSITION = 42;
constexpr uint8_t LOCK = 55;
constexpr uint8_t PRESENT_POSITION = 56;
}  // namespace reg

class StsBus
{
public:
  ~StsBus();

  // echo: the adapter loops TX back to RX (half-duplex adapter on a GPIO UART)
  bool open(const std::string & port, int baud, bool echo);
  void close();
  bool is_open() const {return fd_ >= 0;}
  const std::string & port() const {return port_;}

  // Writes ACC, GOAL_POSITION, GOAL_TIME(0), GOAL_SPEED for all ids in one packet.
  bool sync_write_position(
    const std::vector<uint8_t> & ids, const std::vector<int> & ticks, uint8_t acc,
    uint16_t speed);

  bool sync_write_byte(const std::vector<uint8_t> & ids, uint8_t addr, uint8_t value);

  // Reads present position + speed. ok[i] is false if servo i did not answer.
  void sync_read_position(
    const std::vector<uint8_t> & ids, std::vector<int> & ticks, std::vector<int> & speed,
    std::vector<bool> & ok, int timeout_ms);

private:
  bool write_packet(uint8_t id, uint8_t instr, const std::vector<uint8_t> & params);
  bool read_status(uint8_t & id, std::vector<uint8_t> & data, int timeout_ms);
  bool read_exact(uint8_t * buf, size_t n, int timeout_ms);
  void drop_echo(const std::vector<uint8_t> & pkt);

  int fd_{-1};
  bool echo_{false};
  std::vector<uint8_t> pending_;  // bytes read while looking for the echo
  std::string port_;
};

}  // namespace st3215_hardware
