#pragma once

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "st3215_hardware/sts_bus.hpp"

namespace st3215_hardware
{

class ST3215System : public hardware_interface::SystemInterface
{
public:
  CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Joint
  {
    std::string name;
    uint8_t id{0};
    std::string port;
    double sign{1.0};
    int zero{2048};
    double min{-3.14};
    double max{3.14};
    double pos{0.0};
    double vel{0.0};
    double cmd{0.0};
  };

  struct Bus
  {
    StsBus bus;
    std::vector<size_t> joints;
    std::vector<uint8_t> ids;
    size_t fail_cycles{0};
  };

  bool read_all(bool require_all);
  double ticks_to_rad(const Joint & j, int ticks) const;
  int rad_to_ticks(const Joint & j, double rad) const;

  std::vector<Joint> joints_;
  std::map<std::string, std::unique_ptr<Bus>> buses_;
  int baud_{1000000};
  uint8_t acc_{0};
  uint16_t speed_{0};
  int timeout_ms_{5};
  bool echo_{true};
  size_t read_errors_{0};
};

}
