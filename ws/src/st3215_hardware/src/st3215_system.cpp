#include "st3215_hardware/st3215_system.hpp"

#include <algorithm>
#include <cmath>
#include <sstream>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/rclcpp.hpp"

namespace st3215_hardware
{

namespace
{
constexpr double TICKS_PER_RAD = 4096.0 / (2.0 * M_PI);
const rclcpp::Logger LOGGER = rclcpp::get_logger("ST3215System");

std::string param_or(
  const std::unordered_map<std::string, std::string> & params, const std::string & key,
  const std::string & def)
{
  auto it = params.find(key);
  return it == params.end() ? def : it->second;
}
}

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
hardware_interface::CallbackReturn ST3215System::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }
#pragma GCC diagnostic pop

  baud_ = std::stoi(param_or(info_.hardware_parameters, "baud", "1000000"));
  acc_ = static_cast<uint8_t>(std::stoi(param_or(info_.hardware_parameters, "acc", "0")));
  speed_ = static_cast<uint16_t>(std::stoi(param_or(info_.hardware_parameters, "speed", "0")));
  timeout_ms_ = std::stoi(param_or(info_.hardware_parameters, "timeout_ms", "5"));
  echo_ = param_or(info_.hardware_parameters, "echo", "true") == "true";

  for (const auto & ji : info_.joints) {
    Joint j;
    j.name = ji.name;
    if (!ji.parameters.count("id") || !ji.parameters.count("port")) {
      RCLCPP_FATAL(LOGGER, "Joint '%s' needs <param name=\"id\"> and <param name=\"port\">",
        ji.name.c_str());
      return CallbackReturn::ERROR;
    }
    j.id = static_cast<uint8_t>(std::stoi(ji.parameters.at("id")));
    j.port = ji.parameters.at("port");
    j.sign = std::stod(param_or(ji.parameters, "sign", "1"));
    j.zero = std::stoi(param_or(ji.parameters, "zero", "2048"));
    if (ji.command_interfaces.size() != 1 ||
      ji.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(LOGGER, "Joint '%s' must have exactly one 'position' command interface",
        ji.name.c_str());
      return CallbackReturn::ERROR;
    }
    const auto & ci = ji.command_interfaces[0];
    if (!ci.min.empty()) {j.min = std::stod(ci.min);}
    if (!ci.max.empty()) {j.max = std::stod(ci.max);}
    joints_.push_back(j);
  }

  for (size_t i = 0; i < joints_.size(); ++i) {
    auto & bus = buses_[joints_[i].port];
    if (!bus) {
      bus = std::make_unique<Bus>();
    }
    bus->joints.push_back(i);
    bus->ids.push_back(joints_[i].id);
  }
  return CallbackReturn::SUCCESS;
}

double ST3215System::ticks_to_rad(const Joint & j, int ticks) const
{
  return j.sign * (ticks - j.zero) / TICKS_PER_RAD;
}

int ST3215System::rad_to_ticks(const Joint & j, double rad) const
{
  const int t = j.zero + static_cast<int>(std::lround(j.sign * rad * TICKS_PER_RAD));
  return std::clamp(t, 0, 4095);
}

hardware_interface::CallbackReturn ST3215System::on_configure(const rclcpp_lifecycle::State &)
{
  for (auto & [port, bus] : buses_) {
    if (!bus->bus.open(port, baud_, echo_)) {
      RCLCPP_FATAL(LOGGER, "Cannot open serial port %s", port.c_str());
      return CallbackReturn::ERROR;
    }
    RCLCPP_INFO(LOGGER, "Opened %s (%zu servos)", port.c_str(), bus->ids.size());
  }
  for (int attempt = 0; attempt < 5; ++attempt) {
    if (read_all(true)) {
      return CallbackReturn::SUCCESS;
    }
  }
  return CallbackReturn::ERROR;
}

hardware_interface::CallbackReturn ST3215System::on_cleanup(const rclcpp_lifecycle::State &)
{
  for (auto & [port, bus] : buses_) {
    bus->bus.close();
  }
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ST3215System::on_activate(const rclcpp_lifecycle::State &)
{
  bool ok = false;
  for (int attempt = 0; attempt < 5 && !ok; ++attempt) {
    ok = read_all(attempt == 4);
  }
  if (!ok) {
    RCLCPP_FATAL(LOGGER, "Not all servos answered, refusing to enable torque");
    return CallbackReturn::ERROR;
  }
  for (const auto & j : joints_) {
    if (j.pos < j.min - 0.05 || j.pos > j.max + 0.05) {
      RCLCPP_FATAL(LOGGER, "%s is at %.0f deg, outside its limits [%.0f, %.0f] deg: move it "
        "by hand towards zero before enabling torque", j.name.c_str(), j.pos * 180 / M_PI,
        j.min * 180 / M_PI, j.max * 180 / M_PI);
      return CallbackReturn::ERROR;
    }
  }
  for (auto & j : joints_) {
    j.cmd = j.pos;
  }
  for (auto & [port, bus] : buses_) {
    std::vector<int> ticks;
    for (size_t idx : bus->joints) {
      ticks.push_back(rad_to_ticks(joints_[idx], joints_[idx].cmd));
    }
    bus->bus.sync_write_position(bus->ids, ticks, acc_, speed_);
    bus->bus.sync_write_byte(bus->ids, reg::TORQUE_ENABLE, 1);
  }
  RCLCPP_INFO(LOGGER, "Torque enabled");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ST3215System::on_deactivate(const rclcpp_lifecycle::State &)
{
  for (auto & [port, bus] : buses_) {
    bus->bus.sync_write_byte(bus->ids, reg::TORQUE_ENABLE, 0);
  }
  RCLCPP_INFO(LOGGER, "Torque disabled");
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ST3215System::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> out;
  for (auto & j : joints_) {
    out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.pos);
    out.emplace_back(j.name, hardware_interface::HW_IF_VELOCITY, &j.vel);
  }
  return out;
}

std::vector<hardware_interface::CommandInterface> ST3215System::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> out;
  for (auto & j : joints_) {
    out.emplace_back(j.name, hardware_interface::HW_IF_POSITION, &j.cmd);
  }
  return out;
}

bool ST3215System::read_all(bool require_all)
{
  bool all_ok = true;
  for (auto & [port, bus] : buses_) {
    std::vector<int> ticks, speed;
    std::vector<bool> ok;
    bus->bus.sync_read_position(bus->ids, ticks, speed, ok, timeout_ms_);
    std::ostringstream missing;
    bool bus_ok = true;
    for (size_t k = 0; k < bus->joints.size(); ++k) {
      auto & j = joints_[bus->joints[k]];
      if (ok[k]) {
        j.pos = ticks_to_rad(j, ticks[k]);
        j.vel = j.sign * speed[k] / TICKS_PER_RAD;
      } else {
        all_ok = false;
        bus_ok = false;
        missing << " " << j.name << "(id " << static_cast<int>(j.id) << ")";
      }
    }
    bus->fail_cycles = bus_ok ? 0 : bus->fail_cycles + 1;
    if (bus->fail_cycles >= 50 && bus->fail_cycles % 100 == 50) {
      RCLCPP_ERROR(LOGGER, "BUS %s NOT ANSWERING for %zu cycles (stale positions!):%s - check "
        "RX/GND wiring and servo power", port.c_str(), bus->fail_cycles, missing.str().c_str());
    }
    if (require_all && !missing.str().empty()) {
      RCLCPP_ERROR(LOGGER, "No answer on %s from:%s", port.c_str(), missing.str().c_str());
    }
  }
  return all_ok;
}

hardware_interface::return_type ST3215System::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!read_all(false)) {
    if (++read_errors_ % 100 == 1) {
      RCLCPP_WARN(LOGGER, "Servo read errors so far: %zu", read_errors_);
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type ST3215System::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  for (auto & [port, bus] : buses_) {
    std::vector<int> ticks;
    for (size_t idx : bus->joints) {
      auto & j = joints_[idx];
      if (std::isnan(j.cmd)) {
        j.cmd = j.pos;
      }
      ticks.push_back(rad_to_ticks(j, std::clamp(j.cmd, j.min, j.max)));
    }
    bus->bus.sync_write_position(bus->ids, ticks, acc_, speed_);
  }
  return hardware_interface::return_type::OK;
}

}

PLUGINLIB_EXPORT_CLASS(st3215_hardware::ST3215System, hardware_interface::SystemInterface)
