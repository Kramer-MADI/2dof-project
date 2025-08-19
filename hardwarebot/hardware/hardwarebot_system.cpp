#include <algorithm>
#include <limits>
#include <sstream>

#include "hardwarebot_system.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace hardwarebot_hardware
{

// send one line over the serial port
void HardwarebotSystem::send_line_(const std::string & line)
{
  boost::asio::write(serial_, boost::asio::buffer(line));
}

// init
hardware_interface::CallbackReturn
HardwarebotSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS)
    return CallbackReturn::ERROR;

  try {
    const auto & p = info_.hardware_parameters;
    if (p.count("port"))          port_          = p.at("port");
    if (p.count("baud"))          baud_          = std::stoul(p.at("baud"));
    if (p.count("cmd_prefix"))    cmd_prefix_    = p.at("cmd_prefix");
    if (p.count("steps_per_rad")) steps_per_rad_ = std::stod(p.at("steps_per_rad"));
    if (p.count("steps_per_mm"))  steps_per_mm_  = std::stod(p.at("steps_per_mm"));
    if (p.count("microsteps"))    microsteps_    = std::stoul(p.at("microsteps"));
    if (p.count("default_feed"))  feed_          = std::stod(p.at("default_feed"));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("HardwarebotSystem"), "Parameter error: %s", e.what());
    return CallbackReturn::ERROR;
  }

  const std::size_t n = info_.joints.size();
  pos_.assign(n, 0.0);
  vel_.assign(n, 0.0);
  eff_.assign(n, 0.0);
  cmd_.assign(n, 0.0);
  vel_cmd_.assign(n, 0.0);
  last_steps_.assign(n, std::numeric_limits<long>::min());

  RCLCPP_INFO(rclcpp::get_logger("HardwarebotSystem"),
              "Parsed %zu joints. Opening %s @ %u baud …",
              n, port_.c_str(), baud_);

  try {
    using spb = boost::asio::serial_port_base;
    serial_.open(port_);
    serial_.set_option(spb::baud_rate(baud_));
    serial_.set_option(spb::character_size(8));
    serial_.set_option(spb::parity(spb::parity::none));
    serial_.set_option(spb::stop_bits(spb::stop_bits::one));
    serial_.set_option(spb::flow_control(spb::flow_control::none));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(rclcpp::get_logger("HardwarebotSystem"), "Failed to open serial: %s", e.what());
    return CallbackReturn::ERROR;
  }

  std::ostringstream fcmd;
  fcmd << cmd_prefix_ << "F" << feed_ << '\n';
  send_line_(fcmd.str());
  last_feed_ = static_cast<long>(feed_);

  return CallbackReturn::SUCCESS;
}

// state interfaces
std::vector<hardware_interface::StateInterface>
HardwarebotSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> v;
  for (std::size_t i = 0; i < pos_.size(); ++i) {
    v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &pos_[i]);
    v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_[i]);
    v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_EFFORT,   &eff_[i]);
  }
  return v;
}

// command interfaces
std::vector<hardware_interface::CommandInterface>
HardwarebotSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> v;
  for (std::size_t i = 0; i < cmd_.size(); ++i) {
    v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &cmd_[i]);
    v.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &vel_cmd_[i]);
  }
  return v;
}

// lifecycle
hardware_interface::CallbackReturn
HardwarebotSystem::on_activate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("HardwarebotSystem"), "Activated");
  return CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn
HardwarebotSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(rclcpp::get_logger("HardwarebotSystem"), "Deactivated");
  return CallbackReturn::SUCCESS;
}

// read (echo)
hardware_interface::return_type
HardwarebotSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  pos_ = cmd_;
  std::fill(vel_.begin(), vel_.end(), 0.0);
  std::fill(eff_.begin(), eff_.end(), 0.0);
  return hardware_interface::return_type::OK;
}

// write (map joints → GRBL axes)
hardware_interface::return_type
HardwarebotSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  for (std::size_t i = 0; i < cmd_.size(); ++i) {
    const std::string & j = info_.joints[i].name;

    long steps = (j == "Slider28")
                   ? static_cast<long>(cmd_[i] * steps_per_mm_)   // Z (linear)
                   : static_cast<long>(cmd_[i] * steps_per_rad_); // X/Y/A (rotary)

    if (steps == last_steps_[i]) continue;

    std::ostringstream line;
    if      (j == "Revolute22") line << cmd_prefix_ << "Y " << steps << '\n';
    else if (j == "Revolute27") line << cmd_prefix_ << "X " << steps << '\n';
    else if (j == "Slider28")   line << cmd_prefix_ << "Z " << steps << '\n';
    else if (j == "Slider30")   line << cmd_prefix_ << "A " << steps << '\n';
    else                        line << cmd_prefix_ << "J " << j << ' ' << steps << '\n';

    send_line_(line.str());
    last_steps_[i] = steps;
  }
  return hardware_interface::return_type::OK;
}

}  // namespace hardwarebot_hardware

PLUGINLIB_EXPORT_CLASS(hardwarebot_hardware::HardwarebotSystem,
                       hardware_interface::SystemInterface)
