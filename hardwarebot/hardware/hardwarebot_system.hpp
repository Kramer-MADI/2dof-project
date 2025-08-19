#ifndef HARDWAREBOT_SYSTEM_HPP_
#define HARDWAREBOT_SYSTEM_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include <boost/asio.hpp>
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include <rclcpp/rclcpp.hpp>

namespace hardwarebot_hardware
{

class HardwarebotSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(HardwarebotSystem)

  // lifecycle
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;

  // interfaces
  std::vector<hardware_interface::StateInterface>   export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  // I/O loop
  hardware_interface::return_type read (const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // parameters (defaults)
  std::string port_           {"/dev/ttyS3"};
  uint32_t    baud_           {115200};
  std::string cmd_prefix_     {"~"};
  double      steps_per_rad_  {50.0};
  double      steps_per_mm_   {200.0};
  uint32_t    microsteps_     {16};
  double      feed_           {1200.0};   // mm/min
  long        last_feed_      {-1};

  // state / command
  std::vector<double> pos_, vel_, eff_, cmd_, vel_cmd_;
  std::vector<long>   last_steps_;

  // serial
  boost::asio::io_service  io_;
  boost::asio::serial_port serial_{io_};

  void send_line_(const std::string & line);
};

}  // namespace hardwarebot_hardware

#endif  // HARDWAREBOT_SYSTEM_HPP_

