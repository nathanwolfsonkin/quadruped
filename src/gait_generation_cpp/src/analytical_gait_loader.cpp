#include <rclcpp/rclcpp.hpp>
#include <pybind11/embed.h>

#include "gait_generation_cpp/analytical_gait_loader.hh"

GaitLoader::GaitLoader() : Node("gait_loader")
{
  load_python_parameters();
  std::string filepath = "/workspace/install/gait_generation/share/gait_generation/gait_trajectory/gait_sin_waves.yaml";
  load_yaml(filepath);
  generate_trajectory_functions();
  setup_publishers();

  // Subscribe to /clock topic to get simulated time
  clock_sub_ = this->create_subscription<rosgraph_msgs::msg::Clock>(
      "/clock", 10, std::bind(&GaitLoader::update_clock, this, std::placeholders::_1));

  double gait_command_rate_us = this->gait_command_rate_ * 1e6;
  // Timer to periodically publish joint commands
  timer_ = this->create_wall_timer(
      std::chrono::microseconds(static_cast<int>(gait_command_rate_us)),
      std::bind(&GaitLoader::publish_commands, this));
}

// Function to load parameters from python
void GaitLoader::load_python_parameters()
{
  pybind11::scoped_interpreter guard{};

  try
  {
    pybind11::module sim_params = pybind11::module::import("simulation.parameters");
    this->gait_command_rate_ = sim_params.attr("gait_command_publishing_rate").cast<double>();
    this->start_time_ = sim_params.attr("start_walking_time").cast<double>();

    RCLCPP_DEBUG(this->get_logger(), "Loaded gait command rate: %f", gait_command_rate_);
    RCLCPP_DEBUG(this->get_logger(), "Loaded start walking time: %f", start_time_);
  }
  catch (pybind11::error_already_set &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Python error: %s", e.what());
  }
}

// Function to set up publishers
void GaitLoader::setup_publishers()
{
  for (const auto &[key, _] : trajectory_functions_)
  {
    std::string topic_name = "/quadruped/cmd_" + key + "_joint";
    publishers_[key] = this->create_publisher<std_msgs::msg::Float64>(topic_name, 10);
  }
}

// Function to update simulation time
void GaitLoader::update_clock(const rosgraph_msgs::msg::Clock::SharedPtr msg)
{
  sim_time_ = msg->clock.sec + msg->clock.nanosec * 1e-9;
}

// Function to publish joint commands
void GaitLoader::publish_commands()
{
  for (const auto &[key, func] : trajectory_functions_)
  {
    std_msgs::msg::Float64 msg;

    if (this->sim_time_ < this->start_time_)
    {
      msg.data = func(this->start_time_);
    }
    else
    {
      msg.data = func(this->sim_time_);
    }
    
    publishers_[key]->publish(msg);
  }
}

// Function to compute the sum of sine waves
double GaitLoader::compute_trajectory(double t, const std::vector<FreqAmpPhase> &data)
{
  double total = 0.0;
  for (const auto &[freq, amp, phase] : data)
  {
    total += amp * std::cos(2 * M_PI * freq * t + phase);
  }
  return total;
}

// Generate trajectory functions and store them in a map
void GaitLoader::generate_trajectory_functions()
{
  for (const auto &[leg, joints] : gait_data_)
  {
    for (const auto &[joint, data] : joints)
    {
      std::string key = leg + "_" + joint;

      // Store a lambda function in the map
      trajectory_functions_[key] = [data](double t) -> double
      {
        return GaitLoader::compute_trajectory(t, data);
      };
    }
  }

  // Print confirmation
  for (const auto &[key, func] : trajectory_functions_)
  {
    RCLCPP_DEBUG(this->get_logger(), "Generated function for %s", key.c_str());
  }
}

// Loads yaml data into gait_data_
void GaitLoader::load_yaml(const std::string &file_path)
{
  try
  {
    YAML::Node yaml_data = YAML::LoadFile(file_path);

    for (const auto &leg : yaml_data)
    {
      std::string leg_name = leg.first.as<std::string>();
      gait_data_[leg_name] = {};

      for (const auto &joint : leg.second)
      {
        std::string joint_name = joint.first.as<std::string>();
        gait_data_[leg_name][joint_name] = {};

        for (const auto &freq_amp_phase : joint.second)
        {
          std::vector<double> values;
          for (const auto &val : freq_amp_phase.second)
          {
            values.push_back(val.as<double>());
          }
          if (values.size() == 3)
          {
            gait_data_[leg_name][joint_name].emplace_back(values[0], values[1], values[2]);
          }
        }
      }
    }

    // Print loaded data
    for (const auto &[leg, joints] : gait_data_)
    {
      RCLCPP_DEBUG(this->get_logger(), "Leg: %s", leg.c_str());
      for (const auto &[joint, values] : joints)
      {
        RCLCPP_DEBUG(this->get_logger(), "  Joint: %s", joint.c_str());
        for (const auto &[freq, amp, phase] : values)
        {
          RCLCPP_DEBUG(this->get_logger(), "    Freq: %.3f, Amp: %.3f, Phase: %.3f", freq, amp, phase);
        }
      }
    }
  }
  catch (const std::exception &e)
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to load YAML file: %s", e.what());
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GaitLoader>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}