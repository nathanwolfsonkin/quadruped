#ifndef GAIT_LOADER_HPP
#define GAIT_LOADER_HPP

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <unordered_map>
#include <vector>
#include <string>
#include <tuple>
#include <functional>
#include <cmath>

#include <std_msgs/msg/float64.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

class GaitLoader : public rclcpp::Node
{
public:
    GaitLoader();

private:
    // Structure to store frequency, amplitude, and phase
    using FreqAmpPhase = std::tuple<double, double, double>;

    // Stores raw gait data and trajectory functions
    std::unordered_map<std::string, std::unordered_map<std::string, std::vector<FreqAmpPhase>>> gait_data_;
    std::unordered_map<std::string, std::function<double(double)>> trajectory_functions_;

    // Publishers for each joint
    std::unordered_map<std::string, rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr> publishers_;

    // Timer for periodic publishing
    rclcpp::TimerBase::SharedPtr timer_;

    // Simulated time tracking
    rclcpp::Subscription<rosgraph_msgs::msg::Clock>::SharedPtr clock_sub_;
    double sim_time_ = 0.0;

    void load_yaml(const std::string &file_path);
    void generate_trajectory_functions();
    void setup_publishers();
    void update_clock(const rosgraph_msgs::msg::Clock::SharedPtr msg);
    void publish_commands();

    // Function to compute the sum of sine waves
    static double compute_trajectory(double t, const std::vector<FreqAmpPhase>& data);
};

#endif // GAIT_LOADER_HPP