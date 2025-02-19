#ifndef CROSS_CORRELATION_NODE_HPP
#define CROSS_CORRELATION_NODE_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>
#include <deque>
#include <vector>
#include <mutex>

class CrossCorrelationNode : public rclcpp::Node {
public:
    CrossCorrelationNode();

private:
    void FR_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void FL_callback(const std_msgs::msg::Float64::SharedPtr msg);
    void compute_outer_product();

    size_t window_size_;
    std::deque<double> buffer1_;
    std::deque<double> buffer2_;
    std::mutex buffer_mutex_;

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub1_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr sub2_;
    rclcpp::TimerBase::SharedPtr timer_;
};

#endif // CROSS_CORRELATION_NODE_HPP
