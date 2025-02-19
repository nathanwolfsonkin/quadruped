#include "cross_correlation_node.hpp"
#include <numeric>
#include <iomanip>

CrossCorrelationNode::CrossCorrelationNode()
    : Node("cross_correlation_node"), window_size_(100)
{ // Default window size = 100
    sub1_ = this->create_subscription<std_msgs::msg::Float64>(
        "/quadruped/cmd_FR_thigh_joint", 10, std::bind(&CrossCorrelationNode::FR_callback, this, std::placeholders::_1));

    sub2_ = this->create_subscription<std_msgs::msg::Float64>(
        "/quadruped/cmd_FL_thigh_joint", 10, std::bind(&CrossCorrelationNode::FL_callback, this, std::placeholders::_1));

    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(500),
        std::bind(&CrossCorrelationNode::compute_outer_product, this));
    
    RCLCPP_INFO(this->get_logger(), "CrossCorrelationNode initialized with window size %zu", window_size_);
}

void CrossCorrelationNode::FR_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    buffer1_.push_back(msg->data);
    if (buffer1_.size() > window_size_)
    {
        buffer1_.pop_front();
    }
}

void CrossCorrelationNode::FL_callback(const std_msgs::msg::Float64::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    buffer2_.push_back(msg->data);
    if (buffer2_.size() > window_size_)
    {
        buffer2_.pop_front();
    }
}

void CrossCorrelationNode::compute_outer_product()
{
    std::lock_guard<std::mutex> lock(buffer_mutex_);

    if (buffer1_.size() < window_size_ || buffer2_.size() < window_size_)
    {
        return;
    }

    std::vector<std::vector<double>> outer_product(window_size_, std::vector<double>(window_size_));

    for (size_t i = 0; i < window_size_; ++i)
    {
        for (size_t j = 0; j < window_size_; ++j)
        {
            outer_product[i][j] = buffer1_[i] * buffer2_[j];
        }
    }

    int mat_view_size = 10;

    // Lof a summary of the matrix (first 5x5 section for readability)
    std::ostringstream matrix_stream;
    matrix_stream << "Outer Product Matrix (first 5x5 elements):\n";
    for (size_t i = 0; i < std::min(window_size_, size_t(mat_view_size)); ++i)
    {
        for (size_t j = 0; j < std::min(window_size_, size_t(mat_view_size)); ++j)
        {
            matrix_stream << std::fixed << std::setprecision(2) << outer_product[i][j] << " ";
        }
        matrix_stream << "\n";
    }

    RCLCPP_INFO(this->get_logger(), "%s", matrix_stream.str().c_str());
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CrossCorrelationNode>());
    rclcpp::shutdown();
    return 0;
}