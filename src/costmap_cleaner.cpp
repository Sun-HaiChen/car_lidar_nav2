#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <std_srvs/srv/empty.hpp>

class CostmapClearer : public rclcpp::Node
{
public:
    CostmapClearer()
    : Node("costmap_cleaner")
    {
        setlocale(LC_ALL, "");
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose", 1,
            std::bind(&CostmapClearer::initialPoseCallback, this, std::placeholders::_1));
        clear_costmaps_client_ = this->create_client<std_srvs::srv::Empty>("/move_base/clear_costmaps");
    }

    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        (void)msg;
        if (!clear_costmaps_client_->wait_for_service(std::chrono::seconds(1)))
        {
            RCLCPP_ERROR(this->get_logger(), "clear_costmaps service not available");
            return;
        }
        auto request = std::make_shared<std_srvs::srv::Empty::Request>();
        auto result = clear_costmaps_client_->async_send_request(request);
        if (result.wait_for(std::chrono::seconds(1)) == std::future_status::ready)
        {
            RCLCPP_INFO(this->get_logger(), "Successfully cleared costmaps");
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Costmap clearing failed");
        }
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmaps_client_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CostmapClearer>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
