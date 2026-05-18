#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/region_of_interest.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/LinearMath/Matrix3x3.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/empty.hpp>
#include <vector>
#include <cmath>
#include <limits>

class LidarLoc : public rclcpp::Node
{
public:
    LidarLoc()
    : Node("lidar_loc")
    {
        setlocale(LC_ALL, "");

        this->declare_parameter<std::string>("base_frame", "base_footprint");
        this->declare_parameter<std::string>("odom_frame", "odom");
        this->declare_parameter<std::string>("laser_frame", "laser");
        this->declare_parameter<std::string>("laser_topic", "scan");
        this->declare_parameter<std::string>("clear_costmap_service", "/global_costmap/clear_entirely_global_costmap");
        this->declare_parameter<int>("scan_stride", 2);
        this->declare_parameter<int>("max_iterations", 8);
        this->declare_parameter<double>("convergence_translation_pixels", 0.5);
        this->declare_parameter<double>("convergence_yaw_deg", 0.2);
        this->declare_parameter<double>("max_usable_range", 12.0);
        this->declare_parameter<bool>("auto_initialize_from_map_center", true);

        base_frame_ = this->get_parameter("base_frame").as_string();
        odom_frame_ = this->get_parameter("odom_frame").as_string();
        laser_frame_ = this->get_parameter("laser_frame").as_string();
        laser_topic_ = this->get_parameter("laser_topic").as_string();
        clear_costmap_service_ = this->get_parameter("clear_costmap_service").as_string();
        scan_stride_ = std::max(1, static_cast<int>(this->get_parameter("scan_stride").as_int()));
        max_iterations_ = std::max(1, static_cast<int>(this->get_parameter("max_iterations").as_int()));
        convergence_translation_pixels_ = this->get_parameter("convergence_translation_pixels").as_double();
        convergence_yaw_rad_ = this->get_parameter("convergence_yaw_deg").as_double() * M_PI / 180.0;
        max_usable_range_ = this->get_parameter("max_usable_range").as_double();
        auto_initialize_from_map_center_ = this->get_parameter("auto_initialize_from_map_center").as_bool();

        map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            "map", 1, std::bind(&LidarLoc::mapCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            laser_topic_, 1, std::bind(&LidarLoc::scanCallback, this, std::placeholders::_1));
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 1, std::bind(&LidarLoc::initialPoseCallback, this, std::placeholders::_1));
        clear_costmaps_client_ = this->create_client<std_srvs::srv::Empty>(clear_costmap_service_);

        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        pose_tf_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(33), std::bind(&LidarLoc::pose_tf, this));
    }

private:
    nav_msgs::msg::OccupancyGrid map_msg_;
    cv::Mat map_cropped_;
    cv::Mat map_temp_;
    sensor_msgs::msg::RegionOfInterest map_roi_info_;
    std::vector<cv::Point2f> scan_points_;
    rclcpp::Client<std_srvs::srv::Empty>::SharedPtr clear_costmaps_client_;
    std::string base_frame_;
    std::string odom_frame_;
    std::string laser_frame_;
    std::string laser_topic_;
    std::string clear_costmap_service_;
    int scan_stride_ = 2;
    int max_iterations_ = 8;
    double convergence_translation_pixels_ = 0.5;
    double convergence_yaw_rad_ = 0.2 * M_PI / 180.0;
    double max_usable_range_ = 12.0;
    bool auto_initialize_from_map_center_ = true;

    float lidar_x_ = 250;
    float lidar_y_ = 250;
    float lidar_yaw_ = 0;
    float deg_to_rad_ = M_PI / 180.0;
    int clear_countdown_ = -1;
    int scan_count_ = 0;
    bool lidar_is_inverted_ = false;
    bool has_map_ = false;
    bool pose_initialized_ = false;

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
    rclcpp::TimerBase::SharedPtr pose_tf_timer_;

    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        double map_x = msg->pose.pose.position.x;
        double map_y = msg->pose.pose.position.y;
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);

        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        if (map_msg_.info.resolution <= 0) {
            RCLCPP_ERROR(this->get_logger(), "Map info invalid or not received");
            return;
        }

        lidar_x_ = (map_x - map_msg_.info.origin.position.x) / map_msg_.info.resolution - map_roi_info_.x_offset;
        lidar_y_ = (map_y - map_msg_.info.origin.position.y) / map_msg_.info.resolution - map_roi_info_.y_offset;

        lidar_yaw_ = -yaw;
        pose_initialized_ = true;

        clear_countdown_ = 30;
    }

    void crop_map();
    void processMap();

    void mapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        map_msg_ = *msg;
        has_map_ = true;
        crop_map();
        processMap();
    }

    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg);
    cv::Mat createGradientMask(int size);
    void pose_tf();
    int scorePoints(const std::vector<cv::Point2f>& points, float dx, float dy, float dyaw);
};

void LidarLoc::crop_map()
{
    nav_msgs::msg::MapMetaData info = map_msg_.info;

    size_t xMax, xMin, yMax, yMin;
    xMax = xMin = info.width / 2;
    yMax = yMin = info.height / 2;
    bool bFirstPoint = true;

    cv::Mat map_raw(info.height, info.width, CV_8UC1, cv::Scalar(128));

    for (size_t y = 0; y < info.height; y++)
    {
        for (size_t x = 0; x < info.width; x++)
        {
            size_t index = y * info.width + x;
            map_raw.at<uchar>(y, x) = static_cast<uchar>(map_msg_.data[index]);
            if (map_msg_.data[index] == 100)
            {
                if (bFirstPoint)
                {
                    xMax = xMin = x;
                    yMax = yMin = y;
                    bFirstPoint = false;
                    continue;
                }
                xMin = std::min(xMin, x);
                xMax = std::max(xMax, x);
                yMin = std::min(yMin, y);
                yMax = std::max(yMax, y);
            }
        }
    }

    int cen_x = static_cast<int>((xMin + xMax) / 2);
    int cen_y = static_cast<int>((yMin + yMax) / 2);

    int new_half_width = static_cast<int>(xMax - xMin) / 2 + 50;
    int new_half_height = static_cast<int>(yMax - yMin) / 2 + 50;
    int new_origin_x = cen_x - new_half_width;
    int new_origin_y = cen_y - new_half_height;
    int new_width = new_half_width * 2;
    int new_height = new_half_height * 2;

    if (new_origin_x < 0) new_origin_x = 0;
    if (new_origin_x + new_width > static_cast<int>(info.width)) new_width = static_cast<int>(info.width) - new_origin_x;
    if (new_origin_y < 0) new_origin_y = 0;
    if (new_origin_y + new_height > static_cast<int>(info.height)) new_height = static_cast<int>(info.height) - new_origin_y;

    cv::Rect roi(new_origin_x, new_origin_y, new_width, new_height);
    cv::Mat roi_map = map_raw(roi).clone();
    map_cropped_ = roi_map;

    map_roi_info_.x_offset = new_origin_x;
    map_roi_info_.y_offset = new_origin_y;
    map_roi_info_.width = new_width;
    map_roi_info_.height = new_height;

    if (!pose_initialized_ && auto_initialize_from_map_center_)
    {
        lidar_x_ = static_cast<float>(map_roi_info_.width) * 0.5f;
        lidar_y_ = static_cast<float>(map_roi_info_.height) * 0.5f;
        lidar_yaw_ = 0.0f;
        pose_initialized_ = true;
    }
}

void LidarLoc::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    if (!has_map_ || map_cropped_.empty() || map_temp_.empty() || map_msg_.info.resolution <= 0.0 || !pose_initialized_)
    {
        return;
    }

    scan_points_.clear();
    double angle = msg->angle_min;

    geometry_msgs::msg::TransformStamped transformStamped;
    try {
        transformStamped = tf_buffer_->lookupTransform(
            base_frame_, laser_frame_, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "%s", ex.what());
        return;
    }

    tf2::Quaternion q_lidar;
    tf2::fromMsg(transformStamped.transform.rotation, q_lidar);

    double roll, pitch, yaw;
    tf2::Matrix3x3(q_lidar).getRPY(roll, pitch, yaw);

    const double tolerance = 0.1;
    lidar_is_inverted_ = std::abs(std::abs(roll) - M_PI) < tolerance;
    lidar_is_inverted_ = lidar_is_inverted_ && !(std::abs(std::abs(pitch) - M_PI) < tolerance);

    for (size_t i = 0; i < msg->ranges.size(); i += static_cast<size_t>(scan_stride_))
    {
        if (msg->ranges[i] >= msg->range_min &&
            msg->ranges[i] <= msg->range_max &&
            msg->ranges[i] <= max_usable_range_)
        {
            float x_laser = msg->ranges[i] * cos(angle);
            float y_laser = -msg->ranges[i] * sin(angle);

            geometry_msgs::msg::PointStamped point_laser;
            point_laser.header.frame_id = laser_frame_;
            point_laser.header.stamp = msg->header.stamp;
            point_laser.point.x = x_laser;
            point_laser.point.y = y_laser;
            point_laser.point.z = 0.0;

            geometry_msgs::msg::PointStamped point_base;
            tf2::doTransform(point_laser, point_base, transformStamped);

            float x = point_base.point.x / map_msg_.info.resolution;
            float y = point_base.point.y / map_msg_.info.resolution;
            if (lidar_is_inverted_)
            {
                x = -x;
                y = -y;
            }
            scan_points_.push_back(cv::Point2f(x, y));
        }
        angle += msg->angle_increment * static_cast<double>(scan_stride_);
    }

    if (scan_points_.empty())
    {
        return;
    }

    if (scan_count_ == 0)
        scan_count_++;

    for (int iter = 0; iter < max_iterations_; ++iter)
    {
        const std::vector<cv::Point2f> offsets = {{0, 0}, {1, 0}, {-1, 0}, {0, 1}, {0, -1}};
        const std::vector<float> yaw_offsets = {0.0f, deg_to_rad_, -deg_to_rad_};

        int best_score = std::numeric_limits<int>::min();
        float best_dx = 0.0f;
        float best_dy = 0.0f;
        float best_dyaw = 0.0f;

        for (const auto& offset : offsets)
        {
            for (float yaw_offset : yaw_offsets)
            {
                const int score = scorePoints(scan_points_, offset.x, offset.y, yaw_offset);
                if (score > best_score)
                {
                    best_score = score;
                    best_dx = offset.x;
                    best_dy = offset.y;
                    best_dyaw = yaw_offset;
                }
            }
        }

        lidar_x_ += best_dx;
        lidar_y_ += best_dy;
        lidar_yaw_ += best_dyaw;

        if (std::abs(best_dx) <= convergence_translation_pixels_ &&
            std::abs(best_dy) <= convergence_translation_pixels_ &&
            std::abs(best_dyaw) <= convergence_yaw_rad_)
        {
            break;
        }
    }

    if (clear_countdown_ > -1)
        clear_countdown_--;
    if (clear_countdown_ == 0)
    {
        if (clear_costmaps_client_->wait_for_service(std::chrono::seconds(1)))
        {
            auto request = std::make_shared<std_srvs::srv::Empty::Request>();
            clear_costmaps_client_->async_send_request(request);
        }
    }
}

cv::Mat LidarLoc::createGradientMask(int size)
{
    cv::Mat mask(size, size, CV_8UC1);
    int center = size / 2;
    for (int y = 0; y < size; y++)
    {
        for (int x = 0; x < size; x++)
        {
            double distance = std::hypot(x - center, y - center);
            int value = cv::saturate_cast<uchar>(255 * std::max(0.0, 1.0 - distance / center));
            mask.at<uchar>(y, x) = value;
        }
    }
    return mask;
}

void LidarLoc::processMap()
{
    if (map_cropped_.empty()) return;

    map_temp_ = cv::Mat::zeros(map_cropped_.size(), CV_8UC1);
    cv::Mat gradient_mask = createGradientMask(101);
    for (int y = 0; y < map_cropped_.rows; y++)
    {
        for (int x = 0; x < map_cropped_.cols; x++)
        {
            if (map_cropped_.at<uchar>(y, x) == 100)
            {
                int left = std::max(0, x - 50);
                int top = std::max(0, y - 50);
                int right = std::min(map_cropped_.cols - 1, x + 50);
                int bottom = std::min(map_cropped_.rows - 1, y + 50);

                cv::Rect roi(left, top, right - left + 1, bottom - top + 1);
                cv::Mat region = map_temp_(roi);

                int mask_left = 50 - (x - left);
                int mask_top = 50 - (y - top);
                cv::Rect mask_roi(mask_left, mask_top, roi.width, roi.height);
                cv::Mat mask = gradient_mask(mask_roi);

                cv::max(region, mask, region);
            }
        }
    }
}

int LidarLoc::scorePoints(const std::vector<cv::Point2f>& points, float dx, float dy, float dyaw)
{
    const float yaw = lidar_yaw_ + dyaw;
    const float c = std::cos(yaw);
    const float s = std::sin(yaw);
    int sum = 0;

    for (const auto& point : points)
    {
        const float rotated_x = point.x * c - point.y * s;
        const float rotated_y = point.x * s + point.y * c;
        const int px = static_cast<int>(std::lround(rotated_x + lidar_x_ + dx));
        const int py = static_cast<int>(std::lround(lidar_y_ - rotated_y + dy));

        if (px >= 0 && px < map_temp_.cols && py >= 0 && py < map_temp_.rows)
        {
            sum += map_temp_.at<uchar>(py, px);
        }
    }

    return sum;
}

void LidarLoc::pose_tf()
{
    if (scan_count_ == 0) return;
    if (map_cropped_.empty() || map_msg_.data.empty() || map_msg_.info.resolution <= 0) return;

    double full_map_pixel_x = lidar_x_ + map_roi_info_.x_offset;
    double full_map_pixel_y = lidar_y_ + map_roi_info_.y_offset;

    double x_in_map_frame = full_map_pixel_x * map_msg_.info.resolution + map_msg_.info.origin.position.x;
    double y_in_map_frame = full_map_pixel_y * map_msg_.info.resolution + map_msg_.info.origin.position.y;

    double yaw_in_map_frame = -lidar_yaw_;

    tf2::Transform map_to_base;
    map_to_base.setOrigin(tf2::Vector3(x_in_map_frame, y_in_map_frame, 0.0));
    tf2::Quaternion q;
    q.setRPY(0, 0, yaw_in_map_frame);
    map_to_base.setRotation(q);

    geometry_msgs::msg::TransformStamped odom_to_base_msg;
    try {
        odom_to_base_msg = tf_buffer_->lookupTransform(
            odom_frame_, base_frame_, tf2::TimePointZero);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_WARN(this->get_logger(), "Cannot get transform from '%s' to '%s': %s",
            odom_frame_.c_str(), base_frame_.c_str(), ex.what());
        return;
    }

    tf2::Transform odom_to_base_tf2;
    tf2::fromMsg(odom_to_base_msg.transform, odom_to_base_tf2);
    tf2::Transform map_to_odom = map_to_base * odom_to_base_tf2.inverse();

    geometry_msgs::msg::TransformStamped map_to_odom_msg;
    map_to_odom_msg.header.stamp = this->get_clock()->now();
    map_to_odom_msg.header.frame_id = "map";
    map_to_odom_msg.child_frame_id = odom_frame_;
    map_to_odom_msg.transform = tf2::toMsg(map_to_odom);

    tf_broadcaster_->sendTransform(map_to_odom_msg);
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarLoc>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
