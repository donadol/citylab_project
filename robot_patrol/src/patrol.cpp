#include <chrono>
#include <geometry_msgs/msg/twist.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;
using std::placeholders::_2;

class Patrol : public rclcpp::Node {
   public:
    Patrol() : Node("patrol_node"), direction_(0.0) {
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan", 10,
            std::bind(&Patrol::laser_scan_callback, this, _1));

        cmd_vel_publisher_ =
            this->create_publisher<geometry_msgs::msg::Twist>("/fastbot_1/cmd_vel", 10);

        control_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&Patrol::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "Patrol node initialized");
    }

   private:
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        float max_distance = 0.0;
        float safest_angle = 0.0;
        bool found_valid = false;

        // We need front 180 degrees: from -π/2 to +π/2
        const double FRONT_MIN_ANGLE = -M_PI / 2.0;  // -90 degrees
        const double FRONT_MAX_ANGLE = M_PI / 2.0;   // +90 degrees
        const double MIN_CLEARANCE = 0.35;           // 35 cm obstacle detection threshold

        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float angle = msg->angle_min + (i * msg->angle_increment);

            // Only consider front 180 degrees
            if (angle < FRONT_MIN_ANGLE || angle > FRONT_MAX_ANGLE) {
                continue;
            }

            float distance = msg->ranges[i];

            // Skip invalid readings
            if (distance < msg->range_min || distance > msg->range_max ||
                std::isnan(distance) || std::isinf(distance)) {
                continue;
            }

            // Only consider directions with at least 0.35m clearance
            if (distance < MIN_CLEARANCE) {
                continue;
            }

            // Find maximum distance direction
            if (distance > max_distance) {
                max_distance = distance;
                safest_angle = angle;
                found_valid = true;
            }
        }

        direction_ = safest_angle;

        if (found_valid) {
            direction_ = safest_angle;
            RCLCPP_DEBUG(this->get_logger(),
                         "Safest direction: %.2f rad (%.1f deg), clearance: %.2f m",
                         direction_, direction_ * 180.0 / M_PI, max_range);

            return;
        }

        // Emergency: no safe direction found, stop turning
        direction_ = 0.0;
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "No safe direction found!");
    }

    void control_loop() {
        auto msg = geometry_msgs::msg::Twist();

        msg.linear.x = 0.1;              // Always move forward
        msg.angular.z = direction_ / 2;  // Turn toward safest direction

        cmd_vel_publisher_->publish(msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::TimerBase::SharedPtr control_timer_;
    float direction_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Patrol>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}