#include <cmath>
#include <csignal>
#include <geometry_msgs/msg/twist.hpp>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

#include "robot_patrol/srv/get_direction.hpp"

class DirectionService : public rclcpp::Node {
   public:
    DirectionService() : Node("direction_service_node") {
        std::string name_service = "/direction_service";
        service_ = this->create_service<robot_patrol::srv::GetDirection>(
            name_service, std::bind(&DirectionService::direction_service, this,
                                    std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "%s Service Server Ready...",
                    name_service.c_str());
    }

   private:
    void direction_service(const std::shared_ptr<robot_patrol::srv::GetDirection::Request> request,
                           std::shared_ptr<robot_patrol::srv::GetDirection::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Service Requested");

        // Get the laser scan data from the request
        auto laser_data = request->laser_data;

        // Real robot laser: 0 to 2π (360°) - angle_min=0, angle_max=6.28...
        // Robot faces forward at 0°/360°
        // Front 180° divided into 3 sections of 60° each:
        // Right:  270° to 330° = 3π/2 to 11π/6
        // Front:  330° to 30°  = 11π/6 to π/6  (wraps through 0°)
        // Left:   30° to 90°   = π/6 to π/2

        const double RIGHT_MIN = 3.0 * M_PI / 2.0;   // 270° = 3π/2
        const double RIGHT_MAX = 11.0 * M_PI / 6.0;  // 330° = 11π/6
        const double FRONT_MIN = 11.0 * M_PI / 6.0;  // 330° = 11π/6
        const double FRONT_MAX = M_PI / 6.0;         // 30°  = π/6
        const double LEFT_MIN = M_PI / 6.0;          // 30°  = π/6
        const double LEFT_MAX = M_PI / 2.0;          // 90°  = π/2

        const double MIN_CLEARANCE = 0.5;  // 50 cm threshold

        // Initialize distance totals for each section
        double total_dist_sec_right = 0.0;
        double total_dist_sec_front = 0.0;
        double total_dist_sec_left = 0.0;

        // Track minimum distance in front section for threshold check
        double min_front_distance = std::numeric_limits<double>::max();

        // Sum up distances for each section
        for (size_t i = 0; i < laser_data.ranges.size(); ++i) {
            float distance = laser_data.ranges[i];

            // Skip invalid readings (inf values indicate out-of-range)
            if (std::isinf(distance) || std::isnan(distance)) {
                continue;
            }

            // Calculate the angle for this ray
            float angle = laser_data.angle_min + (i * laser_data.angle_increment);

            // Determine which section this ray belongs to and sum distances
            // Right section: 270° to 330°
            if (angle >= RIGHT_MIN && angle < RIGHT_MAX) {
                total_dist_sec_right += distance;
                continue;
            }

            // Front section: 330° to 360° OR 0° to 30° (wraps through 0°)
            if (angle >= FRONT_MIN || angle <= FRONT_MAX) {
                total_dist_sec_front += distance;
                min_front_distance = std::min(min_front_distance, static_cast<double>(distance));
                continue;
            }

            // Left section: 30° to 90°
            if (angle >= LEFT_MIN && angle <= LEFT_MAX) {
                total_dist_sec_left += distance;
            }
        }

        // Log the section totals for debugging
        RCLCPP_INFO(this->get_logger(),
                    "Section totals - Right: %.2f, Front: %.2f, Left: %.2f | Min front: %.2f",
                    total_dist_sec_right, total_dist_sec_front, total_dist_sec_left, min_front_distance);

        RCLCPP_INFO(this->get_logger(), "Service Completed");

        // Decision logic:
        // 1. If front is clear (min distance > 35cm), go forward
        // 2. Otherwise, choose the direction with the largest total distance

        if (min_front_distance > MIN_CLEARANCE) {
            response->direction = "forward";
            RCLCPP_INFO(this->get_logger(), "Decision: FORWARD (front clear: %.2fm)", min_front_distance);
            return;
        }

        // Front is blocked, choose the safest side based on total distances
        if (total_dist_sec_right > total_dist_sec_front &&
            total_dist_sec_right > total_dist_sec_left) {
            response->direction = "right";
            RCLCPP_INFO(this->get_logger(), "Decision: RIGHT (safest: %.2fm)", total_dist_sec_right);
            return;
        }

        if (total_dist_sec_left > total_dist_sec_front &&
            total_dist_sec_left > total_dist_sec_right) {
            response->direction = "left";
            RCLCPP_INFO(this->get_logger(), "Decision: LEFT (safest: %.2fm)", total_dist_sec_left);
            return;
        }

        // Front has largest total, but is below threshold
        // Choose between left and right
        if (total_dist_sec_left > total_dist_sec_right) {
            response->direction = "left";
            RCLCPP_INFO(this->get_logger(), "Decision: LEFT (fallback)");
            return;
        }

        response->direction = "right";
        RCLCPP_INFO(this->get_logger(), "Decision: RIGHT (fallback)");
    }
    rclcpp::Service<robot_patrol::srv::GetDirection>::SharedPtr service_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<DirectionService> node = std::make_shared<DirectionService>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
