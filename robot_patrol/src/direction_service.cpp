#include <cmath>
#include <csignal>
#include <geometry_msgs/msg/twist.hpp>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <robot_patrol_msg/srv/get_direction.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

class DirectionService : public rclcpp::Node {
   public:
    DirectionService() : Node("direction_service_node") {
        std::string name_service = "/direction_service";
        service_ = this->create_service<robot_patrol_msg::srv::GetDirection>(
            name_service, std::bind(&DirectionService::direction_service, this,
                                    std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "%s Service Server Ready...", name_service.c_str());
    }

   private:
    void direction_service(const std::shared_ptr<robot_patrol_msg::srv::GetDirection::Request> request,
                           std::shared_ptr<robot_patrol_msg::srv::GetDirection::Response> response) {
        RCLCPP_INFO(this->get_logger(), "Received direction request");

        // Get the laser scan data from the request
        auto laser_data = request->laser_data;

        // Laser scan covers front 180 degrees: from -π/2 to +π/2
        // Divide into 3 sections of 60° each:
        // Right:  -π/2 to -π/6  (-90° to -30°)
        // Front:  -π/6 to +π/6  (-30° to +30°)
        // Left:   +π/6 to +π/2  (+30° to +90°)

        const double RIGHT_MIN = -M_PI / 2.0;  // -90°
        const double RIGHT_MAX = -M_PI / 6.0;  // -30°
        const double FRONT_MIN = -M_PI / 6.0;  // -30°
        const double FRONT_MAX = M_PI / 6.0;   // +30°
        const double LEFT_MIN = M_PI / 6.0;    // +30°
        const double LEFT_MAX = M_PI / 2.0;    // +90°

        const double MIN_CLEARANCE = 0.35;  // 35 cm threshold

        // Initialize distance totals for each section
        double total_dist_sec_right = 0.0;
        double total_dist_sec_front = 0.0;
        double total_dist_sec_left = 0.0;

        // Track minimum distance in front section for threshold check
        double min_front_distance = std::numeric_limits<double>::max();

        // Sum up distances for each section
        for (size_t i = 0; i < laser_data.ranges.size(); ++i) {
            // Calculate the angle for this ray
            float angle = laser_data.angle_min + (i * laser_data.angle_increment);
            float distance = laser_data.ranges[i];

            // Skip invalid readings (nan, inf, or out of range)
            if (std::isnan(distance) || std::isinf(distance) ||
                distance < laser_data.range_min || distance > laser_data.range_max) {
                continue;
            }

            // Determine which section this ray belongs to and sum distances
            if (angle >= RIGHT_MIN && angle < RIGHT_MAX) {
                // Right section
                total_dist_sec_right += distance;
                continue;
            }

            if (angle >= FRONT_MIN && angle < FRONT_MAX) {
                // Front section
                total_dist_sec_front += distance;
                min_front_distance = std::min(min_front_distance, static_cast<double>(distance));
                continue;
            }

            if (angle >= LEFT_MIN && angle <= LEFT_MAX) {
                // Left section
                total_dist_sec_left += distance;
            }
        }

        // Log the section totals for debugging
        RCLCPP_INFO(this->get_logger(),
                    "Section totals - Right: %.2f, Front: %.2f, Left: %.2f | Min front: %.2f",
                    total_dist_sec_right, total_dist_sec_front, total_dist_sec_left, min_front_distance);

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
    rclcpp::Service<robot_patrol_msg::srv::GetDirection>::SharedPtr service_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    std::shared_ptr<DirectionService> node = std::make_shared<DirectionService>();

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
