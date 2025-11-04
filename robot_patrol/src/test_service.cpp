#include <rclcpp/rclcpp.hpp>
#include <robot_patrol/srv/get_direction.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;
using std::placeholders::_2;
using namespace std::chrono_literals;

class TestService : public rclcpp::Node {
   public:
    TestService() : Node("test_service_node") {
        laser_scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/fastbot_1/scan", 10,
            std::bind(&TestService::laser_scan_callback, this, _1));

        std::string name_service = "/direction_service";
        client_ =
            this->create_client<robot_patrol::srv::GetDirection>(name_service);

        // Wait for the service to be available (checks every second)
        while (!client_->wait_for_service(1s)) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(),
                             "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(),
                        "Service %s not available, waiting again...",
                        name_service.c_str());
        }

        RCLCPP_INFO(this->get_logger(), "Service client Ready");
    }

   private:
    void laser_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        auto request =
            std::make_shared<robot_patrol::srv::GetDirection::Request>();
        request->laser_data = *msg;

        RCLCPP_INFO(this->get_logger(), "Service Request");

        // Send the request asynchronously
        auto result = client_->async_send_request(
            request,
            [this](rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture
                       future_response) {
                auto response = future_response.get();
                RCLCPP_INFO(this->get_logger(), "Service Response: %s",
                            response->direction.c_str());
            });
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_scan_sub_;
    rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    std::shared_ptr<TestService> node = std::make_shared<TestService>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}