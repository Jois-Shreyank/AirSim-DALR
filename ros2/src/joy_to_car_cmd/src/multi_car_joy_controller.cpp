#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "airsim_interfaces/msg/car_controls.hpp"

#include <map>
#include <vector>
#include <string>

class MultiCarJoyController : public rclcpp::Node {
public:
    MultiCarJoyController()
        : Node("multi_car_joy_controller") {
        // Declare and get the parameter for vehicle names
        this->declare_parameter<std::vector<std::string>>("vehicle_names", {"Car_1", "Car_2", "Car_3"});
        this->get_parameter("vehicle_names", vehicle_names_);

        // Initialize publishers and subscribers for each vehicle
        for (const auto& vehicle : vehicle_names_) {
            // Create a publisher for car_cmd
            std::string car_cmd_topic = "/airsim_node/" + vehicle + "/car_cmd";
            auto car_cmd_publisher = this->create_publisher<airsim_interfaces::msg::CarControls>(car_cmd_topic, 10);
            vehicle_publishers_[vehicle] = car_cmd_publisher;

            // Create a subscriber for joy
            std::string joy_topic = "/" + vehicle + "/joy";
            auto joy_subscriber = this->create_subscription<sensor_msgs::msg::Joy>(
                joy_topic,
                10,
                [this, vehicle](sensor_msgs::msg::Joy::SharedPtr msg) { this->joy_callback(msg, vehicle); });
            joy_subscribers_[vehicle] = joy_subscriber;

            RCLCPP_INFO(this->get_logger(), "Initialized subscriber for %s on topic %s", vehicle.c_str(), joy_topic.c_str());
            RCLCPP_INFO(this->get_logger(), "Initialized publisher for %s on topic %s", vehicle.c_str(), car_cmd_topic.c_str());
        }

        // Initialize camera subscriber and image publisher
        camera_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/airsim_node/Car_1/camera/Scene",
            10,
            std::bind(&MultiCarJoyController::camera_callback, this, std::placeholders::_1));
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/pi_cam/image_raw", 10);

        RCLCPP_INFO(this->get_logger(), "Initialized camera subscriber and image publisher.");
    }

private:
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr joy_msg, const std::string& vehicle) {
        auto controls = airsim_interfaces::msg::CarControls();

        // Map joystick axes to car controls
        controls.steering = -joy_msg->axes[0]; // Invert steering if necessary

        float throttle_brake = joy_msg->axes[1]; // Adjust index if necessary
        if (throttle_brake > 0.0) {
            // Forward
            controls.throttle = throttle_brake;
            controls.brake = 0.0;
            controls.manual = true;
            controls.manual_gear = 1;
            controls.gear_immediate = true;
        } else if (throttle_brake < 0.0) {
            // Reverse
            controls.throttle = -throttle_brake;
            controls.brake = 0.0;
            controls.manual = true;
            controls.manual_gear = -1;
            controls.gear_immediate = true;
        } else {
            // Brake
            controls.throttle = 0.0;
            controls.brake = 1.0;
            controls.manual = false;
        }

        controls.header.stamp = this->get_clock()->now();

        // Publish car controls to the appropriate topic
        vehicle_publishers_[vehicle]->publish(controls);
        RCLCPP_INFO(this->get_logger(), "Published car controls for %s", vehicle.c_str());
    }

    void camera_callback(const sensor_msgs::msg::Image::SharedPtr img_msg) {
        // Republish the image to /pi_cam/image_raw
        image_publisher_->publish(*img_msg);
        RCLCPP_INFO(this->get_logger(), "Republished image from Car_1 camera to /pi_cam/image_raw");
    }

    std::vector<std::string> vehicle_names_;
    std::map<std::string, rclcpp::Publisher<airsim_interfaces::msg::CarControls>::SharedPtr> vehicle_publishers_;
    std::map<std::string, rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr> joy_subscribers_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MultiCarJoyController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
