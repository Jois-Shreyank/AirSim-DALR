#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/image.hpp"

#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <condition_variable>

class JoyRepublisher : public rclcpp::Node {
public:
    JoyRepublisher()
        : Node("joy_republisher"),
          current_car_name_("") {
        // Get the list of available cars from parameters
        this->declare_parameter<std::vector<std::string>>("available_cars", {"Car_1", "Car_2", "Car_3"});
        this->get_parameter("available_cars", available_cars_);

        // Initialize joystick publishers for each car
        for (const auto& car : available_cars_) {
            std::string joy_topic = "/" + car + "/joy";
            auto joy_publisher = this->create_publisher<sensor_msgs::msg::Joy>(joy_topic, 10);
            car_joy_publishers_[car] = joy_publisher;

            RCLCPP_INFO(this->get_logger(), "Initialized joystick publisher for %s: %s", car.c_str(), joy_topic.c_str());
        }

        // Initialize the single publisher for /pi_cam/image_raw
        pi_cam_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/pi_cam/image_raw", 10);

        // Start the car selection thread
        gui_thread_ = std::thread(&JoyRepublisher::start_gui, this);

        // Create a subscriber to the /exomy/joy topic
        joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/exomy/joy",
            10,
            std::bind(&JoyRepublisher::joy_callback, this, std::placeholders::_1)
        );
    }

    ~JoyRepublisher() {
        if (gui_thread_.joinable()) {
            gui_thread_.join();
        }
    }

private:
    void start_gui() {
        while (rclcpp::ok()) {
            std::cout << "\nAvailable cars:\n";
            for (size_t i = 0; i < available_cars_.size(); ++i) {
                std::cout << i + 1 << ". " << available_cars_[i] << "\n";
            }
            std::cout << "Enter the number of the car to select it (or 0 to quit): ";
            int choice;
            std::cin >> choice;

            if (choice == 0) {
                rclcpp::shutdown();
                break;
            }

            if (choice > 0 && static_cast<size_t>(choice) <= available_cars_.size()) {
                std::lock_guard<std::mutex> lock(car_mutex_);
                std::string selected_car = available_cars_[choice - 1];
                if (selected_car != current_car_name_) {
                    current_car_name_ = selected_car;
                    RCLCPP_INFO(this->get_logger(), "Switched to %s", current_car_name_.c_str());
                    setup_camera_subscription(current_car_name_);
                }
            } else {
                std::cout << "Invalid choice. Please try again.\n";
            }
        }
    }

    void setup_camera_subscription(const std::string& car_name) {
        // Reset the previous subscription
        if (camera_sub_) {
            camera_sub_.reset();
        }

        // Subscribe to the selected car's camera topic
        std::string camera_topic = "/airsim_node/" + car_name + "/camera/Scene";
        camera_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
            camera_topic,
            10,
            std::bind(&JoyRepublisher::image_callback, this, std::placeholders::_1)
        );
        RCLCPP_INFO(this->get_logger(), "Subscribed to %s", camera_topic.c_str());
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(car_mutex_);

        if (current_car_name_.empty()) {
            RCLCPP_INFO(this->get_logger(), "No car selected.");
            return;
        }

        auto it = car_joy_publishers_.find(current_car_name_);
        if (it == car_joy_publishers_.end()) {
            RCLCPP_INFO(this->get_logger(), "No joystick publisher for %s", current_car_name_.c_str());
            return;
        }

        // Publish the joystick message to the selected car's topic
        it->second->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Republished joystick message to %s/joy", current_car_name_.c_str());
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        std::lock_guard<std::mutex> lock(car_mutex_);

        if (current_car_name_.empty()) {
            RCLCPP_INFO(this->get_logger(), "No car selected.");
            return;
        }

        // Publish the image message to the /pi_cam/image_raw topic
        pi_cam_publisher_->publish(*msg);
        RCLCPP_INFO(this->get_logger(), "Republished image message to /pi_cam/image_raw");
    }

    std::vector<std::string> available_cars_;
    std::map<std::string, rclcpp::Publisher<sensor_msgs::msg::Joy>::SharedPtr> car_joy_publishers_;
    std::string current_car_name_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_sub_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pi_cam_publisher_;

    std::thread gui_thread_;
    std::mutex car_mutex_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JoyRepublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
