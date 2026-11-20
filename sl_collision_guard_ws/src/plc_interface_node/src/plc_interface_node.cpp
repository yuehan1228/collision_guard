#include <rclcpp/rclcpp.hpp>
#include <stdexcept>
#include <geometry_msgs/msg/pose_array.hpp>
#include <custom_msgs/msg/ship_loader_pose.hpp>
#include <custom_msgs/msg/status_code.hpp>
#include <custom_msgs/msg/collision_detection_enable.hpp>
#include <custom_msgs/msg/collision_detection_result.hpp>
#include "modbus_client.h"

class PlcInterfaceNode : public rclcpp::Node {
public:
    PlcInterfaceNode() : Node("plc_interface_node"), plc_read_client_(nullptr), plc_write_client_(nullptr) {
        plc_ip_ = this->declare_parameter<std::string>("plc_ip", "172.26.216.31");
        plc_read_port_ = this->declare_parameter<int>("plc_read_port", 502);
        plc_write_port_ = this->declare_parameter<int>("plc_write_port", 502);
        read_register_count_ = this->declare_parameter<int>("read_register_count", 20);
        RCLCPP_INFO_STREAM(this->get_logger(), "PLC IP: " << plc_ip_);
        RCLCPP_INFO_STREAM(this->get_logger(), "PLC Read Port: " << plc_read_port_);
        RCLCPP_INFO_STREAM(this->get_logger(), "PLC Write Port: " << plc_write_port_);
        RCLCPP_INFO_STREAM(this->get_logger(), "Read Register Count: " << read_register_count_);
        plc_read_client_ = new ModbusClient();
        plc_write_client_ = new ModbusClient();
        if (!plc_read_client_->init(plc_ip_, plc_read_port_)) {
            throw std::runtime_error("Failed to initialize Modbus read client");
        }
        if (!plc_write_client_->init(plc_ip_, plc_write_port_)) {
            throw std::runtime_error("Failed to initialize Modbus write client");
        }
        if (!plc_read_client_->connect()) { 
            throw std::runtime_error("Failed to connect Modbus read client");
        }
        if (!plc_write_client_->connect()) {
            throw std::runtime_error("Failed to connect Modbus write client");
        }
        if (!plc_read_client_->open_listen_thread()) {  
            throw std::runtime_error("Failed to open read client listen thread");
        }

        RCLCPP_INFO_STREAM(this->get_logger(), "Connect to PLC success");

        plc_read_client_->set_read_register_count(read_register_count_);

        ship_loader_pose_pub_ = this->create_publisher<custom_msgs::msg::ShipLoaderPose>("/plc/ship_loader_pos", 10);

        pub_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        sub_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PlcInterfaceNode::publish_plc_info, this),
            pub_callback_group_
        );
        write_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&PlcInterfaceNode::flush_plc_outputs, this),
            pub_callback_group_
        );

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = sub_callback_group_;
        status_code_sub_ = this->create_subscription<custom_msgs::msg::StatusCode>(
            "/cpc/status_code", 10,
            std::bind(&PlcInterfaceNode::status_code_callback, this, std::placeholders::_1),
            sub_options
        );

        collision_detection_result_sub_ = this->create_subscription<custom_msgs::msg::CollisionDetectionResult>(
            "/cpc/collision_detection_result", 10,
            std::bind(&PlcInterfaceNode::collision_detection_result_callback, this, std::placeholders::_1),
            sub_options
        );
    }

    ~PlcInterfaceNode() {
        if (plc_read_client_ != nullptr) {
            plc_read_client_->close();
            delete plc_read_client_;
            plc_read_client_ = nullptr;
        }
        if (plc_write_client_ != nullptr) {
            plc_write_client_->close();
            delete plc_write_client_;
            plc_write_client_ = nullptr;
        }
    }

private:
    void publish_plc_info() {
        publish_watchdog_count();
        publish_ship_loader_pose();
    }

    void publish_watchdog_count() {
        uint32_t count = plc_read_client_->get_watchdog_count();
        plc_write_client_->set_watchdog_count(count);
    }

    void publish_ship_loader_pose() {
        ShipLoaderPose pose = plc_read_client_->get_ship_loader_pose();
        custom_msgs::msg::ShipLoaderPose msg;
        msg.walking_dist = pose.walking_dist;
        msg.luffing_angle = pose.luffing_angle;
        msg.slewing_angle = pose.slewing_angle;
        msg.stretch_dist = pose.stretch_dist;
        msg.timestamp = this->now();
        ship_loader_pose_pub_->publish(msg);
    }

    void flush_plc_outputs() {
        if (!plc_write_client_->flush_output_registers()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to flush PLC output registers");
        }
    }

    void status_code_callback(const custom_msgs::msg::StatusCode::SharedPtr msg) {
        StatusCode status;
        status.front_lidar_status = msg->front_lidar_status;
        status.left_lidar_status = msg->left_lidar_status;
        status.right_lidar_status = msg->right_lidar_status;
        status.down_lidar_status = msg->down_lidar_status;
        status.collision_detection_node_status = msg->collision_detection_node_status;

        if (!plc_write_client_->set_status_code(status)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set status code");
        }
    }

    void collision_detection_result_callback(const custom_msgs::msg::CollisionDetectionResult::SharedPtr msg) {
        CollisionDetectionResult result;
        result.front_zone_status = msg->front_zone_status;
        result.left_zone_status = msg->left_zone_status;
        result.right_zone_status = msg->right_zone_status;
        result.down_zone_status = msg->down_zone_status;
        result.front_zone_collision_dist = msg->front_zone_collision_dist;
        result.left_zone_collision_dist = msg->left_zone_collision_dist;
        result.right_zone_collision_dist= msg->right_zone_collision_dist;
        result.down_zone_collision_dist = msg->down_zone_collision_dist;
        if (!plc_write_client_->set_collision_detection_result(result)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set collision detection result");
        }
    }

    ModbusClient *plc_read_client_;
    ModbusClient *plc_write_client_;
    std::string plc_ip_;
    int plc_read_port_;
    int plc_write_port_;
    int read_register_count_;

    rclcpp::Publisher<custom_msgs::msg::ShipLoaderPose>::SharedPtr ship_loader_pose_pub_;
    rclcpp::Publisher<custom_msgs::msg::CollisionDetectionEnable>::SharedPtr collision_detection_enable_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr write_timer_;
    rclcpp::Subscription<custom_msgs::msg::StatusCode>::SharedPtr status_code_sub_;
    rclcpp::Subscription<custom_msgs::msg::CollisionDetectionResult>::SharedPtr collision_detection_result_sub_;
    rclcpp::CallbackGroup::SharedPtr pub_callback_group_;
    rclcpp::CallbackGroup::SharedPtr sub_callback_group_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<PlcInterfaceNode>();
        rclcpp::executors::MultiThreadedExecutor executor;
        executor.add_node(node);
        executor.spin();
    } catch (const std::exception &e) {
        RCLCPP_FATAL(rclcpp::get_logger("plc_interface_node"), "%s", e.what());
    }
    rclcpp::shutdown();
    return 0;
}
