#include <arpa/inet.h>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <modbus/modbus.h>
#include <netinet/in.h>
#include <rclcpp/rclcpp.hpp>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <custom_msgs/msg/collision_detection_enable.hpp>
#include <custom_msgs/msg/collision_detection_result.hpp>
#include <custom_msgs/msg/ship_loader_pose.hpp>
#include <custom_msgs/msg/status_code.hpp>

#include "plc_addr.h"

namespace {

constexpr int kReadRegisterCount = 128;
constexpr int kWriteRegisterCount = 128;

uint32_t float_to_abcd(float value) {
  uint32_t raw = 0;
  std::memcpy(&raw, &value, sizeof(raw));
  return raw;
}

float abcd_to_float(uint16_t high, uint16_t low) {
  const uint32_t raw = (static_cast<uint32_t>(high) << 16) | low;
  float value = 0.0f;
  std::memcpy(&value, &raw, sizeof(value));
  return value;
}

} // namespace

class PlcSimulatorNode : public rclcpp::Node {
public:
  PlcSimulatorNode() : Node("plc_simulator_node") {
    bind_ip_ = declare_parameter<std::string>("bind_ip", "0.0.0.0");
    read_port_ = declare_parameter<int>("read_port", 2105);
    write_port_ = declare_parameter<int>("write_port", 2107);
    update_period_ms_ = declare_parameter<int>("update_period_ms", 100);
    auto_increment_watchdog_ = declare_parameter<bool>("auto_increment_watchdog", true);

    pose_.walking_dist = declare_parameter<double>("initial_pose.walking_dist", 0.0);
    pose_.slewing_angle = declare_parameter<double>("initial_pose.slewing_angle", 0.0);
    pose_.luffing_angle = declare_parameter<double>("initial_pose.luffing_angle", 0.0);
    pose_.stretch_dist = declare_parameter<double>("initial_pose.stretch_dist", 0.0);

    enable_.front_collision_detection_enable =
        declare_parameter<bool>("initial_enable.front", true);
    enable_.left_collision_detection_enable =
        declare_parameter<bool>("initial_enable.left", true);
    enable_.right_collision_detection_enable =
        declare_parameter<bool>("initial_enable.right", true);
    enable_.down_collision_detection_enable =
        declare_parameter<bool>("initial_enable.back", true);

    pose_sub_ = create_subscription<custom_msgs::msg::ShipLoaderPose>(
        "~/input/ship_loader_pose", 10,
        std::bind(&PlcSimulatorNode::ship_loader_pose_callback, this, std::placeholders::_1));

    enable_sub_ = create_subscription<custom_msgs::msg::CollisionDetectionEnable>(
        "~/input/collision_detection_enable", 10,
        std::bind(&PlcSimulatorNode::collision_enable_callback, this, std::placeholders::_1));

    plc_pose_pub_ =
        create_publisher<custom_msgs::msg::ShipLoaderPose>("~/state/ship_loader_pose", 10);
    plc_enable_pub_ = create_publisher<custom_msgs::msg::CollisionDetectionEnable>(
        "~/state/collision_detection_enable", 10);
    status_code_pub_ = create_publisher<custom_msgs::msg::StatusCode>("~/state/status_code", 10);
    collision_result_pub_ = create_publisher<custom_msgs::msg::CollisionDetectionResult>(
        "~/state/collision_detection_result", 10);

    update_timer_ = create_wall_timer(
        std::chrono::milliseconds(update_period_ms_),
        std::bind(&PlcSimulatorNode::update_and_publish_state, this));

    running_.store(true);
    read_server_thread_ = std::thread(&PlcSimulatorNode::server_loop, this, read_port_, true);
    write_server_thread_ = std::thread(&PlcSimulatorNode::server_loop, this, write_port_, false);

    RCLCPP_INFO(get_logger(), "PLC simulator listening on %s, read_port=%d, write_port=%d",
                bind_ip_.c_str(), read_port_, write_port_);
  }

  ~PlcSimulatorNode() override {
    running_.store(false);
    shutdown_server(read_server_socket_);
    shutdown_server(write_server_socket_);

    if (read_server_thread_.joinable()) {
      read_server_thread_.join();
    }
    if (write_server_thread_.joinable()) {
      write_server_thread_.join();
    }
  }

private:
  void ship_loader_pose_callback(const custom_msgs::msg::ShipLoaderPose::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    pose_ = *msg;
    write_read_mapping_locked();
  }

  void collision_enable_callback(
      const custom_msgs::msg::CollisionDetectionEnable::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    enable_ = *msg;
    write_read_mapping_locked();
  }

  void update_and_publish_state() {
    custom_msgs::msg::ShipLoaderPose pose_msg;
    custom_msgs::msg::CollisionDetectionEnable enable_msg;
    custom_msgs::msg::StatusCode status_msg;
    custom_msgs::msg::CollisionDetectionResult result_msg;

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (auto_increment_watchdog_) {
        ++watchdog_count_;
      }
      write_read_mapping_locked();
      read_write_mapping_locked();

      pose_msg = pose_;
      enable_msg = enable_;
      status_msg = status_code_;
      result_msg = collision_result_;
    }

    const auto now = this->now();
    pose_msg.timestamp = now;
    enable_msg.timestamp = now;
    status_msg.timestamp = now;
    result_msg.timestamp = now;

    plc_pose_pub_->publish(pose_msg);
    plc_enable_pub_->publish(enable_msg);
    status_code_pub_->publish(status_msg);
    collision_result_pub_->publish(result_msg);
  }

  void write_read_mapping_locked() {
    if (read_mapping_ == nullptr) {
      return;
    }

    const uint16_t flags =
        (static_cast<uint16_t>(enable_.front_collision_detection_enable) << P2C_FRONT_ZONE_ENABLE_BIT) |
        (static_cast<uint16_t>(enable_.left_collision_detection_enable) << P2C_LEFT_ZONE_ENABLE_BIT) |
        (static_cast<uint16_t>(enable_.right_collision_detection_enable) << P2C_RIGHT_ZONE_ENABLE_BIT) |
        (static_cast<uint16_t>(enable_.down_collision_detection_enable) << P2C_BACK_ZONE_ENABLE_BIT);
    read_mapping_->tab_registers[P2C_COLLISION_DETECTION_ENABLE_OFFSET] =
        static_cast<uint16_t>(flags << 8);

    read_mapping_->tab_registers[P2C_WATCHDOG_COUNTER_OFFSET] =
        static_cast<uint16_t>((watchdog_count_ >> 16) & 0xFFFF);
    read_mapping_->tab_registers[P2C_WATCHDOG_COUNTER_OFFSET + 1] =
        static_cast<uint16_t>(watchdog_count_ & 0xFFFF);

    write_float_to_mapping(read_mapping_, P2C_WALKING_POSITION_OFFSET,
                           static_cast<float>(pose_.walking_dist));
    write_float_to_mapping(read_mapping_, P2C_SLEWING_POSITION_OFFSET,
                           static_cast<float>(pose_.slewing_angle));
    write_float_to_mapping(read_mapping_, P2C_LUFFING_POSITION_OFFSET,
                           static_cast<float>(pose_.luffing_angle));
    write_float_to_mapping(read_mapping_, P2C_STRETCH_POSITION_OFFSET,
                           static_cast<float>(pose_.stretch_dist));
  }

  void read_write_mapping_locked() {
    if (write_mapping_ == nullptr) {
      return;
    }

    status_code_.front_lidar_status =
        write_mapping_->tab_registers[C2P_STATUS_CODE_OFFSET + C2P_FRONT_LIDAR_STATUS_OFFSET];
    status_code_.left_lidar_status =
        write_mapping_->tab_registers[C2P_STATUS_CODE_OFFSET + C2P_LEFT_LIDAR_STATUS_OFFSET];
    status_code_.right_lidar_status =
        write_mapping_->tab_registers[C2P_STATUS_CODE_OFFSET + C2P_RIGHT_LIDAR_STATUS_OFFSET];
    status_code_.down_lidar_status =
        write_mapping_->tab_registers[C2P_STATUS_CODE_OFFSET + C2P_BACK_LIDAR_STATUS_OFFSET];
    status_code_.collision_detection_node_status = write_mapping_
        ->tab_registers[C2P_STATUS_CODE_OFFSET + C2P_COLLISION_DETECTION_NODE_STATUS_OFFSET];

    collision_result_.front_zone_status = write_mapping_->tab_registers[C2P_FRONT_ZONE_STATUS_OFFSET];
    collision_result_.left_zone_status = write_mapping_->tab_registers[C2P_LEFT_ZONE_STATUS_OFFSET];
    collision_result_.right_zone_status = write_mapping_->tab_registers[C2P_RIGHT_ZONE_STATUS_OFFSET];
    collision_result_.down_zone_status = write_mapping_->tab_registers[C2P_BACK_ZONE_STATUS_OFFSET];

    collision_result_.front_zone_collision_dist =
        read_float_from_mapping(write_mapping_, C2P_FRONT_ZONE_DISTANCE_OFFSET);
    collision_result_.left_zone_collision_dist =
        read_float_from_mapping(write_mapping_, C2P_LEFT_ZONE_DISTANCE_OFFSET);
    collision_result_.right_zone_collision_dist =
        read_float_from_mapping(write_mapping_, C2P_RIGHT_ZONE_DISTANCE_OFFSET);
    collision_result_.down_zone_collision_dist =
        read_float_from_mapping(write_mapping_, C2P_BACK_ZONE_DISTANCE_OFFSET);
  }

  static void write_float_to_mapping(modbus_mapping_t *mapping, int offset, float value) {
    const uint32_t raw = float_to_abcd(value);
    mapping->tab_registers[offset] = static_cast<uint16_t>((raw >> 16) & 0xFFFF);
    mapping->tab_registers[offset + 1] = static_cast<uint16_t>(raw & 0xFFFF);
  }

  static float read_float_from_mapping(modbus_mapping_t *mapping, int offset) {
    return abcd_to_float(mapping->tab_registers[offset], mapping->tab_registers[offset + 1]);
  }

  void server_loop(int port, bool is_read_server) {
    modbus_t *ctx = modbus_new_tcp(bind_ip_.c_str(), port);
    if (ctx == nullptr) {
      RCLCPP_ERROR(get_logger(), "Failed to create modbus context for port %d", port);
      return;
    }

    timeval response_timeout{};
    response_timeout.tv_sec = 0;
    response_timeout.tv_usec = 200000;
    modbus_set_response_timeout(ctx, response_timeout.tv_sec, response_timeout.tv_usec);

    modbus_mapping_t *mapping = modbus_mapping_new(0, 0,
                                                   is_read_server ? kReadRegisterCount : kWriteRegisterCount,
                                                   0);
    if (mapping == nullptr) {
      RCLCPP_ERROR(get_logger(), "Failed to create modbus mapping for port %d", port);
      modbus_free(ctx);
      return;
    }

    {
      std::lock_guard<std::mutex> lock(state_mutex_);
      if (is_read_server) {
        read_mapping_ = mapping;
        write_read_mapping_locked();
      } else {
        write_mapping_ = mapping;
      }
    }

    int server_socket = modbus_tcp_listen(ctx, 1);
    if (server_socket == -1) {
      RCLCPP_ERROR(get_logger(), "Failed to listen on port %d: %s", port, modbus_strerror(errno));
      clear_mapping(is_read_server, mapping);
      modbus_mapping_free(mapping);
      modbus_free(ctx);
      return;
    }

    if (is_read_server) {
      read_server_socket_ = server_socket;
    } else {
      write_server_socket_ = server_socket;
    }

    std::vector<uint8_t> query(MODBUS_TCP_MAX_ADU_LENGTH);
    while (running_.load()) {
      int client_socket = modbus_tcp_accept(ctx, &server_socket);
      if (client_socket == -1) {
        if (running_.load()) {
          RCLCPP_WARN(get_logger(), "Accept failed on port %d: %s", port, modbus_strerror(errno));
        }
        continue;
      }

      RCLCPP_INFO(get_logger(), "PLC simulator accepted client on port %d", port);

      while (running_.load()) {
        const int rc = modbus_receive(ctx, query.data());
        if (rc > 0) {
          std::lock_guard<std::mutex> lock(state_mutex_);
          if (is_read_server) {
            write_read_mapping_locked();
          }
          const int reply_rc = modbus_reply(ctx, query.data(), rc, mapping);
          if (reply_rc == -1) {
            if (running_.load()) {
              RCLCPP_WARN(get_logger(), "Reply failed on port %d: %s", port,
                          modbus_strerror(errno));
            }
            break;
          }
          if (!is_read_server) {
            read_write_mapping_locked();
          }
          continue;
        }

        if (rc == -1 && running_.load()) {
          RCLCPP_INFO(get_logger(), "Client disconnected from port %d", port);
        }
        break;
      }

      modbus_set_socket(ctx, -1);
      ::close(client_socket);
    }

    clear_mapping(is_read_server, mapping);
    modbus_mapping_free(mapping);
    if (is_read_server) {
      read_server_socket_ = -1;
    } else {
      write_server_socket_ = -1;
    }
    modbus_free(ctx);
  }

  void clear_mapping(bool is_read_server, modbus_mapping_t *mapping) {
    std::lock_guard<std::mutex> lock(state_mutex_);
    if (is_read_server && read_mapping_ == mapping) {
      read_mapping_ = nullptr;
    }
    if (!is_read_server && write_mapping_ == mapping) {
      write_mapping_ = nullptr;
    }
  }

  static void shutdown_server(int server_socket) {
    if (server_socket >= 0) {
      ::shutdown(server_socket, SHUT_RDWR);
      ::close(server_socket);
    }
  }

  std::string bind_ip_;
  int read_port_ = 2105;
  int write_port_ = 2107;
  int update_period_ms_ = 100;
  bool auto_increment_watchdog_ = true;

  std::atomic<bool> running_{false};
  std::thread read_server_thread_;
  std::thread write_server_thread_;
  int read_server_socket_ = -1;
  int write_server_socket_ = -1;

  std::mutex state_mutex_;
  modbus_mapping_t *read_mapping_ = nullptr;
  modbus_mapping_t *write_mapping_ = nullptr;
  uint32_t watchdog_count_ = 0;

  custom_msgs::msg::ShipLoaderPose pose_;
  custom_msgs::msg::CollisionDetectionEnable enable_;
  custom_msgs::msg::StatusCode status_code_;
  custom_msgs::msg::CollisionDetectionResult collision_result_;

  rclcpp::Subscription<custom_msgs::msg::ShipLoaderPose>::SharedPtr pose_sub_;
  rclcpp::Subscription<custom_msgs::msg::CollisionDetectionEnable>::SharedPtr enable_sub_;
  rclcpp::Publisher<custom_msgs::msg::ShipLoaderPose>::SharedPtr plc_pose_pub_;
  rclcpp::Publisher<custom_msgs::msg::CollisionDetectionEnable>::SharedPtr plc_enable_pub_;
  rclcpp::Publisher<custom_msgs::msg::StatusCode>::SharedPtr status_code_pub_;
  rclcpp::Publisher<custom_msgs::msg::CollisionDetectionResult>::SharedPtr collision_result_pub_;
  rclcpp::TimerBase::SharedPtr update_timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlcSimulatorNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
