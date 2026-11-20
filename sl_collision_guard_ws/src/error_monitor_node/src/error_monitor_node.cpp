#include <rclcpp/rclcpp.hpp>
#include <mutex>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <custom_msgs/msg/heart_beat.hpp>
#include <custom_msgs/msg/status_code.hpp>
#include <unordered_map>

constexpr size_t GNSS_ANT_SIZE = 4;

struct NodeHeartbeatInfo {
    rclcpp::Time timestamp;
    uint32_t seq;
    uint16_t status;
};

class ErrorMonitorNode : public rclcpp::Node {
public:
    ErrorMonitorNode() : Node("error_monitor_node") {
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/rslidar_points", 10,
            std::bind(&ErrorMonitorNode::lidar_callback, this, std::placeholders::_1)
        );

        heartbeat_sub_ = this->create_subscription<custom_msgs::msg::HeartBeat>(
            "/heartbeat", 10,
            std::bind(&ErrorMonitorNode::on_heart_beat, this, std::placeholders::_1)
        );

        check_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100),
            std::bind(&ErrorMonitorNode::check_heart_beats, this)
        );

        last_lidar_msg_time_ = clock_->now();
        timeout_sec_ = 1.0;
        std::vector<std::string> expected_nodes_ = {
            "collision_guard_node"
        };
        for (const auto &name : expected_nodes_) {
            NodeHeartbeatInfo info;
            info.timestamp = rclcpp::Time(0,0,this->get_clock()->get_clock_type());
            info.seq = 0;
            info.status = 1;
            heartbeat_map_[name] = info;
            last_seq_map_[name] = 0;
        }

        init_status_msg();
        status_pub_ = this->create_publisher<custom_msgs::msg::StatusCode>("/status_code", 10);
    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        RCLCPP_INFO(this->get_logger(), "接收到 RSLiDAR 数据");
        last_lidar_msg_time_ = clock_->now();
        if (!is_lidar_alive_) {
            is_lidar_alive_ = true;
            RCLCPP_INFO(this->get_logger(), "RSLiDAR 数据恢复正常");
        }
    }

    void on_heart_beat(const custom_msgs::msg::HeartBeat::SharedPtr msg) {
        auto &info = heartbeat_map_[msg->node_name];
        info.timestamp = msg->timestamp;
        info.seq = msg->heartbeat_seq;
        info.status = msg->status_code;
    }

    void check_heart_beats() {
        rclcpp::Time now = clock_->now();
        for (auto &[name, info] : heartbeat_map_) {
            double elapsed = (now - info.timestamp).seconds();
            for (auto &[name, info] : heartbeat_map_) {
                auto it = node_status_map_.find(name);
                if (it == node_status_map_.end()) continue; 
            
                uint16_t *status_field = it->second;
                double elapsed = (now - info.timestamp).seconds();
            
                if (elapsed < 0) { 
                    *status_field = 255;
                } 
                else if (elapsed > timeout_sec_) {
                    *status_field = 1;
                } 
                else if (info.seq == last_seq_map_[name]) { 
                    *status_field = 2;
                } 
                else { 
                    *status_field = 0;
                    if (name == "rtk_gnss_driver_0" || name == "rtk_gnss_driver_1") {
                        *status_field = info.status == 0 ? 0 : 3;
                    } 
                }
            }
        }

        double lidar_elapsed = (now - last_lidar_msg_time_).seconds();
        if (lidar_elapsed > timeout_sec_ && is_lidar_alive_) {
            is_lidar_alive_ = false;
            RCLCPP_ERROR(this->get_logger(), "RSLiDAR 心跳丢失，已超时 %.2f 秒", lidar_elapsed);
        }
        *node_status_map_["lidar_status"] = is_lidar_alive_ ? 0 : 1;

        status_msg_.timestamp = now;
        status_pub_->publish(status_msg_);
    }

    void init_status_msg() {
        status_msg_.timestamp = clock_->now();

        status_msg_.front_lidar_status = 0;
        status_msg_.left_lidar_status = 0;
        status_msg_.right_lidar_status = 0;
        status_msg_.down_lidar_status = 0;
        status_msg_.collision_detection_node_status = 0;
    }

    rclcpp::Clock::SharedPtr clock_ = std::make_shared<rclcpp::Clock>(RCL_ROS_TIME);
    rclcpp::Subscription<custom_msgs::msg::HeartBeat>::SharedPtr heartbeat_sub_;
    rclcpp::TimerBase::SharedPtr check_timer_;
    std::unordered_map<std::string, NodeHeartbeatInfo> heartbeat_map_;
    std::unordered_map<std::string, uint32_t> last_seq_map_;
    rclcpp::Time last_lidar_msg_time_;
    bool is_lidar_alive_ = true;
    double timeout_sec_;

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr lidar_sub_;

    std::array<sensor_msgs::msg::NavSatFix::SharedPtr, GNSS_ANT_SIZE> cur_gnss_data_;
    std::mutex mutex_;

    rclcpp::Publisher<custom_msgs::msg::StatusCode>::SharedPtr status_pub_;
    custom_msgs::msg::StatusCode status_msg_;

    std::unordered_map<std::string, uint16_t*> node_status_map_ = {
        {"front_lidar_status", &status_msg_.front_lidar_status},
        {"left_lidar_status", &status_msg_.left_lidar_status},
        {"right_lidar_status", &status_msg_.right_lidar_status},
        {"down_lidar_status", &status_msg_.down_lidar_status},
        {"collision_detection_node_status", &status_msg_.collision_detection_node_status}
    };
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ErrorMonitorNode>());
    rclcpp::shutdown();
    return 0;
}
