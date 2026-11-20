#pragma once

#include "plc_addr.h"
#include <modbus/modbus.h>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct ShipLoaderPose {
  float walking_dist;  // 大车行走距离
  float slewing_angle; // 大臂回转角度
  float luffing_angle; // 大臂俯仰距离
  float stretch_dist;  // 小臂伸缩距离

  ShipLoaderPose()
      : walking_dist(0.0f), slewing_angle(0.0f), luffing_angle(0.0f),
        stretch_dist(0.0f) {}

  ShipLoaderPose(float w, float s, float l, float st)
      : walking_dist(w), slewing_angle(s), luffing_angle(l), stretch_dist(st) {}
};

struct StatusCode {
  uint16_t front_lidar_status;              // 前侧激光雷达状态
  uint16_t left_lidar_status;               // 左侧激光雷达状态
  uint16_t right_lidar_status;              // 右侧激光雷达状态
  uint16_t down_lidar_status;               // 下侧激光雷达状态
  uint16_t collision_detection_node_status; // 碰撞检测节点状态

  StatusCode()
      : front_lidar_status(0), left_lidar_status(0), right_lidar_status(0),
        down_lidar_status(0), collision_detection_node_status(0) {}

  StatusCode(uint16_t fls, uint16_t lfs, uint16_t rls, uint16_t dls,
             uint16_t cdns)
      : front_lidar_status(fls), left_lidar_status(lfs),
        right_lidar_status(rls), down_lidar_status(dls),
        collision_detection_node_status(cdns) {}
};

struct CollisionDetectionEnable {
  bool front_collision_detection_enable; // 前侧碰撞区使能
  bool left_collision_detection_enable;  // 左侧碰撞区使能
  bool right_collision_detection_enable; // 右侧碰撞区使能
  bool down_collision_detection_enable;  // 下侧碰撞区使能

  CollisionDetectionEnable()
      : front_collision_detection_enable(false),
        left_collision_detection_enable(false),
        right_collision_detection_enable(false),
        down_collision_detection_enable(false) {}

  CollisionDetectionEnable(bool fce, bool lce, bool rce, bool dce)
      : front_collision_detection_enable(fce),
        left_collision_detection_enable(lce),
        right_collision_detection_enable(rce),
        down_collision_detection_enable(dce) {}
};

struct CollisionDetectionResult {
  uint16_t front_zone_status;       // 前侧碰撞区状态
  uint16_t left_zone_status;        // 左侧碰撞区状态
  uint16_t right_zone_status;       // 右侧碰撞区状态
  uint16_t down_zone_status;        // 下侧碰撞区状态
  double front_zone_collision_dist; // 前侧碰撞区距离
  double left_zone_collision_dist;  // 左侧碰撞区距离
  double right_zone_collision_dist; // 右侧碰撞区距离
  double down_zone_collision_dist;  // 下侧碰撞区距离

  CollisionDetectionResult()
      : front_zone_status(0), left_zone_status(0), right_zone_status(0),
        down_zone_status(0), front_zone_collision_dist(0.0),
        left_zone_collision_dist(0.0), right_zone_collision_dist(0.0),
        down_zone_collision_dist(0.0) {}

  CollisionDetectionResult(uint16_t fzs, uint16_t lzs, uint16_t rzs,
                           uint16_t bzs, double fzd, double lzd, double rzd,
                           double bzd)
      : front_zone_status(fzs), left_zone_status(lzs), right_zone_status(rzs),
        down_zone_status(bzs), front_zone_collision_dist(fzd),
        left_zone_collision_dist(lzd), right_zone_collision_dist(rzd),
        down_zone_collision_dist(bzd) {}
};

class ModbusClient {
public:
  ModbusClient();
  ~ModbusClient();

  bool init(const std::string &ip, const unsigned int port);
  bool connect();
  void close();
  bool open_listen_thread();
  void set_read_register_count(int count) { read_register_count_ = count; }
  uint32_t get_watchdog_count();
  ShipLoaderPose get_ship_loader_pose();
  CollisionDetectionEnable get_collision_detection_enable();

  bool read_holding_registers(int addr, int nb, std::vector<uint16_t> &dest);
  bool read_input_registers(int addr, int nb, std::vector<uint16_t> &dest);
  bool write_holding_register(int addr, uint16_t value);
  bool write_holding_registers(int addr, int nb,
                               const std::vector<uint16_t> &value);
  bool write_holding_bit(int addr, int bit, bool value);

  bool set_watchdog_count(const uint32_t &watchdog_count);
  bool set_status_code(const StatusCode &status_code);
  bool set_collision_detection_result(const CollisionDetectionResult &result);
  bool flush_output_registers();

  bool is_connected_;

private:
  static unsigned int listen_thread(void *arg);
  void read_plc_info();
  bool flush_output_registers_locked();
  bool reconnect_locked();
  void mark_disconnected();

  std::string ip_;
  int port_;
  modbus_t *ctx_;
  int read_register_count_;
  ShipLoaderPose ship_loader_pose_;
  CollisionDetectionEnable collision_detection_enable_;
  std::thread *listen_thread_;
  std::mutex mutex_;
  bool is_exit_;
  uint32_t watchdog_count_;
  uint32_t output_watchdog_count_ = 0;
  StatusCode output_status_code_;
  CollisionDetectionResult output_collision_result_;
};
