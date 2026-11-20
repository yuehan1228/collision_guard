#include "modbus_client.h"
#include <chrono>
#include <cmath>
#include <cstring>
#include <iostream>

namespace {

void encode_float_abcd(float value, uint16_t &high, uint16_t &low) {
    uint32_t raw = 0;
    std::memcpy(&raw, &value, sizeof(raw));
    high = static_cast<uint16_t>((raw >> 16) & 0xFFFF);
    low = static_cast<uint16_t>(raw & 0xFFFF);
}

float round_to_3_decimals(double value) {
    return static_cast<float>(std::round(value * 1000.0) / 1000.0);
}

} // namespace

ModbusClient::ModbusClient()
    : ctx_(nullptr), is_connected_(false), listen_thread_(nullptr), is_exit_(false), watchdog_count_(0) {
}

ModbusClient::~ModbusClient() {
    close();
    if (ctx_) {
        modbus_free(ctx_);
    }
    if (listen_thread_ != nullptr) {
        is_exit_ = true;
        if (listen_thread_->joinable()) {
            listen_thread_->join();
        }
        delete listen_thread_;
        listen_thread_ = nullptr;
    }
}

bool ModbusClient::init(const std::string& ip, const unsigned int port) {
    ctx_ = modbus_new_tcp(ip.c_str(), port);
    if (ctx_ == nullptr) {
        std::cerr << "[ERROR] Unable to allocate libmodbus context\n";
        return false;
    }
    timeval response_timeout{};
    response_timeout.tv_sec = 0;
    response_timeout.tv_usec = 200000;
    modbus_set_response_timeout(ctx_, response_timeout.tv_sec, response_timeout.tv_usec);
    modbus_set_error_recovery(ctx_, static_cast<modbus_error_recovery_mode>(
                                        MODBUS_ERROR_RECOVERY_LINK |
                                        MODBUS_ERROR_RECOVERY_PROTOCOL));
    return true;
}

bool ModbusClient::connect() {
    if (ctx_ == nullptr) return false;
    int rc = modbus_set_slave(ctx_, 1);
    if (rc == -1) {
        std::cerr << "Invalid slave ID" << std::endl;
        modbus_free(ctx_);
        return -1;
    }
    if (modbus_connect(ctx_) == -1) {
        std::cerr << "Connection failed: " << modbus_strerror(errno) << "\n";
        return false;
    }
    is_connected_ = true;
    std::cout << "[INFO] Modbus connected to " << ip_ << ":" << port_ << std::endl;
    return true;
}

void ModbusClient::close() {
    is_exit_ = true;
    if (is_connected_) {
        modbus_close(ctx_);
        is_connected_ = false;
    }
}

uint32_t ModbusClient::get_watchdog_count() {
    std::lock_guard<std::mutex> lock(mutex_);
    return watchdog_count_;
}

ShipLoaderPose ModbusClient::get_ship_loader_pose()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return ship_loader_pose_;
}

CollisionDetectionEnable ModbusClient::get_collision_detection_enable()
{
    std::lock_guard<std::mutex> lock(mutex_);
    return collision_detection_enable_;
}

bool ModbusClient::read_holding_registers(int addr, int nb, std::vector<uint16_t>& dest) {
    if (!is_connected_) return false;
    dest.resize(nb);
    int rc = modbus_read_registers(ctx_, addr, nb, dest.data());
    if (rc == -1) {
        std::cerr << "Read failed: " << modbus_strerror(errno) << "\n";
        mark_disconnected();
        return false;
    }
    return true;
}

bool ModbusClient::read_input_registers(int addr, int nb, std::vector<uint16_t>& dest) {
    if (!is_connected_) return false;
    dest.resize(nb);
    int rc = modbus_read_input_registers(ctx_, addr, nb, dest.data());
    if (rc == -1) {
        std::cerr << "Read failed: " << modbus_strerror(errno) << "\n";
        mark_disconnected();
        return false;
    }
    return true;
}

bool ModbusClient::write_holding_register(int addr, uint16_t value) {
    if (!is_connected_) return false;
    int rc = modbus_write_register(ctx_, addr, value);
    if (rc == -1) {
        std::cerr << "Write failed: " << modbus_strerror(errno) << "\n";
        mark_disconnected();
        return false;
    }
    return true;
}

bool ModbusClient::write_holding_registers(int addr, int nb, const std::vector<uint16_t>& value) {
    if (!is_connected_) return false;
    int rc = modbus_write_registers(ctx_, addr, nb, value.data());
    if (rc == -1) {
        std::cerr << "Write failed: " << modbus_strerror(errno) << "\n";
        mark_disconnected();
        return false;
    }
    return true;
}

bool ModbusClient::write_holding_bit(int addr, int bit, bool value) {
    if (!is_connected_) return false;
    std::vector<uint16_t> reg(1);
    if (read_holding_registers(addr, 1, reg)) {
        if (value)
            reg[0] |= (1 << bit);
        else
            reg[0] &= ~(1 << bit);

        if (write_holding_registers(addr, 1, reg)) {
            return true;
        } else {
            std::cerr << "Write failed: " << modbus_strerror(errno) << "\n";
            return false;
        }
    } else {
        std::cerr << "Read failed: " << modbus_strerror(errno) << "\n";
        return false;
    }
}

bool ModbusClient::set_watchdog_count(const uint32_t &watchdog_count) {
    std::lock_guard<std::mutex> lock(mutex_);
    output_watchdog_count_ = watchdog_count + 1;
    return true;
}

bool ModbusClient::set_status_code(const StatusCode &status_code)
{
    std::lock_guard<std::mutex> lock(mutex_);
    output_status_code_ = status_code;
    if (!is_connected_) return false;
    return true;
}

bool ModbusClient::set_collision_detection_result(const CollisionDetectionResult &result)
{
    std::lock_guard<std::mutex> lock(mutex_);
    output_collision_result_ = result;
    if (!is_connected_) return false;
    return true;
}

bool ModbusClient::flush_output_registers() {
    std::lock_guard<std::mutex> lock(mutex_);
    return flush_output_registers_locked();
}

bool ModbusClient::flush_output_registers_locked() {
    if (!is_connected_ && !reconnect_locked()) return false;

    constexpr int kOutputRegisterStart = C2P_STATUS_CODE_OFFSET;
    constexpr int kOutputRegisterCount = 20;
    std::vector<uint16_t> registers(kOutputRegisterCount, 0);

    registers[C2P_FRONT_LIDAR_STATUS_OFFSET] = output_status_code_.front_lidar_status;
    registers[C2P_LEFT_LIDAR_STATUS_OFFSET] = output_status_code_.left_lidar_status;
    registers[C2P_RIGHT_LIDAR_STATUS_OFFSET] = output_status_code_.right_lidar_status;
    registers[C2P_BACK_LIDAR_STATUS_OFFSET] = output_status_code_.down_lidar_status;
    registers[C2P_COLLISION_DETECTION_NODE_STATUS_OFFSET] =
        output_status_code_.collision_detection_node_status;

    registers[C2P_WATCHDOG_COUNTER_OFFSET] =
        static_cast<uint16_t>((output_watchdog_count_ >> 16) & 0xFFFF);
    registers[C2P_WATCHDOG_COUNTER_OFFSET + 1] =
        static_cast<uint16_t>(output_watchdog_count_ & 0xFFFF);

    registers[C2P_FRONT_ZONE_STATUS_OFFSET] = output_collision_result_.front_zone_status;
    registers[C2P_LEFT_ZONE_STATUS_OFFSET] = output_collision_result_.left_zone_status;
    registers[C2P_RIGHT_ZONE_STATUS_OFFSET] = output_collision_result_.right_zone_status;
    registers[C2P_BACK_ZONE_STATUS_OFFSET] = output_collision_result_.down_zone_status;

    encode_float_abcd(round_to_3_decimals(output_collision_result_.front_zone_collision_dist),
                      registers[C2P_FRONT_ZONE_DISTANCE_OFFSET],
                      registers[C2P_FRONT_ZONE_DISTANCE_OFFSET + 1]);
    encode_float_abcd(round_to_3_decimals(output_collision_result_.left_zone_collision_dist),
                      registers[C2P_LEFT_ZONE_DISTANCE_OFFSET],
                      registers[C2P_LEFT_ZONE_DISTANCE_OFFSET + 1]);
    encode_float_abcd(round_to_3_decimals(output_collision_result_.right_zone_collision_dist),
                      registers[C2P_RIGHT_ZONE_DISTANCE_OFFSET],
                      registers[C2P_RIGHT_ZONE_DISTANCE_OFFSET + 1]);
    encode_float_abcd(round_to_3_decimals(output_collision_result_.down_zone_collision_dist),
                      registers[C2P_BACK_ZONE_DISTANCE_OFFSET],
                      registers[C2P_BACK_ZONE_DISTANCE_OFFSET + 1]);

    return write_holding_registers(C2P_BASE_ADDRESS + kOutputRegisterStart, kOutputRegisterCount,
                                   registers);
}

bool ModbusClient::open_listen_thread() {
    listen_thread_ = new std::thread(&ModbusClient::listen_thread, this);
    if (!listen_thread_->joinable()) {
        std::cout << "[ERROR] Thread is not joinable." << std::endl;
        return false;
    }
    return true;
}

unsigned int ModbusClient::listen_thread(void *arg) {
    ModbusClient* client = static_cast<ModbusClient*>(arg);
    while (client->is_exit_ == false) {
        auto start_time = std::chrono::steady_clock::now();
        client->read_plc_info();
        auto end_time = std::chrono::steady_clock::now();
        auto duration_ms = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
        if (duration_ms < 100) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100 - duration_ms));
        }
    }

    return 0;
}

void ModbusClient::read_plc_info() {
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!is_connected_ && !reconnect_locked()) {
            return;
        }
    }

    auto start_time = std::chrono::high_resolution_clock::now();
    std::vector<uint16_t> data;
    if (!read_holding_registers(P2C_BASE_ADDRESS, read_register_count_, data)) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(mutex_);

        watchdog_count_ = (static_cast<uint32_t>(data[P2C_WATCHDOG_COUNTER_OFFSET]) << 16) | 
                           data[P2C_WATCHDOG_COUNTER_OFFSET + 1];

        auto parse_float_abcd = [&](int offset) -> float {
            if (offset + 1 >= data.size()) return 0.0f; 
            uint32_t raw = (static_cast<uint32_t>(data[offset]) << 16) | data[offset + 1];
            float val;
            std::memcpy(&val, &raw, sizeof(val));
            return val;
        };

        /**
         * ============= Ship Loader Position =============
         */
        ship_loader_pose_.walking_dist  = parse_float_abcd(P2C_WALKING_POSITION_OFFSET);
        ship_loader_pose_.slewing_angle = parse_float_abcd(P2C_SLEWING_POSITION_OFFSET);
        ship_loader_pose_.luffing_angle = parse_float_abcd(P2C_LUFFING_POSITION_OFFSET);
        ship_loader_pose_.stretch_dist  = parse_float_abcd(P2C_STRETCH_POSITION_OFFSET);

        /**
         * ============= Collision Detection Enable =============
         */
        uint16_t enable_reg = data[P2C_COLLISION_DETECTION_ENABLE_OFFSET];
        uint16_t flags = enable_reg >> 8;
        collision_detection_enable_.front_collision_detection_enable = 
            (flags & (1 << P2C_FRONT_ZONE_ENABLE_BIT)) ? 1 : 0;
            
        collision_detection_enable_.left_collision_detection_enable = 
            (flags & (1 << P2C_LEFT_ZONE_ENABLE_BIT)) ? 1 : 0;
            
        collision_detection_enable_.right_collision_detection_enable = 
            (flags & (1 << P2C_RIGHT_ZONE_ENABLE_BIT)) ? 1 : 0;
            
        collision_detection_enable_.down_collision_detection_enable = 
            (flags & (1 << P2C_BACK_ZONE_ENABLE_BIT)) ? 1 : 0;

    } 

    (void)start_time;
}

bool ModbusClient::reconnect_locked() {
    if (ctx_ == nullptr) {
        return false;
    }

    if (is_connected_) {
        return true;
    }

    modbus_close(ctx_);
    if (modbus_connect(ctx_) == -1) {
        std::cerr << "Reconnect failed: " << modbus_strerror(errno) << "\n";
        is_connected_ = false;
        return false;
    }

    is_connected_ = true;
    std::cout << "[INFO] Modbus reconnected" << std::endl;
    return true;
}

void ModbusClient::mark_disconnected() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!is_connected_) {
        return;
    }
    modbus_close(ctx_);
    is_connected_ = false;
}
