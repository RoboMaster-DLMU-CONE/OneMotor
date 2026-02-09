#include <chrono>
#include <one/motor/Error.hpp>
#include <one/motor/dji/MotorManager.hpp>
#include <one/thread/Othread.hpp>

#include <one/utils/CCM.h>
#include <one/utils/Panic.hpp>

#include <one/utils/DoubleBuffer.hpp>
#include <one/utils/DtcmAllocator.hpp>

using tl::unexpected;
using enum one::motor::ErrorCode;

namespace one::motor::dji {
template <typename T> using Hash = std::hash<T>;
template <typename K, typename V>
using FastMap = std::unordered_map<K, V, Hash<K>, std::equal_to<K>,
                                   DtcmAllocator<std::pair<const K, V>>>;

using OutputArray = std::array<uint8_t, 8>;
using ControlBuffers = FastMap<uint16_t, DoubleBuffer<OutputArray>>;

/// @brief 记录每个CAN驱动下注册了哪些电机ID
OM_CCM_ATTR FastMap<can::CanDriver *, std::set<uint16_t>> g_driver_motor_ids;
/// @brief 存储每个CAN驱动要发送的电机电流数据
OM_CCM_ATTR FastMap<can::CanDriver *, ControlBuffers> g_driver_motor_outputs;

MotorManager::~MotorManager() {
    stop_.store(true, std::memory_order_release);
    if (thread_->joinable()) {
        thread_->join();
    }
}

MotorManager &MotorManager::getInstance() {
    static MotorManager instance;
    return instance;
}

tl::expected<void, Error>
MotorManager::registerMotor(can::CanDriver &driver,
                            const uint16_t canId) noexcept {
    if (auto &set = g_driver_motor_ids[&driver]; !set.contains(canId)) {
        if (set.size() >= OM_CAN_MAX_DJI_MOTOR) {
            return unexpected(
                Error{DJIMotorManagerError,
                      "Specified CanDriver has exceeded Max DJI motor count."});
        }
        set.insert(canId);
        return {};
    }
    return unexpected(
        Error{DJIMotorManagerError, "Re-registration detected on CAN ID."});
}

// ReSharper disable once CppParameterMayBeConstPtrOrRef
tl::expected<void, Error>
MotorManager::deregisterMotor(can::CanDriver &driver,
                              const uint16_t canId) noexcept {
    if (const auto it = g_driver_motor_ids.find(&driver);
        it != g_driver_motor_ids.end()) {
        if (it->second.empty()) {
            return unexpected(Error{DJIMotorManagerError,
                                    "Too many times of deregistration."});
        }
        it->second.erase(canId);
        if (it->second.empty()) {
            g_driver_motor_ids.erase(it);
        }
        return {};
    }
    return unexpected(Error{DJIMotorManagerError,
                            "Can't find specified CanDriver in MotorManager."});
}

void MotorManager::pushOutput(can::CanDriver &driver,
                              const uint16_t control_can_id,
                              const uint8_t offset, const uint8_t lo_value,
                              const uint8_t hi_value) noexcept {
    // 直接写入工作缓冲区
    auto &driver_buffers = g_driver_motor_outputs[&driver];
    auto &output_buffer = driver_buffers[control_can_id];
    auto &raw_buffer = output_buffer.write();

    raw_buffer[offset] = hi_value;
    raw_buffer[offset + 1] = lo_value;
    // 每次写入后交换缓冲区，使发送线程能看到最新数据
    output_buffer.swap();
}

MotorManager::MotorManager() {
    thread_ = std::make_unique<thread::Othread>([&]() {
        while (!stop_.load(std::memory_order_acquire)) {
            for (auto &[driver, driver_buffers] : g_driver_motor_outputs) {
                for (auto &[control_can_id, output_buffer] : driver_buffers) {
                    can::CanFrame frame;
                    frame.dlc = 8;
                    frame.id = control_can_id;
                    std::copy_n(output_buffer.readView().data(), 8, frame.data);
                    if (const auto result = driver->send(frame); !result) {
                        thread::sleep_for(std::chrono::milliseconds(10));
                    }
                }
            }

#ifdef CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME
#if CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME != 0
            thread::sleep_for(std::chrono::milliseconds(
                CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME * 5));
#else
            thread::sleep_for(std::chrono::milliseconds(1));
#endif
#else
            thread::sleep_for(std::chrono::milliseconds(1));
#endif
        }
    });
}
} // namespace one::motor::dji
