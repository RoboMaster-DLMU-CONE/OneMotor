#include "OneMotor/Can/CanFrame.hpp"
#include "OneMotor/Motor/DJI/MotorManager.hpp"
#include "OneMotor/Thread/Othread.hpp"
#include <chrono>
#include <format>

#include <OneMotor/Util/CCM.h>
#include <OneMotor/Util/Panic.hpp>
#include <ankerl/unordered_dense.h>

#include <OneMotor/Util/DoubleBuffer.hpp>
#include <OneMotor/Util/DtcmAllocator.hpp>

using tl::unexpected;
using enum OneMotor::ErrorCode;

namespace OneMotor::Motor::DJI
{
    template <typename T>
    using AHash = ankerl::unordered_dense::hash<T>;
    template <typename K, typename V>
    using FastMap = ankerl::unordered_dense::map<K, V, AHash<K>, std::equal_to<K>,
                                                 DtcmAllocator<std::pair<K, V>>>;

    using OutputArray = std::array<uint8_t, 8>;
    using ControlBuffers = FastMap<uint16_t, DoubleBuffer<OutputArray>>;
    /// @brief 记录每个CAN驱动下注册了哪些电机ID
    OM_CCM_ATTR FastMap<Can::CanDriver*, std::set<uint16_t>> g_driver_motor_ids;
    /// @brief 存储每个CAN驱动要发送的电机电流数据
    OM_CCM_ATTR FastMap<Can::CanDriver*, ControlBuffers> g_driver_motor_outputs;

    MotorManager::~MotorManager()
    {
        stop_.store(true, std::memory_order_release);
        if (thread_->joinable())
        {
            thread_->join();
        }
    }

    MotorManager& MotorManager::getInstance()
    {
        static MotorManager instance;
        return instance;
    }

    tl::expected<void, Error> MotorManager::registerMotor(Can::CanDriver& driver,
                                                          uint16_t canId) noexcept
    {
        if (auto& set = g_driver_motor_ids[&driver]; !set.contains(canId))
        {
            if (set.size() >= OM_CAN_MAX_DJI_MOTOR)
            {
                return unexpected(
                    Error{
                        DJIMotorManagerError,
                        std::format("Specified CanDriver has exceeded Max DJI "
                                    "Motor count ({}).",
                                    OM_CAN_MAX_DJI_MOTOR)
                    });
            }
            set.insert(canId);
            return {};
        }
        return unexpected(
            Error{
                DJIMotorManagerError,
                std::format("Re-registration detected on CAN ID: {}.", canId)
            });
    }

    // ReSharper disable once CppParameterMayBeConstPtrOrRef
    tl::expected<void, Error>
    MotorManager::deregisterMotor(Can::CanDriver& driver,
                                  const uint16_t canId) noexcept
    {
        if (const auto it = g_driver_motor_ids.find(&driver);
            it != g_driver_motor_ids.end())
        {
            if (it->second.empty())
            {
                return unexpected(Error{
                    DJIMotorManagerError,
                    "Too many times of deregistration."
                });
            }
            it->second.erase(canId);
            if (it->second.empty())
            {
                g_driver_motor_ids.erase(it);
            }
            return {};
        }
        return unexpected(Error{
            DJIMotorManagerError,
            "Can't find specified CanDriver in MotorManager."
        });
    }

    void MotorManager::pushOutput(Can::CanDriver& driver,
                                  const uint16_t control_can_id,
                                  const uint8_t offset, const uint8_t lo_value,
                                  const uint8_t hi_value) noexcept
    {
        // 直接写入工作缓冲区
        auto& driver_buffers = g_driver_motor_outputs[&driver];
        auto& output_buffer = driver_buffers[control_can_id];
        auto& raw_buffer = output_buffer.write();

        raw_buffer[offset] = hi_value;
        raw_buffer[offset + 1] = lo_value;
        // 每次写入后交换缓冲区，使发送线程能看到最新数据
        output_buffer.swap();
    }

    MotorManager::MotorManager()
    {
        thread_ = std::make_unique<Thread::Othread>([&]()
        {
            while (!stop_.load(std::memory_order_acquire))
            {
                for (auto& [driver, driver_buffers] : g_driver_motor_outputs)
                {
                    for (auto& [control_can_id, output_buffer] : driver_buffers)
                    {
                        Can::CanFrame frame;
                        frame.dlc = 8;
                        frame.id = control_can_id;
                        std::copy_n(output_buffer.readView().data(), 8, frame.data);
                        if (const auto result = driver->send(frame); !result)
                        {
                            Thread::sleep_for(std::chrono::milliseconds(10));
                        }
                    }
                }

#ifdef CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME
#if CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME != 0
                Thread::sleep_for(std::chrono::milliseconds(
                    CONFIG_OM_DJI_MOTOR_SKIP_N_FRAME * 5));
#else
                Thread::sleep_for(std::chrono::milliseconds(1));
#endif
#else
                Thread::sleep_for(std::chrono::milliseconds(1));
#endif
            }
        });
    }
} // namespace OneMotor::Motor::DJI
