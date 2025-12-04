#include "OneMotor/Can/CanFrame.hpp"
#include "OneMotor/Motor/DJI/MotorManager.hpp"
#ifdef ONE_MOTOR_LINUX
#include <format>
#endif
#include <sstream>
#include <vector>

#include <OneMotor/Util/Panic.hpp>
#include <OneMotor/Util/CCM.h>
using tl::unexpected;
using enum OneMotor::ErrorCode;

namespace OneMotor::Motor::DJI
{
    /// @brief 记录每个CAN驱动下注册了哪些电机ID
    OM_CCM_ATTR std::unordered_map<Can::CanDriver*, std::set<uint16_t>> g_driver_motor_ids;
    /// @brief 存储每个CAN驱动要发送的电机电流数据
    OM_CCM_ATTR std::unordered_map<Can::CanDriver*, MotorManager::DriverOutputBuffers> g_driver_motor_outputs;

    MotorManager::~MotorManager()
    {
        stop_.store(true, std::memory_order_release);
        if (thread_->joinable()) { thread_->join(); }
    }

    MotorManager& MotorManager::getInstance()
    {
        static MotorManager instance;
        return instance;
    }

    tl::expected<void, Error> MotorManager::registerMotor(Can::CanDriver& driver, uint16_t canId) noexcept
    {
        if (auto& set = g_driver_motor_ids[&driver]; !set.contains(canId))
        {
            if (set.size() >= OM_CAN_MAX_DJI_MOTOR)
            {
#ifdef ONE_MOTOR_LINUX
                return unexpected(Error{
                    DJIMotorManagerError, std::format(
                        "Specified CanDriver has exceeded Max DJI Motor count ({}).",
                        OM_CAN_MAX_DJI_MOTOR)
                });
#else
                std::ostringstream oss;
                oss << "Specified CanDriver has exceeded Max DJI Motor count (" << OM_CAN_MAX_DJI_MOTOR << ").";
                return unexpected(Error{DJIMotorManagerError, oss.str()});
#endif
            }
            set.insert(canId);
            return {};
        }
#ifdef ONE_MOTOR_LINUX
        return unexpected(Error{
            DJIMotorManagerError, std::format("Re-registration detected on CAN ID: {}.", canId)
        });
#else
        {
            std::ostringstream oss;
            oss << "Re-registration detected on CAN ID: " << canId << ".";
            return unexpected(Error{DJIMotorManagerError, oss.str()});
        }
#endif
    }

    // ReSharper disable once CppParameterMayBeConstPtrOrRef
    tl::expected<void, Error> MotorManager::deregisterMotor(Can::CanDriver& driver, const uint16_t canId) noexcept
    {
        if (const auto it = g_driver_motor_ids.find(&driver); it != g_driver_motor_ids.end())
        {
            if (it->second.empty())
            {
                return unexpected(Error{DJIMotorManagerError, "Too many times of deregistration."});
            }
            it->second.erase(canId);
            if (it->second.empty())
            {
                g_driver_motor_ids.erase(it);
            }
            return {};
        }
        return unexpected(Error{DJIMotorManagerError, "Can't find specified CanDriver in MotorManager."});
    }

    template <uint8_t id>
    void MotorManager::pushOutput(Can::CanDriver& driver, const uint8_t lo_value, const uint8_t hi_value) noexcept
    {
        constexpr uint8_t base_index = (id - 1) * 2;
        // 直接写入工作缓冲区
        auto& driver_buffers = g_driver_motor_outputs[&driver];
        auto& output_buffer = *driver_buffers.current_write_buffer;

        output_buffer[base_index] = hi_value;
        output_buffer[base_index + 1] = lo_value;
        // 每次写入后交换缓冲区，使发送线程能看到最新数据
        swapBuffers(driver_buffers);
    }

    void MotorManager::swapBuffers(DriverOutputBuffers& driver_buffers) noexcept
    {
        // 原子交换读写缓冲区指针
        auto* old_read = driver_buffers.current_read_buffer.exchange(
            driver_buffers.current_write_buffer, std::memory_order_acq_rel);
        driver_buffers.current_write_buffer = old_read;
    }

    MotorManager::MotorManager()
    {
        thread_ = std::make_unique<Thread::Othread>([&]()
        {
            while (!stop_.load(std::memory_order_acquire))
            {
                for (auto& [driver, driver_buffers] : g_driver_motor_outputs)
                {
                    // 直接从读取缓冲区获取数据
                    auto* read_buffer = driver_buffers.current_read_buffer.load(std::memory_order_acquire);

                    Can::CanFrame frame;
                    frame.dlc = 8;
                    frame.id = 0x200;
                    std::copy_n(read_buffer->data(), 8, frame.data);
                    auto result = driver->send(frame);

                    static constexpr uint8_t zero_bytes[8] = {0};
                    if (memcmp(read_buffer->data() + 8, zero_bytes, 8) != 0)
                    {
                        frame.id = 0x1FF;
                        std::copy_n(read_buffer->data() + 8, 8, frame.data);
                        result = driver->send(frame);
                    }
                    if (!result)
                    {
                        panic("Failed to send CAN Message in MotorManager");
                    }
                }

                Thread::sleep_for(std::chrono::milliseconds(1));
            }
        });
    }

    template void MotorManager::pushOutput<1>(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;
    template void MotorManager::pushOutput<2>(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;
    template void MotorManager::pushOutput<3>(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;
    template void MotorManager::pushOutput<4>(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;
    template void MotorManager::pushOutput<5>(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;
    template void MotorManager::pushOutput<6>(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;
    template void MotorManager::pushOutput<7>(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;
    template void MotorManager::pushOutput<8>(Can::CanDriver& driver, uint8_t lo_value, uint8_t hi_value) noexcept;
}
