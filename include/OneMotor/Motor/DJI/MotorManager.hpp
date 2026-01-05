/**
 * @file MotorManager.hpp
 * @brief 定义了DJI电机管理器，用于统一发送电机控制指令。
 * @author MoonFeather
 * @date 2025-06-26
 */
#ifndef MOTORMANAGER_HPP
#define MOTORMANAGER_HPP

#include <array>
#include <atomic>
#include <memory>
#include <set>
#include <unordered_map>

#include "OneMotor/Can/CanDriver.hpp"
#include "OneMotor/Thread/Othread.hpp"
#include "OneMotor/Util/Error.hpp"
#include <tl/expected.hpp>

namespace OneMotor::Motor::DJI {
/**
 * @class MotorManager
 * @brief DJI电机管理器单例类。
 * @details
 * 该类负责管理所有DJI电机的控制指令发送。它作为一个单例，确保全局只有一个实例存在。
 * MotorManager内部会启动一个独立的发送线程。各个M3508电机实例在计算完控制电流后，
 * 调用 `pushOutput` 方法将电流值推送到管理器的缓冲区中。
 * 发送线程会以固定的时间间隔（例如1ms），将缓冲区中的所有电机电流值打包成CAN帧，
 * 并通过对应的CAN-bus驱动发送出去。
 * 这种设计将电机控制计算与CAN发送解耦，并保证了发送的周期性。
 * 此类是不可拷贝、不可移动的。
 */
class MotorManager {
  public:
    MotorManager(const MotorManager &) = delete;
    MotorManager(MotorManager &&) = delete;
    MotorManager &operator=(MotorManager &) = delete;
    MotorManager &operator=(MotorManager &&) = delete;

    /**
     * @brief 析构函数。
     * @details 停止发送线程并等待其结束。
     */
    ~MotorManager();

    /**
     * @brief 获取MotorManager的单例实例。
     * @return 对MotorManager唯一实例的引用。
     */
    static MotorManager &getInstance();

    /**
     * @brief 向管理器注册一个电机。
     * @param driver 电机所挂载的CAN总线驱动。
     * @param canId 电机的CAN ID。
     * @return Result 操作结果。
     */
    tl::expected<void, Error> registerMotor(Can::CanDriver &driver,
                                            uint16_t canId) noexcept;

    /**
     * @brief 从管理器中注销一个电机。
     * @param driver 电机所挂载的CAN总线驱动。
     * @param canId 电机的CAN ID。
     * @return 操作结果。
     */
    tl::expected<void, Error> deregisterMotor(Can::CanDriver &driver,
                                              uint16_t canId) noexcept;

    /**
     * @brief 将一个电机的目标电流值推送到发送缓冲区。
     * @tparam id 电机的ID (1-8)，对应其在CAN帧数据区的位置。
     * @param driver 电机所挂载的CAN总线驱动。
     * @param control_can_id
     * @param offset
     * @param lo_value 电流值的低8位。
     * @param hi_value 电流值的高8位。
     */
    void pushOutput(Can::CanDriver &driver, uint16_t control_can_id,
                    uint8_t offset, uint8_t lo_value,
                    uint8_t hi_value) noexcept;

  private:
    /**
     * @brief 私有构造函数，在`getInstance`中首次调用时执行。
     * @details 初始化并启动后台发送线程。
     */
    MotorManager();

    std::atomic<bool> stop_{false}; ///< 用于通知发送线程停止的原子标志
    std::unique_ptr<Thread::Othread> thread_; ///< 后台发送线程的封装
};
} // namespace OneMotor::Motor::DJI

#endif // MOTORMANAGER_HPP
