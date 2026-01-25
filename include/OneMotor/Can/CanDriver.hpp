/**
 * @file CanDriver.hpp
 * @brief 提供一个统一的CAN总线驱动接口。
 */
#ifndef CANDRIVER_HPP
#define CANDRIVER_HPP

#include "CanFrame.hpp"
#include "OneMotor/Util/Error.hpp"
#include <concepts>
#include <functional>
#include <memory>
#include <set>
#include <string>
#include <tl/expected.hpp>

#ifdef ONE_MOTOR_LINUX
#include <HyCAN/Interface/Interface.hpp>
#endif

namespace OneMotor::Can {
/**
 * @class CanDriver
 * @brief CAN总线驱动类，封装了底层CAN接口的打开、关闭、发送和接收回调注册等操作。
 * @details
 * 此类旨在提供一个与平台无关的CAN驱动层。
 * 目前，在Linux平台下，它内部使用 `HyCAN` 库来实现。
 * 在Zephyr平台上，它使用Zephyr的原生CAN驱动。
 * 此类不可拷贝或赋值。
 */
class CanDriver {
  public:
#ifdef ONE_MOTOR_LINUX
    /**
     * @brief 默认构造函数（Linux平台）
     *
     * 创建一个未初始化的CanDriver实例。
     * 需要通过init方法进行初始化。
     */
    CanDriver() = default;

    /**
     * @brief CanDriver 的构造函数（Linux平台）。
     * @param interface_name CAN接口的名称 (例如 "can0")。
     *
     * 构造一个CanDriver实例并指定CAN接口名称。
     */
    explicit CanDriver(std::string interface_name);
#else
    /**
     * @brief 默认构造函数（Zephyr平台）
     *
     * 创建一个未初始化的CanDriver实例。
     * 需要通过init方法进行初始化。
     */
    CanDriver() = default;

    /**
     * @brief CanDriver 的构造函数（Zephyr平台）。
     * @param device Zephyr CAN接口设备。
     *
     * 构造一个CanDriver实例并绑定到指定的Zephyr CAN设备。
     */
    explicit CanDriver(const device *device);
#endif

    /**
     * @brief CanDriver 的析构函数。
     *
     * 释放CAN驱动占用的资源。
     */
    ~CanDriver();

  #ifdef ONE_MOTOR_LINUX
    /**
     * @brief 初始化CAN驱动（Linux平台）
     * @param interface_name CAN接口的名称 (例如 "can0")
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 初始化CAN驱动，指定要使用的接口名称。
     */
    tl::expected<void, Error> init(std::string interface_name);
  #else
    /**
     * @brief 初始化CAN驱动（Zephyr平台）
     * @param device Zephyr CAN接口设备
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 初始化CAN驱动，绑定到指定的Zephyr CAN设备。
     */
    tl::expected<void, Error> init(const device *device);
  #endif

    /**
     * @brief 删除拷贝构造函数
     *
     * CanDriver不可拷贝，以避免资源管理问题。
     */
    CanDriver(const CanDriver &) = delete;

    /**
     * @brief 删除赋值运算符
     *
     * CanDriver不可赋值，以避免资源管理问题。
     */
    CanDriver &operator=(const CanDriver &) = delete;

    /**
     * @brief 检查CAN接口是否已打开
     * @return 操作结果，成功返回bool值表示是否已打开，失败返回Error
     */
    tl::expected<bool, Error> is_open();

    /**
     * @brief 打开CAN接口。
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 打开CAN接口，准备进行数据收发。
     */
    tl::expected<void, Error> open();

    /**
     * @brief 关闭CAN接口。
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 关闭CAN接口，释放相关资源。
     */
    tl::expected<void, Error> close();

    /**
     * @brief 发送一帧CAN数据。
     * @param frame 要发送的CanFrame对象
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 将指定的CAN帧发送到总线上。
     */
    tl::expected<void, Error> send(const CanFrame &frame);

    /**
     * @brief 注册一个回调函数，用于处理特定CAN ID的数据帧。
     * @param can_ids 一个包含需要监听的CAN ID的集合
     * @param func 当接收到指定ID的CAN帧时要调用的回调函数
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 注册回调函数以处理特定ID的CAN消息。
     * 回调函数应接受一个CanFrame参数。
     */
#ifdef ONE_MOTOR_LINUX
    template <typename Func>
    tl::expected<void, Error> registerCallback(const std::set<size_t> &can_ids,
                                               Func &&func)
        requires(std::invocable<Func, CanFrame>)
    {
        if (auto guard = ensureInitialized(); !guard) {
            return tl::make_unexpected(guard.error());
        }
        return interface->register_callback<can_frame>(
                   can_ids, [func = std::forward<decltype(func)>(func)](
                                can_frame frame) {
                       func(std::bit_cast<CanFrame>(frame));
                   })
            .map_error([&](const auto &) {
                return Error{ErrorCode::CanDriverInternalError,
                             "HyCAN register callback failed"};
            });
    }
#else
    /**
     * @brief 注册回调函数的内部实现（Zephyr平台）
     * @param can_ids 要监听的CAN ID集合
     * @param func 回调函数
     * @return 操作结果
     */
    tl::expected<void, Error>
    registerCallbackImpl(const std::set<size_t> &can_ids,
                         const std::function<void(CanFrame)> &func) const;

    template <typename Func>
    tl::expected<void, Error> registerCallback(const std::set<size_t> &can_ids,
                                               Func &&func)
        requires(std::invocable<Func, CanFrame>)
    {
        if (auto guard = ensureInitialized(); !guard) {
            return tl::make_unexpected(guard.error());
        }
        return registerCallbackImpl(
            can_ids, [func = std::forward<decltype(func)>(func)](
                         CanFrame can_frame) { func(can_frame); });
    }
#endif

  private:
#ifdef ONE_MOTOR_LINUX
    std::string interface_name;                      ///< CAN接口名称
    std::unique_ptr<HyCAN::CANInterface> interface;  ///< 底层的HyCAN接口实例
#else
    const device *can_dev = nullptr;  ///< Zephyr CAN设备指针

#endif
    /**
     * @var m_initialized
     * @brief 初始化状态标志
     *
     * 标记CAN驱动是否已完成初始化。
     */
    bool m_initialized = false;

    /**
     * @brief 确保CAN驱动已初始化
     * @return 操作结果，成功返回void，失败返回Error
     *
     * 检查CAN驱动是否已正确初始化，如果没有则返回错误。
     */
    tl::expected<void, Error> ensureInitialized() const;
};

/**
 * @brief 确保CAN驱动已初始化的内联实现
 * @return 操作结果，成功返回void，失败返回Error
 *
 * 检查CAN驱动是否已正确初始化，如果没有则返回错误。
 */
inline tl::expected<void, Error> CanDriver::ensureInitialized() const {
#ifdef ONE_MOTOR_LINUX
    if (interface && m_initialized)
#else
    if (m_initialized)
#endif
        return {};
    return tl::make_unexpected(
        Error{ErrorCode::CanDriverNotInitialized, "CanDriver not initialized"});
}

} // namespace OneMotor::Can

#endif // CANDRIVER_HPP
