/**
 * @file CanDriver.hpp
 * @brief 提供一个统一的CAN总线驱动接口。
 * @author MoonFeather
 * @date 2025-06-26
 */
#ifndef CANDRIVER_HPP
#define CANDRIVER_HPP

#include <string>
#include <functional>
#include <set>
#include <expected>
#include "CanFrame.hpp"

#ifdef ONE_MOTOR_LINUX
#include <HyCAN/Interface/Interface.hpp>
#endif


namespace OneMotor::Can
{
    /**
     * @class CanDriver
     * @brief CAN总线驱动类，封装了底层CAN接口的打开、关闭、发送和接收回调注册等操作。
     * @details
     * 此类旨在提供一个与平台无关的CAN驱动层。
     * 目前，在Linux平台下，它内部使用 `HyCAN` 库来实现。
     * 此类不可拷贝或赋值。
     */
    class CanDriver
    {
    public:
        /**
         * @brief CAN帧接收回调函数的类型定义。
         * @param CanFrame&& 接收到的CAN帧 (右值引用)。
         */
        using CallbackFunc = std::function<void(CanFrame&&)>;

        /**
         * @brief 操作结果的类型定义。
         * @details
         * - `std::expected<void, std::string>`: 如果操作成功，包含 `void`；如果失败，包含一个描述错误的字符串。
         */
        using Result = std::expected<void, std::string>;

        /**
         * @brief CanDriver 的构造函数。
         * @param interface_name CAN接口的名称 (例如 "can0")。
         */
        explicit CanDriver(std::string interface_name);

        /**
         * @brief CanDriver 的析构函数。
         */
        ~CanDriver();

        CanDriver(const CanDriver&) = delete;
        CanDriver& operator=(const CanDriver&) = delete;

        /**
         * @brief 打开CAN接口。
         * @return Result 操作结果。
         */
        Result open();

        /**
         * @brief 关闭CAN接口。
         * @return Result 操作结果。
         */
        Result close();

        /**
         * @brief 发送一帧CAN数据。
         * @param frame 要发送的CanFrame对象。
         * @return Result 操作结果。
         */
        Result send(const CanFrame& frame);

        /**
         * @brief 注册一个回调函数，用于处理特定CAN ID的数据帧。
         * @param can_ids 一个包含需要监听的CAN ID的集合。
         * @param func 当接收到指定ID的CAN帧时要调用的回调函数。
         * @return Result 操作结果。
         */
        Result registerCallback(const std::set<size_t>& can_ids, const CallbackFunc& func);

    private:
        std::string interface_name; ///< CAN接口名称
#ifdef ONE_MOTOR_LINUX
        HyCAN::Interface interface; ///< 底层的HyCAN接口实例
#endif
    };
}


#endif //CANDRIVER_HPP
