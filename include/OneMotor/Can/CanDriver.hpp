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
#include <memory>
#include <unordered_map>
#include <tl/expected.hpp>
#include "CanFrame.hpp"
#include "OneMotor/Util/Error.hpp"

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

#ifdef ONE_MOTOR_LINUX
        /**
         * @brief CanDriver 的构造函数。
         * @param interface_name CAN接口的名称 (例如 "can0")。
         */
        explicit CanDriver(std::string interface_name);
#else
        /**
         * @brief CanDriver 的构造函数。
         * @param device Zephyr CAN接口设备。
         */
        explicit CanDriver(const device* device);
#endif

        /**
         * @brief CanDriver 的析构函数。
         */
        ~CanDriver();

        CanDriver(const CanDriver&) = delete;
        CanDriver& operator=(const CanDriver&) = delete;

        tl::expected<bool, Error> is_open();

        /**
         * @brief 打开CAN接口。
         * @return 操作结果。
         */
        tl::expected<void, Error> open();

        /**
         * @brief 关闭CAN接口。
         * @return 操作结果。
         */
        tl::expected<void, Error> close();

        /**
         * @brief 发送一帧CAN数据。
         * @param frame 要发送的CanFrame对象。
         * @return 操作结果。
         */
        tl::expected<void, Error> send(const CanFrame& frame);

        /**
         * @brief 注册一个回调函数，用于处理特定CAN ID的数据帧。
         * @param can_ids 一个包含需要监听的CAN ID的集合。
         * @param func 当接收到指定ID的CAN帧时要调用的回调函数。
         * @return 操作结果。
         */
#ifdef ONE_MOTOR_LINUX
        tl::expected<void, Error> registerCallback(const std::set<size_t>& can_ids, const CallbackFunc& func);
#else
        tl::expected<void, Error> registerCallback(const std::set<size_t>& can_ids, const CallbackFunc& func);
#endif

    private:
#ifdef ONE_MOTOR_LINUX
        std::string interface_name; ///< CAN接口名称
        HyCAN::CANInterface interface; ///< 底层的HyCAN接口实例
#else
        const device* can_dev;
        std::unordered_map<uint16_t, CallbackFunc> callbacks;
        std::unordered_map<uint16_t, std::pair<can_filter, int>> filters{};

#endif
    };
}


#endif //CANDRIVER_HPP
