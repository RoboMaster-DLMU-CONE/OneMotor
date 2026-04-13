/**
 * @file CanFrame.hpp
 * @brief 定义了与平台无关的CAN帧结构体。
 */
#ifndef CANFRAME_HPP
#define CANFRAME_HPP
#include <cstddef>
#include <cstdint>
#include <type_traits>

#ifdef ONE_MOTOR_LINUX
#include <linux/can.h>
#define ONE_MOTOR_CAN_MAX_DLEN CAN_MAX_DLEN
#else
#include <zephyr/drivers/can.h>
// 使用 Zephyr 的 CAN_MAX_DLEN（CAN 2.0 为 8，CAN FD 为 64）
#define ONE_MOTOR_CAN_MAX_DLEN CAN_MAX_DLEN
#endif

namespace one::can {
/**
 * @brief 一个与平台无关的CAN帧结构体。
 * @details
 * 这个结构体的设计旨在与Linux SocketCAN 和 Zephyr 的 `can_frame`
 * 结构体在内存布局上兼容， 以便在对应系统上可以直接进行类型转换。 通过
 * `static_assert` 在编译时保证其兼容性。
 */
struct CanFrame {
    CanFrame() = default;
    uint32_t id{};      ///< CAN ID (标准帧或扩展帧)
    uint8_t dlc{};      ///< 数据长度码 (0-8)
#ifdef ONE_MOTOR_LINUX
    uint8_t __pad{};    ///< 填充字节，用于对齐
    uint8_t __res0{};   ///< 保留字节
    uint8_t len8_dlc{}; ///< CAN FD中真实的数据长度 (兼容普通CAN)
    alignas(8) uint8_t data[ONE_MOTOR_CAN_MAX_DLEN]{};
#else
    uint8_t flags{};    ///< CAN帧标志位 (FDF, BRS, ESI等)
    uint16_t timestamp{}; ///< 时间戳或保留字段
    uint8_t data[ONE_MOTOR_CAN_MAX_DLEN]{}; ///< CAN数据负载
#endif
};
#ifdef ONE_MOTOR_LINUX
static_assert(sizeof(CanFrame) == sizeof(can_frame),
              "Size mismatch between CanFrame and can_frame.");
static_assert(alignof(CanFrame) == alignof(can_frame),
              "Alignment mismatch between CanFrame and can_frame.");
static_assert(std::is_standard_layout_v<CanFrame>,
              "CanFrame must be a standard-layout type for safe casting.");
static_assert(offsetof(CanFrame, id) == offsetof(can_frame, can_id),
              "Offset mismatch for 'id' member.");
static_assert(offsetof(CanFrame, dlc) == offsetof(can_frame, can_dlc),
              "Offset mismatch for 'dlc' member.");
static_assert(offsetof(CanFrame, data) == offsetof(can_frame, data),
              "Offset mismatch for 'data' member.");
#else
static_assert(sizeof(CanFrame) == sizeof(can_frame),
              "Size mismatch between CanFrame and can_frame.");
static_assert(alignof(CanFrame) == alignof(can_frame),
              "Alignment mismatch between CanFrame and can_frame.");
static_assert(std::is_standard_layout_v<CanFrame>,
              "CanFrame must be a standard-layout type for safe casting.");
static_assert(offsetof(CanFrame, id) == offsetof(can_frame, id),
              "Offset mismatch for 'id' member.");
static_assert(offsetof(CanFrame, dlc) == offsetof(can_frame, dlc),
              "Offset mismatch for 'dlc' member.");
static_assert(offsetof(CanFrame, flags) == offsetof(can_frame, flags),
              "Offset mismatch for 'flags' member.");
#if defined(CONFIG_CAN_RX_TIMESTAMP)
static_assert(offsetof(CanFrame, timestamp) == offsetof(can_frame, timestamp),
              "Offset mismatch for 'timestamp' member.");
#else
static_assert(offsetof(CanFrame, timestamp) == offsetof(can_frame, reserved),
              "Offset mismatch for 'timestamp/reserved' member.");
#endif
static_assert(offsetof(CanFrame, data) == offsetof(can_frame, data),
              "Offset mismatch for 'data' member.");
#endif
} // namespace one::can

#endif // CANFRAME_HPP
