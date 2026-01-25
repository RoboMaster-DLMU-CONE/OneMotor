#ifndef ONE_MOTOR_UNITS_HPP_
#define ONE_MOTOR_UNITS_HPP_
/**
 * @file Units.hpp
 * @brief 物理量单位定义
 *
 * 该文件定义了OneMotor项目中使用的各种物理量单位类型，
 * 基于mp-units库实现类型安全的物理量计算。
 */

#include <cstdint>
#include <mp-units/framework.h>
#include <mp-units/systems/angular.h>
#include <mp-units/systems/angular/units.h>
#include <mp-units/systems/si.h>
#include <mp-units/systems/si/unit_symbols.h>
#include <mp-units/systems/si/units.h>

/**
 * @namespace OneMotor::Units
 * @brief OneMotor项目的单位系统命名空间
 *
 * 包含了电机控制中常用的物理量单位定义，
 * 如角度、角速度、扭矩、电流等。
 */
namespace OneMotor::Units {

// 导入mp-units库中的基础类型
using mp_units::quantity;
using mp_units::quantity_point;
using mp_units::angular::degree;
using mp_units::angular::radian;
using mp_units::angular::revolution;

using mp_units::si::ampere;
using mp_units::si::degree_Celsius;
using mp_units::si::metre;
using mp_units::si::milli;
using mp_units::si::minute;
using mp_units::si::newton;
using mp_units::si::second;

/**
 * @typedef Angle
 * @brief 角度类型，以弧度为单位
 *
 * 表示角度的类型，使用弧度作为单位，浮点数存储。
 */
using Angle = quantity<radian, float>;

/**
 * @typedef AngleDeg
 * @brief 角度类型，以度为单位
 *
 * 表示角度的类型，使用度作为单位，浮点数存储。
 */
using AngleDeg = quantity<degree, float>;

/**
 * @typedef AngularVelocity
 * @brief 角速度类型，以弧度/秒为单位
 *
 * 表示角速度的类型，使用弧度/秒作为单位，浮点数存储。
 */
using AngularVelocity = quantity<radian / second, float>;

/**
 * @typedef AngulurVelocityDeg
 * @brief 角速度类型，以度/秒为单位
 *
 * 表示角速度的类型，使用度/秒作为单位，浮点数存储。
 */
using AngulurVelocityDeg = quantity<degree / second, float>;

/**
 * @typedef Round
 * @brief 转数类型
 *
 * 表示转数的类型，使用转数作为单位，整数存储。
 */
using Round = quantity<revolution, int32_t>;

/**
 * @typedef RPM
 * @brief 转速类型，以转/分钟为单位
 *
 * 表示转速的类型，使用转/分钟作为单位，浮点数存储。
 */
using RPM = quantity<revolution / minute, float>;

/**
 * @typedef Torque
 * @brief 扭矩类型
 *
 * 表示扭矩的类型，使用牛·米作为单位，浮点数存储。
 */
using Torque = quantity<newton * metre, float>;

/**
 * @typedef Current
 * @brief 电流类型
 *
 * 表示电流的类型，使用安培作为单位，整数存储。
 */
using Current = quantity<ampere, uint16_t>;

/**
 * @typedef CurrentMilli
 * @brief 毫安电流类型
 *
 * 表示电流的类型，使用毫安作为单位，整数存储。
 */
using CurrentMilli = quantity<milli<ampere>, uint16_t>;

/**
 * @typedef CurrentMilliF
 * @brief 毫安电流类型（浮点）
 *
 * 表示电流的类型，使用毫安作为单位，浮点数存储。
 */
using CurrentMilliF = quantity<milli<ampere>, float>;

/**
 * @typedef Temperature
 * @brief 温度类型
 *
 * 表示温度的类型，使用摄氏度作为单位，相对于绝对零度，
 * 浮点数存储。
 */
using Temperature =
    quantity_point<degree_Celsius, mp_units::si::absolute_zero, float>;

/**
 * @namespace OneMotor::Units::literals
 * @brief 单位字面量命名空间
 *
 * 包含了常用的单位字面量，方便在代码中直接使用，
 * 如 90_deg 表示90度，2_rad 表示2弧度等。
 */
namespace literals {
using mp_units::angular::unit_symbols::deg;  ///< 度的字面量符号
using mp_units::angular::unit_symbols::rad;  ///< 弧度的字面量符号
using mp_units::angular::unit_symbols::rev;  ///< 转数的字面量符号
using mp_units::si::unit_symbols::deg_C;     ///< 摄氏度的字面量符号
using mp_units::si::unit_symbols::m;         ///< 米的字面量符号
using mp_units::si::unit_symbols::mA;        ///< 毫安的字面量符号
using mp_units::si::unit_symbols::N;         ///< 牛顿的字面量符号
using mp_units::si::unit_symbols::s;         ///< 秒的字面量符号
} // namespace literals

} // namespace OneMotor::Units

#endif // ONE_MOTOR_UNITS_HPP_
