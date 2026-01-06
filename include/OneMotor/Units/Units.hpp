#ifndef ONE_MOTOR_UNITS_HPP_
#define ONE_MOTOR_UNITS_HPP_
/**
 * @file Units.hpp
 * @brief 物理量单位定义
 */

#include <cstdint>
#include <mp-units/framework.h>
#include <mp-units/systems/angular.h>
#include <mp-units/systems/angular/units.h>
#include <mp-units/systems/si.h>
#include <mp-units/systems/si/unit_symbols.h>
#include <mp-units/systems/si/units.h>

namespace OneMotor::Units {

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

/// 角度类型，以弧度为单位
using Angle = quantity<radian, float>;
/// 角度类型，以度为单位
using AngleDeg = quantity<degree, float>;

/// 角速度类型，以弧度/秒为单位
using AngularVelocity = quantity<radian / second, float>;
/// 角速度类型，以度/秒为单位
using AngulurVelocityDeg = quantity<degree / second, float>;

/// 转数类型
using Round = quantity<revolution, int32_t>;
/// 转速类型，以转/分钟为单位
using RPM = quantity<revolution / minute, float>;

/// 扭矩类型
using Torque = quantity<newton * metre, float>;

/// 电流类型
using Current = quantity<ampere, uint16_t>;
/// 毫安电流类型
using CurrentMilli = quantity<milli<ampere>, uint16_t>;
/// 毫安电流类型（浮点）
using CurrentMilliF = quantity<milli<ampere>, float>;

/// 温度类型
using Temperature =
    quantity_point<degree_Celsius, mp_units::si::absolute_zero, float>;

namespace literals {
using mp_units::angular::unit_symbols::deg;
using mp_units::angular::unit_symbols::rad;
using mp_units::angular::unit_symbols::rev;
using mp_units::si::unit_symbols::deg_C;
using mp_units::si::unit_symbols::m;
using mp_units::si::unit_symbols::mA;
using mp_units::si::unit_symbols::N;
using mp_units::si::unit_symbols::s;
} // namespace literals

} // namespace OneMotor::Units

#endif // INCLUDE_UNITS_UNITS_HPP_
