#ifndef ONE_MOTOR_UNITS_HPP_
#define ONE_MOTOR_UNITS_HPP_

#include <mp-units/framework.h>
#include <mp-units/systems/angular.h>
#include <mp-units/systems/si.h>

namespace OneMotor::Units {

using mp_units::quantity;
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

using Angle = quantity<radian, float>;
using AngleDeg = quantity<degree, float>;

using AngularVelocity = quantity<radian / second, float>;
using AngulurVelocityDeg = quantity<degree / second, float>;
using RPM = quantity<revolution / minute, float>;

using Torque = quantity<newton * metre, float>;

using Current = quantity<ampere, float>;
using CurrentMilli = quantity<milli<ampere>, float>;

using Temperature = quantity<degree_Celsius, float>;

namespace literals {
using namespace mp_units::si::unit_symbols;
using namespace mp_units::angular::unit_symbols;
} // namespace literals

} // namespace OneMotor::Units

#endif // INCLUDE_UNITS_UNITS_HPP_
