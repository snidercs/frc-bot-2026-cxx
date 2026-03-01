#pragma once

#include <cmath>

namespace bot {

/** Applies a normalized exponential curve to a raw joystick axis value.
 
    The input is first checked against the deadband. If within the deadband,
    returns 0. Otherwise the value is normalized so that the deadband edge maps
    to 0 and full deflection maps to 1, the curve is applied, and the result is
    scaled back with the original sign. This guarantees the curve passes through
    (deadband, 0) and (1, 1) on the input/output graph.

    Formula:
    @code
        normalized = (|raw| - deadband) / (1 - deadband)
        output     = sign(raw) * normalized^exponent
    @endcode

    @param raw      Raw axis value in the range [-1, 1]
    @param deadband Deadband threshold in the range [0, 1)
    @param exponent Curve exponent (1.0 = linear, 2.0 = quadratic, etc.)
    @return Shaped output value in the range [-1, 1]
*/
inline double applyCurve(double raw, double deadband, double exponent) noexcept
{
    if (std::abs(raw) < deadband)
        return 0.0;

    const double normalized = (std::abs(raw) - deadband) / (1.0 - deadband);
    return std::copysign(std::pow(normalized, exponent), raw);
}

}
