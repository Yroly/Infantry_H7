#ifndef SP__MATH_TOOLS_HPP
#define SP__MATH_TOOLS_HPP

#include "math.h"
#include "stdint.h"

namespace at{
constexpr float PI = 3.14159265358979323846;

// (-PI, PI]
float limit_angle(float angle);

// [min, max]
float limit_min_max(float input, float min, float max);

// [-max, max]
float limit_max(float input, float max);

// [min, max]
bool scope_min_max(float input, float min, float max);

// [-max, max]
bool scope_max(float input, float max);

// [min, max]
float uint_to_float(uint32_t input, float min, float max, size_t bits);

// [min, max]
uint32_t float_to_uint(float input, float min, float max, size_t bits);

// sgn
int8_t sgn(float input);
}
#endif  // AT__MATH_TOOLS_HPP