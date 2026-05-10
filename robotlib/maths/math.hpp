#ifndef _ROBOTLIB_MATH_HPP
#define _ROBOTLIB_MATH_HPP

#include <math.h>

#define F32_PI 3.14159265358979f
#define F32_PI_2 1.57079632679489f
#define F32_2_PI 6.28318530717958f

#define RAD2DEG 57.2957795130823f
#define DEG2RAD 0.0174532925199433f

template <typename t>
inline t map(t x, t in_min, t in_max, t out_min, t out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

template <typename t>
inline t map(t x, t out_min, t out_max)
{
  return map(x, 0, 1, out_min, out_max);
}

template <typename t>
t clamp(t x, t outmin, t outmax)
{
  if (x > outmax)
    return outmax;
  else if (x < outmin)
    return outmin;
  return x;
}

template <typename t>
inline t trim(t x, t a, t b, t c)
{
  if (x > a && x <= b)
    return (x - a) / (b - a);
  else if (x > b && x <= c)
    return (c - x) / (c - b);
  return 0;
}

template <typename t>
inline t invTrim(t x, t a, t b, t c)
{
  t out1, out2;
  out1 = (b - a) * x + a;
  out2 = c - (c - b) * x;
  return (out1 + out2) / 2;
}

inline float angleChange(const float curr, const float prev)
{
  float change;

  if ((prev > F32_PI_2) && (curr < -F32_PI_2))
  {
    change = (F32_PI - prev) + (F32_PI + curr);
  }
  else if ((prev < -F32_PI_2) && (curr > F32_PI_2))
  {
    change = -(F32_PI + prev) - (F32_PI - curr);
  }
  else
  {
    change = curr - prev;
  }
  return change;
}

inline float wrapAngle(float angle)
{
  angle = fmod(angle + F32_PI, F32_2_PI);
  if (angle < 0)
  {
    angle += F32_2_PI;
  }
  return angle - F32_PI;
}

inline float rad2Deg(float rad)
{
  return (rad * 180.0f / F32_PI);
}

inline float deg2Rad(float rad)
{
  return (rad * F32_PI / 180.0f);
}

template <typename t>
struct Vector2
{
  t x;
  t y;
};

template <typename t>
struct Vector3
{
  t x;
  t y;
  t z;
};

template <typename t>
struct Vector4
{
  t w;
  t x;
  t y;
  t z;
};

inline int float_to_uint(float x, float x_min, float x_max, unsigned int bits)
{
  float span = x_max - x_min;
  if (x < x_min)
    x = x_min;
  else if (x > x_max)
    x = x_max;
  return (int)((x - x_min) * ((float)((1 << bits) - 1) / span));
}

inline float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
  /// converts unsigned int to float, given range and number of bits ///
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

#endif // _ROBOTLIB_MATH_HPP