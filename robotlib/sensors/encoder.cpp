#include <stdio.h>
#include "../maths/math.hpp"
#include "encoder.hpp"

bool Encoder::init(void)
{
  if (henc == nullptr)
    return false;

  HAL_TIM_Encoder_Stop(henc, TIM_CHANNEL_ALL);

  if (HAL_TIM_Encoder_Start(henc, TIM_CHANNEL_ALL) != HAL_OK)
    return false;

  reset_encoder_count();
  reset_count_aggregate();

  return true;
}

int32_t Encoder::get_count(void)
{
  int32_t count = henc->Instance->CNT;
  if (count > (int32_t)32768)
    count = count - (int32_t)65536;

  if (opposite)
    return (-1 * count);

  return count;
}

void Encoder::reset_encoder_count(void)
{
  count_aggregate += get_count();
  henc->Instance->CNT = 0;
  last_reset_time = HAL_GetTick();
}

float Encoder::get_omega(void)
{
  int32_t sampling_time = HAL_GetTick() - last_reset_time;
  if (sampling_time >= sample_time)
  {
    int32_t count = get_count();
    omega = 2.0f * F32_PI * (float)count * 1000 / (((float)cpr + calibration) * (float)sampling_time);
    // reset encoder count as soon as possible after reading count for omega
    reset_encoder_count();
  }

  return omega;
}

int32_t Encoder::get_count_aggregate(void)
{
  return count_aggregate + get_count();
}
