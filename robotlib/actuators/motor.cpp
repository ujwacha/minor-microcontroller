
/****************************************************************************************************
 * @file motor.cpp
 * @brief Implementation file for the Motor class providing PWM control and direction for a motor.
 * @author Robotics Team, Robotics Club Pulchowk Campus
 * @date 2023
 ****************************************************************************************************/

#include <math.h>
#include "../maths/math.hpp"
#include "motor.hpp"

/**
 * @brief Initializes the motor configuration.
 * @details Starts PWM for the motor and sets the initial direction and speed to zero.
 * @return True if the motor is successfully initialized.
 */
bool Motor::init()
{
  if (pwm_timer == nullptr)
    return false;

  HAL_StatusTypeDef status;

  if (isChannelN)
  {
    /*N Channel PWN starts only using exclusive function*/
    status = HAL_TIMEx_PWMN_Start(pwm_timer, pwm_timer_channel);
  }
  else
  {
    /*Start Normally*/
    status = HAL_TIM_PWM_Start(pwm_timer, pwm_timer_channel);
  }

  if (status != HAL_OK)
    return false;

  set_speed(0); /*Set spped 0*/

  return true;
}

/**
 * @brief Sets the motor speed using PWM.
 * @param _speed Speed value between -1 and 1.
 */
void Motor::set_speed(float _speed)
{
  speed = clamp<float>(_speed, -1.0, 1.0);

  if (speed < 0)
  {
    direction = ANTI_CLOCKWISE;
  }
  else
  {
    direction = CLOCKWISE;
  }

  if (is_opposite)
  {
    direction = (direction == CLOCKWISE) ? ANTI_CLOCKWISE : CLOCKWISE;
  }

  pwm_signal = fabs(speed) * pwm_timer->Instance->ARR;

  update();
}

/**
 * @brief Updates the motor control parameters.
 * @details Updates the motor direction and PWM signal based on the current settings.
 */
inline void Motor::update()
{
  /*Set direction and write*/
  GPIO_PinState direction_pin_state = (direction == ANTI_CLOCKWISE) ? GPIO_PIN_SET : GPIO_PIN_RESET;
  HAL_GPIO_WritePin(direction_port, direction_pin, direction_pin_state);
  /*Set PWM signal to CNT register*/
  __HAL_TIM_SET_COMPARE(pwm_timer, pwm_timer_channel, pwm_signal);
}

/**
 * @brief Gets the current motor speed.
 * @return Current motor speed.
 */
inline float Motor::get_speed() const
{
  return speed;
}