
/**********************************************************************************************
 * @file Motor.hpp
 * @brief Header file for the Motor class providing PWM control and direction for a motor.
 * @author Robotics Team, Robotics Club Pulchowk Campus
 * @date 2023
 **********************************************************************************************/
/**
 *  @warning Some pins might be internally pulled up, which make actuators to kick on reset.
 */

#ifndef ROBOTLIB_MOTOR_HPP_
#define ROBOTLIB_MOTOR_HPP_

#include "../common/processor_type.h"

/**
 * @enum Direction
 * @brief Enumeration for motor rotation direction.
 */
enum Direction
{
  CLOCKWISE,     /**< Clockwise direction. */
  ANTI_CLOCKWISE /**< Anti-clockwise direction. */
};

/**
 * @class Motor
 * @brief Class representing a motor with PWM control and direction.
 */
class Motor
{
private:
  TIM_HandleTypeDef *pwm_timer; /**< Pointer to the PWM timer structure. */
  int pwm_timer_channel;        /**< PWM timer channel for motor control. */
  GPIO_TypeDef *direction_port; /**< Pointer to the GPIO port for motor direction control. */
  uint16_t direction_pin;       /**< Pin for motor direction control. */
  bool isChannelN;              /**< Flag indicating whether the PWM channel is complementary (ChannelN). */
  Direction direction;          /**< Current motor rotation direction. */
  float speed;                  /**< Current motor speed. */
  uint16_t pwm_signal;          /**< Current PWM signal value. */
  bool is_opposite;             /**< Flag indicating whether the motor is in opposite direction. */

  /**
   * @brief Updates the motor control parameters.
   */
  void update();

public:
  /**
   * @brief Default constructor for Motor class.
   */
  Motor() : pwm_timer(nullptr) {};

  /**
   * @brief Motor constructor.
   * @param _pwm_timer Pointer to the PWM timer structure.
   * @param _pwm_timer_channel PWM timer channel for motor control.
   * @param _direction_port Pointer to the GPIO port for motor direction control.
   * @param _direction_pin Pin for motor direction control.
   * @param _isChannelN Flag indicating whether the PWM channel is complementary (ChannelN).
   */
  Motor(TIM_HandleTypeDef *_pwm_timer,
        int _pwm_timer_channel,
        GPIO_TypeDef *_direction_port,
        uint16_t _direction_pin,
        bool _isChannelN = false,
        bool _is_opposite = false)
      : pwm_timer(_pwm_timer),
        pwm_timer_channel(_pwm_timer_channel),
        direction_port(_direction_port),
        direction_pin(_direction_pin),
        isChannelN(_isChannelN),
        is_opposite(_is_opposite)
  {
  }

  /**
   * @brief Copy constructor for Motor class.
   * @param _motor Motor object to copy.
   */
  Motor(const Motor &_motor) = default;

  /**
   * @brief Initializes the motor configuration.
   * @details Starts PWM for the motor and sets the initial direction and speed to zero.
   * @return True if initialization is successful, false otherwise.
   */
  bool init(void);

  /**
   * @brief Sets the motor speed using PWM.
   * @param _speed Speed value between 0 and 1.
   */
  void set_speed(float _speed);

  /**
   * @brief Gets the current motor speed.
   * @return Current motor speed.
   */
  float get_speed() const;
};

#endif // ROBOTLIB_MOTOR_HPP_
