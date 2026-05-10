#ifndef CAR_HPP__
#define CAR_HPP__
#ifdef __cplusplus

#include <cstdint>

#include "gpio.h"
#include "dma.h"
#include "spi.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "stm32f1xx_hal_conf.h"

#include "actuators/motor.hpp"
#include "controllers/pid.hpp"
#include "sensors/encoder.hpp"
#include "communication/uart.hpp"
#include "sensors/bno08_spi.hpp"
#include "msgs/state_msg.hpp"

/* Robot Dimensions */
#define BASE_DIAMETER 0.61f
#define WHEEL_DIAMETER 0.152f

/* Robot Maximum Limits */
#define MAXIMUM_VELOCITY 1.0f
#define MAXIMUM_OMEGA 1.0f

/* Base Motors Limit */
#define MAX_MOTOR_OMEGA 91.0f // rad/s
#define MAX_MOTOR0_PWM 0.95f
#define MAX_MOTOR1_PWM 0.93f

#define ROBOT_LOOP_PERIOD 10

class Car
{
private:
  uint32_t robot_loop_tick;
  Twist twist;

  const float kp[2] = {0.7f, 0.7f};
  const float ki[2] = {12.0f, 12.0f};
  const float kd[2] = {0.0005f, 0.0005f};

public:
  Motor left_motor = Motor(&MLP_TIMER, TIM_CHANNEL_1, MLD_GPIO_Port, MLD_Pin);
  Motor right_motor = Motor(&MRP_TIMER, TIM_CHANNEL_2, MLD_GPIO_Port, MLD_Pin);

  Encoder left_encoder = Encoder(&htim3, 1000, ROBOT_LOOP_PERIOD);
  Encoder right_encoder = Encoder(&htim4, 1000, ROBOT_LOOP_PERIOD);

  PID left_pid = PID(kp[0], ki[0], kd[0], -MAX_MOTOR_OMEGA, MAX_MOTOR_OMEGA, ROBOT_LOOP_PERIOD);
  PID right_pid = PID(kp[0], ki[0], kd[0], -MAX_MOTOR_OMEGA, MAX_MOTOR_OMEGA, ROBOT_LOOP_PERIOD);

  UART main_uart = UART(&huart3, UART_BOTH, sizeof(Twist), sizeof(ImuData));

  void init();
  void run();
  void set_twist(const Twist &_twist);
  bool update();
};

extern Car car;

#endif

#ifdef __cplusplus
extern "C"
{
#endif

  void run_car(void);

#ifdef __cplusplus
}
#endif

#endif