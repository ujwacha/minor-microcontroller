#ifndef CAR_HPP__
#define CAR_HPP__
#ifdef __cplusplus

#include <cstdint>

#include "gpio.h"
#include "dma.h"
// #include "spi.h"
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "stm32f1xx_hal_conf.h"

#include "motor.hpp"
#include "pid.hpp"
#include "encoder.hpp"
#include "uart.hpp"
#include "state_msg.hpp"

/* Robot Dimensions */
#define BASE_DIAMETER 0.24f
#define WHEEL_DIAMETER 0.0657f

/* Robot Maximum Limits */
#define MAXIMUM_VELOCITY 1.0f
#define MAXIMUM_OMEGA 1.0f

/* Base Motors Limit */
#define MAX_MOTOR_OMEGA 76.0f // rad/s
#define MAX_MOTOR_PWM 0.95f

#define ROBOT_LOOP_PERIOD 10

class Car
{
private:
  uint32_t robot_loop_tick;
  Twist twist;

  const float kp[2] = {6.0f, 6.0f};
  const float ki[2] = {8.0f, 8.00f};
  const float kd[2] = {0.0050f, 0.0050f};

public:
  Motor left_motor = Motor(&MLP_TIMER, TIM_CHANNEL_1, MLD_GPIO_Port, MLD_Pin);
  Motor right_motor = Motor(&MRP_TIMER, TIM_CHANNEL_2, MRD_GPIO_Port, MRD_Pin);

  Encoder left_encoder = Encoder(&htim2, 830, ROBOT_LOOP_PERIOD);
  Encoder right_encoder = Encoder(&htim1, 830, ROBOT_LOOP_PERIOD, true);

	int32_t left_count = 0;
	int32_t right_count = 0;

  PID left_pid = PID(kp[0], ki[0], kd[0], -MAX_MOTOR_OMEGA, MAX_MOTOR_OMEGA, ROBOT_LOOP_PERIOD);
  PID right_pid = PID(kp[1], ki[1], kd[1], -MAX_MOTOR_OMEGA, MAX_MOTOR_OMEGA, ROBOT_LOOP_PERIOD);

  UART main_uart = UART(&huart3, UART_BOTH, sizeof(Twist), sizeof(char)*6);

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
