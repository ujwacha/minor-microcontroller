/***********************************************************************************************
 * @file processor_type.h
 * @brief Header including driver of processor
 * @author Robotics Team, Robotics Club Pulchowk Campus
 * @date 2023
 **********************************************************************************************/

#ifndef _PROCESSOR_TYPE_H
#define _PROCESSOR_TYPE_H

#if defined STM32F407xx
    #include "stm32f4xx_hal.h"
#elif defined STM32F103xB || defined STM32F103x6
    #include "stm32f1xx_hal.h"
#else
    #error "Unknown Processor to Robotlib, Add here."
#endif

#endif // _PROCESSOR_TYPE_H