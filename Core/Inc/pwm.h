/**
  ******************************************************************************
  * @file           : pwm.h
  * @author         : Dionisie Stratulat
  * @brief          : TODO
  ******************************************************************************
  */

#ifndef INC_PWM_H_
#define INC_PWM_H_

#include <stdint.h>
#include "main.h"

static inline volatile uint32_t* _chooseCCR(TIM_HandleTypeDef *htim, uint32_t channel) {
  switch (channel) {
  case TIM_CHANNEL_1:
    return &htim->Instance->CCR1;
  case TIM_CHANNEL_2:
    return &htim->Instance->CCR2;
  case TIM_CHANNEL_3:
    return &htim->Instance->CCR3;
  case TIM_CHANNEL_4:
    return &htim->Instance->CCR4;
  default:
    return NULL;
  }
}

static inline void analogWrite(TIM_HandleTypeDef *htim, uint32_t channel, uint16_t compare_value) {
  if (compare_value > htim->Instance->ARR)
    compare_value = htim->Instance->ARR;

  volatile uint32_t* CCR = _chooseCCR(htim, channel);
  *CCR = compare_value;
}

static inline void analogWritePercent(TIM_HandleTypeDef *htim, uint32_t channel, uint8_t duty_cycle) {
  if (duty_cycle > 100)
    duty_cycle = 100;

  volatile uint32_t* CCR = _chooseCCR(htim, channel);
  *CCR = ((float)duty_cycle / 100.0f) * htim->Instance->ARR;
}

#endif /* INC_PWM_H_ */
