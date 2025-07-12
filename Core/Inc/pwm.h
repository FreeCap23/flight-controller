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

void PWM_Initialize(TIM_HandleTypeDef *htim, uint16_t ARR);
void analogWrite(uint32_t channel, uint16_t duty_cycle);
void analogWritePercent(uint32_t channel, uint8_t duty_cycle);

#endif /* INC_PWM_H_ */
