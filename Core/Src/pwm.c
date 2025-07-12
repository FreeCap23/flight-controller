/**
  ******************************************************************************
  * @file           : pwm.c
  * @author         : Dionisie Stratulat
  * @brief          : TODO
  ******************************************************************************
  */

#include "pwm.h"

static struct {
  uint16_t ARR; /// Auto-Reload Register. Basically the resolution of the PWM signal.
  TIM_HandleTypeDef *htim; /// Handle to the timer used to generate the PWM signal.
} _cfg;

void PWM_Initialize(TIM_HandleTypeDef *htim, uint16_t ARR) {
  _cfg.htim = htim;
  _cfg.ARR = ARR;
}

void analogWrite(uint32_t channel, uint16_t duty_cycle) {
  if (duty_cycle < 0) {
    duty_cycle = 0;
  } else if (duty_cycle > _cfg.ARR) {
    duty_cycle = _cfg.ARR;
  }
  HAL_TIM_PWM_Start(_cfg.htim, channel);
  _cfg.htim->Instance->CCR1 = duty_cycle;
}

void analogWritePercent(uint32_t channel, uint8_t duty_cycle) {
  if (duty_cycle > 100) {
    duty_cycle = 100;
  }
  HAL_TIM_PWM_Start(_cfg.htim, channel);
  uint32_t *CCR = NULL;
  switch (channel) {
  case TIM_CHANNEL_1:
    CCR = &_cfg.htim->Instance->CCR1;
    break;
  case TIM_CHANNEL_2:
    CCR = &_cfg.htim->Instance->CCR2;
    break;
  case TIM_CHANNEL_3:
    CCR = &_cfg.htim->Instance->CCR3;
    break;
  case TIM_CHANNEL_4:
    CCR = &_cfg.htim->Instance->CCR4;
    break;
  }
  if (CCR != NULL)
    *CCR = ((float)duty_cycle / 100) * _cfg.ARR;
}
