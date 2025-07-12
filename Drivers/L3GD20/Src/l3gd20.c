/**
  ******************************************************************************
  * @file    l3gd20.c
  * @author  Dionisie Stratulat
  * @brief   This file provides a set of functions needed to manage the L3GD20,
  *          ST MEMS motion sensor, 3-axis digital output gyroscope.
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "l3gd20.h"


void L3GD20_Initialize(SPI_HandleTypeDef *handle, GPIO_TypeDef *CS_Port, uint16_t CS_Pin) {
  _cfg.hspi = handle;
  _cfg.CS_Port = CS_Port;
  _cfg.CS_Pin = CS_Pin;
}

uint8_t L3GD20_ReadRegister(uint8_t reg) {
  uint8_t tx[2], rx[2];
  tx[0] = reg | 0x80; // Set MSB to indicate we want to read from this register.
  tx[1] = 0; // Dummy

  // Enable SPI Communication by setting CS Low
  HAL_GPIO_WritePin(_cfg.CS_Port, _cfg.CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(_cfg.hspi, tx, rx, 2, 50); // Send and receive 2 bytes, timeout 50ms.
  // Disable SPI Communication by setting CS High
  HAL_GPIO_WritePin(_cfg.CS_Port, _cfg.CS_Pin, GPIO_PIN_SET);

  // Only rx[1] holds information.
  return rx[1];
}

void L3GD20_WriteRegister(uint8_t reg, uint8_t value) {
  uint8_t tx[2];
  tx[0] = reg & ~0x80; // Reset MSB to indicate we want to write to this register.
  tx[1] = value; // Dummy

  // Enable SPI Communication by setting CS Low
  HAL_GPIO_WritePin(_cfg.CS_Port, _cfg.CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_Transmit(_cfg.hspi, tx, 2, 50); // Send 2 bytes, timeout 50ms.
  // Disable SPI Communication by setting CS High
  HAL_GPIO_WritePin(_cfg.CS_Port, _cfg.CS_Pin, GPIO_PIN_SET);
}
