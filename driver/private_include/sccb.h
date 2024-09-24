/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver.
 *
 */
#ifndef __SCCB_H__
#define __SCCB_H__
#include <stdint.h>
// CIRCUITPY-CHANGE: need i2c_master_bus_handle_t. Remove after updating to ESP-IDF v5.4.
#include "driver/i2c_types.h"
int SCCB_Init(int pin_sda, int pin_scl);
// CIRCUITPY-CHANGE: pass in handle as an extra argument. Remove extra arg after update to ESP-IDF v5.4.
int SCCB_Use_Port(int i2c_num, i2c_master_bus_handle_t handle);
int SCCB_Deinit(void);
uint8_t SCCB_Probe(void);
uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg);
int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data);
uint8_t SCCB_Read16(uint8_t slv_addr, uint16_t reg);
int SCCB_Write16(uint8_t slv_addr, uint16_t reg, uint8_t data);
uint16_t SCCB_Read_Addr16_Val16(uint8_t slv_addr, uint16_t reg);
int SCCB_Write_Addr16_Val16(uint8_t slv_addr, uint16_t reg, uint16_t data);
#endif // __SCCB_H__
