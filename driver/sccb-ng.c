/*
 * This file is part of the OpenMV project.
 * Copyright (c) 2013/2014 Ibrahim Abdelkader <i.abdalkader@gmail.com>
 * This work is licensed under the MIT license, see the file LICENSE for details.
 *
 * SCCB (I2C like) driver with the new esp-idf I2C API.
 *
 */
#include <stdbool.h>
#include <string.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "sccb.h"
#include "sensor.h"
#include <stdio.h>
#include "sdkconfig.h"
#if defined(ARDUINO_ARCH_ESP32) && defined(CONFIG_ARDUHAL_ESP_LOG)
#include "esp32-hal-log.h"
#else
#include "esp_log.h"
static const char *TAG = "sccb-ng";
#endif

#define LITTLETOBIG(x) ((x << 8) | (x >> 8))

// CIRCUITPY-CHANGE: not available until ESP-IDF v5.4, and not needed
// #include "esp_private/i2c_platform.h"
#include "driver/i2c_master.h"
#include "driver/i2c_types.h"

// support IDF 5.x
#ifndef portTICK_RATE_MS
#define portTICK_RATE_MS portTICK_PERIOD_MS
#endif

#define TIMEOUT_MS 1000                /*!< I2C timeout duration */
#define SCCB_FREQ CONFIG_SCCB_CLK_FREQ /*!< I2C master frequency */
#if CONFIG_SCCB_HARDWARE_I2C_PORT1
const int SCCB_I2C_PORT_DEFAULT = 1;
#else
const int SCCB_I2C_PORT_DEFAULT = 0;
#endif

// CIRCUITPY-CHANGE: device handles are registered and dregistered on each read and write, so they are not
// remembered.

// CIRCUITPY-CHANGE: remember the passed-in bus handle insted of the port.
// The I2C bus handle is owned and managed by CircuitPython.
static i2c_master_bus_handle_t sccb_i2c_master_bus_handle;

static esp_err_t register_device(uint8_t device_addr, i2c_master_dev_handle_t *dev_handle)
{
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = device_addr,
        .scl_speed_hz = SCCB_FREQ,
    };

    esp_err_t ret = i2c_master_bus_add_device(sccb_i2c_master_bus_handle, &dev_config, dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "register_device failed bus_handle:%p, device_addr: 0x%02x, ret:%d", sccb_i2c_master_bus_handle, device_addr, ret);
    }

    return ret;
}

static esp_err_t deregister_device(i2c_master_dev_handle_t dev_handle)
{
    esp_err_t ret = i2c_master_bus_rm_device(dev_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "deregister_device failed ret:%d", ret);
    }

    return ret;
}

static esp_err_t i2c_transmit_with_addr(uint8_t device_addr, const uint8_t *write_buffer, size_t write_size, int xfer_timeout_ms) {
    esp_err_t ret;
    i2c_master_dev_handle_t dev_handle = NULL;

    ret = register_device(device_addr, &dev_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_master_transmit(dev_handle, write_buffer, write_size, xfer_timeout_ms);
    // Always deregister.
    deregister_device(dev_handle);
    return ret;
}

static esp_err_t i2c_transmit_receive_with_addr(uint8_t device_addr, const uint8_t *write_buffer, size_t write_size, uint8_t *read_buffer, size_t read_size, int xfer_timeout_ms) {
    esp_err_t ret;
    i2c_master_dev_handle_t dev_handle = NULL;

    ret = register_device(device_addr, &dev_handle);
    if (ret != ESP_OK) {
        return ret;
    }
    ret = i2c_master_transmit_receive(dev_handle, write_buffer, write_size, read_buffer, read_size, xfer_timeout_ms);
    // Always deregister.
    deregister_device(dev_handle);
    return ret;
}


int SCCB_Install_Device(uint8_t device_addr)
{
    // CIRCUITPY-CHANGE: don't install device: each read or write will do that temporarily,
    return 0;
}

int SCCB_Init(int pin_sda, int pin_scl)
{
    // CIRCUITPY-CHANGE: initialization via pins not available; do not use
    return ESP_FAIL;
}

// CIRCUITPY-CHANGE: pass in handle instead of port, because handle is owned by CircuitPython
int SCCB_Use_Handle(i2c_master_bus_handle_t handle)
{
    sccb_i2c_master_bus_handle = handle;

    return ESP_OK;
}

int SCCB_Deinit(void)
{
    // CIRCUITPY-CHANGE: don't install device: each read or write will do that temporarily,
    return ESP_OK;
}

uint8_t SCCB_Probe(void)
{
    uint8_t slave_addr = 0x0;
    esp_err_t ret;
    // CIRCUITPY-CHANGE: already have handle

    for (size_t i = 0; i < CAMERA_MODEL_MAX; i++)
    {
        if (slave_addr == camera_sensor[i].sccb_addr)
        {
            continue;
        }
        slave_addr = camera_sensor[i].sccb_addr;

        // CIRCUITPY-CHANGE: use saved handle
        ret = i2c_master_probe(sccb_i2c_master_bus_handle, slave_addr, TIMEOUT_MS);

        if (ret == ESP_OK)
        {
            if (SCCB_Install_Device(slave_addr) != 0)
            {
                return 0;
            }
            return slave_addr;
        }
    }
    return 0;
}

uint8_t SCCB_Read(uint8_t slv_addr, uint8_t reg)
{
    // CIRCUITPY-CHANGE: register device only while in use

    uint8_t tx_buffer[1];
    uint8_t rx_buffer[1];

    tx_buffer[0] = reg;

    esp_err_t ret = i2c_transmit_receive_with_addr(slv_addr, tx_buffer, 1, rx_buffer, 1, TIMEOUT_MS);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SCCB_Read Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, rx_buffer[0], ret);
    }

    return rx_buffer[0];
}

int SCCB_Write(uint8_t slv_addr, uint8_t reg, uint8_t data)
{
    // CIRCUITPY-CHANGE: register device only while in use

    uint8_t tx_buffer[2];
    tx_buffer[0] = reg;
    tx_buffer[1] = data;

    esp_err_t ret = i2c_transmit_with_addr(slv_addr, tx_buffer, 2, TIMEOUT_MS);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "SCCB_Write Failed addr:0x%02x, reg:0x%02x, data:0x%02x, ret:%d", slv_addr, reg, data, ret);
    }

    return ret == ESP_OK ? 0 : -1;
}

uint8_t SCCB_Read16(uint8_t slv_addr, uint16_t reg)
{
    // CIRCUITPY-CHANGE: register device only while in use

    uint8_t rx_buffer[1];

    uint16_t reg_htons = LITTLETOBIG(reg);
    uint8_t *reg_u8 = (uint8_t *)&reg_htons;

    esp_err_t ret = i2c_transmit_receive_with_addr(slv_addr, reg_u8, 2, rx_buffer, 1, TIMEOUT_MS);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "W [%04x]=%02x fail\n", reg, rx_buffer[0]);
    }

    return rx_buffer[0];
}

int SCCB_Write16(uint8_t slv_addr, uint16_t reg, uint8_t data)
{
    // CIRCUITPY-CHANGE: register device only while in use

    uint8_t tx_buffer[3];
    tx_buffer[0] = reg >> 8;
    tx_buffer[1] = reg & 0x00ff;
    tx_buffer[2] = data;

    esp_err_t ret = i2c_transmit_with_addr(slv_addr, tx_buffer, 3, TIMEOUT_MS);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "W [%04x]=%02x fail\n", reg, data);
    }
    return ret == ESP_OK ? 0 : -1;
}

uint16_t SCCB_Read_Addr16_Val16(uint8_t slv_addr, uint16_t reg)
{
    // CIRCUITPY-CHANGE: register device only while in use

    uint8_t rx_buffer[2];

    uint16_t reg_htons = LITTLETOBIG(reg);
    uint8_t *reg_u8 = (uint8_t *)&reg_htons;

    esp_err_t ret = i2c_transmit_receive_with_addr(slv_addr, reg_u8, 2, rx_buffer, 2, TIMEOUT_MS);
    uint16_t data = ((uint16_t)rx_buffer[0] << 8) | (uint16_t)rx_buffer[1];

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "W [%04x]=%02x fail\n", reg, data);
    }

    return data;
}

int SCCB_Write_Addr16_Val16(uint8_t slv_addr, uint16_t reg, uint16_t data)
{
    // CIRCUITPY-CHANGE: register device only while in use

    uint8_t tx_buffer[4];
    tx_buffer[0] = reg >> 8;
    tx_buffer[1] = reg & 0x00ff;
    tx_buffer[2] = data >> 8;
    tx_buffer[3] = data & 0x00ff;

    esp_err_t ret = i2c_transmit_with_addr(slv_addr, tx_buffer, 4, TIMEOUT_MS);

    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "W [%04x]=%02x fail\n", reg, data);
    }
    return ret == ESP_OK ? 0 : -1;
}
