#pragma once
/**
 * @file spi_device.h
 *
 * @brief Class File (header) contains information about the SPI device pins / scan timing / others
 *
 * @author Kishwar Kumar
 * Contact: kumar.kishwar@gmail.com
 * 
 * @date 01/09/2024
 *
 */

#include <cstdint>
#include <cstddef>

#include <driver/spi_master.h>

#include "esp_log.h"

const char TAG[] = "SPIDEVICE";

class SpiDevice {
  public:
    SpiDevice(uint16_t interval, size_t stack, uint16_t prio, spi_host_device_t host,
              int miso, int mosi, int sck, int sda, int intrupt, int clock, uint32_t flag) :
              scan_interval_ms(interval), task_stack_size(stack), task_priority(prio),
              host(host), miso_gpio(miso), mosi_gpio(mosi), sck_gpio(sck), sda_gpio(sda),
              clock_speed_hz(clock), device_flags(flag), is_bus_initialized(false) {
    }

    SpiDevice() = delete;
    ~SpiDevice() {
      ESP_LOGI(TAG, "SPIDEVICE object destroyed");
    }

    friend class Rc522Spi;

  private:
    uint16_t scan_interval_ms;          /*<! How fast will ESP32 scan for nearby tags, in miliseconds */
    size_t task_stack_size;             /*<! Stack size of rc522 task */
    uint8_t task_priority;              /*<! Priority of rc522 task */
    spi_host_device_t host;
    int miso_gpio;
    int mosi_gpio;
    int sck_gpio;
    int sda_gpio;
    int clock_speed_hz;
    uint32_t device_flags;     /*<! Bitwise OR of SPI_DEVICE_* flags */
    /**
     * @brief Set to true if the bus is already initialized. 
     *        NOTE: This property will be removed in future,
     *        once when https://github.com/espressif/esp-idf/issues/8745 is resolved
     * 
     */
    bool is_bus_initialized;

    spi_device_handle_t spi_handle = nullptr;
};