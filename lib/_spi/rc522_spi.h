#pragma once
/**
 * @file rc522_spi.cc
 *
 * @brief Class File (header) response to handle read / write operations to RC522 using SPI
 *
 * @author Kishwar Kumar
 * Contact: kumar.kishwar@gmail.com
 * 
 * @date 01/09/2024
 *
 */

#include "spi_device.h"
#include "soc/gpio_num.h"

// This class has nothing public.
class Rc522Spi {
  public:
    void Thread();
  protected:
    Rc522Spi();
    ~Rc522Spi();

  private:
    const int PIN_SS                            = 05;
    const int PIN_MOSI                          = GPIO_NUM_23;
    const int PIN_MISO                          = GPIO_NUM_19;
    const int PIN_SCK                           = GPIO_NUM_18;
    const int PIN_IRQ                           = 04;      // need to check this interrupt..
    const int RC522_DEFAULT_SCAN_INTERVAL_MS    = 125;
    const int RC522_DEFAULT_TASK_STACK_SIZE     = (4 * 1024);
    const int RC522_DEFAULT_TASK_STACK_PRIORITY = 4;
    const int RC522_DEFAULT_SPI_CLOCK_SPEED_HZ  = 5 * 1000 * 1000;

    bool _thread_running = false;
    SpiDevice *spi = nullptr;

    void init_spi();
    void thread_spi();

    esp_err_t Send(uint8_t *buffer, uint8_t length);
    esp_err_t Receive(uint8_t *buffer, uint8_t length, uint8_t addr);
    esp_err_t GetTag(uint8_t **result);
    esp_err_t Write(uint8_t addr, uint8_t n, uint8_t *data);
    esp_err_t Write(uint8_t addr, uint8_t val);
    esp_err_t Request(uint8_t *res_n, uint8_t **result);
    esp_err_t Write(uint8_t cmd, uint8_t *data, uint8_t n, uint8_t *res_n, uint8_t **result);
    esp_err_t ClearBitMask(uint8_t addr, uint8_t mask);
    esp_err_t Read(uint8_t addr, uint8_t n, uint8_t* buffer);
    esp_err_t Read(uint8_t addr, uint8_t* value_ref);
    esp_err_t SetBitMask(uint8_t addr, uint8_t mask);
    esp_err_t AntiCollision(uint8_t **result);
    esp_err_t CRC(uint8_t *data, uint8_t n, uint8_t* buffer);
};