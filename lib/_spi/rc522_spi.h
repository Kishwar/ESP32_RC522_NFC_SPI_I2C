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
#include <memory>

// This class has nothing public.
class Rc522Spi {
  protected:
    Rc522Spi();
    ~Rc522Spi();
  private:
    const int PIN_SS                            = 05;
    const int PIN_MOSI                          = 23;
    const int PIN_MISO                          = 19;
    const int PIN_SCK                           = 18;
    const int PIN_IRQ                           = 04;      // need to check this interrupt..
    const int RC522_DEFAULT_SCAN_INTERVAL_MS    = 125;
    const int RC522_DEFAULT_TASK_STACK_SIZE     = (4 * 1024);
    const int RC522_DEFAULT_TASK_STACK_PRIORITY = 4;
    const int RC522_DEFAULT_SPI_CLOCK_SPEED_HZ  = 5 * 1000 * 1000;

    void init_spi(std::unique_ptr<SpiDevice>& spi);

    void thread_spi(const std::unique_ptr<SpiDevice>& spi);
};