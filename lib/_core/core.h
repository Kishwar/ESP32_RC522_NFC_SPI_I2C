#pragma once
/**
 * @file core.cc
 *
 * @brief Class File (header) response to receive / send messages to Application for any NFC traffic.
 *
 * @author Kishwar Kumar
 * Contact: kumar.kishwar@gmail.com
 * 
 * @date 01/09/2024
 *
 */

#include "rc522_spi.h"
#include "rc522_i2c.h"

class Core: private Rc522Spi, Rc522I2C {
  public:
    static Core& getInstance(void);
    void Thread();

  private:
    bool _thread_running = false;
    Core();
    ~Core();
};