/**
 * @file rc522_spi.cc
 *
 * @brief Class File (source) response to handle read / write operations to RC522 using I2C
 *
 * @author Kishwar Kumar
 * Contact: kumar.kishwar@gmail.com
 *
 * @date 01/09/2024
 *
 */

#include "rc522_i2c.h"
#include "esp_log.h"

const char TAG[] = "RC522I2C";

Rc522I2C::Rc522I2C() {
  ESP_LOGI(TAG, "Rc522I2C created");
}

Rc522I2C::~Rc522I2C() {
  ESP_LOGI(TAG, "Rc522I2C destroyed");
}