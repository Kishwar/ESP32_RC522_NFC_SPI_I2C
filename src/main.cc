/**
 * @file main.cc
 *
 * @brief Project responsible to read / write operations
 *        from RFID RC522 (both polling and interrupt)
 *
 * @author Kishwar Kumar
 * Contact: kumar.kishwar@gmail.com
 * 
 * @date 01/09/2024
 *
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "esp_log.h"
#include "core.h"

extern "C" void app_main() {
    Core::getInstance();
}