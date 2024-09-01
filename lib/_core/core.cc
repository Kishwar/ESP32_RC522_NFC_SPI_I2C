/**
 * @file core.cc
 *
 * @brief Class File (source) response to receive / send messages to Application for any NFC traffic.
 *
 * @author Kishwar Kumar
 * Contact: kumar.kishwar@gmail.com
 * 
 * @date 01/09/2024
 *
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstring>

#include "core.h"
#include "esp_log.h"

static void taskWrapper(void *pvParameters);

static Core *instance = nullptr;
const char TAG_CORE[] = "CORE";

Core &Core::getInstance(void)
{
  if(instance == nullptr) {
    ESP_LOGI(TAG_CORE, "new Core Instance created");
    instance = new Core();
  }
  return *instance;
}

void Core::Thread() {
  if(_thread_running) { 
    ESP_LOGE(TAG_CORE, "Thread already running..");
    return;
  }

  _thread_running = true;
  while(true) {
    ESP_LOGI(TAG_CORE, "Core looping");
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

static void taskWrapper(void *pvParameters) {
  Core *core = static_cast<Core *>(pvParameters);
  core->Thread();
}

Core::Core()
{
  // create a RTOS task (this should be only once)
  xTaskCreate(taskWrapper, "TaskName", 4096, this, 1, nullptr);
}

Core::~Core() {
  // let's do some clean up work
}
