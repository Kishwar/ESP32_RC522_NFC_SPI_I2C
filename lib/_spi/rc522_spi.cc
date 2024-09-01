/**
 * @file rc522_spi.cc
 *
 * @brief Class File (source) response to handle read / write operations to RC522 using SPI
 *
 * @author Kishwar Kumar
 * Contact: kumar.kishwar@gmail.com
 *
 * @date 01/09/2024
 *
 */

#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>

#include "rc522_spi.h"
#include "esp_log.h"

const char TAG_RC522SPI[] = "RC522SPI";

Rc522Spi::Rc522Spi() {
  ESP_LOGI(TAG_RC522SPI, "%d: %s(): ENTER", __LINE__, __FILE__);
  SpiDevice *spi = new SpiDevice(RC522_DEFAULT_SCAN_INTERVAL_MS,
                                 RC522_DEFAULT_TASK_STACK_SIZE, RC522_DEFAULT_TASK_STACK_PRIORITY,
                                 SPI2_HOST, PIN_MISO, PIN_MISO, PIN_SCK, PIN_SS,
                                 0 /*not used*/, RC522_DEFAULT_SPI_CLOCK_SPEED_HZ, 0);

  // initialize hardware peripherals
  init_spi(spi);

  esp_event_loop_args_t event_args = {
    .queue_size = 1,
    .task_name = nullptr, // no task will be created
  };

  esp_event_loop_create(&event_args, &(spi->event_handle));

  // trigger a thread to scan any incoming card
  thread_spi(spi);
  ESP_LOGI(TAG_RC522SPI, "%d: %s(): EXIT", __LINE__, __FILE__);
}

Rc522Spi::~Rc522Spi() {
  ESP_LOGI(TAG_RC522SPI, "%d: %s(): ENTER", __LINE__, __FILE__);
}

void Rc522Spi::init_spi(SpiDevice *spi) {
  spi_device_interface_config_t devcfg = {
    .mode = 0,
    .clock_speed_hz = spi->clock_speed_hz,
    .spics_io_num = spi->sda_gpio,
    .flags = spi->device_flags,
    .queue_size = 7,
  };

  spi_bus_config_t buscfg = {
    .mosi_io_num = spi->mosi_gpio,
    .miso_io_num = spi->miso_gpio,
    .sclk_io_num = spi->sck_gpio,
    .quadwp_io_num = -1,
    .quadhd_io_num = -1,
  };

  spi_bus_initialize(spi->host, &buscfg, 0);
  spi_bus_add_device(spi->host, &devcfg, &spi->spi_handle);
}

void Rc522Spi::Thread() {
  if(_thread_running) { 
    ESP_LOGI(TAG_RC522SPI, "%d: %s(): Thread already running..", __LINE__, __FILE__);
    return;
  }

  _thread_running = true;
  while(true) {
    ESP_LOGI(TAG_RC522SPI, "%d: %s(): SPI looping..", __LINE__, __FILE__);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

static void taskWrapper(void *pvParameters) {
  Rc522Spi *rc522Spi = static_cast<Rc522Spi *>(pvParameters);
  rc522Spi->Thread();
}

void Rc522Spi::thread_spi(SpiDevice *spi)
{
  xTaskCreate(taskWrapper, "taskWrapper", 4096, this, 1, nullptr);
}
