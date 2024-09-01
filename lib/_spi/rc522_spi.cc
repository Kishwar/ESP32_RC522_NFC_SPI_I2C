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

#include "rc522_spi.h"
#include "esp_log.h"

const char TAG[] = "RC522SPI";

Rc522Spi::Rc522Spi() {
  ESP_LOGI(TAG, "RC522SPI");
  std::unique_ptr<SpiDevice> spi = std::make_unique<SpiDevice>(RC522_DEFAULT_SCAN_INTERVAL_MS, RC522_DEFAULT_TASK_STACK_SIZE,
                                      RC522_DEFAULT_TASK_STACK_PRIORITY, (spi_host_device_t)0,
                                      PIN_MISO, PIN_MISO, PIN_SCK, PIN_SS, 0 /*not used*/,
                                      RC522_DEFAULT_SPI_CLOCK_SPEED_HZ, 0);

  // initialize hardware peripherals
  init_spi(spi);

  // trigger a thread to scan any incoming card
  thread_spi(spi);
  ESP_LOGI(TAG, "RC522SPI created");
}

Rc522Spi::~Rc522Spi() {
  ESP_LOGI(TAG, "RC522SPI destroyed");
}

void Rc522Spi::init_spi(std::unique_ptr<SpiDevice>& spi) {
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

void thread_spi(const std::unique_ptr<SpiDevice>& spi) {

}