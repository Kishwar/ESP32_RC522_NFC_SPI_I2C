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
#include <memory>
#include <cstring>

#include "rc522_spi.h"
#include "rc522_registers.h"
#include "esp_log.h"

const char TAG_RC522SPI[] = "RC522SPI";

Rc522Spi::Rc522Spi() {
  ESP_LOGI(TAG_RC522SPI, "%d: %s(): ENTER", __LINE__, __func__);
  spi = new SpiDevice(RC522_DEFAULT_SCAN_INTERVAL_MS,
                      RC522_DEFAULT_TASK_STACK_SIZE, RC522_DEFAULT_TASK_STACK_PRIORITY,
                      VSPI_HOST, PIN_MISO, PIN_MOSI, PIN_SCK, PIN_SS,
                      0 /*not used*/, RC522_DEFAULT_SPI_CLOCK_SPEED_HZ, 0);

  // initialize hardware peripherals
  init_spi();

  esp_event_loop_args_t event_args;
  memset(&event_args, 0, sizeof(esp_event_loop_args_t));
  event_args.queue_size = 1;
  event_args.task_name = nullptr;

  esp_event_loop_create(&event_args, &(spi->event_handle));

  // trigger a thread to scan any incoming card
  thread_spi();
  ESP_LOGI(TAG_RC522SPI, "%d: %s(): EXIT", __LINE__, __func__);
}

Rc522Spi::~Rc522Spi() {
  ESP_LOGI(TAG_RC522SPI, "%d: %s(): ENTER", __LINE__, __func__);
}

void Rc522Spi::init_spi() {
  ESP_LOGI(TAG_RC522SPI, "%d: %s(): ENTER", __LINE__, __func__);
  
  spi_device_interface_config_t devcfg;
  memset(&devcfg, 0, sizeof(spi_device_interface_config_t));
  devcfg.mode = 0;
  devcfg.clock_speed_hz = spi->clock_speed_hz;
  devcfg.spics_io_num = spi->sda_gpio;
  devcfg.flags = spi->device_flags;
  devcfg.queue_size = 7;

  spi_bus_config_t buscfg;
  memset(&buscfg, 0, sizeof(spi_bus_config_t));
  buscfg.mosi_io_num = spi->mosi_gpio;
  buscfg.miso_io_num = spi->miso_gpio;
  buscfg.sclk_io_num = spi->sck_gpio;
  buscfg.quadwp_io_num = -1;
  buscfg.quadhd_io_num = -1;

  ESP_LOGI(TAG, "devcfg: [%u, %u, %u, %u, %u]", (unsigned int)devcfg.mode, (unsigned int)devcfg.clock_speed_hz,
                                                (unsigned int)devcfg.spics_io_num, (unsigned int)devcfg.flags,
                                                (unsigned int)devcfg.queue_size);

  ESP_LOGI(TAG, "buscfg: [%u, %u, %u, %u, %u, %u]", (unsigned int)spi->host, (unsigned int)buscfg.mosi_io_num, (unsigned int)buscfg.miso_io_num,
                                                (unsigned int)buscfg.sclk_io_num, (unsigned int)buscfg.quadwp_io_num,
                                                (unsigned int)buscfg.quadhd_io_num);

  spi_bus_initialize(spi->host, &buscfg, 0);
  spi_bus_add_device(spi->host, &devcfg, &spi->spi_handle);
  ESP_LOGI(TAG_RC522SPI, "%d: %s(): handle: %p EXIT", __LINE__, __func__, spi->spi_handle);
}

void Rc522Spi::Thread() {
  if(_thread_running) { 
    ESP_LOGI(TAG_RC522SPI, "%d: %s(): Thread already running..", __LINE__, __func__);
    return;
  }

  _thread_running = true;
  while(true) {
    //if(!spi->scanning) { // are we scanning, if yes then wait
      vTaskDelay(100 / portTICK_PERIOD_MS);
    //  continue;
    //}

    uint8_t* serial_no_array = nullptr;
    if(ESP_OK != GetTag(&serial_no_array)) {
      ESP_LOGI(TAG_RC522SPI, "%d: %s(): Tag not present", __LINE__, __func__);
    }

    vTaskDelay((spi->scan_interval_ms * 5) / portTICK_PERIOD_MS);
  }
}

static void taskWrapper(void *pvParameters) {
  Rc522Spi *rc522Spi = static_cast<Rc522Spi *>(pvParameters);
  rc522Spi->Thread();
}

void Rc522Spi::thread_spi()
{
  xTaskCreate(taskWrapper, "taskWrapper", 4096, this, 1, nullptr);
}

esp_err_t Rc522Spi::Send(uint8_t *buffer, uint8_t length)
{
  if(buffer == nullptr || length == 0) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): Invalid arguments", __LINE__, __func__);
    return ESP_ERR_INVALID_ARG;
  }

  buffer[0] = (buffer[0] << 1) & 0x7E;
  spi_transaction_t transaction;
  std::memset(&transaction, 0, sizeof(spi_transaction_t));
  transaction.length = (8 * length);
  transaction.tx_buffer = buffer;

  esp_err_t err = spi_device_transmit(spi->spi_handle, &transaction);
  return err;
}

esp_err_t Rc522Spi::Receive(uint8_t *buffer, uint8_t length, uint8_t addr)
{
  if(buffer == nullptr || length == 0) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): Invalid arguments", __LINE__, __func__);
    return ESP_ERR_INVALID_ARG;
  }

  esp_err_t err = ESP_OK;
  addr = ((addr << 1) & 0x7E) | 0x80;
  spi_transaction_t transaction;
  std::memset(&transaction, 0, sizeof(spi_transaction_t));
  if(SPI_DEVICE_HALFDUPLEX & spi->device_flags) {
    transaction.flags = SPI_TRANS_USE_TXDATA;
    transaction.length = 8;
    transaction.rxlength = (8 * length);
    transaction.tx_data[0] = addr;
    transaction.rx_buffer = buffer;
    err = spi_device_transmit(spi->spi_handle, &transaction);
  } else { // Fullduplex
    transaction.flags = SPI_TRANS_USE_TXDATA;
    transaction.length = 8;
    transaction.tx_data[0] = addr;
    err = spi_device_transmit(spi->spi_handle, &transaction);
    if(err == ESP_OK) {
      transaction.flags = 0x00;
      transaction.length = 8;
      transaction.rxlength = (8 * length);
      transaction.rx_buffer = buffer;
      transaction.tx_buffer = nullptr;
      err = spi_device_transmit(spi->spi_handle, &transaction);
    }
  }
  return err;
}

esp_err_t Rc522Spi::GetTag(uint8_t **result) {
  esp_err_t err = ESP_OK;
  uint8_t *_result = nullptr;
  uint8_t *res_data = nullptr;
  uint8_t res_data_n;
  ESP_LOGI(TAG_RC522SPI, "%d: %s()", __LINE__, __func__);
  if((err = Request(&res_data_n, &res_data))) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): Request failed with error %d", __LINE__, __func__, err);
    return err;
  }

  if(res_data != nullptr) {
    delete[] res_data;
    res_data = nullptr;

    if((err = AntiCollision(&_result))) {
      ESP_LOGE(TAG_RC522SPI, "%d: %s(): AntiCollision failed with error %d", __LINE__, __func__, err);
      return err;
    }

    if(_result != nullptr) {
      uint8_t buf[] = { 0x50, 0x00, 0x00, 0x00 };
      if((err = CRC(buf, 2, buf + 2))) {
        ESP_LOGE(TAG_RC522SPI, "%d: %s(): CRC failed with error %d", __LINE__, __func__, err);
        return err;
      }

      if((err = Write(0x0C, buf, 4, &res_data_n, &res_data))) {
        ESP_LOGE(TAG_RC522SPI, "%d: %s(): Write failed with error %d", __LINE__, __func__, err);
        return err;
      }

      delete[] res_data;
      if((err = ClearBitMask(RC522_STATUS_2_REG, 0x08))) {
        ESP_LOGE(TAG_RC522SPI, "%d: %s(): ClearBitMask failed with error %d", __LINE__, __func__, err);
        return err;
      }
    }
  }

  *result = _result;
  return err;
}

esp_err_t Rc522Spi::CRC(uint8_t *data, uint8_t n, uint8_t* buffer)
{
  esp_err_t err = ESP_OK;
  uint8_t i = 255;
  uint8_t nn = 0;
  ESP_LOGI(TAG_RC522SPI, "%d: %s()", __LINE__, __func__);
  if((err = ClearBitMask(RC522_DIV_INT_REQ_REG, 0x04))) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): ClearBitMask failed with error %d", __LINE__, __func__, err);
    return err;
  }

  if((err = SetBitMask(RC522_FIFO_LEVEL_REG, 0x80))) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): SetBitMask failed with error %d", __LINE__, __func__, err);
    return err;
  }

  if((err = Write(RC522_FIFO_DATA_REG, n, data))) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): Write failed with error %d", __LINE__, __func__, err);
    return err;
  }

  if((err = Write(RC522_COMMAND_REG, 0x03))) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): Write failed with error %d", __LINE__, __func__, err);
    return err;
  }

  while(true) {
    if((err = Read(RC522_DIV_INT_REQ_REG, &nn))) {
      ESP_LOGE(TAG_RC522SPI, "%d: %s(): Read failed with error %d", __LINE__, __func__, err);
      return err;
    }
    i--;
    if(! (i != 0 && ! (nn & 0x04))) {
      break;
    }
  }

  uint8_t tmp;
  if((err = Read(RC522_CRC_RESULT_LSB_REG, &tmp))) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): Read failed with error %d", __LINE__, __func__, err);
    return err;
  }
  buffer[0] = tmp;
  if((err = Read(RC522_CRC_RESULT_MSB_REG, &tmp))) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): Read failed with error %d", __LINE__, __func__, err);
    return err;
  }
  buffer[1] = tmp;
  return ESP_OK;
}

esp_err_t Rc522Spi::AntiCollision(uint8_t **result) {
  esp_err_t err = ESP_OK;
  uint8_t* _result = NULL;
  uint8_t _res_n;

  ESP_LOGI(TAG_RC522SPI, "%d: %s()", __LINE__, __func__);
  if((err = Write(RC522_BIT_FRAMING_REG, 0x00))) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): Write failed with error %d", __LINE__, __func__, err);
    return err;
  }

  uint8_t _tmp[] = { 0x93, 0x20 };
  if((err = Write(0x0C, _tmp, 2, &_res_n, &_result))) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): Write failed with error %d", __LINE__, __func__, err);
    return err;
  }

  // TODO: Some cards have length of 4, and some of them have length of 7 bytes
  //       here we are using one extra byte which is not part of UID.
  //       Implement logic to determine the length of the UID and use that info
  //       to retrieve the serial number aka UID
  if(_result && _res_n != 5) { // all cards/tags serial numbers is 5 bytes long (??)
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): invalid length of serial number _res_n: %d", __LINE__, __func__, _res_n);
    return ESP_ERR_INVALID_RESPONSE;
  }

  *result = _result;
  return err;
}

esp_err_t Rc522Spi::Request(uint8_t *res_n, uint8_t **result) {
  esp_err_t err = ESP_OK;
  uint8_t *_result = nullptr;
  uint8_t _res_n = 0;
  uint8_t req_mode = 0x26;
  ESP_LOGI(TAG_RC522SPI, "%d: %s()", __LINE__, __func__);
  if((err = Write(RC522_BIT_FRAMING_REG, 0x07))) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): Write failed with error %d", __LINE__, __func__, err);
    return err;
  }

  if((err = Write(0x0C, &req_mode, 1, &_res_n, &_result))) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): Write failed with error %d", __LINE__, __func__, err);
    return err;
  }

  if((_res_n * 8) != 0x10) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): _res_n error %d", __LINE__, __func__, err);
    delete[] _result;
    _result =  nullptr;
    *res_n = _res_n;
    *result = _result;
    return ESP_ERR_INVALID_STATE;
  }

  *res_n = _res_n;
  *result = _result;
  return err;
}

esp_err_t Rc522Spi::Read(uint8_t addr, uint8_t n, uint8_t* buffer)
{
  return Receive(buffer, n, addr);
}

esp_err_t Rc522Spi::Read(uint8_t addr, uint8_t* value_ref)
{
  return Read(addr, 1, value_ref);
}

esp_err_t Rc522Spi::ClearBitMask(uint8_t addr, uint8_t mask)
{
    esp_err_t err = ESP_OK;
    uint8_t tmp;

    if((err = Read(addr, &tmp))) {
      ESP_LOGE(TAG_RC522SPI, "%d: %s(): Read failed with error %d", __LINE__, __func__, err);
      return err;
    }

    return Write(addr, tmp & ~mask);
}

esp_err_t Rc522Spi::SetBitMask(uint8_t addr, uint8_t mask)
{
    esp_err_t err = ESP_OK;
    uint8_t tmp;

    if((err = Read(addr, &tmp))) {
      ESP_LOGE(TAG_RC522SPI, "%d: %s(): Read failed with error %d", __LINE__, __func__, err);
      return err;
    }

    return Write(addr, tmp | mask);
}

esp_err_t Rc522Spi::Write(uint8_t cmd, uint8_t *data, uint8_t n, uint8_t *res_n, uint8_t **result) {
  esp_err_t err = ESP_OK;
  uint8_t* _result = NULL;
  uint8_t _res_n = 0;
  uint8_t irq = 0x00;
  uint8_t irq_wait = 0x00;
  uint8_t last_bits = 0;
  uint8_t nn = 0;
  uint8_t tmp;

  if(cmd == 0x0E) {
    irq = 0x12;
    irq_wait = 0x10;
  }
  else if(cmd == 0x0C) {
    irq = 0x77;
    irq_wait = 0x30;
  }

  if((err = Write(RC522_COMM_INT_EN_REG, irq | 0x80) != ESP_OK)) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): Write failed with error %d", __LINE__, __func__, err);
    return err;
  }

  if((err = ClearBitMask(RC522_COMM_INT_REQ_REG, 0x80) != ESP_OK)) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): ClearBitMask failed with error %d", __LINE__, __func__, err);
    return err;
  }

  if((err = SetBitMask(RC522_FIFO_LEVEL_REG, 0x80) != ESP_OK)) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): SetBitMask failed with error %d", __LINE__, __func__, err);
    return err;
  }

  if((err = Write(RC522_COMMAND_REG, 0x00) != ESP_OK)) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): Write failed with error %d", __LINE__, __func__, err);
    return err;
  }

  if((err = Write(RC522_FIFO_DATA_REG, n, data) != ESP_OK)) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): Write failed with error %d", __LINE__, __func__, err);
    return err;
  }

  if((err = Write(RC522_COMMAND_REG, cmd) != ESP_OK)) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): Write failed with error %d", __LINE__, __func__, err);
    return err;
  }

  if(cmd == 0x0C) {
    if((err = SetBitMask(RC522_BIT_FRAMING_REG, 0x80) != ESP_OK)) {
      ESP_LOGE(TAG_RC522SPI, "%d: %s(): SetBitMask failed with error %d", __LINE__, __func__, err);
      return err;
    }
  }

  uint16_t i = 1000;

  while(true) {
    if((err = Read(RC522_COMM_INT_REQ_REG, &nn) != ESP_OK)) {
      ESP_LOGE(TAG_RC522SPI, "%d: %s(): Read failed with error %d", __LINE__, __func__, err);
      return err;
    }
    i--;

    if(!(i != 0 && (((nn & 0x01) == 0) && ((nn & irq_wait) == 0)))) {
      break;
    }
  }

  if((err = ClearBitMask(RC522_BIT_FRAMING_REG, 0x80) != ESP_OK)) {
    ESP_LOGE(TAG_RC522SPI, "%d: %s(): ClearBitMask failed with error %d", __LINE__, __func__, err);
    return err;
  }

  if(i != 0) {
    if((err = Read(RC522_ERROR_REG, &tmp) != ESP_OK)) {
      ESP_LOGE(TAG_RC522SPI, "%d: %s(): Read failed with error %d", __LINE__, __func__, err);
      return err;
    }

    if((tmp & 0x1B) == 0x00) {
      if(cmd == 0x0C) {
        if((err = Read(RC522_FIFO_LEVEL_REG, &nn) != ESP_OK)) {
          ESP_LOGE(TAG_RC522SPI, "%d: %s(): Read failed with error %d", __LINE__, __func__, err);
          return err;
        }

        if((err = Read(RC522_CONTROL_REG, &tmp) != ESP_OK)) {
          ESP_LOGE(TAG_RC522SPI, "%d: %s(): Read failed with error %d", __LINE__, __func__, err);
          return err;
        }

        last_bits = tmp & 0x07;
        if (last_bits != 0) {
          _res_n = (nn - 1) + last_bits;
        } else {
          _res_n = nn;
        }
        if(_res_n > 0) {
          _result = new uint8_t[_res_n];
          for(i = 0; i < _res_n; i++) {
            if((err = Read(RC522_FIFO_DATA_REG, &tmp) != ESP_OK)) {
              ESP_LOGE(TAG_RC522SPI, "%d: %s(): Read failed with error %d", __LINE__, __func__, err);
              delete[] _result;
              _result = nullptr;
              return err;
            }
            _result[i] = tmp;
          }
        }
      }
    }
  }
  
  *res_n = _res_n;
  *result = _result;

  return err;
}

esp_err_t Rc522Spi::Write(uint8_t addr, uint8_t n, uint8_t *data) {
  std::unique_ptr<uint8_t[]> buffer = std::make_unique<uint8_t[]>(n+1);
  buffer.get()[0] = addr;
  std::memcpy(buffer.get() + 1, data, n);

  return Send(buffer.get(), n+1);
}

esp_err_t Rc522Spi::Write(uint8_t addr, uint8_t val) {
  return Write(addr, 1, &val);
}