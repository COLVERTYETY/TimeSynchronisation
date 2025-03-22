// #ifndef ESP_HAL_H
// #define ESP_HAL_H

// // include RadioLib
// #include <RadioLib.h>


// // include all the dependencies
// #include "freertos/FreeRTOS.h"
// #include "freertos/task.h"
// #include "driver/gpio.h"
// #include "driver/spi_master.h"
// #include "hal/gpio_hal.h"
// #include "esp_timer.h"
// #include "esp_log.h"
// #include <stdint.h>


// #define TAG_ "ESP_HAL"
// // define Arduino-style macros
// #define LOW                         (0x0)
// #define HIGH                        (0x1)
// #define INPUT                       (0x01)
// #define OUTPUT                      (0x02)
// #define RISING                      (0x01)
// #define FALLING                     (0x02)
// #define NOP()                       asm volatile ("nop")

// #define MATRIX_DETACH_OUT_SIG       (0x100)
// #define MATRIX_DETACH_IN_LOW_PIN    (0x30)


// spi_device_handle_t spi_handle;

// // create a new ESP-IDF hardware abstraction layer
// // the HAL must inherit from the base RadioLibHal class
// // and implement all of its virtual methods


// class EspHal : public RadioLibHal {
//   public:
//     // default constructor - initializes the base HAL and any needed private members
//     EspHal(uint8_t sck, uint8_t miso, uint8_t mosi, spi_host_device_t spi_host = SPI2_HOST, spi_device_handle_t existing_handle = nullptr)
//         : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING),
//           spiSCK(sck), spiMISO(miso), spiMOSI(mosi), spi_host(spi_host), spi_handle(existing_handle) {
//     }

//     void init() override {
//       // we only need to init the SPI here
//       spiBegin();
//     }

//     void term() override {
//       // we only need to stop the SPI here
//       spiEnd();
//     }

//     // GPIO-related methods (pinMode, digitalWrite etc.) should check
//     // RADIOLIB_NC as an alias for non-connected pins
//     void pinMode(uint32_t pin, uint32_t mode) override {
//         if (pin == RADIOLIB_NC) return;
//         gpio_set_direction((gpio_num_t)pin, (gpio_mode_t)mode);
//     }


//     void digitalWrite(uint32_t pin, uint32_t value) override {
//       if(pin == RADIOLIB_NC) {
//         return;
//       }
//       gpio_set_level((gpio_num_t)pin, value);
//     }

//     uint32_t digitalRead(uint32_t pin) override {
//       if(pin == RADIOLIB_NC) {
//         return(0);
//       }

//       return(gpio_get_level((gpio_num_t)pin));
//     }

//     void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override {
//       if(interruptNum == RADIOLIB_NC) {
//         return;
//       }

//       gpio_config_t io_conf = {
//         .pin_bit_mask = (1ULL << interruptNum),
//         .mode = GPIO_MODE_INPUT,
//         .pull_up_en = GPIO_PULLUP_DISABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_POSEDGE,  // Rising edge trigger
//       };
//       gpio_config(&io_conf);
//       gpio_install_isr_service(1);
//       gpio_isr_handler_add(static_cast<gpio_num_t>(interruptNum), (void (*)(void*))interruptCb, NULL);
//       gpio_intr_enable(static_cast<gpio_num_t>(interruptNum));
//     }

//     void detachInterrupt(uint32_t interruptNum) override {
//       if(interruptNum == RADIOLIB_NC) {
//         return;
//       }
//       gpio_isr_handler_remove((gpio_num_t)interruptNum);
// 	    gpio_wakeup_disable((gpio_num_t)interruptNum);
//       gpio_set_intr_type((gpio_num_t)interruptNum, GPIO_INTR_DISABLE);
//     }

//     void delay(unsigned long ms) override {
//       // vTaskDelay(ms / portTICK_PERIOD_MS);
//       vTaskDelay(pdMS_TO_TICKS(ms));
//     }

//     void delayMicroseconds(unsigned long us) override {
//       esp_rom_delay_us(us);
//     }

//     unsigned long millis() override {
//       return((unsigned long)(esp_timer_get_time() / 1000ULL));
//     }

//     unsigned long micros() override {
//       return((unsigned long)(esp_timer_get_time()));
//     }

//     long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override {
//       if(pin == RADIOLIB_NC) {
//         return(0);
//       }

//       this->pinMode(pin, INPUT);
//       uint32_t start = this->micros();
//       uint32_t curtick = this->micros();

//       while(this->digitalRead(pin) == state) {
//         if((this->micros() - curtick) > timeout) {
//           return(0);
//         }
//       }

//       return(this->micros() - start);
//     }

//     void spiBegin() {
//         if (spi_handle) {
//             ESP_LOGI(TAG_, "SPI device already attached to bus");
//             return;  // Device already added, skip re-adding
//         }

//         // Device configuration (can be adjusted based on requirements)
//         spi_device_interface_config_t devcfg = {
//             .mode = 0,                          // SPI mode 0
//             .clock_speed_hz = 2 * 1000 * 1000,  // 2 MHz
//             .spics_io_num = -1,                 // CS handled manually
//             .queue_size = 1,
//         };

//         // Always attempt to add the RadioLib device
//         esp_err_t ret = spi_bus_add_device(spi_host, &devcfg, &spi_handle);
//         if (ret != ESP_OK) {
//             ESP_LOGE(TAG_, "Failed to add RadioLib device to SPI bus: %d", ret);
//         } else {
//             ESP_LOGI(TAG_, "RadioLib device successfully added to SPI bus");
//         }
//     }


//     void spiBeginTransaction() {
//       // not needed - in ESP32 Arduino core, this function
//       // repeats clock div, mode and bit order configuration
//     //   gpio_set_level((gpio_num_t)cs_pin, 0);  // Assert CS (Active low)
//       spi_device_acquire_bus(spi_handle, portMAX_DELAY);  // Lock the SPI bus
//     }

//     uint8_t spiTransferByte(uint8_t b) {
//       spi_transaction_t t;
//       uint8_t rx_data = 0;
//       memset(&t, 0, sizeof(t));
//       t.length = 8;
//       t.tx_buffer = &b;
//       t.rx_buffer = &rx_data;
//       spi_device_polling_transmit(spi_handle, &t);
//       return rx_data;
//     }

//     void spiTransfer(uint8_t* out, size_t len, uint8_t* in) {
//         spi_transaction_t t;
//         memset(&t, 0, sizeof(t));
//         t.length = len * 8;
//         t.tx_buffer = out;
//         t.rx_buffer = in;
//         spi_device_polling_transmit(spi_handle, &t);
//     }


//     void spiEndTransaction() {
//       // nothing needs to be done here
//     //   gpio_set_level((gpio_num_t)cs_pin, 1);  // Deassert CS
//       spi_device_release_bus(spi_handle);  // Unlock the SPI bus
//     }

//     void spiEnd() {
//         if (spi_handle) {
//             spi_bus_remove_device(spi_handle);
//             spi_handle = NULL;
//             ESP_LOGI(TAG_, "SPI device removed from bus");
//         }
//     }


//   private:
//     // the HAL can contain any additional private members
//     uint8_t spiSCK;
//     uint8_t spiMISO;
//     uint8_t spiMOSI;
//     spi_host_device_t spi_host;
//     spi_device_handle_t spi_handle;
// };



// #endif
#ifndef ESP_HAL_H
#define ESP_HAL_H

// Include RadioLib base class
#include <RadioLib.h>

// Include ESP-IDF and FreeRTOS dependencies
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <stdint.h>
#include <string.h>
#include <map>

// Arduino-style macros
#define LOW                         (0x0)
#define HIGH                        (0x1)
// Note: while the original driver-based HAL defined OUTPUT as 0x02,
// the second implementation uses 0x03 (meaning “input/output”)
// so here we use 0x03 for more flexibility.
#define INPUT                       (0x01)
#define OUTPUT                      (0x03)
#define RISING                      (0x01)
#define FALLING                     (0x02)
#define NOP()                       asm volatile ("nop")

// These macros are only needed for detaching SPI pins (not used with the driver)
#define MATRIX_DETACH_OUT_SIG       (0x100)
#define MATRIX_DETACH_IN_LOW_PIN    (0x30)

// Logging tag
static const char* TAG_ = "ESP_HAL";

//------------------------------------------------------------------------------
// The EspHal class
// This version uses the ESP-IDF SPI driver for SPI transfers and
// incorporates improvements from the second implementation in GPIO,
// interrupt, and delay functions.
//------------------------------------------------------------------------------
class EspHal : public RadioLibHal {
  public:
  // Constructor with optional SPI host and pre-existing device handle
  EspHal(uint8_t sck, uint8_t miso, uint8_t mosi, spi_host_device_t spi_host = SPI2_HOST )
    : RadioLibHal(INPUT, OUTPUT, LOW, HIGH, RISING, FALLING),
    spiSCK(sck), spiMISO(miso), spiMOSI(mosi),
    spi_host(spi_host) {
    }

    // Initialize the HAL (SPI setup)
    void init() override {
      spiBegin();
    }

    // Terminate the HAL (SPI teardown)
    void term() override {
      spiEnd();
    }

    // Configure a GPIO pin using the ESP-IDF gpio_config_t structure.
    // This version disables pull-ups/downs and interrupts by default.
    void pinMode(uint32_t pin, uint32_t mode) override {
      // ESP_LOGI(TAG_, "Setting pin %" PRIu32 " to mode %" PRIu32, pin, mode);
      if(pin == RADIOLIB_NC)
        return;
        
      gpio_config_t conf = {
        .pin_bit_mask = (1ULL << pin),
        .mode         = (gpio_mode_t)mode,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE
      };
      gpio_config(&conf);
    }

    // Write a digital level to a pin
    void digitalWrite(uint32_t pin, uint32_t value) override {
      // ESP_LOGI(TAG_, "Writing %" PRIu32 " to pin %" PRIu32, value, pin);
      if(pin == RADIOLIB_NC)
        return;
      gpio_set_level((gpio_num_t)pin, value);
    }

    // Read the digital level from a pin
    uint32_t digitalRead(uint32_t pin) override {
      // ESP_LOGI(TAG_, "Reading from pin %" PRIu32, pin);
      if(pin == RADIOLIB_NC)
        return 0;
      return gpio_get_level((gpio_num_t)pin);
    }

    // Attach an interrupt to a pin.
    // This version installs the ISR service using ESP_INTR_FLAG_IRAM for faster response
    // and sets the interrupt type based on the passed mode.
    void attachInterrupt(uint32_t interruptNum, void (*interruptCb)(void), uint32_t mode) override {
      // ESP_LOGI(TAG_, "Attaching interrupt to pin %" PRIu32, interruptNum);
      if(interruptNum == RADIOLIB_NC)
        return;
      if(pin2interrupt.find(interruptNum) == pin2interrupt.end() || !pin2interrupt[interruptNum])
      {
        pin2interrupt[interruptNum] = true;
        gpio_install_isr_service(ESP_INTR_FLAG_LEVEL1); // ESP_INTR_FLAG_IRAM
        gpio_set_intr_type((gpio_num_t)interruptNum, (gpio_int_type_t)(mode));
        gpio_isr_handler_add((gpio_num_t)interruptNum, (void (*)(void*))interruptCb, NULL);
      } 
      // Install the ISR service with IRAM flag (if not already installed, ESP-IDF ignores duplicate calls)
      gpio_intr_enable((gpio_num_t)interruptNum);
    }

    // Detach the interrupt from a pin
    void detachInterrupt(uint32_t interruptNum) override {
      // ESP_LOGI(TAG_, "Detaching interrupt from pin %" PRIu32, interruptNum);
      if(interruptNum == RADIOLIB_NC)
        return;
      // gpio_isr_handler_remove((gpio_num_t)interruptNum);
      // gpio_wakeup_disable((gpio_num_t)interruptNum);
      // gpio_set_intr_type((gpio_num_t)interruptNum, GPIO_INTR_DISABLE);
      gpio_intr_disable((gpio_num_t)interruptNum);
    }

    // Delay for a specified number of milliseconds using FreeRTOS vTaskDelay.
    void delay(unsigned long ms) override {
      vTaskDelay(pdMS_TO_TICKS(ms));
    }

    // Delay for a specified number of microseconds using a busy-wait loop.
    void delayMicroseconds(unsigned long us) override {
      esp_rom_delay_us(us);
    }

    // Return the number of milliseconds since boot.
    unsigned long millis() override {
      return (unsigned long)(esp_timer_get_time() / 1000);
    }

    // Return the number of microseconds since boot.
    unsigned long micros() override {
      return (unsigned long)(esp_timer_get_time());
    }

    // Measure the length of a pulse on a pin.
    long pulseIn(uint32_t pin, uint32_t state, unsigned long timeout) override {
      if(pin == RADIOLIB_NC)
        return 0;
      pinMode(pin, INPUT);
      uint32_t start = micros();
      uint32_t now = micros();
      while (digitalRead(pin) == state) {
        if ((micros() - now) > timeout)
          return 0;
      }
      return micros() - start;
    }

    void yield() override {
      // ESP_LOGI(TAG_, "Yielding to other tasks");
      // Yield to other tasks
      vTaskDelay(1);
    }

    //-------------------------------------------------------------------------
    // SPI functions using the ESP-IDF SPI driver
    //-------------------------------------------------------------------------

    // Begin SPI communication by adding a device to the SPI bus.
    void spiBegin() {
      // if (spi_handle) {
      //   ESP_LOGI(TAG_, "SPI device already attached to bus");
      //   return;
      // }

      spi_device_interface_config_t devcfg = {
          .mode = 0,                        // SPI mode 0
          .clock_speed_hz = 5*1000000,          // 5 MHz SPI clock
          .spics_io_num = -1,               // CS handled manually
          .queue_size = 1,
      };
      size_t maxsize = 0;
      spi_bus_get_max_transaction_len(spi_host, &maxsize);
      ESP_LOGI(TAG_, "max size is %d", (int)maxsize);
      esp_err_t ret = spi_bus_add_device(spi_host, &devcfg, &spi_handle);
      if(ret != ESP_OK) {
        ESP_LOGE(TAG_, "Failed to add SPI device to bus: %d", ret);
      } else {
        ESP_LOGI(TAG_, "SPI device added successfully");
      }
    }

    // Begin an SPI transaction (acquire the bus)
    void spiBeginTransaction() {
      // ESP_LOGI(TAG_, "BEGINNING SPI TRANSACTION");
      spi_device_acquire_bus(spi_handle, portMAX_DELAY);
    }

    // Transfer one byte over SPI.
    uint8_t spiTransferByte(uint8_t b) {
      // ESP_LOGI(TAG_, "SPI BYTE TRANSFER");
      spi_transaction_t t;
      uint8_t rx_data = 0;
      memset(&t, 0, sizeof(t));
      t.length = 8;  // 8 bits
      t.tx_buffer = &b;
      t.rx_buffer = &rx_data;
      t.rxlength = 8;
      esp_err_t ret = spi_device_polling_transmit(spi_handle, &t);
      if (ret!=ESP_OK) {
        ESP_LOGE(TAG_, "SPI transfer failed: %d", ret);
      }
      return rx_data;
    }

    // Transfer a buffer over SPI.
    void spiTransfer(uint8_t* out, size_t len, uint8_t* in) override {
      // ESP_LOGI(TAG_, "SPI TRANSFER of size %d", len);
      spi_transaction_t t;
      memset(&t, 0, sizeof(t));
      t.length = len * 8;
      t.tx_buffer = out;
      t.rx_buffer = in;
      t.rxlength = len * 8;
      spi_device_polling_transmit(spi_handle, &t);
    }

    // End an SPI transaction (release the bus)
    void spiEndTransaction() {
      spi_device_release_bus(spi_handle);
      // ESP_LOGI(TAG_, "ENDING SPI TRANSACTION");
    }

    void spiEnd() {
      ESP_LOGI(TAG_, "Ending SPI");
      if (spi_handle) {
        spi_bus_remove_device(spi_handle);
        spi_handle = NULL;
        ESP_LOGI(TAG_, "SPI device removed from bus");
      }
    }


  private:
    uint8_t spiSCK;
    uint8_t spiMISO;
    uint8_t spiMOSI;
    spi_host_device_t spi_host;
    spi_device_handle_t spi_handle;
    bool interruptAttached = false;
    std::map<uint32_t, bool> pin2interrupt;  // Map to store pin interrupt status

};

#endif
