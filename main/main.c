#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"

#include "driver/spi_master.h"
#include "driver/gpio.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"
#include "esp_system.h"
#include "nvs_flash.h"

#include "TI_CC_CC1100-CC2500.h"
#include "cc1101.h"

#include "esp32wmbusmeters.h"

/* Sources:
 * https://github.com/espressif/esp-idf.git/examples/peripherals/spi_master
 * https://github.com/nopnop2002/esp-idf-cc1101/components/cc1101
 * http://www.ti.com/lit/zip/slaa325
 * http://www.ti.com/lit/zip/SWRA234
 */

const char *wmbus_name   = CONFIG_WMBUS_NAME;
const char *wmbus_driver = CONFIG_WMBUS_DRIVER;
const char *wmbus_id     = CONFIG_WMBUS_ID;
const char *wmbus_key    = CONFIG_WMBUS_KEY;

#if CONFIG_SPI2_HOST
#define SPI_HOST SPI2_HOST
#elif CONFIG_SPI3_HOST
#define SPI_HOST SPI3_HOST
#endif

gpio_num_t miso = CONFIG_SPI_MISO_GPIO;
gpio_num_t mosi = CONFIG_SPI_MOSI_GPIO;
gpio_num_t sck  = CONFIG_SPI_SCK_GPIO;
gpio_num_t csn  = CONFIG_SPI_CSN_GPIO;
gpio_num_t gpi0 = CONFIG_SPI_GDO0_GPIO;
gpio_num_t gpi2 = CONFIG_SPI_GDO2_GPIO;
spi_device_handle_t spi;

void wifi_init(void);
void obtain_time(void);

static RingbufHandle_t rxring = NULL;
static TaskHandle_t rxtask = NULL;
static TaskHandle_t wmbus = NULL;

void app_main(void)
{
  gpio_config_t gpi0cfg = {
    .pin_bit_mask=1ULL<<gpi0,
    .mode = GPIO_MODE_INPUT,
    .intr_type=GPIO_INTR_POSEDGE,
    .pull_up_en = 1,
  };
  gpio_config_t gpi2cfg = {
    .pin_bit_mask=1ULL<<gpi2,
    .mode = GPIO_MODE_INPUT,
    .intr_type=GPIO_INTR_POSEDGE,
    .pull_up_en = 1,
  };
  gpio_config_t csncfg = {
    .pin_bit_mask=1ULL<<csn,
    .mode = GPIO_MODE_OUTPUT,
  };
  spi_bus_config_t buscfg = {
    .miso_io_num=miso,
    .mosi_io_num=mosi,
    .sclk_io_num=sck,
    .quadwp_io_num=-1,
    .quadhd_io_num=-1,
  };
  spi_device_interface_config_t devcfg = {
    .clock_speed_hz = 5000000, // SPI clock is 5 MHz!
    .queue_size = 7,
    .mode = 0, // SPI mode 0
    .spics_io_num = csn,
    .flags = SPI_DEVICE_NO_DUMMY,
  };
  //Init NVS
  esp_err_t ret = nvs_flash_init();
  if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
    ESP_ERROR_CHECK(nvs_flash_erase());
    ret = nvs_flash_init();
  }
  ESP_ERROR_CHECK(ret);
  //Connect to wifi, sync time
  wifi_init();
  obtain_time();
  //Create meter
  rxring = xRingbufferCreate(1028, RINGBUF_TYPE_NOSPLIT);
  esp32meter(wmbus_name, wmbus_driver, wmbus_id, wmbus_key);
  xTaskCreate(&esp32frame, "WMBUS", 1024*16, rxring, tskIDLE_PRIORITY, &wmbus);
  //Setup GPIO
  ESP_ERROR_CHECK(gpio_config(&gpi0cfg));
  ESP_ERROR_CHECK(gpio_config(&gpi2cfg));
  ESP_ERROR_CHECK(gpio_config(&csncfg));
  //Initialize the SPI bus, attach the CC1101
  ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
  ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi));
  //Initialize and configure CC1101
  cc1101_setup(csn, miso, gpi0, gpi2, spi);
  xTaskCreate(&cc1101_rxtask, "CC1101", 1024*16, rxring, configMAX_PRIORITIES - 1, &rxtask);
  //Restart
  for (int i = 0x10000; i > 0; i--) {
    printf("Restarting in %d seconds...\n", 8*i);
    vTaskDelay(8000 / portTICK_PERIOD_MS);
  }
  printf("Restarting now.\n");
  fflush(stdout);
  esp_restart();    
}
