#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/ringbuf.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "driver/gpio.h"

#define LOG_LOCAL_LEVEL ESP_LOG_VERBOSE
#include "esp_log.h"

#include "TI_CC_CC1100-CC2500.h"
#include "smode_rf_settings.h"
#include "tmode_rf_settings.h"
#include "cc1101.h"

/* Sources:
 * https://github.com/espressif/esp-idf.git/examples/peripherals/spi_master
 * https://github.com/nopnop2002/esp-idf-cc1101/components/cc1101
 * http://www.ti.com/lit/zip/slaa325
 * http://www.ti.com/lit/zip/SWRA234
 */

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

static RingbufHandle_t buf_handle = NULL;
static QueueHandle_t evt_queue = NULL;

static void IRAM_ATTR gpio_isr(void *arg)
{
  xQueueSendFromISR(evt_queue, arg, NULL);  
}

void rx_task(void *arg)
{
  int i;
  gpio_num_t gp;
  uint8_t len, val;
  uint8_t off, buf[256];
  ESP_LOGI(pcTaskGetName(0), "Start");
  //Poll for data
  for (;;) {
    //Wait for interrupt
    if (xQueueReceive(evt_queue, &gp, portMAX_DELAY)) {
      ESP_LOGV("CC1101", "%-20s event %d", __FUNCTION__, gp);
    } else {
      ESP_LOGD("CC1101", "%-20s timeout", __FUNCTION__);
    }
    //Acquire bus
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
    //Drain FIFO
    off = 0;
    for (;;) {
      val = cc1101_read(csn, miso, spi, TI_CCxxx0_RXBYTES | TI_CCxxx0_READ_BURST, &len);
      ESP_LOGV("CC1101", "%-20s 0x%02x 0x%02x", "CCxxx0_RXBYTES", val, len);
      len &= 0x7f;
      if (len > sizeof(buf) - off) len = sizeof(buf) - off;
      if (len == 0) break;
      cc1101_xmit(csn, miso, spi, TI_CCxxx0_RXFIFO | TI_CCxxx0_READ_BURST, NULL, buf+off, len);
      off += len;
    }
    if (off) {
      printf("%-20s 0x%02x bytes ","CCxxx0_RXFIFO", off);
      for (i=0;i<off;i++) {
	printf("%02x",buf[i]);
      }
      printf("\n");
    }
    switch (val & 0x70) {
    case 0x10: /* RX */
      ESP_LOGV("CC1101", "%-20s 0x%02x", "CCxxx0_RXBYTES", val);
      break;
    default:
      //Idle state
      do {
	val = cc1101_send(csn, miso, spi, TI_CCxxx0_SIDLE);
	ESP_LOGV("CC1101", "%-20s 0x%02x", "CCxxx0_SIDLE", val);
      } while ((val & 0x70) != 0x00);
      //Flush FIFO
      val = cc1101_send(csn, miso, spi, TI_CCxxx0_SFRX);
      ESP_LOGV("CC1101", "%-20s 0x%02x", "CCxxx0_SFRX", val);
      //RX state
      val = cc1101_send(csn, miso, spi, TI_CCxxx0_SRX);
      ESP_LOGV("CC1101", "%-20s 0x%02x", "CCxxx0_SRX", val);
      break;
    }
    //Release bus
    spi_device_release_bus(spi);
  }
  // never reach here
  vTaskDelete(NULL);
}

void app_main(void)
{
  int i;
  uint8_t buf[256];
  uint8_t val, ans;
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
  //Ring buffer and event queue
  buf_handle = xRingbufferCreate(1028, RINGBUF_TYPE_NOSPLIT);
  evt_queue = xQueueCreate(2, sizeof(gpio_num_t));
  //Setup GPIO
  ESP_ERROR_CHECK(gpio_config(&gpi0cfg));
  ESP_ERROR_CHECK(gpio_config(&gpi2cfg));
  ESP_ERROR_CHECK(gpio_config(&csncfg));
  //Initialize the SPI bus
  ESP_ERROR_CHECK(spi_bus_initialize(SPI_HOST, &buscfg, SPI_DMA_CH_AUTO));
  //Attach the CC1101 to the SPI bus
  ESP_ERROR_CHECK(spi_bus_add_device(SPI_HOST, &devcfg, &spi));
  //Pulse CSN
  gpio_set_level(csn, 1);
  esp_rom_delay_us(5);
  gpio_set_level(csn, 0);
  esp_rom_delay_us(10);
  gpio_set_level(csn, 1);
  esp_rom_delay_us(41);
  //Install handlers
  ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3));
  ESP_ERROR_CHECK(gpio_isr_handler_add(gpi0, gpio_isr, &gpi0));
  ESP_ERROR_CHECK(gpio_isr_handler_add(gpi0, gpio_isr, &gpi2));
  //Acquire bus
  ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
  //Strobe reset 
  val = cc1101_send(csn, miso, spi, TI_CCxxx0_SRES);
  ESP_LOGI("CC1101", "%-20s 0x%02x", "CCxxx0_SRES", val);
  esp_rom_delay_us(100);
  //Read P/N
  val = cc1101_read(csn, miso, spi, TI_CCxxx0_PARTNUM | TI_CCxxx0_READ_BURST, &ans);
  ESP_LOGI("CC1101", "%-20s 0x%02x 0x%02x", "CC1101_PARTNUM", val, ans);
  //Read version
  val = cc1101_read(csn, miso, spi, TI_CCxxx0_VERSION | TI_CCxxx0_READ_BURST, &ans);
  ESP_LOGI("CC1101", "%-20s 0x%02x 0x%02x", "CC1101_VERSION", val, ans);
  //Write mode settings
  for (i=0; i<sizeof(tModeRfConfig)/sizeof(uint8_t); i+=2) {
    val = cc1101_write(csn, miso, spi, tModeRfConfig[i], tModeRfConfig[i+1]);      
    cc1101_read(csn, miso, spi, tModeRfConfig[i]  | TI_CCxxx0_READ_SINGLE, &ans);
    if (1 || ans != tModeRfConfig[i+1]) {
      ESP_LOGI("CC1101", "0x%02x 0x%02x 0x%02x 0x%02x", val, tModeRfConfig[i], tModeRfConfig[i+1], ans);
    }
  }
  //Write patable
  val = cc1101_xmit(csn, miso, spi, TI_CCxxx0_PATABLE | TI_CCxxx0_WRITE_BURST, tModePaTable, NULL, tModePaTableLen);
  ans = cc1101_xmit(csn, miso, spi, TI_CCxxx0_PATABLE | TI_CCxxx0_READ_BURST, NULL, buf, tModePaTableLen);
  for (i=0; i<tModePaTableLen; i++) {
    if (1 || buf[i] != tModePaTable[i]) {
      ESP_LOGI("CC1101", "0x%02x 0x%02x 0x%02x 0x%02x", val, tModePaTable[i], buf[i], ans);
    }
  }
  //MCSM1.RXOFF_MODE[3:2]   0 RX -> IDLE after a packet has been received
  val = cc1101_write(csn, miso, spi, TI_CCxxx0_MCSM1, 0x00);
  cc1101_read(csn, miso, spi, TI_CCxxx0_MCSM1 | TI_CCxxx0_READ_SINGLE, &ans);
  ESP_LOGI("CC1101", "%-20s 0x%02x 0x%02x", "CC1101_MCSM1", val, ans);
  //SYNC1 = 0x54
  val = cc1101_write(csn, miso, spi, TI_CCxxx0_SYNC1, 0x54);
  cc1101_read(csn, miso, spi, TI_CCxxx0_SYNC1 | TI_CCxxx0_READ_SINGLE, &ans);
  ESP_LOGI("CC1101", "%-20s 0x%02x 0x%02x", "CC1101_SYNC1", val, ans);
  //SYNC0 = 0x3D
  val = cc1101_write(csn, miso, spi, TI_CCxxx0_SYNC0, 0x3D);
  cc1101_read(csn, miso, spi, TI_CCxxx0_SYNC0 | TI_CCxxx0_READ_SINGLE, &ans);
  ESP_LOGI("CC1101", "%-20s 0x%02x 0x%02x", "CC1101_SYNC0", val, ans);
  // MDMCFG2[2:0] SYNC_MODE 3 - 30/32 sync word bits detected
  //              SYNC_MODE 7 - 30/32 + carrier sense above threshold
  val = cc1101_write(csn, miso, spi, TI_CCxxx0_MDMCFG2, 0x07);
  cc1101_read(csn, miso, spi, TI_CCxxx0_MDMCFG2 | TI_CCxxx0_READ_SINGLE, &ans);
  ESP_LOGI("CC1101", "%-20s 0x%02x 0x%02x", "CC1101_MDMCFG2", val, ans);
  // PKTLEN[7:0] PACKET_LENGTH 0x80
  val = cc1101_write(csn, miso, spi, TI_CCxxx0_PKTLEN, 0x80);    
  cc1101_read(csn, miso, spi, TI_CCxxx0_PKTLEN | TI_CCxxx0_READ_SINGLE, &ans);
  ESP_LOGI("CC1101", "%-20s 0x%02x 0x%02x", "CC1101_PKTLEN", val, ans);
  // PKTCTRL1[7:5] PQT - quality estimator
  // PKTCTRL1[2:2] APPEND_STATUS               
  val = cc1101_write(csn, miso, spi, TI_CCxxx0_PKTCTRL1, 0x04);    
  cc1101_read(csn, miso, spi, TI_CCxxx0_PKTCTRL1 | TI_CCxxx0_READ_SINGLE, &ans);
  ESP_LOGI("CC1101", "%-20s 0x%02x 0x%02x", "CC1101_PKTCTRL1", val, ans);
  // PKTCTRL0[2:2] CRC_EN        0 - CRC disabled
  // PKTCTRL0[1:0] LENGTH_CONFIG 0 - Fixed packet length
  //               LENGTH_CONFIG 1 - Variable packet length
  //               LENGTH_CONFIG 2 - Infinite packet length
  
  val = cc1101_write(csn, miso, spi, TI_CCxxx0_PKTCTRL0, 0x01);    
  cc1101_read(csn, miso, spi, TI_CCxxx0_PKTCTRL0 | TI_CCxxx0_READ_SINGLE, &ans);
  ESP_LOGI("CC1101", "%-20s 0x%02x 0x%02x", "CC1101_PKTCTRL0", val, ans);
  //RX FIFO threshold is 4 bytes
  val = cc1101_write(csn, miso, spi, TI_CCxxx0_FIFOTHR, 0x00);    
  cc1101_read(csn, miso, spi, TI_CCxxx0_FIFOTHR | TI_CCxxx0_READ_SINGLE, &ans);
  ESP_LOGI("CC1101", "%-20s 0x%02x 0x%02x", "CC1101_FIFOTHR", val, ans);
  //GDO0 is RX FIFO threshold signal
  val = cc1101_write(csn, miso, spi, TI_CCxxx0_IOCFG0, 0x01);
  cc1101_read(csn, miso, spi, TI_CCxxx0_IOCFG0 | TI_CCxxx0_READ_SINGLE, &ans);
  ESP_LOGI("CC1101", "%-20s 0x%02x 0x%02x", "CC1101_IOCFG0", val, ans);
  //GDO2 is packet received signal
  val = cc1101_write(csn, miso, spi, TI_CCxxx0_IOCFG2, 0x06);
  cc1101_read(csn, miso, spi, TI_CCxxx0_IOCFG2 | TI_CCxxx0_READ_SINGLE, &ans);
  ESP_LOGI("CC1101", "%-20s 0x%02x 0x%02x", "CC1101_IOCFG2", val, ans);
  //Idle state
  do {
    val = cc1101_send(csn, miso, spi, TI_CCxxx0_SIDLE);
    ESP_LOGI("CC1101", "%-20s 0x%02x", "CCxxx0_SIDLE", val);
  } while ((val & 0x70) != 0x00);
  //Flush FIFO
  val = cc1101_send(csn, miso, spi, TI_CCxxx0_SFRX);
  ESP_LOGI("CC1101", "%-20s 0x%02x", "CCxxx0_SFRX", val);
  //RX state
  val = cc1101_send(csn, miso, spi, TI_CCxxx0_SRX);
  ESP_LOGI("CC1101", "%-20s 0x%02x", "CCxxx0_SRX", val);
  //Release bus
  spi_device_release_bus(spi);
  //Interrupt and handlers
  xTaskCreate(&rx_task, "RX", 1024*3, NULL, tskIDLE_PRIORITY, NULL);    
  //Restart
  for (int i = 100; i > 0; i--) {
    printf("Restarting in %d seconds...\n", 20*i);
    vTaskDelay(20000 / portTICK_PERIOD_MS);
  }
  printf("Restarting now.\n");
  fflush(stdout);
  esp_restart();    
}
