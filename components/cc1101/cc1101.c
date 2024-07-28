#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "freertos/ringbuf.h"

#include "TI_CC_CC1100-CC2500.h"
#include "smode_rf_settings.h"
#include "tmode_rf_settings.h"
#include "cc1101.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO
#include "esp_log.h"


static gpio_num_t csn = -1;
static gpio_num_t miso = -1;
static gpio_num_t gpi0 = -1;
static gpio_num_t gpi2 = -1;
static spi_device_handle_t spi = NULL;

// Initialize CC1101
void cc1101_setup(gpio_num_t csn_, gpio_num_t miso_, gpio_num_t gpi0_, gpio_num_t gpi2_, spi_device_handle_t spi_)
{
  //Set static variables
  csn  = csn_;
  miso = miso_;
  gpi0 = gpi0_;
  gpi2 = gpi2_;
  spi  = spi_;
  //Acquire bus
  ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
  //Pulse CSN
  gpio_set_level(csn, 1);
  esp_rom_delay_us(5);
  gpio_set_level(csn, 0);
  esp_rom_delay_us(10);
  gpio_set_level(csn, 1);
  esp_rom_delay_us(41);
  //Send reset
  cc1101_send(csn, miso, spi, TI_CCxxx0_SRES);
  //Read P/N and version
  cc1101_info(csn, miso, spi, TI_CCxxx0_PARTNUM|TI_CCxxx0_READ_BURST);
  cc1101_info(csn, miso, spi, TI_CCxxx0_VERSION|TI_CCxxx0_READ_BURST);
  //RF mode
  cc1101_tmode(csn, miso, spi);
  cc1101_cmode(csn, miso, spi);  
  //Release bus
  spi_device_release_bus(spi);
}

static uint8_t rxbuf[0x100];
static uint8_t rxlen = 0;

void IRAM_ATTR cc1101_rxisr(void *arg)
{
  static spi_transaction_t txn;
  //Acquire bus
  ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
  //Check fifo
  memset(&txn, 0, sizeof(spi_transaction_t));
  txn.length = 8*2;
  txn.flags |= SPI_TRANS_USE_TXDATA;
  txn.flags |= SPI_TRANS_USE_RXDATA;
  txn.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
  txn.tx_data[0] = TI_CCxxx0_RXBYTES | TI_CCxxx0_READ_BURST;
  gpio_set_level(csn, 0);
  while(gpio_get_level(miso)>0);
  ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &txn));
  uint8_t val = txn.rx_data[0] & 0x70; 
  uint8_t rem = txn.rx_data[1] & 0x7f;
  uint8_t num = (rem < (sizeof(rxbuf) - rxlen)) ? rem : (sizeof(rxbuf) - rxlen);
  //Prepare transfer
  memset(&txn, 0, sizeof(spi_transaction_t));
  txn.length = 8*1;
  txn.flags |= SPI_TRANS_USE_TXDATA;
  txn.flags |= SPI_TRANS_USE_RXDATA;
  txn.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
  txn.tx_data[0] = TI_CCxxx0_RXFIFO | TI_CCxxx0_READ_BURST;
  ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &txn));
  //Release bus
  spi_device_release_bus(spi);
  //Transfer data
  memset(&txn, 0, sizeof(spi_transaction_t));
  txn.length = 8*num;
  txn.tx_buffer = NULL;
  txn.rx_buffer = rxbuf+rxlen;
  rxlen += num;
  ESP_ERROR_CHECK(spi_device_queue_trans(spi, &txn, portMAX_DELAY));
  //Send to rxtask
  xQueueSendFromISR(arg, &val, NULL);
}

void cc1101_rxtask(void *ring)
{
  spi_transaction_t *ptr;
  //Install interrupt handlers
  ESP_LOGI("CC1101", "%s install ISR", __FUNCTION__);
  QueueHandle_t event = xQueueCreate(5, sizeof(uint32_t));
  ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3));
  ESP_ERROR_CHECK(gpio_isr_handler_add(gpi0, cc1101_rxisr, event));
  //Setup HW for IRQ
  ESP_LOGI("CC1101", "%s enable IRQ", __FUNCTION__);
  //Acquire bus
  ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
  // MCSM1.RXOFF_MODE[3:2] 0 - RX -> IDLE after a packet has been received
  cc1101_check(csn, miso, spi, TI_CCxxx0_MCSM1, 0x00);
  // FIFOTHR[7:0] 0 - RX FIFO 4 bytes
  //              3 - RX FIFO 16 bytes
  cc1101_check(csn, miso, spi, TI_CCxxx0_FIFOTHR, 0x03);    
  // IOCFG0[5:0] GDOn_CFG 0x00 - RX FIFO >= threshold
  //             GDOn_CFG 0x01 - RX FIFO >= threshold or end of packet
  //             GDOn_CFG 0x04 - RX FIFO has overflowed
  //             GDOn_CFG 0x06 - Sync word received
  cc1101_check(csn, miso, spi, TI_CCxxx0_IOCFG0, 0x01);
  cc1101_check(csn, miso, spi, TI_CCxxx0_IOCFG2, 0x04);
  //Start RX mode
  ESP_LOGI("CC1101", "%s starting RXMODE", __FUNCTION__);
  cc1101_rxmode(csn, miso, spi);
  //Release bus
  spi_device_release_bus(spi);
  //Wait for event
  for (;;) {
    uint8_t val;
    if (!xQueueReceive(event, &val, 120000/portTICK_PERIOD_MS)) {
      memset(rxbuf, 0, sizeof(rxbuf));
      rxlen = 0;
      ESP_LOGW("CC1101", "%s xQueueReceive timeout", __FUNCTION__);
      //Acquire bus
      ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
      //Start RX mode
      cc1101_rxmode(csn, miso, spi);
      //Release bus
      spi_device_release_bus(spi);
      continue;
    }
    //Wait for transaction
    ESP_ERROR_CHECK(spi_device_get_trans_result(spi, &ptr, portMAX_DELAY));
    //Format string
    char str[2*sizeof(rxbuf)+1];
    for (int i=0;i<rxlen;i++) {
      snprintf(str+2*i, 3, "%02x",rxbuf[i]);
    }
    str[2*rxlen] = 0;
    ESP_LOGI("CC1101", "%s 0x%02x %s", __FUNCTION__, rxlen, str);
    //Pass data
    if ((val & 0x70) == 0x00) {
      if (!xRingbufferSend(ring, rxbuf, rxlen, portMAX_DELAY)) {
	ESP_LOGW("CC1101", "%s 0x%02x %s", __FUNCTION__, rxlen, str);
      }
      memset(rxbuf, 0, sizeof(rxbuf));
      rxlen = 0;
    }
    //Acquire bus
    ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
    //Handle state
    switch (val & 0x70) {
    case 0x00: /* IDLE */
      ESP_LOGI("CC1101", "%-15s 0x%02x", "IDLE", val);
      cc1101_rxmode(csn, miso, spi);
      break;
    case 0x10: /* RX */
      ESP_LOGW("CC1101", "%-15s 0x%02x", "RX", val);
      break;
    case 0x20: /* TX */
      ESP_LOGE("CC1101", "%-15s 0x%02x", "TX", val);
      break;
    case 0x30: /* FSTXON */
      ESP_LOGE("CC1101", "%-15s 0x%02x", "FSTXON", val);
      break;
    case 0x40: /* CALIBRATE */
      ESP_LOGV("CC1101", "%-15s 0x%02x", "CALIBRATE", val);
      break;
    case 0x50: /* SETTLING */
      ESP_LOGV("CC1101", "%-15s 0x%02x", "SETTLING", val);
      break;
    case 0x60: /* RXFIFO_OVERFLOW */
      // Flush RXFIFO
      ESP_LOGW("CC1101", "%-15s 0x%02x", "RXFIFO_OVERFLOW", val);
      cc1101_rxmode(csn, miso, spi);
      break;
    case 0x70: /* TXFIFO_OVERFLOW */
      // Flush TXFIFO
      ESP_LOGE("CC1101", "%-15s 0x%02x", "TXFIFO_OVERFLOW", val);
      break;
    default:
      break;
    }
    //Release bus
    spi_device_release_bus(spi);
  }
  //Never reached
  vTaskDelete(NULL);  
}

uint8_t cc1101_txmode(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi)
{
  uint8_t val;
  //Enter IDLE state
  do {
    val = cc1101_send(csn, miso, spi, TI_CCxxx0_SIDLE);
  } while ((val & 0x70) != 0x00);
  //Flush TXFIFO
  cc1101_send(csn, miso, spi, TI_CCxxx0_SFTX);
  //Enter TX state
  do {
    val = cc1101_send(csn, miso, spi, TI_CCxxx0_STX);
  } while ((val & 0x70) != 0x20);
  return val;
}

uint8_t cc1101_rxmode(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi)
{
  uint8_t val;
  //Enter IDLE state
  do {
    val = cc1101_send(csn, miso, spi, TI_CCxxx0_SIDLE);
  } while ((val & 0x70) != 0x00);
  //Flush RXFIFO
  val = cc1101_send(csn, miso, spi, TI_CCxxx0_SFRX);
  //Enter RX state
  do {
    val = cc1101_send(csn, miso, spi, TI_CCxxx0_SRX);
  } while ((val & 0x70) != 0x10);
  return val;
}

// T-mode setup
uint8_t cc1101_tmode(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi)
{
  int i;
  //Write t-mode settings
  for (i=0; i<sizeof(tModeRfConfig)/sizeof(uint8_t); i+=2) {
    cc1101_check(csn, miso, spi, tModeRfConfig[i], tModeRfConfig[i+1]);      
  }
  //Write patable
  cc1101_xmit(csn, miso, spi, TI_CCxxx0_PATABLE | TI_CCxxx0_WRITE_BURST, tModePaTable, NULL, tModePaTableLen);
  //Return status
  return cc1101_send(csn, miso, spi, TI_CCxxx0_SNOP);
}

// C-mode setup
uint8_t cc1101_cmode(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi)
{
  //SYNC1 = 0x54
  cc1101_check(csn, miso, spi, TI_CCxxx0_SYNC1, 0x54);
  //SYNC0 = 0x3D
  cc1101_check(csn, miso, spi, TI_CCxxx0_SYNC0, 0x3D);
  // MDMCFG2[2:0] SYNC_MODE 3 - 30/32 sync word bits detected
  //              SYNC_MODE 7 - 30/32 + carrier sense above threshold
  cc1101_check(csn, miso, spi, TI_CCxxx0_MDMCFG2, 0x03);
  // PKTLEN[7:0] PACKET_LENGTH 0x80
  cc1101_check(csn, miso, spi, TI_CCxxx0_PKTLEN, 0x80);    
  // PKTCTRL1[7:5] PQT - quality estimator
  // PKTCTRL1[2:2] APPEND_STATUS               
  cc1101_check(csn, miso, spi, TI_CCxxx0_PKTCTRL1, 0x04);    
  // PKTCTRL0[2:2] CRC_EN        0 - CRC disabled
  // PKTCTRL0[1:0] LENGTH_CONFIG 0 - Fixed packet length
  //               LENGTH_CONFIG 1 - Variable packet length
  //               LENGTH_CONFIG 2 - Infinite packet length
  cc1101_check(csn, miso, spi, TI_CCxxx0_PKTCTRL0, 0x01);    
  //Return status
  return cc1101_send(csn, miso, spi, TI_CCxxx0_SNOP);
}

// Interrupt transactions
uint8_t cc1101_send(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd)
{
  spi_transaction_t txn;
  memset(&txn, 0, sizeof(spi_transaction_t));
  txn.length = 8*1;
  txn.flags |= SPI_TRANS_USE_TXDATA;
  txn.flags |= SPI_TRANS_USE_RXDATA;
  txn.tx_data[0] = cmd;
  gpio_set_level(csn, 0);
  while(gpio_get_level(miso)>0);
  ESP_ERROR_CHECK(spi_device_transmit(spi, &txn));
  return txn.rx_data[0];
}

uint8_t cc1101_info(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd)
{
  spi_transaction_t txn;
  memset(&txn, 0, sizeof(spi_transaction_t));
  txn.length = 8*2;
  txn.flags |= SPI_TRANS_USE_TXDATA;
  txn.flags |= SPI_TRANS_USE_RXDATA;
  txn.tx_data[0] = cmd;
  gpio_set_level(csn, 0);
  while(gpio_get_level(miso)>0);
  ESP_ERROR_CHECK(spi_device_transmit(spi, &txn));
  ESP_LOGV("CC1101", "Info: 0x%02x: 0x%02x", cmd, txn.rx_data[1]);
  return txn.rx_data[0];
}

uint8_t cc1101_read(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, uint8_t *inp)
{
  spi_transaction_t txn;
  memset(&txn, 0, sizeof(spi_transaction_t));
  txn.length = 8*2;
  txn.flags |= SPI_TRANS_USE_TXDATA;
  txn.flags |= SPI_TRANS_USE_RXDATA;
  txn.tx_data[0] = cmd;
  gpio_set_level(csn, 0);
  while(gpio_get_level(miso)>0);
  ESP_ERROR_CHECK(spi_device_transmit(spi, &txn));
  *inp = txn.rx_data[1];
  return txn.rx_data[0];
}

uint8_t cc1101_write(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, uint8_t out)
{
  spi_transaction_t txn;
  memset(&txn, 0, sizeof(spi_transaction_t));
  txn.length = 8*2;
  txn.flags |= SPI_TRANS_USE_TXDATA;
  txn.flags |= SPI_TRANS_USE_RXDATA;
  txn.tx_data[0] = cmd;
  txn.tx_data[1] = out;
  gpio_set_level(csn, 0);
  while(gpio_get_level(miso)>0);
  ESP_ERROR_CHECK(spi_device_transmit(spi, &txn));
  return txn.rx_data[0];
}

uint8_t cc1101_check(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, uint8_t out)
{
  spi_transaction_t txn;
  memset(&txn, 0, sizeof(spi_transaction_t));
  txn.length = 8*4;
  txn.flags |= SPI_TRANS_USE_TXDATA;
  txn.flags |= SPI_TRANS_USE_RXDATA;
  txn.tx_data[0] = cmd | TI_CCxxx0_READ_SINGLE;
  txn.tx_data[1] = out;
  txn.tx_data[2] = cmd;
  txn.tx_data[3] = out;
  gpio_set_level(csn, 0);
  while(gpio_get_level(miso)>0);
  ESP_ERROR_CHECK(spi_device_transmit(spi, &txn));
  if (txn.rx_data[1] != txn.tx_data[3])
    ESP_LOGV("CC1101", "Update: 0x%02x: 0x%02x -> 0x%02x", cmd, txn.rx_data[1], txn.tx_data[3]);
  return txn.rx_data[2];
}

uint8_t cc1101_xmit(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, const uint8_t *out, uint8_t *inp, size_t len)
{
  uint8_t val;
  spi_transaction_t txn;
  memset(&txn, 0, sizeof(spi_transaction_t));
  txn.length = 8*1;
  txn.flags |= SPI_TRANS_USE_TXDATA;
  txn.flags |= SPI_TRANS_USE_RXDATA;
  txn.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
  txn.tx_data[0] = cmd;
  gpio_set_level(csn, 0);
  while(gpio_get_level(miso)>0);
  ESP_ERROR_CHECK(spi_device_transmit(spi, &txn));
  val = txn.rx_data[0];
  memset(&txn, 0, sizeof(spi_transaction_t));
  txn.length = 8*len;
  txn.tx_buffer = out;
  txn.rx_buffer = inp;
  ESP_ERROR_CHECK(spi_device_transmit(spi, &txn));
  return val;
}

// Polling transactions
uint8_t cc1101_polling_send(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd)
{
  spi_transaction_t txn;
  memset(&txn, 0, sizeof(spi_transaction_t));
  txn.length = 8*1;
  txn.flags |= SPI_TRANS_USE_TXDATA;
  txn.flags |= SPI_TRANS_USE_RXDATA;
  txn.tx_data[0] = cmd;
  gpio_set_level(csn, 0);
  while(gpio_get_level(miso)>0);
  ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &txn));
  return txn.rx_data[0];
}

uint8_t cc1101_polling_read(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, uint8_t *inp)
{
  spi_transaction_t txn;
  memset(&txn, 0, sizeof(spi_transaction_t));
  txn.length = 8*2;
  txn.flags |= SPI_TRANS_USE_TXDATA;
  txn.flags |= SPI_TRANS_USE_RXDATA;
  txn.tx_data[0] = cmd;
  gpio_set_level(csn, 0);
  while(gpio_get_level(miso)>0);
  ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &txn));
  *inp = txn.rx_data[1];
  return txn.rx_data[0];
}

uint8_t cc1101_polling_write(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, uint8_t out)
{
  spi_transaction_t txn;
  memset(&txn, 0, sizeof(spi_transaction_t));
  txn.length = 8*2;
  txn.flags |= SPI_TRANS_USE_TXDATA;
  txn.flags |= SPI_TRANS_USE_RXDATA;
  txn.tx_data[0] = cmd;
  txn.tx_data[1] = out;
  gpio_set_level(csn, 0);
  while(gpio_get_level(miso)>0);
  ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &txn));
  return txn.rx_data[0];
}

uint8_t cc1101_polling_check(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, uint8_t out)
{
  spi_transaction_t txn;
  memset(&txn, 0, sizeof(spi_transaction_t));
  txn.length = 8*4;
  txn.flags |= SPI_TRANS_USE_TXDATA;
  txn.flags |= SPI_TRANS_USE_RXDATA;
  txn.tx_data[0] = cmd | TI_CCxxx0_READ_SINGLE;
  txn.tx_data[1] = out;
  txn.tx_data[2] = cmd;
  txn.tx_data[3] = out;
  gpio_set_level(csn, 0);
  while(gpio_get_level(miso)>0);
  ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &txn));
  if (txn.rx_data[1] != txn.tx_data[3])
    ESP_LOGV("CC1101", "0x%02x 0x%02x 0x%02x", cmd, txn.rx_data[1], txn.tx_data[3]);
  return txn.rx_data[2];
}

uint8_t cc1101_polling_xmit(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, const uint8_t *out, uint8_t *inp, size_t len)
{
  uint8_t val;
  spi_transaction_t txn;
  memset(&txn, 0, sizeof(spi_transaction_t));
  txn.length = 8*1;
  txn.flags |= SPI_TRANS_USE_TXDATA;
  txn.flags |= SPI_TRANS_USE_RXDATA;
  txn.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
  txn.tx_data[0] = cmd;
  gpio_set_level(csn, 0);
  while(gpio_get_level(miso)>0);
  ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &txn));
  val = txn.rx_data[0];
  memset(&txn, 0, sizeof(spi_transaction_t));
  txn.length = 8*len;
  txn.tx_buffer = out;
  txn.rx_buffer = inp;
  ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &txn));
  return val;
}

// Simple ISR handler
void IRAM_ATTR cc1101_polling_rxisr(void *ring)
{
  static uint8_t buf[0x80];
  static uint8_t off = 0;
  uint8_t val;
  uint8_t rem;
  uint8_t num;
  //Acquire bus
  ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
  //Drain FIFO
  for (;;) {
    val = cc1101_polling_read(csn, miso, spi, TI_CCxxx0_RXBYTES | TI_CCxxx0_READ_BURST, &rem);
    rem &= 0x7f;
    if (rem == 0) break;
    //Read data into buffer
    num = (rem < (sizeof(buf) - off)) ? rem : (sizeof(buf) - off);
    if (num > 0) {
      cc1101_polling_xmit(csn, miso, spi, TI_CCxxx0_RXFIFO | TI_CCxxx0_READ_BURST, NULL, buf+off, num);
      off += num;
      rem -= num;
    }
    //Flush remaining data
    if (rem > 0) {
      cc1101_polling_xmit(csn, miso, spi, TI_CCxxx0_RXFIFO | TI_CCxxx0_READ_BURST, NULL, NULL, rem);
    }
  }
  //Handle chip state
  switch (val & 0x70) {
  case 0x00: /* IDLE */
    xRingbufferSendFromISR(ring, buf, off, NULL);
    memset(buf, 0, sizeof(buf));
    off = 0;
    val = cc1101_polling_send(csn, miso, spi, TI_CCxxx0_SRX);
    ESP_LOGV("CC1101", "%-15s 0x%02x", "CCxxx0_SRX", val);
    break;
  case 0x10: /* RX */
    ESP_LOGV("CC1101", "%-15s 0x%02x", "CCxxx0_RXBYTES", val);
    break;
  case 0x20: /* TX */
  case 0x30: /* FSTXON */
    val = cc1101_polling_send(csn, miso, spi, TI_CCxxx0_SIDLE);
    ESP_LOGV("CC1101", "%-15s 0x%02x", "CCxxx0_SIDLE", val);
    break;
  case 0x40: /* CALIBRATE */
  case 0x50: /* SETTLING */
    ESP_LOGV("CC1101", "%-15s 0x%02x", "CCxxx0_SRX", val);
    break;
  case 0x60: /* RXFIFO_OVERFLOW */
    // Flush RXFIFO
    val = cc1101_polling_send(csn, miso, spi, TI_CCxxx0_SFRX);
    ESP_LOGV("CC1101", "%-15s 0x%02x", "CCxxx0_SFRX", val);
    break;
  case 0x70: /* TXFIFO_OVERFLOW */
    // Flush FIFO
    val = cc1101_polling_send(csn, miso, spi, TI_CCxxx0_SFTX);
    ESP_LOGV("CC1101", "%-15s 0x%02x", "CCxxx0_SFTX", val);
    break;
  default:
    break;
  }
  //Release bus
  spi_device_release_bus(spi);
}
