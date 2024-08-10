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
void cc1101_setup(gpio_num_t csn_, gpio_num_t miso_, spi_device_handle_t spi_, gpio_num_t gpi0_, gpio_num_t gpi2_)
{
  //Set static variables
  csn  = csn_;
  miso = miso_;
  spi  = spi_;
  gpi0 = gpi0_;
  gpi2 = gpi2_;
  //Pulse CSN
  ESP_LOGI("CC1101", "%s reset chip", __FUNCTION__);
  //Acquire bus
  ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
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
  //Release bus
  spi_device_release_bus(spi);
  //RF mode and RX fifo
  cc1101_tmode(csn, miso, spi);
  cc1101_cmode(csn, miso, spi);
}

static QueueHandle_t evt;
static uint8_t rxbuf[0x100];
static uint8_t rxlen = 0;

void IRAM_ATTR cc1101_rxisr(void *arg)
{
  uint8_t val;
  uint8_t rem;
  ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
  //Prepare transfer
  val = cc1101_polling_read(csn, miso, spi, TI_CCxxx0_RXBYTES | TI_CCxxx0_READ_BURST, &rem);
  //Read data into buffer
  uint8_t num = (rem < (sizeof(rxbuf) - rxlen)) ? rem : (sizeof(rxbuf) - rxlen);
  if (num > 0) {
    cc1101_polling_xmit(csn, miso, spi, TI_CCxxx0_RXFIFO | TI_CCxxx0_READ_BURST, NULL, rxbuf+rxlen, num);
    rxlen += num;
  }
  //Flush remaining data
  if (rem > num) {
    cc1101_polling_xmit(csn, miso, spi, TI_CCxxx0_RXFIFO | TI_CCxxx0_READ_BURST, NULL, NULL, rem - num);
  }
  spi_device_release_bus(spi);
  //Return status
  uint8_t data[2] = {val, rem};
  xQueueSendFromISR(evt, data, NULL);
}

static RingbufHandle_t ring;
void cc1101_rxtask(void *arg)
{
  //Interrupt service
  uint8_t data[2];
  //Handle events
  for (;;) {
    //RX start
    rxlen = 0;
    memset(rxbuf, 0, sizeof(rxbuf));
    cc1101_rxmode(csn, miso, spi);
    ESP_LOGV("CC1101", "RXMODE");
    //Wait for event
    if (!xQueueReceive(evt, &data, 100000/portTICK_PERIOD_MS)) {
      ESP_LOGW("CC1101", "TIMEOUT");
      continue;
    }
    //Drain event queue
    do {
      uint8_t val = data[0];    
      uint8_t rem = data[1];
      switch (val & 0x70) {
      case 0x00: /* IDLE */
	ESP_LOGV("CC1101", "%-15s 0x%02x 0x%02x", "IDLE", val, rem);
	break;
      case 0x10: /* RX */
	ESP_LOGV("CC1101", "%-15s 0x%02x 0x%02x", "RX", val, rem);
	break;
      case 0x60: /* RXFIFO_OVERFLOW */
	ESP_LOGV("CC1101", "%-15s 0x%02x 0x%02x", "RXFIFO_OVERFLOW", val, rem);
	break;
      default:
	ESP_LOGV("CC1101", "%-15s 0x%02x 0x%02x", "UNEXPECTED", val, rem);
	break;
      }
    } while (xQueueReceive(evt, &data, 0));
    //Format telegram 
    char rxstr[2*sizeof(rxbuf)+1];
    for (int i=0;i<rxlen;i++) snprintf(rxstr+2*i, 3, "%02x",rxbuf[i]);
    rxstr[2*rxlen] = 0;    
    ESP_LOGI("CC1101", "%-15s 0x%02x %s", "TELEGRAM", rxlen, rxstr);
    //Send data to wmbusmeter
    xRingbufferSend(ring, rxbuf, rxlen, portMAX_DELAY);
  }
  //Never reached
  vTaskDelete(NULL);  
}

static TaskHandle_t rxtask = NULL;
void cc1101_start(RingbufHandle_t _ring)
{
  ring = _ring;
  evt = xQueueCreate(64, 2*sizeof(uint8_t));
  xTaskCreate(&cc1101_rxtask, "CC1101", 1024*16, ring, 7, &rxtask);
  ESP_ERROR_CHECK(gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3));
  ESP_ERROR_CHECK(gpio_isr_handler_add(gpi0, cc1101_rxisr, evt));
  ESP_LOGV("CC1101", "ISR Installed");
  ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
  // MCSM1.RXOFF_MODE[3:2] 0 - RX -> IDLE after a packet has been received
  cc1101_check(csn, miso, spi, TI_CCxxx0_MCSM1, 0x00);
  // FIFOTHR[7:0] 0 - RX FIFO 4 bytes
  // FIFOTHR[7:0] 3 - RX FIFO 16 bytes
  // FIFOTHR[7:0] 7 - RX FIFO 32 bytes
  cc1101_check(csn, miso, spi, TI_CCxxx0_FIFOTHR, 0x07);    
  // IOCFG0[5:0] GDOn_CFG 0x00 - RX FIFO >= threshold
  // IOCFG0[5:0] GDOn_CFG 0x01 - RX FIFO >= threshold or end of packet
  // IOCFG0[5:0] GDOn_CFG 0x04 - RX FIFO has overflowed
  // IOCFG0[5:0] GDOn_CFG 0x06 - sync word has been received
  // IOCFG0[5:0] GDOn_CFG 0x07 - packet has been received with CRC OK
  // IOCFG0[5:0] GDOn_CFG 0x2f - HW to 0
  cc1101_check(csn, miso, spi, TI_CCxxx0_IOCFG0, 0x01);
  spi_device_release_bus(spi);
  ESP_LOGV("CC1101", "IRQ Started");
}

uint8_t cc1101_txmode(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi)
{
  uint8_t val;
  //Acquire bus
  ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
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
  //Release bus
  spi_device_release_bus(spi);
  return val;
}

uint8_t cc1101_rxmode(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi)
{
  uint8_t val;
  //Acquire bus
  ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
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
  //Release bus
  spi_device_release_bus(spi);
  return val;
}

// T-mode setup
uint8_t cc1101_tmode(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi)
{
  int i;
  //Acquire bus
  ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
  //Write t-mode settings
  for (i=0; i<sizeof(tModeRfConfig)/sizeof(uint8_t); i+=2) {
    cc1101_check(csn, miso, spi, tModeRfConfig[i], tModeRfConfig[i+1]);      
  }
  //Write patable
  cc1101_xmit(csn, miso, spi, TI_CCxxx0_PATABLE | TI_CCxxx0_WRITE_BURST, tModePaTable, NULL, tModePaTableLen);
  //Get status
  uint8_t val = cc1101_send(csn, miso, spi, TI_CCxxx0_SNOP);
  //Release bus
  spi_device_release_bus(spi);
  return val;  
}

// C-mode setup
uint8_t cc1101_cmode(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi)
{
  //Acquire bus
  ESP_ERROR_CHECK(spi_device_acquire_bus(spi, portMAX_DELAY));
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
  uint8_t val = cc1101_check(csn, miso, spi, TI_CCxxx0_PKTCTRL0, 0x01);    
  //Release bus
  spi_device_release_bus(spi);
  return val;
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
  ESP_LOGI("CC1101", "Info: 0x%02x: 0x%02x", cmd, txn.rx_data[1]);
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
    ESP_LOGI("CC1101", "Update: 0x%02x: 0x%02x -> 0x%02x", cmd, txn.rx_data[1], txn.tx_data[3]);
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
    ESP_LOGI("CC1101", "0x%02x 0x%02x 0x%02x", cmd, txn.rx_data[1], txn.tx_data[3]);
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
    ESP_LOGI("CC1101", "%-15s 0x%02x", "CCxxx0_SRX", val);
    break;
  case 0x10: /* RX */
    ESP_LOGI("CC1101", "%-15s 0x%02x", "CCxxx0_RXBYTES", val);
    break;
  case 0x20: /* TX */
  case 0x30: /* FSTXON */
    val = cc1101_polling_send(csn, miso, spi, TI_CCxxx0_SIDLE);
    ESP_LOGI("CC1101", "%-15s 0x%02x", "CCxxx0_SIDLE", val);
    break;
  case 0x40: /* CALIBRATE */
  case 0x50: /* SETTLING */
    ESP_LOGI("CC1101", "%-15s 0x%02x", "CCxxx0_SRX", val);
    break;
  case 0x60: /* RXFIFO_OVERFLOW */
    // Flush RXFIFO
    val = cc1101_polling_send(csn, miso, spi, TI_CCxxx0_SFRX);
    ESP_LOGI("CC1101", "%-15s 0x%02x", "CCxxx0_SFRX", val);
    break;
  case 0x70: /* TXFIFO_OVERFLOW */
    // Flush FIFO
    val = cc1101_polling_send(csn, miso, spi, TI_CCxxx0_SFTX);
    ESP_LOGI("CC1101", "%-15s 0x%02x", "CCxxx0_SFTX", val);
    break;
  default:
    break;
  }
  //Release bus
  spi_device_release_bus(spi);
}
