#include <string.h>
#include "driver/spi_master.h"
#include "driver/gpio.h"
#include "TI_CC_CC1100-CC2500.h"
#include "cc1101.h"

uint8_t cc1101_send(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd)
{
    uint8_t val;
    spi_transaction_t txn;
    memset(&txn, 0, sizeof(spi_transaction_t));
    txn.length = 8*1;
    txn.flags |= SPI_TRANS_USE_TXDATA;
    txn.flags |= SPI_TRANS_USE_RXDATA;
    txn.tx_data[0] = cmd;
    gpio_set_level(csn, 0);
    while(gpio_get_level(miso)>0);
    ESP_ERROR_CHECK(spi_device_transmit(spi, &txn));
    val = txn.rx_data[0];
    gpio_set_level(csn, 1);
    return val;
}

uint8_t cc1101_write(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t reg, uint8_t ans)
{
    uint8_t val;
    spi_transaction_t txn;
    memset(&txn, 0, sizeof(spi_transaction_t));
    txn.length = 8*2;
    txn.flags |= SPI_TRANS_USE_TXDATA;
    txn.flags |= SPI_TRANS_USE_RXDATA;
    txn.tx_data[0] = reg;
    txn.tx_data[1] = ans;
    gpio_set_level(csn, 0);
    while(gpio_get_level(miso)>0);
    ESP_ERROR_CHECK(spi_device_transmit(spi, &txn));
    val = txn.rx_data[1];
    gpio_set_level(csn, 1);
    return val;
}

uint8_t cc1101_read(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t reg, uint8_t *ans)
{
    uint8_t val;    
    spi_transaction_t txn;
    memset(&txn, 0, sizeof(spi_transaction_t));
    txn.length = 8*2;
    txn.flags |= SPI_TRANS_USE_TXDATA;
    txn.flags |= SPI_TRANS_USE_RXDATA;
    txn.tx_data[0] = reg;
    gpio_set_level(csn, 0);
    while(gpio_get_level(miso)>0);
    ESP_ERROR_CHECK(spi_device_transmit(spi, &txn));
    val = txn.rx_data[0];
    *ans = txn.rx_data[1];
    gpio_set_level(csn, 1);
    return val;
}

uint8_t cc1101_xmit(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t reg, const uint8_t *out, uint8_t *inp, size_t len)
{
    uint8_t val;
    spi_transaction_t txn;
    memset(&txn, 0, sizeof(spi_transaction_t));
    txn.length = 8*1;
    txn.flags |= SPI_TRANS_USE_TXDATA;
    txn.flags |= SPI_TRANS_USE_RXDATA;
    txn.flags |=  SPI_TRANS_CS_KEEP_ACTIVE;
    txn.tx_data[0] = reg;
    gpio_set_level(csn, 0);
    while(gpio_get_level(miso)>0);
    ESP_ERROR_CHECK(spi_device_transmit(spi, &txn));
    val = txn.rx_data[0];
    memset(&txn, 0, sizeof(spi_transaction_t));
    txn.length = 8*len;
    txn.tx_buffer = out;
    txn.rx_buffer = inp;
    ESP_ERROR_CHECK(spi_device_transmit(spi, &txn));
    gpio_set_level(csn, 1);
    return val;
}

uint8_t cc1101_poll(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t reg, const uint8_t *out, uint8_t *inp, size_t len)
{
    uint8_t val;
    spi_transaction_t txn;
    memset(&txn, 0, sizeof(spi_transaction_t));
    txn.length = 8*1;
    txn.flags |= SPI_TRANS_USE_TXDATA;
    txn.flags |= SPI_TRANS_USE_RXDATA;
    txn.flags |= SPI_TRANS_CS_KEEP_ACTIVE;
    txn.tx_data[0] = reg;
    gpio_set_level(csn, 0);
    while(gpio_get_level(miso)>0);
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &txn));
    val = txn.rx_data[0];
    memset(&txn, 0, sizeof(spi_transaction_t));
    txn.length = 8*len;
    txn.tx_buffer = out;
    txn.rx_buffer = inp;
    ESP_ERROR_CHECK(spi_device_polling_transmit(spi, &txn));
    gpio_set_level(csn, 1);
    return val;
}

