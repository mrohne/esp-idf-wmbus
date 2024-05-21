// Initialize CC1101
void cc1101_init(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi);
void cc1101_rf_tmode();
void cc1101_rf_cmode();
void cc1101_rx_start();
void IRAM_ATTR cc1101_rx_isr(void *ring);
// Polling transactions
uint8_t cc1101_send(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd);
uint8_t cc1101_read(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, uint8_t *inp);
uint8_t cc1101_write(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, uint8_t out);
uint8_t cc1101_xmit(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, const uint8_t *out, uint8_t *inp, size_t len);
