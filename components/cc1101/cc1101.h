// Initialize CC1101
void cc1101_setup(gpio_num_t csn, gpio_num_t miso, gpio_num_t gpi0, gpio_num_t gpi2, spi_device_handle_t spi);
void cc1101_rxtask(void *ring);
// Chip setup 
uint8_t cc1101_tmode(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi);
uint8_t cc1101_cmode(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi);
uint8_t cc1101_rxmode(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi);
uint8_t cc1101_txmode(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi);
// Interrupt transactions
uint8_t cc1101_send(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd);
uint8_t cc1101_info(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd);
uint8_t cc1101_read(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, uint8_t *inp);
uint8_t cc1101_write(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, uint8_t out);
uint8_t cc1101_check(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, uint8_t out);
uint8_t cc1101_xmit(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, const uint8_t *out, uint8_t *inp, size_t len);
// Polling transactions
uint8_t cc1101_polling_send(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd);
uint8_t cc1101_polling_read(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, uint8_t *inp);
uint8_t cc1101_polling_check(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, uint8_t out);
uint8_t cc1101_polling_xmit(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd, const uint8_t *out, uint8_t *inp, size_t len);
