uint8_t cc1101_send(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t cmd);
uint8_t cc1101_write(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t reg, uint8_t ans);
uint8_t cc1101_read(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t reg, uint8_t *ans);
uint8_t cc1101_xmit(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t reg, const uint8_t *out, uint8_t *inp, size_t len);
uint8_t cc1101_poll(gpio_num_t csn, gpio_num_t miso, spi_device_handle_t spi, uint8_t reg, const uint8_t *out, uint8_t *inp, size_t len);


