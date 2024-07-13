idf_component_register(SRCS 
		"main.c"
		"wifi.c"
		"sntp.c"
                INCLUDE_DIRS "."
                PRIV_REQUIRES cc1101 wmbusmeters esp_wifi nvs_flash)
