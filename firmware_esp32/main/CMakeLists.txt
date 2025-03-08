idf_component_register(SRCS "usb_cdc_wifi.c"
                    INCLUDE_DIRS "."
                    PRIV_REQUIRES usb
                                  esp_wifi
                                  nvs_flash
                    )
