idf_component_register(SRCS "esp_hid_gap.c" "hid_host_example.c"
                       INCLUDE_DIRS "." "../components/hid/include"
                       REQUIRES esp_hid driver usb bt
                       PRIV_REQUIRES nvs_flash)
