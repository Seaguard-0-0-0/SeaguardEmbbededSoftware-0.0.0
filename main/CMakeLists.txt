idf_component_register(
    SRCS "main.c"
    INCLUDE_DIRS "."
    REQUIRES
        driver
        esp_timer
        esp_wifi
        esp_netif
        esp_event
        nvs_flash
        mqtt
        cjson
        onewire_bus
        ds18b20
        esp-dht
)
