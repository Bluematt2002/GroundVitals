file(GLOB_RECURSE srcs "main.c" "src/*.c")

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "./include")
idf_component_register(
    SRCS
        "main.c"
        "ble_receiver.c"
    INCLUDE_DIRS
        "."
    REQUIRES
        bt
        nvs_flash
        driver
        esp_common
)