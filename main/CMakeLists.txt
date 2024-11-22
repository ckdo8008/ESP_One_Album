idf_component_register(SRCS "hello_world_main.c"
                    INCLUDE_DIRS ".")

spiffs_create_partition_image(storage ${PROJECT_DIR}/data FLASH_IN_PROJECT)