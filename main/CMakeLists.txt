idf_component_register(SRCS "GUI_Paint.c" "font8.c" "font12.c" "font16.c" "font20.c" "font24.c" "hello_world_main.c"
                    INCLUDE_DIRS ".")

spiffs_create_partition_image(storage ${PROJECT_DIR}/data FLASH_IN_PROJECT)