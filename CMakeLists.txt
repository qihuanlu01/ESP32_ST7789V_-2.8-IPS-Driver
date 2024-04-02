<<<<<<< HEAD
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(spi_lcd_touch)
=======
idf_component_register(SRCS "St7789.c"
                    INCLUDE_DIRS "include"
                    REQUIRES "esp_lcd"
                    PRIV_REQUIRES "driver" 
                    )
>>>>>>> 17ec9607af5f02a471fe5757e35867f414be5020
