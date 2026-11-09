add_library(halspi OBJECT
    src/hal_dw_spi.c
    src/hal_spi_platform.c
    src/hal_spi_platform_io.c
)
target_include_directories(halspi PRIVATE
    include
    ../config
    ${CMAKE_INSTALL_INC_PREFIX}/arch
    ${CMAKE_INSTALL_INC_PREFIX}/common
    ${CMAKE_INSTALL_INC_PREFIX}/kernel
)
install(FILES include/hal_dw_spi.h DESTINATION include/hal/spi)





