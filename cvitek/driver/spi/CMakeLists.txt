add_library(spi OBJECT src/spi.c)
target_include_directories(spi PRIVATE
    include
    ${TOP_DIR}/hal/cv181x/spi/include
    ${CMAKE_INSTALL_INC_PREFIX}/arch
    ${CMAKE_INSTALL_INC_PREFIX}/common
    ${CMAKE_INSTALL_INC_PREFIX}/kernel
)
install(FILES include/spi.h DESTINATION include/driver/spi)
