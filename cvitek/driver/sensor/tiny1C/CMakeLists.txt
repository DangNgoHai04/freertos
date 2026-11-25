cmake_minimum_required(VERSION 3.16.3)
project(thermal_tiny1c C)
set(CMAKE_C_STANDARD 11)
set(T3_SRC src/data_rw.c src/vdcmd.c src/falcon_cmd.c src/hal_spi.c src/vospi.c src/core.c)
add_library(thermal_tiny1c STATIC ${T3_SRC})
target_include_directories(thermal_tiny1c PUBLIC include)
if(DEFINED THERMAL_I2C_FREQ_HZ)
    if(THERMAL_I2C_FREQ_HZ LESS 1000 OR THERMAL_I2C_FREQ_HZ GREATER 100000)
        message(FATAL_ERROR "THERMAL_I2C_FREQ_HZ must be 1000..100000")
    endif()
    target_compile_definitions(thermal_tiny1c PRIVATE THERMAL_I2C_FREQ_HZ=${THERMAL_I2C_FREQ_HZ}U)
endif()
if(DEFINED TOP_DIR)
    target_sources(thermal_tiny1c PRIVATE
        rtos/thermal_service.c
        ${TOP_DIR}/driver/i2c/src/soft_i2c.c
        ${TOP_DIR}/hal/cv181x/i2c/src/hal_i2c_bitbang.c
        ${TOP_DIR}/hal/cv181x/i2c/src/hal_i2c_gpio_platform.c)
    target_include_directories(thermal_tiny1c PRIVATE
        ${TOP_DIR}/driver/spi/include ${TOP_DIR}/driver/i2c/include
        ${TOP_DIR}/hal/cv181x/spi/include ${TOP_DIR}/hal/cv181x/i2c/include
        ${TOP_DIR}/hal/cv181x/config ${TOP_DIR}/hal/cv181x/pinmux/include
        ${TOP_DIR}/driver/pinmux/include ${TOP_DIR}/driver/rtos_cmdqu/include
        ${TOP_DIR}/driver/common/include ${TOP_DIR}/task/comm/include
        ${TOP_DIR}/common/include ${TOP_DIR}/common/include/riscv64
        ${TOP_DIR}/arch/riscv64/include ${TOP_DIR}/kernel/include/riscv64
        ${TOP_DIR}/../Source/include ${TOP_DIR}/../Source/portable/GCC/RISC-V)
    target_compile_definitions(thermal_tiny1c PRIVATE THEAD_C906 CONFIG_64BIT)
else()
    target_compile_definitions(thermal_tiny1c PUBLIC THERMAL_HOST_TEST)
    target_link_libraries(thermal_tiny1c PUBLIC pthread)
endif()
