FREERTOS_ROOT := $(abspath ../../../..)
SDK_ROOT := $(abspath ../../../../..)
CC ?= cc
CFLAGS := -std=c11 -O2 -Wall -Wextra -Werror -Iinclude -DTHERMAL_HOST_TEST -pthread
SRCS := src/data_rw.c src/vdcmd.c src/falcon_cmd.c src/hal_spi.c src/vospi.c src/core.c
RISCV_CC ?= $(SDK_ROOT)/host-tools/gcc/riscv64-elf-x86_64/bin/riscv64-unknown-elf-gcc
RISCV_FLAGS := -std=gnu11 -O2 -Wall -Wextra -Werror -mcmodel=medany -mabi=lp64d -march=rv64imafdc -DTHEAD_C906 -DCONFIG_64BIT -Iinclude
INCS := driver/i2c/include hal/cv181x/i2c/include driver/spi/include hal/cv181x/spi/include common/include common/include/riscv64 arch/riscv64/include kernel/include/riscv64 hal/cv181x/config hal/cv181x/pinmux/include driver/pinmux/include driver/rtos_cmdqu/include driver/common/include task/comm/include
RISCV_FLAGS += $(addprefix -I$(FREERTOS_ROOT)/cvitek/,$(INCS)) -I$(FREERTOS_ROOT)/Source/include -I$(FREERTOS_ROOT)/Source/portable/GCC/RISC-V
.PHONY: all test riscv-check clean
all: test
build:
	mkdir -p $@
build/test_driver: $(SRCS) tests/test_driver.c $(wildcard include/*.h) | build
	$(CC) $(CFLAGS) $(SRCS) tests/test_driver.c -o $@
build/test_i2c_failure: $(SRCS) tests/test_i2c_failure.c $(wildcard include/*.h) | build
	$(CC) $(CFLAGS) $(SRCS) tests/test_i2c_failure.c -o $@
build/test_vospi: src/vospi.c tests/test_vospi.c $(wildcard include/*.h) | build
	$(CC) $(CFLAGS) src/vospi.c tests/test_vospi.c -o $@
test: build/test_driver build/test_i2c_failure build/test_vospi
	./build/test_driver
	./build/test_i2c_failure
	./build/test_vospi
riscv-check: | build
	@set -e; for f in $(SRCS) rtos/thermal_service.c; do \
	 echo "[REAL RISCV] $$f"; $(RISCV_CC) $(RISCV_FLAGS) -c $$f -o build/$$(basename $$f .c).o; done
	$(dir $(RISCV_CC))riscv64-unknown-elf-nm -u build/hal_spi.o | grep -q spi_transfer_one
	$(dir $(RISCV_CC))riscv64-unknown-elf-nm -u build/data_rw.o | grep -q soft_i2c_read_reg16
	$(dir $(RISCV_CC))riscv64-unknown-elf-nm -u build/vdcmd.o | grep -q xQueueSemaphoreTake
	$(dir $(RISCV_CC))riscv64-unknown-elf-nm build/thermal_service.o | grep -q ' T thermal_service_task$$'
clean:
	rm -rf build thermal_t3_host_test
