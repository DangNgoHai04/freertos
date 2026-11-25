#!/usr/bin/env bash
set -euo pipefail
d="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
r="$(cd -- "$d/../../../../.." && pwd)"
repo="$(cd -- "$r/.." && pwd)"
cc="${RISCV_CC:-$repo/host-tools/gcc/riscv64-elf-x86_64/bin/riscv64-unknown-elf-gcc}"
out="$(mktemp -d /tmp/i2c-riscv.XXXXXX)"
trap 'rm -rf -- "$out"' EXIT
args=(-std=c11 -Wall -Wextra -Werror -march=rv64imafdc -mabi=lp64d -DCONFIG_64BIT -DTHEAD_C906
-I"$r/Source/include" -I"$r/Source/portable/GCC/RISC-V"
-I"$r/cvitek/kernel/include/riscv64" -I"$r/cvitek/arch/riscv64/include"
-I"$r/cvitek/common/include/riscv64" -I"$d/../include" -I"$r/cvitek/driver/i2c/include")
for src in "$d/../src/hal_i2c_bitbang.c" "$d/../src/hal_i2c_gpio_platform.c" "$r/cvitek/driver/i2c/src/soft_i2c.c"; do
    "$cc" "${args[@]}" -c "$src" -o "$out/$(basename "$src").o"
done
printf '%s\n' 'RISC-V T0/T1/T2 compile PASS (-Wall -Wextra -Werror, real RTOS headers)'
