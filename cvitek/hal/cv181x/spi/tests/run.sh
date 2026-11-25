#!/usr/bin/env bash
set -euo pipefail
spi_test_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
spi_root="$(cd -- "$spi_test_dir/../../../.." && pwd)"
spi_test_bin="$(mktemp /tmp/spi-hal-tests.XXXXXX)"
trap 'rm -f -- "$spi_test_bin"' EXIT
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror \
    -I"$spi_test_dir/mocks" -I"$spi_test_dir/../include" -I"$spi_test_dir/../src" \
    -I"$spi_root/driver/spi/include" \
    "$spi_test_dir/test_spi.c" "$spi_test_dir/../src/hal_dw_spi.c" \
    "$spi_root/driver/spi/src/spi.c" -o "$spi_test_bin"
"$spi_test_bin"
