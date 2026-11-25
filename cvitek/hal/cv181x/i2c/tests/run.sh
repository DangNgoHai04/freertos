#!/usr/bin/env bash
set -euo pipefail
d="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
r="$(cd -- "$d/../../../.." && pwd)"
b="$(mktemp /tmp/i2c-tests.XXXXXX)"
trap 'rm -f -- "$b"' EXIT
"${CC:-cc}" -std=c11 -Wall -Wextra -Werror -I"$d/mocks" -I"$d/../include" -I"$r/driver/i2c/include" "$d/test_i2c.c" "$d/../src/hal_i2c_bitbang.c" "$r/driver/i2c/src/soft_i2c.c" -o "$b"
"$b"
