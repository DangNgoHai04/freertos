# Kiểm thử SPI0 external loopback trên core FreeRTOS

Tài liệu dành cho LicheeRV Nano / SG2002, core RTOS RISC-V, cấu hình SDK
`sg2002_licheervnano_sd`. Các đường dẫn bắt đầu bằng `cvitek/` được tính từ
thư mục `freertos`; các lệnh build SDK được chạy tại thư mục cha của `freertos`.

## 1. Mục tiêu và phạm vi

Bài test phát dữ liệu từ SPI0 SDO/MOSI, đưa qua dây nối ngoài về SDI/MISO,
rồi so sánh dữ liệu gửi và nhận. Nó kiểm tra đường truyền qua controller,
pinmux, D-PHY, pad và dây nối bên ngoài.

- SPI0 base: `0x04180000`; thanh ghi dữ liệu DR: `0x04180060`.
- Master, Motorola SPI, Mode 0, frame 8 bit, full duplex.
- Internal loopback tắt: `CTRLR0.SRL = 0`.
- Nguồn SPI 25 MHz, `BAUDR = 250`, SCK khoảng 100 kHz.
- Truyền bằng MMIO/polling trực tiếp; không gọi driver `spi_transfer()`.
- Mỗi byte là một giao dịch CS riêng. Packet 4 byte chưa có CS liên tục.
- Không kiểm thử DMA, mutex của driver hoặc giao thức sensor.

## 2. Source và những thay đổi cần có

Commit external loopback tối thiểu gồm ba file sau:

| File | Nội dung |
| --- | --- |
| `cvitek/task/comm/include/spi_ext_loopback.h` | Khai báo `spi_ext_loopback_run()` và `start_spi_ext_loopback_task()` |
| `cvitek/task/comm/src/riscv64/spi_ext_loopback.c` | Clock/reset, D-PHY, pinmux/pad, truyền nhận polling, test pattern, task định kỳ, UART và telemetry |
| `cvitek/task/comm/src/riscv64/comm_main.c` | Include header và gọi hàm khởi động test trước scheduler |

Trong `comm_main.c`, thêm header:

```c
#include "spi_ext_loopback.h"
```

Trong `main_cvirtos()`, sau đăng ký IRQ mailbox và trước khi tạo các task
khác/chạy scheduler:

```c
start_spi_ext_loopback_task();
```

`cvitek/task/comm/CMakeLists.txt` đã dùng:

```cmake
file(GLOB _SOURCES "src/${RUN_ARCH}/*.c")
```

Vì vậy không cần thêm tên file C vào CMakeLists. Khi thêm file mới phải chạy
lại bước configure CMake; `build_rtos` đã làm bước này.

### Những file không bắt buộc cho bài test này

- `spi_smoke_test.c/.h`: include và lời gọi smoke test hiện đã được comment.
  Có thể bỏ các dòng comment này; không cần đưa module smoke test vào commit.
- `cvitek/driver/spi/`, `cvitek/hal/cv181x/spi/` và CMake đăng ký hai module:
  bài test hiện không dùng driver mới.
- Thay đổi `top_reg.h` và `hal_pinmux.c`: module test tự khai báo địa chỉ và
  tự cấu hình pinmux bằng MMIO.
- Thay đổi `driver/uart/src/uart.c`: module test có hàm ghi UART0 trực tiếp.

Các file không liên quan vẫn có thể ảnh hưởng một lần build toàn workspace;
chỉ tách chúng khỏi commit không có nghĩa build tree tự quay về trạng thái cũ.

## 3. Luồng thực thi

```text
main_cvirtos()
  -> đăng ký IRQ mailbox
  -> start_spi_ext_loopback_task()
       -> khởi tạo telemetry tại 0x8FF10000
       -> spi_ext_loopback_run()          [chạy trước scheduler]
            -> chuẩn bị clock/reset, D-PHY và pinmux/pad
            -> truyền và kiểm tra 8 pattern
       -> xTaskCreate(SPI0_EXT)
  -> tạo các task khác
  -> vTaskStartScheduler()
       -> SPI0_EXT                        [chạy sau scheduler]
            -> đợi khoảng 1 giây
            -> chuẩn bị lại phần cứng
            -> truyền AA, count, 55, ~count
            -> so sánh, cập nhật telemetry
            -> in heartbeat mỗi 5 chu kỳ
```

Task có priority `tskIDLE_PRIORITY + 3`, stack depth `512` phần tử stack
FreeRTOS, không mặc định là 512 byte.

Với mỗi byte, code tắt SSI, cấu hình Mode 0/8 bit và BAUDR, bật SSI,
ghi TX vào DR, chọn SER, polling `SR.RFNE`, đọc RX từ DR, polling `SR.BUSY`,
rồi bỏ SER và tắt SSI. CPU vẫn bận phục vụ truyền nhận trong các vòng polling.

## 4. Chuẩn bị phần cứng và quyền sở hữu

| Tín hiệu | Pad | Chân FPC2 theo ánh xạ trong source |
| --- | --- | --- |
| SCK | MIPI_TXM2 | 15 |
| CS | MIPI_TXP2 | 14 |
| SDO/MOSI | MIPI_TXM1 | 18 |
| SDI/MISO | MIPI_TXP1 | 17 |

1. Đối chiếu chiều đánh số FPC với board/adapter đang dùng.
2. Nối SDO với SDI bằng dây ngắn. Với loopback trực tiếp trên FPC2 là
   chân 18 nối chân 17, cùng miền điện áp pad của board.
3. Nếu dùng adapter chuyển mức, nối vòng ở hai kênh tương ứng phía ngoài
   adapter; cấp nguồn, OE và GND đúng thiết kế adapter. Không đưa điện áp
   phía ngoài trực tiếp vào pad FPC nếu khác miền điện áp.
4. Mở UART0 bằng cấu hình console của firmware để quan sát log lúc boot.
   Module không tự thiết lập baud rate UART.
5. Dành SPI0 và D-PHY cho RTOS; không chạy display, SPI Linux hay task khác
   cùng ghi các tài nguyên này. Không chạy đồng thời driver SPI mới.

### Cấu hình Linux trong SDK cha

Board DTS hiện có tại:

```text
build/boards/sg200x/sg2002_licheervnano_sd/dts_riscv/sg2002_licheervnano_sd.dts
```

Trong workspace hiện tại, `&spi0` và `&mipi_tx` đều được đặt
`status = "disabled"`. Khi mang commit sang SDK khác, phải kiểm tra lại
cấu hình này. Thay đổi DTS thuộc repository SDK cha, không nằm trong commit
ba file của repository `freertos`. Nếu thay DTS, cần build và nạp lại image
kernel/DTB; chỉ cập nhật `fip.bin` không cập nhật cấu hình Linux.

### Vùng nhớ telemetry

Module ghi trực tiếp `sizeof(struct spi_ext_loopback_telemetry)` byte từ
`0x8FF10000`; cần dành cả các cache line bao phủ vùng này.
Board memmap hiện khai báo DDR 256 MiB, vùng FreeRTOS 2 MiB ở cuối DDR
(`0x8FE00000..0x90000000`), nên địa chỉ telemetry nằm trong vùng RTOS.
Tuy nhiên linker chưa tự cấp một section riêng cho đối tượng này: phải kiểm
tra ELF/map và layout stack/heap để bảo đảm không chồng lấn. Không dùng nguyên
địa chỉ này cho board có dung lượng/layout DDR khác mà chưa kiểm tra.

## 5. Build theo thứ tự

### 5.1. Chuẩn bị môi trường SDK

Chạy tại SDK root, tức thư mục chứa đồng thời `build/` và `freertos/`:

```bash
source build/cvisetup.sh
defconfig sg2002_licheervnano_sd

command -v cmake
command -v ninja
command -v riscv64-unknown-elf-gcc
```

Cần SDK và các thành phần/toolchain đã được chuẩn bị theo quy trình của SDK.
Xác nhận `CONFIG_ENABLE_FREERTOS=y` trong `build/.config` sau khi chọn board.
Toolchain RTOS là RISC-V ELF; không dùng script `freertos/build_freertos.sh`
ở root vì script đó đang cấu hình CV1835/AArch64.

### 5.2. Build FreeRTOS

```bash
build_rtos
```

Chuỗi gọi thực tế:

```text
build_rtos
  -> make rtos trong build/
  -> sinh/cập nhật memory map
  -> freertos/cvitek/build_cv181x.sh
```

Script `build_cv181x.sh` cấu hình `RUN_CHIP=cv181x`, `RUN_ARCH=riscv64`,
`RUN_TYPE=CVIRTOS` và build/install lần lượt:

| Thứ tự | Thành phần | Vai trò |
| --- | --- | --- |
| 1 | `arch` | Startup, hỗ trợ kiến trúc và cache |
| 2 | `kernel` | FreeRTOS kernel |
| 3 | `common` | Các thành phần SDK dùng chung |
| 4 | `hal/cv181x` | HAL của platform |
| 5 | `driver` | Các driver SDK |
| 6 | `task` | Build module `comm`, gồm external loopback; link firmware |
| 7 | `cvirtos.bin` | Objcopy ELF thành binary |

Module test phụ thuộc các header/thành phần sẵn có: FreeRTOS, task API,
MMIO, cache helper và C runtime. Không có binary riêng cho mỗi bài test.
Các file C được build thành object, đóng vào `libcomm.a`, rồi link vào ELF.

Output chính:

```text
freertos/cvitek/install/bin/cvirtos.elf
freertos/cvitek/install/bin/cvirtos.bin
freertos/cvitek/install/bin/cvirtos.map
freertos/cvitek/install/bin/cvirtos.dis
```

Script cũng copy binary đến `freertos/cvirtos.bin`. Bước đóng gói FSBL dùng
binary trong `cvitek/install/bin/`, không phải bản copy này.

Không nên chạy riêng script CV181x từ một shell chưa nạp SDK: nó cần
`BUILD_PATH`, `.config`, memory map và các output/phụ thuộc SDK.

### 5.3. Đóng gói firmware boot

Sau khi `build_rtos` thành công:

```bash
build_fsbl
```

Trong SDK này, khi `CONFIG_ENABLE_FREERTOS=y`, `fsbl-build` có dependency
`rtos` và truyền `cvitek/install/bin/cvirtos.bin` qua `BLCP_2ND_PATH`.
Vì vậy bước này có thể chạy lại build RTOS; hai lệnh tách riêng giúp phát
hiện lỗi RTOS trước khi đóng gói FSBL/U-Boot/OpenSBI.

Với cấu hình board trên, output dự kiến:

```text
install/soc_sg2002_licheervnano_sd/fip.bin
```

Nếu vừa sửa DTS Linux, chạy thêm `build_kernel` trong cùng môi trường SDK.
Hàm này gọi build kernel rồi `pack_boot` mặc định; cập nhật image boot/DTB
được sinh theo cấu hình SDK, cùng với firmware mới trên board.

## 6. Nạp và chạy

1. Kiểm tra output đúng board và sao lưu firmware đang dùng.
2. Với board SD đang boot từ `/boot/fip.bin`, chép `fip.bin` mới vào vị trí
   đó. Có thể chép qua mạng hoặc gắn thẻ SD trên host.
3. Nếu thay Linux DTS, cập nhật cả image chứa kernel/DTB tương ứng.
4. Đồng bộ filesystem, mở UART console rồi khởi động lại board.

Ví dụ chép qua SSH từ SDK root; thay IP và tài khoản theo board thực tế:

```bash
scp install/soc_sg2002_licheervnano_sd/fip.bin root@BOARD_IP:/boot/fip.bin
ssh root@BOARD_IP 'sync && reboot'
```

Chỉ chép `cvirtos.bin` vào rootfs không thay thế payload RTOS đã đóng trong
FIP. Kiểm tra thời gian/hash của artifact để tránh nạp nhầm bản cũ.

## 7. Kết quả mong đợi

### Test lúc boot

Tám pattern:

```text
5A A5 3C C3 00 FF 12 34
```

Log mong đợi:

```text
SPI0 EXTERNAL PHYSICAL LOOPBACK TEST (SDO <-> SDI)
...
Summary: 8/8 patterns matched on physical wire!
===> PHYSICAL SDO-SDI JUMPER WIRE CONNECTION IS VERIFIED!
[RTOS SPI0-EXT] Background task created successfully!
```

### Test định kỳ

Task đợi khoảng 1 giây giữa các lần thử; packet gồm:

```text
AA, count & 0xFF, 55, (~count) & 0xFF
```

Mỗi 5 chu kỳ sẽ có heartbeat chứa `WIRE MATCH OK` nếu packet khớp.
Chu kỳ thực tế còn bao gồm thời gian chạy test và lập lịch RTOS.

Các trường telemetry chính, offset tính từ `0x8FF10000`:

| Offset | Trường | Ý nghĩa |
| --- | --- | --- |
| `0x00` | `magic` | `0x4558544C` |
| `0x04` | `state` | 1=INIT, 2=TESTING, 3=RUNNING |
| `0x08` | `cycle_count` | Số chu kỳ task định kỳ |
| `0x0C` | `match_count` | Số lần khớp, gồm cả pattern boot |
| `0x10` | `mismatch_count` | Số lần sai/timeout, gồm cả pattern boot |
| `0x14` | `last_match` | Kết quả gần nhất |
| `0x18` | `last_tx_data[4]` | Packet TX gần nhất |
| `0x1C` | `last_rx_data[4]` | Packet RX gần nhất |
| `0x20` | `baud_divider` | BAUDR đọc lại |
| `0x24` | `reg_sr` | Trạng thái SR |

Module flush cache sau cập nhật. Cấu trúc không có sequence lock nên việc
đọc nhiều trường từ core khác có thể bắt gặp một snapshot đang cập nhật.

## 8. Checklist kiểm thử và chẩn đoán

- Build/link thành công và đúng artifact đã được nạp.
- Boot test đạt 8/8 pattern, task tạo thành công.
- Theo dõi ít nhất 60 chu kỳ; `mismatch_count` không tăng khi dây nối tốt.
- Tháo dây nối để kiểm tra phát hiện lỗi, sau đó nối lại và kiểm tra hồi phục.
  Mất dây thường gây mismatch; không bắt buộc gây timeout.
- Nếu có logic analyzer: kiểm tra SCK khoảng 100 kHz, Mode 0, 8 bit và CS
  tách theo từng byte. Loopback dữ liệu tự nó chưa xác minh chân CS.

| Hiện tượng | Điểm cần kiểm tra |
| --- | --- |
| Không có banner | FIP mới có RTOS payload chưa, console UART, điểm gọi trong `main_cvirtos()` |
| RX luôn 00/FF hoặc mismatch | Chiều FPC, dây, adapter/nguồn/OE, pinmux và D-PHY |
| Boot khớp nhưng sau đó lỗi | Linux/display/driver khác có chiếm lại SPI0 hoặc D-PHY không |
| Timeout | Clock/reset/controller và nguồn tranh chấp; log không đủ để kết luận chỉ do dây |
| Task không được tạo | Heap FreeRTOS và kết quả `xTaskCreate()` |
| Telemetry bất thường | Layout DDR, chồng lấn stack/heap, cache và artifact cũ |

Giới hạn code hiện tại: timeout dựa trên số vòng lặp; vòng chờ BUSY cuối
byte chưa trả lỗi riêng khi hết hạn; test không dùng mutex. Không dùng kết
quả PASS để khẳng định driver hoặc giao tiếp sensor đã được xác minh.

## 9. Phạm vi commit và ghi nhận validation

Để tạo một commit mới chứa code test và tài liệu, chọn đúng các file:

```bash
# Chạy từ freertos/
git add cvitek/task/comm/include/spi_ext_loopback.h \
        cvitek/task/comm/src/riscv64/spi_ext_loopback.c \
        cvitek/task/comm/src/riscv64/comm_main.c \
        cvitek/task/comm/tests/spi_ext_loopback/README.md
git diff --cached --stat
git diff --cached --check
git diff --cached
```

Nếu ba file source đã được commit, chỉ stage README để tạo commit tài liệu
bổ sung. Kiểm tra cả những file đã được stage trước đó để tránh đưa nhầm
thay đổi khác vào commit. Các thay đổi DTS SDK cha cần commit riêng tại repo
cha và cập nhật tham chiếu submodule nếu SDK quản lý `freertos` bằng submodule.

Ghi kết quả thực tế trong commit/PR; không coi log mẫu là bằng chứng đã chạy:

```text
Board / SDK configuration:
Build command and result:
Firmware artifact/hash:
Startup patterns: .../8
Periodic cycles / mismatch count:
Disconnect/reconnect result:
Logic analyzer result (if available):
```

Tài liệu mô tả source và script trong workspace. Việc tạo README không đồng
nghĩa đã build firmware, nạp board hoặc hoàn thành kiểm thử phần cứng.
