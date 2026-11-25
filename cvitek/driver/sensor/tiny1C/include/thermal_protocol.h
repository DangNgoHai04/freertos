/*
 * Giao thức truyền nhận khung hình Camera nhiệt Tiny1C giữa 3 miền:
 *   1. RTOS C906 (freertos / thermal_service.c)
 *   2. Linux C906 (thermal_bridge.c qua /dev/cvi-rtos-cmdqu + /dev/mem)
 *   3. Máy trạm / Trình xem (thermal_viewer.py qua TCP socket)
 *
 * Quy ước ABI:
 *   - Tất cả các trường trong thermal_header_t đều cố định 4 byte (Little-Endian),
 *     tổng kích thước Header luôn đúng bằng 64 byte (1 dòng D-Cache C906).
 *   - Không truyền con trỏ bộ nhớ qua TCP; chỉ truyền 64-byte Header nối tiếp
 *     bởi payload_bytes dữ liệu khung hình (98,304 byte hoặc 196,608 byte).
 *   - Cơ chế đồng bộ kéo (Single-Owner Pull Model): RTOS giữ cố định vùng nhớ
 *     phản hồi cho tới khi Linux đọc xong và gửi yêu cầu tiếp theo.
 */
#ifndef THERMAL_PROTOCOL_H
#define THERMAL_PROTOCOL_H
#include <stdint.h>

/* Định danh gói tin "THM1" (0x314D4854 ở định dạng Little-Endian) */
#define THERMAL_MAGIC 0x314d4854U /* bytes THM1 */
#define THERMAL_VERSION 1U

/* Kích thước Payload tối đa: Chế độ Kép (MODE_DUAL) = 256 * 192 * 4 = 196,608 byte */
#define THERMAL_MAX_PAYLOAD (256U * 192U * 4U)

/* Định danh IP Mailbox trên RTOS (khớp với IP_CAMERA = 7 trong rtos_cmdqu.h) */
#define THERMAL_IP 7U

/* Tập mã lệnh Mailbox điều khiển Camera nhiệt qua hàng đợi E_QUEUE_CAMERA */
#define THERMAL_START   0x40U /* Khởi tạo T3 (I2C5 + SPI0) và bật luồng Preview (MODE_IMAGE/TEMP/DUAL) */
#define THERMAL_CAPTURE 0x41U /* Thu 1 khung hình hoàn chỉnh từ cảm biến qua VoSPI SPI0 */
#define THERMAL_STOP    0x42U /* Dừng luồng Preview trên cảm biến */
#define THERMAL_FFC     0x43U /* Kích hoạt hiệu chuẩn màn trập FFC (OOC B-Update) */
#define THERMAL_INFO    0x44U /* Truy vấn trạng thái khởi tạo thiết bị Tiny1C */

/* Các cờ xác nhận quyền sở hữu phần cứng gửi kèm trong param_ptr của lệnh THERMAL_START */
#define THERMAL_GPIO_OWNED (1U << 16) /* Xác nhận Linux đã nhường quyền điều khiển chân GPIOA15/GPIOA27 */
#define THERMAL_DMA_OWNED  (1U << 17) /* Xác nhận kênh DMA 0/1 dành riêng cho SPI0 trên RTOS */

/* Các cờ trạng thái chất lượng khung hình trong trường flags của thermal_header_t */
#define THERMAL_FLAG_SIMULATED           1U  /* Bit 0: Khung hình giả lập từ --simulate (không phải cảm biến thật) */
#define THERMAL_FLAG_SHUTTER_CLOSED      2U  /* Bit 1: Màn trập đang đóng (shutter_state == 0 trong VoSPI header) */
#define THERMAL_FLAG_FROZEN              4U  /* Bit 2: Điểm ảnh đang đóng băng (pix_freeze_state != 0) */
#define THERMAL_FLAG_HIGH_GAIN           8U  /* Bit 3: Cảm biến đang ở chế độ High Gain (gain_state == 1) */
#define THERMAL_FLAG_DUAL_INDEX_MISMATCH 16U /* Bit 4: Chỉ số khung ảnh (0xAA) và khung nhiệt (0xCC) lệch nhau */

/*
 * Cấu trúc Header cố định 64-byte dùng chung giữa RTOS C906, Linux và PC.
 * Payload đi kèm ngay sau Header:
 *   - MODE_IMAGE (0): 98,304 byte YUYV422 (256x192x2)
 *   - MODE_TEMP  (1): 98,304 byte Little-Endian Y16 nhiệt độ thô (256x192x2, Kelvin * 64)
 *   - MODE_DUAL  (2): 196,608 byte (98,304 byte đầu là YUYV422 + 98,304 byte sau là Y16)
 */
typedef struct {
    uint32_t magic, version, header_bytes;
    int32_t status;
    uint32_t seq, mode, width, height, payload_bytes, timestamp_ms;
    /* reserved[0]: signed BB_* status (two's complement);
     * reserved[1]: probe stage bits 31:24, NACK byte phase bits 23:16,
     *              I2C register bits 15:0. */
    uint32_t index_image, index_temp, flags, vtemp, reserved[2];
} thermal_header_t;

_Static_assert(sizeof(thermal_header_t) == 64, "thermal ABI");
#endif
