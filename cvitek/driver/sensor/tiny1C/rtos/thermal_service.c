/*
 * Dịch vụ thu nhận dữ liệu Camera nhiệt Tiny1C trên nhân RTOS C906 (thermal_service.c).
 *
 * Kiến trúc vận hành:
 *   - Một tác vụ FreeRTOS duy nhất (thermal_service_task) sở hữu hoàn toàn Driver Tầng 3
 *     (I2C5 + SPI0) và vùng đệm phản hồi tĩnh `reply` (64B Header + 192KB Payload).
 *   - Tác vụ nhận lệnh từ Linux thông qua ngắt Mailbox -> hàng đợi E_QUEUE_CAMERA (IP_CAMERA = 7).
 *   - Sau khi xử lý lệnh và điền dữ liệu vào `reply`, tác vụ thực hiện flush D-Cache
 *     (`flush_dcache_range` + `fence rw, rw`) rồi đẩy thông điệp trả lời sang hàng đợi
 *     `E_QUEUE_CMDQU` để tác vụ `prvCmdQuRunTask` gửi trả địa chỉ vật lý `&reply` về Linux.
 *   - Nhờ mô hình kéo tuần tự (Linux gửi 1 lệnh -> chờ nhận -> copy xong mới gửi lệnh tiếp),
 *     không bao giờ xảy ra xung đột ghi đè bộ đệm (Zero Producer-Consumer Race) và không
 *     cần gán cứng địa chỉ bộ nhớ chia sẻ.
 */
#include <stdint.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "comm.h"
#include "rtos_cmdqu.h"
#include "arch_helpers.h"
#include "thermal_protocol.h"
#include "core.h"
#include "data_rw.h"
#include "vospi.h"
#include <string.h>

/*
 * Vùng đệm phản hồi đặt trong phân vùng BSS của RTOS (nằm trong vùng DDR 2MB
 * 0x8fe00000..0x8fffffff của C906), căn chỉnh 64-byte theo kích thước dòng Cache.
 */
static struct {
    thermal_header_t header;
    uint8_t payload[THERMAL_MAX_PAYLOAD];
} reply __attribute__((aligned(64)));

void thermal_service_task(void *unused)
{
    (void)unused;
    bool initialized = false, streaming = false;
    uint32_t sequence = 0;

    /* Lấy handle hàng đợi nhận lệnh (E_QUEUE_CAMERA) và hàng đợi trả kết quả (E_QUEUE_CMDQU) */
    QueueHandle_t input = main_GetMODHandle(E_QUEUE_CAMERA);
    QueueHandle_t output = main_GetMODHandle(E_QUEUE_CMDQU);
    configASSERT(input && output);

    for (;;) {
        cmdqu_t req;
        if (xQueueReceive(input, &req, portMAX_DELAY) != pdPASS) continue;
        if (req.ip_id != IP_CAMERA) continue;

        /* Khởi tạo mặc định các trường mô tả của gói Header 64-byte */
        memset(&reply.header, 0, sizeof(reply.header));
        reply.header.magic = THERMAL_MAGIC;
        reply.header.version = THERMAL_VERSION;
        reply.header.header_bytes = sizeof(thermal_header_t);
        reply.header.seq = sequence;
        reply.header.width = IMG_W;
        reply.header.height = IMG_H;

        int status = IR_UNSUPPORTED_CMD;

        if (req.cmd_id == THERMAL_START) {
            /*
             * Lệnh THERMAL_START (0x40):
             *   - Byte thấp của param_ptr chứa chế độ: MODE_IMAGE (0), MODE_TEMP (1), MODE_DUAL (2).
             *   - Bit 16 (THERMAL_GPIO_OWNED) xác nhận quyền sở hữu GPIOA15/GPIOA27.
             *   - Bit 17 (THERMAL_DMA_OWNED) xác nhận quyền dùng kênh DMA cho SPI0.
             */
            uint32_t mode = req.param_ptr & 0xffU;
            if (!(req.param_ptr & THERMAL_GPIO_OWNED) || mode > MODE_DUAL) {
                status = IR_ERROR_PARAM;
            } else {
                if (!initialized) {
                    data_rw_set_gpioa_ownership(true);
                    hal_spi_set_dma_reserved((req.param_ptr & THERMAL_DMA_OWNED) != 0);
                    status = core_init(5U, I2C_SLAVE_ID);
                    if (status == IR_SUCCESS && spi_stream_init(0, SPI0_DSI_CS_PIN_DEFAULT) != 0)
                        status = IR_START_STREAMING_FAIL;
                    initialized = (status == IR_SUCCESS);
                } else status = IR_SUCCESS;
                if (status == IR_SUCCESS) {
                    streaming = false;
                    status = core_start((camera_mode_t)mode, 25);
                    streaming = (status == IR_SUCCESS);
                    spi_reset_vospi_stats();
                    if (streaming) delay_ms(100); /* Chờ 100ms để cảm biến xuất khung hình đầu tiên */
                }
            }
        } else if (req.cmd_id == THERMAL_CAPTURE) {
            /*
             * Lệnh THERMAL_CAPTURE (0x41):
             *   - Thu 1 khung hình từ cảm biến qua Tầng 3 `spi_get_frame`:
             *     + Chế độ đơn (MODE_IMAGE / MODE_TEMP): 98,304 byte (gói lệnh VoSPI 0xAA).
             *     + Chế độ kép (MODE_DUAL): 196,608 byte (96KB đầu 0xAA + 96KB sau 0xCC).
             *   - Đính kèm siêu dữ liệu từ VoSPI Frame Header (frame_index, vtemp, shutter, freeze, gain).
             */
            status = IR_START_STREAMING_FAIL;
            if (streaming) {
                uint32_t size = core_get_mode() == MODE_DUAL ? DUAL_FRAME_BYTES : FRAME_BYTES;
                status = spi_get_frame(reply.payload, size, VOSPI_HEADER_IMAGE);
                if (status == 0) {
                    const vospi_frame_header_t *h = spi_get_last_header();
                    const vospi_frame_header_t *t = spi_get_last_temp_header();
                    reply.header.payload_bytes = size;
                    reply.header.seq = ++sequence;
                    reply.header.index_image = h->frame_index;
                    reply.header.index_temp = core_get_mode() == MODE_DUAL ? t->frame_index : h->frame_index;
                    reply.header.vtemp = h->vtemp;
                    if (!h->shutter_state) reply.header.flags |= THERMAL_FLAG_SHUTTER_CLOSED;
                    if (h->pix_freeze_state) reply.header.flags |= THERMAL_FLAG_FROZEN;
                    if (h->gain_state) reply.header.flags |= THERMAL_FLAG_HIGH_GAIN;
                    if (core_get_mode() == MODE_DUAL && h->frame_index != t->frame_index)
                        reply.header.flags |= THERMAL_FLAG_DUAL_INDEX_MISMATCH;
                } else if (status == -4) {
                    const uint8_t *rx = spi_get_last_rx_dummy();
                    memcpy(&reply.header.index_image, &rx[64], 4U);
                    memcpy(&reply.header.index_temp, &rx[68], 4U);
                    uint32_t nz_off = 0xFFFFU;
                    uint32_t nz_val = 0U;
                    for (uint32_t k = 0U; k < VOSPI_TX_DUMMY_LEN; ++k) {
                        if (rx[k] != 0U) {
                            nz_off = k;
                            nz_val = rx[k];
                            break;
                        }
                    }
                    reply.header.vtemp = (nz_off << 16U) | (nz_val << 8U) | rx[0];
                }
            }
        } else if (req.cmd_id == THERMAL_STOP) {
            /* Lệnh THERMAL_STOP (0x42): Dừng phát luồng trên ASIC-2121W */
            status = core_stop();
            if (status == IR_SUCCESS) streaming = false;
        } else if (req.cmd_id == THERMAL_FFC) {
            /* Lệnh THERMAL_FFC (0x43): Kích hoạt hiệu chuẩn màn trập (OOC B-Update) */
            status = streaming ? trigger_shutter() : IR_START_STREAMING_FAIL;
        } else if (req.cmd_id == THERMAL_INFO) {
            /* Lệnh THERMAL_INFO (0x44): Kiểm tra trạng thái khởi tạo thiết bị */
            status = initialized ? IR_SUCCESS : IR_NOT_GET_DEVICE_INFO;
            if (initialized) {
                const tiny1c_device_info_t *info = core_get_device_info();
                if (info != NULL) {
                    memcpy(&reply.header.index_image, &info->project_info[0], 4U);
                    memcpy(&reply.header.index_temp, &info->project_info[4], 4U);
                    memcpy(&reply.header.flags, &info->fw_compile_date[0], 4U);
                    memcpy(&reply.header.vtemp, &info->fw_compile_date[4], 4U);
                }
            } else {
                uint8_t first_ack, ack_count;
                if (data_rw_scan_i2c(&first_ack, &ack_count) == 0) {
                    reply.header.index_image = first_ack;
                    reply.header.index_temp = ack_count;
                }
            }
        }

        reply.header.status = status;
        reply.header.mode = core_get_mode();
        reply.header.height = reply.header.mode == MODE_DUAL ? DUAL_HEIGHT : IMG_H;
        reply.header.timestamp_ms = (uint32_t)data_rw_now_ms();
        tiny1c_i2c_diag_t diag = data_rw_get_i2c_diag();
        reply.header.reserved[0] = (uint32_t)diag.bb_status;
        reply.header.reserved[1] = ((uint32_t)diag.stage << 24U) |
                                   ((uint32_t)diag.nack_phase << 16U) | diag.reg;
        if (status < 0 && !(req.cmd_id == THERMAL_CAPTURE && status == -4)) {
            reply.header.vtemp = (uint32_t)diag.vdcmd_status;
            reply.header.flags = (uint32_t)diag.probe_ok_mask;
            if (req.cmd_id == THERMAL_START) {
                memcpy(&reply.header.index_image, &diag.cmd_echo[0], 4U);
                memcpy(&reply.header.index_temp, &diag.cmd_echo[4], 4U);
            }
        }

        /* Đẩy toàn bộ dữ liệu từ D-Cache của C906 RTOS xuống DDR vật lý trước khi báo cho Linux */
        flush_dcache_range((uintptr_t)&reply, sizeof(reply.header) + reply.header.payload_bytes);
        __asm__ volatile("fence rw, rw" ::: "memory");

        /* Trả địa chỉ vật lý 32-bit của cấu trúc `reply` qua trường param_ptr của Mailbox */
        req.param_ptr = (uint32_t)(uintptr_t)&reply;
        req.block = 1;
        req.resv.valid.linux_valid = 0;
        req.resv.valid.rtos_valid = 1;
        /* Ủy quyền cho tác vụ CMDQU sẵn có thực hiện gửi Mailbox phần cứng có bảo vệ spinlock */
        (void)xQueueSend(output, &req, portMAX_DELAY);
    }
}
