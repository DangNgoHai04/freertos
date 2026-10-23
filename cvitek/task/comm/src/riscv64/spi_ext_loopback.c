#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "FreeRTOS.h"
#include "task.h"
#include "mmio.h"
#include "arch_helpers.h"
// #include "spi_smoke_test.h"
#include "spi_ext_loopback.h"

#define SPI0_BASE              0x04180000U

/* DesignWare SSI Register Offsets */
#define DW_SPI_CTRLR0          0x00U
#define DW_SPI_SSIENR          0x08U
#define DW_SPI_SER             0x10U
#define DW_SPI_BAUDR           0x14U
#define DW_SPI_TXFTLR          0x18U
#define DW_SPI_RXFTLR          0x1CU
#define DW_SPI_SR              0x28U
#define DW_SPI_IMR             0x2CU
#define DW_SPI_IDR             0x58U
#define DW_SPI_VERSION         0x5CU
#define DW_SPI_DR              0x60U

/* Clock, Reset, and Pinmux Registers */
#define REG_CLK_EN_1           0x03002004U
#define REG_CLK_EN_3           0x0300200CU
#define REG_CLK_BYP_0          0x03002030U
#define REG_RST_BANK1          0x03003004U

#define PINMUX_PAD_MIPI_TXM2   0x030011A4U /* SCK */
#define PINMUX_PAD_MIPI_TXP2   0x030011A8U /* CS */
#define PINMUX_PAD_MIPI_TXM1   0x030011ACU /* MOSI / SDO */
#define PINMUX_PAD_MIPI_TXP1   0x030011B0U /* MISO / SDI */

#define PADCTRL_PAD_MIPI_TXM2  0x03001C70U
#define PADCTRL_PAD_MIPI_TXP2  0x03001C74U
#define PADCTRL_PAD_MIPI_TXM1  0x03001C78U
#define PADCTRL_PAD_MIPI_TXP1  0x03001C7CU

#define SR_BUSY                (1U << 0)
#define SR_TFNF                (1U << 1)
#define SR_TFE                 (1U << 2)
#define SR_RFNE                (1U << 3)

#define SPI_FUNCTION_MODE_6    6U
#define SPI_EXT_BAUD_DIV       250U /* ~100 kHz: toi uu nhat cho day cau vat ly jumper wire */
#define SPI_CS0               1U
#define SPI_TRANSFER_TIMEOUT   100000U

/* ========================================================================= */
/* [ADDED]: Vung nho Shared Telemetry DDR danh rieng cho External Loopback   */
/* Dia chi: 0x8FF10000, Magic: 0x4558544C ("EXTL")                          */
/* ========================================================================= */
#define SPI_EXT_TELEMETRY_ADDR   0x8FF10000U
#define SPI_EXT_TELEMETRY_MAGIC  0x4558544CU /* "EXTL" (External Loopback) */

struct spi_ext_loopback_telemetry {
    uint32_t magic;              /* 0x00: 0x4558544C ("EXTL") */
    uint32_t state;              /* 0x04: 1=INIT, 2=TESTING, 3=RUNNING */
    uint32_t cycle_count;        /* 0x08: Tong so chu ky da truyen */
    uint32_t match_count;        /* 0x0C: So goi loopback khop thanh cong */
    uint32_t mismatch_count;     /* 0x10: So goi loopback mismatch hoac timeout */
    uint32_t last_match;         /* 0x14: 1 neu lan truyen gan nhat khop, 0 neu loi */
    uint8_t  last_tx_data[4];    /* 0x18: 4 byte gui gan nhat */
    uint8_t  last_rx_data[4];    /* 0x1C: 4 byte nhan gan nhat */
    uint32_t baud_divider;       /* 0x20: DW_SPI_BAUDR */
    uint32_t reg_sr;             /* 0x24: DW_SPI_SR */
    uint32_t pinmux_sck;         /* 0x28: Readback 0x030011a4 (Func 6) */
    uint32_t pinmux_cs;          /* 0x2C: Readback 0x030011a8 (Func 6) */
    uint32_t pinmux_sdo;         /* 0x30: Readback 0x030011ac (Func 6) */
    uint32_t pinmux_sdi;         /* 0x34: Readback 0x030011b0 (Func 6) */
    uint32_t log_len;            /* 0x38: Do dai chuoi log */
    char     log_buf[2048];      /* 0x3C: ASCII log buffer */
};

static volatile struct spi_ext_loopback_telemetry * const g_ext_telemetry =
    (volatile struct spi_ext_loopback_telemetry *)SPI_EXT_TELEMETRY_ADDR;

/* ========================================================================= */
/* [ADDED]: Helper in truc tiep vao thanh ghi phan cung UART0 (0x04140000)   */
/* Co timeout 100000 vong lap chong treo khi he thong chua on dinh           */
/* ========================================================================= */
static inline void raw_uart_putc(char c)
{
    volatile uint32_t *lsr = (volatile uint32_t *)0x04140014U;
    volatile uint32_t *thr = (volatile uint32_t *)0x04140000U;
    uint32_t timeout = 100000U;
    while (!(*lsr & 0x20U) && --timeout)
        ;
    if (timeout) {
        *thr = (uint8_t)c;
    }
}

static void raw_uart_puts(const char *s)
{
    while (*s) {
        if (*s == '\n') raw_uart_putc('\r');
        raw_uart_putc(*s++);
    }
}

/* ========================================================================= */
/* [ADDED]: Helper Dual Log in ra UART0 va luu vao DDR Telemetry Buffer      */
/* ========================================================================= */
static void ext_log_puts(const char *s)
{
    raw_uart_puts(s);
    if (g_ext_telemetry) {
        while (*s && g_ext_telemetry->log_len < (sizeof(g_ext_telemetry->log_buf) - 2)) {
            g_ext_telemetry->log_buf[g_ext_telemetry->log_len++] = *s++;
        }
        g_ext_telemetry->log_buf[g_ext_telemetry->log_len] = '\0';
        flush_dcache_range((uintptr_t)g_ext_telemetry, sizeof(*g_ext_telemetry));
    }
}

static void ext_log_put_hex8(uint8_t val)
{
    const char hex[] = "0123456789ABCDEF";
    char buf[5];
    buf[0] = '0';
    buf[1] = 'x';
    buf[2] = hex[(val >> 4) & 0xFU];
    buf[3] = hex[val & 0xFU];
    buf[4] = '\0';
    ext_log_puts(buf);
}

static void ext_log_put_u32(uint32_t val)
{
    char buf[12];
    int i = 0;
    if (val == 0) {
        ext_log_puts("0");
        return;
    }
    while (val > 0) {
        buf[i++] = '0' + (val % 10);
        val /= 10;
    }
    char rev[12];
    int j = 0;
    while (i > 0) {
        rev[j++] = buf[--i];
    }
    rev[j] = '\0';
    ext_log_puts(rev);
}

/* ========================================================================= */
/* [ADDED]: Khoi tao phan cung cho SPI0 (Clock, Reset, Pinmux, DSI D-PHY)    */
/* ========================================================================= */
#define DSI_PHY_BASE           0x0A0D1000U
#define REG_DSI_PHY_EN         (DSI_PHY_BASE + 0x00U)
#define REG_DSI_PHY_LPTX_OV    (DSI_PHY_BASE + 0x4CU)
#define REG_DSI_PHY_PD_TXDRV   (DSI_PHY_BASE + 0x50U)
#define REG_DSI_PHY_PD_EN_TX   (DSI_PHY_BASE + 0x54U)
#define REG_DSI_PHY_PD         (DSI_PHY_BASE + 0x64U)
#define REG_DSI_PHY_LANE_SEL   (DSI_PHY_BASE + 0x9CU)
#define REG_DSI_PHY_GPO        (DSI_PHY_BASE + 0xC4U)
#define REG_DSI_PHY_GPI        (DSI_PHY_BASE + 0xC8U)

static void spi0_ensure_hw_ready(void)
{
    /* 1. Kich hoat Functional Clock (bit 6) va APB Clock (bit 9) */
    mmio_write_32(REG_CLK_EN_3, mmio_read_32(REG_CLK_EN_3) | (1U << 6));
    mmio_write_32(REG_CLK_EN_1, mmio_read_32(REG_CLK_EN_1) | (1U << 9));

    /* 2. 25MHz OSC bypass cho clk_spi */
    mmio_write_32(REG_CLK_BYP_0, mmio_read_32(REG_CLK_BYP_0) | (1U << 30));

    /* 3. Giai phong Soft Reset SPI0 (active-low) */
    mmio_write_32(REG_RST_BANK1, mmio_read_32(REG_RST_BANK1) | (1U << 8));

    for (volatile int i = 0; i < 500; i++) {
        __asm__ __volatile__("nop");
    }

    /* 4. Chuyen MIPI DSI D-PHY (Lanes 0..4) sang che do Digital GPO/GPI */
    mmio_write_32(REG_DSI_PHY_EN,       0x00000000U);
    mmio_write_32(REG_DSI_PHY_LANE_SEL, 0x000FFFFFU);
    mmio_write_32(REG_DSI_PHY_PD,       0x00001F1FU);
    mmio_write_32(REG_DSI_PHY_LPTX_OV,  0x001F001FU);
    mmio_write_32(REG_DSI_PHY_PD_EN_TX, 0x00001F1FU);
    mmio_write_32(REG_DSI_PHY_PD_TXDRV, 0x1F1F1F1FU);
    mmio_write_32(REG_DSI_PHY_GPO,      0x00001F1FU);
    mmio_write_32(REG_DSI_PHY_GPI,      0x00001F1FU);

    /* 5. Cau hinh PADCTRL (Pull-Up + Drive Strength) va Pinmux Func 6 (SPI0) */
    mmio_write_32(PADCTRL_PAD_MIPI_TXM2, 0x44U);
    mmio_write_32(PADCTRL_PAD_MIPI_TXP2, 0x44U);
    mmio_write_32(PADCTRL_PAD_MIPI_TXM1, 0x44U);
    mmio_write_32(PADCTRL_PAD_MIPI_TXP1, 0x44U);

    mmio_write_32(PINMUX_PAD_MIPI_TXM2, SPI_FUNCTION_MODE_6); /* SCK */
    mmio_write_32(PINMUX_PAD_MIPI_TXP2, SPI_FUNCTION_MODE_6); /* CS */
    mmio_write_32(PINMUX_PAD_MIPI_TXM1, SPI_FUNCTION_MODE_6); /* MOSI / SDO */
    mmio_write_32(PINMUX_PAD_MIPI_TXP1, SPI_FUNCTION_MODE_6); /* MISO / SDI */
}

static inline void spi0_write_reg(uint32_t offset, uint32_t value)
{
    mmio_write_32(SPI0_BASE + offset, value);
}

static inline uint32_t spi0_read_reg(uint32_t offset)
{
    return mmio_read_32(SPI0_BASE + offset);
}

static void spi0_clear_rx_fifo(void)
{
    uint32_t timeout = 10000U;
    while (((spi0_read_reg(DW_SPI_SR) & SR_RFNE) != 0U) && (--timeout > 0U)) {
        (void)spi0_read_reg(DW_SPI_DR);
    }
}

/* ========================================================================= */
/* [ADDED]: Ham truyen nhan vat ly Full-Duplex qua day cau ngoai (SRL = 0)   */
/* Gửi 1 byte ra SDO -> tin hieu chay qua day ngoai vao SDI -> doc tu RX FIFO */
/* ========================================================================= */
static int spi0_transfer_byte_physical(uint8_t tx_val, uint8_t *rx_val)
{
    /* 1. Disable SSI va huy chon slave */
    spi0_write_reg(DW_SPI_SSIENR, 0U);
    spi0_write_reg(DW_SPI_SER, 0U);

    /* 2. CTRLR0: 8-bit, Mode 0, Full Duplex, SRL = 0 (EXTERNAL PIN MODE!) */
    uint32_t ctrlr0 = (0x07U << 0)   /* DFS = 8-bit */
                    | (0x00U << 4)   /* Motorola SPI */
                    | (0x00U << 8);  /* Transmit & Receive, bit 11 SRL = 0 */
    spi0_write_reg(DW_SPI_CTRLR0, ctrlr0);
    spi0_write_reg(DW_SPI_BAUDR, SPI_EXT_BAUD_DIV);

    /* 3. Xoa sach RX FIFO truoc khi truyen */
    spi0_clear_rx_fifo();

    /* 4. Enable SSI controller */
    spi0_write_reg(DW_SPI_SSIENR, 1U);

    /* 5. Ghi du lieu vao TX FIFO */
    spi0_write_reg(DW_SPI_DR, tx_val);

    /* 6. Chon Slave CS0 de bat dau xuat xung SCK va day du lieu ra SDO */
    spi0_write_reg(DW_SPI_SER, SPI_CS0);

    /* 7. Cho RX FIFO co du lieu quay ve qua day cau SDI */
    uint32_t timeout = SPI_TRANSFER_TIMEOUT;
    while (((spi0_read_reg(DW_SPI_SR) & SR_RFNE) == 0U) && (--timeout > 0U)) {
        __asm__ __volatile__("nop");
    }

    if (timeout == 0U) {
        spi0_write_reg(DW_SPI_SER, 0U);
        spi0_write_reg(DW_SPI_SSIENR, 0U);
        return -1; /* Timeout: day cau chua tiep xuc tot */
    }

    /* 8. Doc byte nhan duoc */
    uint8_t received = (uint8_t)spi0_read_reg(DW_SPI_DR);
    if (rx_val) {
        *rx_val = received;
    }

    /* 9. Cho bus idle va ket thuc giao tiep */
    timeout = 10000U;
    while ((spi0_read_reg(DW_SPI_SR) & SR_BUSY) && (--timeout > 0U))
        ;

    spi0_write_reg(DW_SPI_SER, 0U);
    spi0_write_reg(DW_SPI_SSIENR, 0U);

    return 0;
}

/* ========================================================================= */
/* [ADDED]: Truyen nhan goi 4 byte qua day cau SDO-SDI bang byte-by-byte     */
/* Co framing CS doc lap va timeout ro rang giup do nhay cao tren day cau    */
/* ========================================================================= */
static int spi0_transfer_packet_physical(const uint8_t *tx_buf, uint8_t *rx_buf, uint32_t len)
{
    if (!tx_buf || !rx_buf || len == 0) return -1;

    for (uint32_t i = 0; i < len; i++) {
        int ret = spi0_transfer_byte_physical(tx_buf[i], &rx_buf[i]);
        if (ret != 0) {
            return ret;
        }
    }
    return 0;
}

/* ========================================================================= */
/* [ADDED]: Kiem tra toan dien Physical Loopback khi board khoi dong         */
/* ========================================================================= */
void spi_ext_loopback_run(void)
{
    g_ext_telemetry->state = 2; // TESTING
    flush_dcache_range((uintptr_t)g_ext_telemetry, sizeof(*g_ext_telemetry));

    spi0_ensure_hw_ready();

    ext_log_puts("\n=======================================================\n");
    ext_log_puts("    SPI0 EXTERNAL PHYSICAL LOOPBACK TEST (SDO <-> SDI) \n");
    ext_log_puts("=======================================================\n");
    ext_log_puts("Testing physical jumper wire between:\n");
    ext_log_puts("  - SDO / MOSI : PAD_MIPI_TXM1 (DSI FPC Pin)\n");
    ext_log_puts("  - SDI / MISO : PAD_MIPI_TXP1 (DSI FPC Pin)\n");
    ext_log_puts("  - Mode       : Hardware External Pins (SRL = 0)\n");
    ext_log_puts("  - Speed      : ~100 kHz (Baud divider 250)\n\n");

    const uint8_t test_patterns[] = { 0x5A, 0xA5, 0x3C, 0xC3, 0x00, 0xFF, 0x12, 0x34 };
    const int num_tests = sizeof(test_patterns) / sizeof(test_patterns[0]);
    int pass_count = 0;

    for (int i = 0; i < num_tests; i++) {
        uint8_t tx = test_patterns[i];
        uint8_t rx = 0;

        ext_log_puts("  - Pattern [");
        ext_log_put_u32(i + 1);
        ext_log_puts("] TX: ");
        ext_log_put_hex8(tx);
        ext_log_puts(" -> ");

        int ret = spi0_transfer_byte_physical(tx, &rx);
        if (ret == 0 && rx == tx) {
            ext_log_puts("RX: ");
            ext_log_put_hex8(rx);
            ext_log_puts(" [PASS MATCH]\n");
            pass_count++;
            g_ext_telemetry->match_count++;
        } else if (ret == 0) {
            ext_log_puts("RX: ");
            ext_log_put_hex8(rx);
            ext_log_puts(" [FAIL MISMATCH]\n");
            g_ext_telemetry->mismatch_count++;
        } else {
            ext_log_puts("[FAIL TIMEOUT - Check wire!]\n");
            g_ext_telemetry->mismatch_count++;
        }
    }

    ext_log_puts("-------------------------------------------------------\n");
    ext_log_puts("Summary: ");
    ext_log_put_u32(pass_count);
    ext_log_puts("/");
    ext_log_put_u32(num_tests);
    ext_log_puts(" patterns matched on physical wire!\n");

    if (pass_count == num_tests) {
        ext_log_puts("===> PHYSICAL SDO-SDI JUMPER WIRE CONNECTION IS VERIFIED!\n");
        g_ext_telemetry->last_match = 1;
    } else {
        ext_log_puts("===> ONE OR MORE PATTERNS FAILED. Check wiring on DSI connector.\n");
        g_ext_telemetry->last_match = 0;
    }
    ext_log_puts("=======================================================\n\n");

    // Cap nhat thanh ghi readback vao telemetry
    g_ext_telemetry->baud_divider = spi0_read_reg(DW_SPI_BAUDR);
    g_ext_telemetry->reg_sr       = spi0_read_reg(DW_SPI_SR);
    g_ext_telemetry->pinmux_sck   = mmio_read_32(PINMUX_PAD_MIPI_TXM2);
    g_ext_telemetry->pinmux_cs    = mmio_read_32(PINMUX_PAD_MIPI_TXP2);
    g_ext_telemetry->pinmux_sdo   = mmio_read_32(PINMUX_PAD_MIPI_TXM1);
    g_ext_telemetry->pinmux_sdi   = mmio_read_32(PINMUX_PAD_MIPI_TXP1);
    flush_dcache_range((uintptr_t)g_ext_telemetry, sizeof(*g_ext_telemetry));
}

/* ========================================================================= */
/* [ADDED]: FreeRTOS Background Task truyen nhan packet dinh ky 1 giay       */
/* ========================================================================= */
static void spi_ext_loopback_task(void *pvParameters)
{
    (void)pvParameters;

    g_ext_telemetry->state = 3; // RUNNING
    flush_dcache_range((uintptr_t)g_ext_telemetry, sizeof(*g_ext_telemetry));

    raw_uart_puts("[RTOS SPI0-EXT] Background task running! Testing physical wire every 1s...\r\n");

    uint32_t count = 0;
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
        count++;
        g_ext_telemetry->cycle_count = count;

        /* Dam bao phan cung luon san sang ke ca khi Linux hoac tien trinh khac can thiep */
        spi0_ensure_hw_ready();

        /* Tao goi tin 4-byte co ma kiem tra */
        uint8_t tx_pkt[4] = {
            0xAA,
            (uint8_t)(count & 0xFF),
            0x55,
            (uint8_t)((~count) & 0xFF)
        };
        uint8_t rx_pkt[4] = { 0, 0, 0, 0 };

        int ret = spi0_transfer_packet_physical(tx_pkt, rx_pkt, 4);

        memcpy((void *)g_ext_telemetry->last_tx_data, tx_pkt, 4);
        memcpy((void *)g_ext_telemetry->last_rx_data, rx_pkt, 4);

        int match = (ret == 0 &&
                     rx_pkt[0] == tx_pkt[0] &&
                     rx_pkt[1] == tx_pkt[1] &&
                     rx_pkt[2] == tx_pkt[2] &&
                     rx_pkt[3] == tx_pkt[3]);

        if (match) {
            g_ext_telemetry->match_count++;
            g_ext_telemetry->last_match = 1;
        } else {
            g_ext_telemetry->mismatch_count++;
            g_ext_telemetry->last_match = 0;
        }

        g_ext_telemetry->reg_sr = spi0_read_reg(DW_SPI_SR);
        flush_dcache_range((uintptr_t)g_ext_telemetry, sizeof(*g_ext_telemetry));

        /* In heartbeat ra UART0 moi 5 giay */
        if ((count % 5) == 0) {
            raw_uart_puts("[RTOS SPI0-EXT] Packet #");
            ext_log_put_u32(count);
            if (match) {
                raw_uart_puts(" -> WIRE MATCH OK [AA ");
                ext_log_put_hex8(rx_pkt[1]);
                raw_uart_puts(" 55 ");
                ext_log_put_hex8(rx_pkt[3]);
                raw_uart_puts("]\r\n");
            } else {
                raw_uart_puts(" -> WIRE MISMATCH (Check jumper)\r\n");
            }
        }
    }
}

/* ========================================================================= */
/* [ADDED]: Ham khoi dong task External Loopback                             */
/* ========================================================================= */
void start_spi_ext_loopback_task(void)
{
    /* 1. Khoi tao vung Shared Telemetry cho External Loopback tai 0x8FF10000 */
    memset((void *)g_ext_telemetry, 0, sizeof(*g_ext_telemetry));
    g_ext_telemetry->magic = SPI_EXT_TELEMETRY_MAGIC;
    g_ext_telemetry->state = 1; // INIT
    flush_dcache_range((uintptr_t)g_ext_telemetry, sizeof(*g_ext_telemetry));

    /* 2. Chay smoke test phan cung ban dau */
    // spi_smoke_test_run();

    /* 3. Chay test External Loopback tren day cau SDO-SDI */
    spi_ext_loopback_run();

    /* 4. Tao FreeRTOS Background Task de truyen nhan lien tuc */
    BaseType_t ret = xTaskCreate(spi_ext_loopback_task, "SPI0_EXT", 512, NULL, tskIDLE_PRIORITY + 3, NULL);
    if (ret == pdPASS) {
        ext_log_puts("[RTOS SPI0-EXT] Background task created successfully!\n");
    } else {
        ext_log_puts("[RTOS SPI0-EXT] Warning: xTaskCreate failed!\n");
    }
    flush_dcache_range((uintptr_t)g_ext_telemetry, sizeof(*g_ext_telemetry));
}
