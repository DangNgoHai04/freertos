/* T0 clock, reset, MIPI DSI D-PHY, pinmux FPC2 và GPIO CS cho CV181x SPI0.
 * Phần xử lý ngắt PLIC, System DMA và đồng bộ D-Cache nằm trong hal_spi_platform.c.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "hal_dw_spi.h"
#include "hal_spi_platform_limits.h"
#include "mmio.h"
#include "top_reg.h"

/* --------------------------------------------------------------------------
 * 1. Địa chỉ thanh ghi hệ thống SoC SG2002 (Clock, Reset, DSI D-PHY, Pinmux, GPIO)
 * -------------------------------------------------------------------------- */

#define SPI0_BASE_ADDR                  0x04180000UL

/* Khối Clock Generator (0x03002000) theo TRM Chương 8 */
#define CLK_EN_0_REG                    0x03002000UL
#define CLK_EN_0_APB_GPIO_BIT           (1U << 29)  /* clk_en_0_29: clk_apb_gpio */

#define CLK_EN_1_REG                    0x03002004UL
#define CLK_EN_1_APB_SPI0_BIT           (1U << 9)   /* clk_en_1_9: clk_apb_spi0 */

#define CLK_EN_3_REG                    0x0300200CUL
#define CLK_EN_3_SPI_BIT                (1U << 6)   /* clk_en_3_6: clk_spi */

#define CLK_BYP_0_REG                   0x03002030UL
#define CLK_BYP_0_SPI_BIT               (1U << 30)  /* clk_byp_0_30: 1 = XTAL 25MHz, 0 = FPLL div */

#define DIV_CLK_SPI_REG                 0x03002100UL
#define DIV_CLK_SPI_RESET_DIS_BIT       (1U << 0)   /* Bit 0: 1 = nhả reset bộ chia, 0 = giữ reset */
#define DIV_CLK_SPI_REG_SEL_BIT         (1U << 3)   /* Bit 3: 1 = lấy hệ số chia từ [20:16], 0 = mặc định 8 */
#define DIV_CLK_SPI_FACTOR_SHIFT        16U
#define DIV_CLK_SPI_FACTOR_MASK         0x1FU
#define DIV_CLK_SPI_DEFAULT_DIV         8U          /* Hệ số chia phần cứng mặc định: 1500 MHz / 8 = 187.5 MHz */

#define SG2002_XTAL_HZ                  25000000UL
#define SG2002_FPLL_HZ                  1500000000UL

/* Khối Reset Controller (0x03003000) theo TRM Bảng 7-3 (SOFT_RSTN_1) */
#define RESET_BANK1_REG                 0x03003004UL
#define RESET_BANK1_SPI0_BIT            (1U << 8)   /* reg_soft_reset_x_spi0 (tích cực mức thấp) */

/* Khối MIPI DSI D-PHY (0x0A0D1000) cho các chân MIPI_TX trên cổng FPC2 */
#define DSI_DPHY_BASE_ADDR              0x0A0D1000UL
#define REG_DSI_PHY_EN                  (DSI_DPHY_BASE_ADDR + 0x00UL)
#define REG_DSI_PHY_LPTX_OV             (DSI_DPHY_BASE_ADDR + 0x4CUL)
#define REG_DSI_PHY_PD_TXDRV            (DSI_DPHY_BASE_ADDR + 0x50UL)
#define REG_DSI_PHY_PD_EN_TX            (DSI_DPHY_BASE_ADDR + 0x54UL)
#define REG_DSI_PHY_PD                  (DSI_DPHY_BASE_ADDR + 0x64UL)
#define REG_DSI_PHY_LANE_SEL            (DSI_DPHY_BASE_ADDR + 0x9CUL)
#define REG_DSI_PHY_GPO                 (DSI_DPHY_BASE_ADDR + 0xC4UL)
#define REG_DSI_PHY_GPI                 (DSI_DPHY_BASE_ADDR + 0xC8UL)
#define DSI_PHY_DIGITAL_IO_MASK         0x00001F1FU

/* Thanh ghi cấu hình điện học Pad (PADCTRL) cho nhóm chân MIPI_TX trên FPC2 */
#define PADCTRL_PAD_MIPI_TXM2_REG       0x03001C70UL /* FPC2.15: SPI0_SCK */
#define PADCTRL_PAD_MIPI_TXP2_REG       0x03001C74UL /* FPC2.14: GPIOC_17 (CS) */
#define PADCTRL_PAD_MIPI_TXM1_REG       0x03001C78UL /* FPC2.18: SPI0_SDO (MOSI) */
#define PADCTRL_PAD_MIPI_TXP1_REG       0x03001C7CUL /* FPC2.17: SPI0_SDI (MISO) */
#define PADCTRL_SPI0_DEFAULT_CFG        0x44U        /* Bật Pull-Up và cấu hình Drive Strength */

/* Thanh ghi chọn chức năng chân (FMUX) theo SDK cv181x_reg_fmux_gpio.h */
#define PINMUX_PAD_MIPI_TXM2_REG        0x030011A4UL /* FPC2.15: Func 6 = SPI0_SCK */
#define PINMUX_PAD_MIPI_TXP2_REG        0x030011A8UL /* FPC2.14: Func 3 = XGPIOC_17 */
#define PINMUX_PAD_MIPI_TXM1_REG        0x030011ACUL /* FPC2.18: Func 6 = SPI0_SDO */
#define PINMUX_PAD_MIPI_TXP1_REG        0x030011B0UL /* FPC2.17: Func 6 = SPI0_SDI */
#define PINMUX_FUNCSEL_MASK             0x7U

#define PINMUX_FUNC_XGPIOC_17           3U // Chức năng GPIOC_17 (CS) trên FPC2.14
#define PINMUX_FUNC_SPI0                6U

/* Khối điều khiển GPIO Port C (0x03022000) cho chân Chip-Select phần mềm GPIOC17 */
#define GPIOC_BASE_ADDR                 0x03022000UL
#define GPIO_SWPORTA_DR_OFF             0x00UL
#define GPIO_SWPORTA_DDR_OFF            0x04UL
#define SPI0_CS_GPIOC_PIN               17U
#define SPI0_CS_GPIOC_BIT               (1U << SPI0_CS_GPIOC_PIN)

static void platform_io_short_delay(unsigned int loops)
{
    for (volatile unsigned int i = 0; i < loops; ++i) {
        __asm__ volatile("nop");
    }
}

/* --------------------------------------------------------------------------
 * 2. Triển khai điều khiển Clock, Reset và đọc tần số nguồn clk_spi
 * -------------------------------------------------------------------------- */

uint32_t hal_spi_platform_get_ref_clock_hz(void)
{
    uint32_t byp_reg;
    uint32_t div_reg;
    uint32_t div_factor;

    byp_reg = mmio_read_32(CLK_BYP_0_REG);
    if ((byp_reg & CLK_BYP_0_SPI_BIT) != 0U) {
        return (uint32_t)SG2002_XTAL_HZ;
    }

    div_reg = mmio_read_32(DIV_CLK_SPI_REG);
    if ((div_reg & DIV_CLK_SPI_RESET_DIS_BIT) == 0U) {
        return 0U;
    }

    if ((div_reg & DIV_CLK_SPI_REG_SEL_BIT) != 0U) {
        div_factor = (div_reg >> DIV_CLK_SPI_FACTOR_SHIFT) & DIV_CLK_SPI_FACTOR_MASK;
    } else {
        div_factor = DIV_CLK_SPI_DEFAULT_DIV;
    }

    if (div_factor == 0U) {
        return 0U;
    }

    return (uint32_t)(SG2002_FPLL_HZ / (unsigned long)div_factor);
}

hal_spi_status_t hal_spi_platform_clock_enable(uintptr_t reg_base) //uintptr_t la kieu so nguyen khong dau co kich thuoc tuong ung voi kich thuoc con tro
{
    uint32_t clk_en0;
    uint32_t clk_en1;
    uint32_t clk_en3;

    if (reg_base != SPI0_BASE_ADDR) {
        return HAL_SPI_ERR_INVALID;
    }

    /* Mở cổng clock APB cho GPIO, clock APB cho SPI0 và functional clock clk_spi */
    clk_en0 = mmio_read_32(CLK_EN_0_REG);
    mmio_write_32(CLK_EN_0_REG, clk_en0 | CLK_EN_0_APB_GPIO_BIT);

    clk_en1 = mmio_read_32(CLK_EN_1_REG);
    mmio_write_32(CLK_EN_1_REG, clk_en1 | CLK_EN_1_APB_SPI0_BIT);

    clk_en3 = mmio_read_32(CLK_EN_3_REG);
    mmio_write_32(CLK_EN_3_REG, clk_en3 | CLK_EN_3_SPI_BIT);

    /* Đọc kiểm chứng các cổng clock của SPI0 đã mở và nguồn clock hợp lệ */
    if ((mmio_read_32(CLK_EN_1_REG) & CLK_EN_1_APB_SPI0_BIT) == 0U ||
        (mmio_read_32(CLK_EN_3_REG) & CLK_EN_3_SPI_BIT) == 0U ||
        hal_spi_platform_get_ref_clock_hz() == 0U) {
        return HAL_SPI_ERR_INVALID;
    }

    return HAL_SPI_OK;
}

hal_spi_status_t hal_spi_platform_reset(uintptr_t reg_base)
{
    uint32_t rst;

    if (reg_base != SPI0_BASE_ADDR) {
        return HAL_SPI_ERR_INVALID;
    }

    /* Tạo xung Reset cứng (tích cực mức thấp) tại SOFT_RSTN_1 bit 8 cho khối SPI0 */
    rst = mmio_read_32(RESET_BANK1_REG);
    mmio_write_32(RESET_BANK1_REG, rst & ~RESET_BANK1_SPI0_BIT);
    platform_io_short_delay(64U);

    mmio_write_32(RESET_BANK1_REG, rst | RESET_BANK1_SPI0_BIT);
    platform_io_short_delay(64U);

    if ((mmio_read_32(RESET_BANK1_REG) & RESET_BANK1_SPI0_BIT) == 0U) {
        return HAL_SPI_ERR_INVALID;
    }

    return HAL_SPI_OK;
}

/* --------------------------------------------------------------------------
 * 3. Triển khai GPIO Chip-Select (GPIOC17) và Pinmux FPC2 (MIPI_TX)
 * -------------------------------------------------------------------------- */

void hal_spi_platform_cs_init(void)
{
    uint32_t dr = mmio_read_32(GPIOC_BASE_ADDR + GPIO_SWPORTA_DR_OFF);
    uint32_t ddr = mmio_read_32(GPIOC_BASE_ADDR + GPIO_SWPORTA_DDR_OFF);

    /* Nạp mức logic CAO (nhả CS) vào thanh ghi dữ liệu trước khi bật hướng Output */
    mmio_write_32(GPIOC_BASE_ADDR + GPIO_SWPORTA_DR_OFF, dr | SPI0_CS_GPIOC_BIT);
    mmio_write_32(GPIOC_BASE_ADDR + GPIO_SWPORTA_DDR_OFF, ddr | SPI0_CS_GPIOC_BIT);
}

void hal_spi_platform_cs_set(bool asserted)
{
    uint32_t dr = mmio_read_32(GPIOC_BASE_ADDR + GPIO_SWPORTA_DR_OFF);

    if (asserted) {
        dr &= ~SPI0_CS_GPIOC_BIT; /* Kéo chân CS xuống THẤP (chọn thiết bị Slave) */
    } else {
        dr |= SPI0_CS_GPIOC_BIT;  /* Kéo chân CS lên CAO (nhả thiết bị Slave) */
    }
    mmio_write_32(GPIOC_BASE_ADDR + GPIO_SWPORTA_DR_OFF, dr);
}

hal_spi_status_t hal_spi_platform_pinmux_init(void)
{
    /* 1. Mở khóa chế độ Digital GPO/GPI cho 5 lane trên khối MIPI DSI D-PHY (0x0A0D1000) */
    mmio_write_32(REG_DSI_PHY_EN, 0x00000000U);
    mmio_write_32(REG_DSI_PHY_LANE_SEL, 0x000FFFFFU);
    mmio_write_32(REG_DSI_PHY_PD, DSI_PHY_DIGITAL_IO_MASK);
    mmio_write_32(REG_DSI_PHY_LPTX_OV, 0x001F001FU);
    mmio_write_32(REG_DSI_PHY_PD_EN_TX, DSI_PHY_DIGITAL_IO_MASK);
    mmio_write_32(REG_DSI_PHY_PD_TXDRV, 0x1F1F1F1FU);
    mmio_write_32(REG_DSI_PHY_GPO, DSI_PHY_DIGITAL_IO_MASK);
    mmio_write_32(REG_DSI_PHY_GPI, DSI_PHY_DIGITAL_IO_MASK);

    /* 2. Thiết lập PADCTRL (Pull-Up và Drive Strength) cho 4 chân SPI0 trên FPC2 */
    mmio_write_32(PADCTRL_PAD_MIPI_TXM2_REG, PADCTRL_SPI0_DEFAULT_CFG);
    mmio_write_32(PADCTRL_PAD_MIPI_TXP2_REG, PADCTRL_SPI0_DEFAULT_CFG);
    mmio_write_32(PADCTRL_PAD_MIPI_TXM1_REG, PADCTRL_SPI0_DEFAULT_CFG);
    mmio_write_32(PADCTRL_PAD_MIPI_TXP1_REG, PADCTRL_SPI0_DEFAULT_CFG);

    /* 3. Ánh xạ FMUX: CS về GPIOC17 (Func 3), SCK/MOSI/MISO về SPI0 (Func 6) */
    mmio_write_32(PINMUX_PAD_MIPI_TXP2_REG, PINMUX_FUNC_XGPIOC_17);
    mmio_write_32(PINMUX_PAD_MIPI_TXM2_REG, PINMUX_FUNC_SPI0);
    mmio_write_32(PINMUX_PAD_MIPI_TXM1_REG, PINMUX_FUNC_SPI0);
    mmio_write_32(PINMUX_PAD_MIPI_TXP1_REG, PINMUX_FUNC_SPI0);

    /* 4. Đọc kiểm chứng (Readback) cấu hình D-PHY, GPIO CS và FMUX */
    if ((mmio_read_32(REG_DSI_PHY_GPO) & DSI_PHY_DIGITAL_IO_MASK) != DSI_PHY_DIGITAL_IO_MASK ||
        (mmio_read_32(REG_DSI_PHY_GPI) & DSI_PHY_DIGITAL_IO_MASK) != DSI_PHY_DIGITAL_IO_MASK ||
        (mmio_read_32(GPIOC_BASE_ADDR + GPIO_SWPORTA_DDR_OFF) & SPI0_CS_GPIOC_BIT) == 0U ||
        (mmio_read_32(PINMUX_PAD_MIPI_TXP2_REG) & PINMUX_FUNCSEL_MASK) != PINMUX_FUNC_XGPIOC_17 ||
        (mmio_read_32(PINMUX_PAD_MIPI_TXM2_REG) & PINMUX_FUNCSEL_MASK) != PINMUX_FUNC_SPI0 ||
        (mmio_read_32(PINMUX_PAD_MIPI_TXM1_REG) & PINMUX_FUNCSEL_MASK) != PINMUX_FUNC_SPI0 ||
        (mmio_read_32(PINMUX_PAD_MIPI_TXP1_REG) & PINMUX_FUNCSEL_MASK) != PINMUX_FUNC_SPI0) {
        return HAL_SPI_ERR_INVALID;
    }

    return HAL_SPI_OK;
}
