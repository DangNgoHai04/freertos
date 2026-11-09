/*
 * Tầng 2 (T2 - RTOS SPI Bus & Device Driver): Điều phối giao dịch đa phân đoạn,
 * quản lý khóa tương tranh FreeRTOS Mutex, điều khiển chân Chip-Select và chọn lựa
 * cơ chế thực thi (POLL / IRQ / DMA / AUTO).
 *
 * Tác dụng: Cung cấp API cấp cao thread-safe cho Sensor Drivers (T3), tự động nạp cấu hình,
 * chọn backend truyền nhận tối ưu theo độ dài gói và tự phục hồi khi có lỗi phần cứng.
 */

#include "spi.h"

#define SPI_DEFAULT_TIMEOUT_MS          1000U
#define SPI_POLL_SPINS_PER_MS           1000U
#define SPI_MIN_POLL_BUDGET             2048U

/* --------------------------------------------------------------------------
 * 1. Các hàm nội bộ: ánh xạ lỗi, tính toán thời gian & chọn Backend
 * -------------------------------------------------------------------------- */

/* Tác dụng: Ánh xạ mã lỗi phần cứng T1 (hal_spi_status_t) sang mã lỗi T2 (spi_status_t) */
static spi_status_t map_hal_status(hal_spi_status_t status)
{
    switch (status) {
    case HAL_SPI_OK:
        return SPI_OK;
    case HAL_SPI_ERR_INVALID:
        return SPI_ERR_INVALID;
    case HAL_SPI_ERR_BUSY:
        return SPI_ERR_BUSY;
    case HAL_SPI_ERR_FIFO:
        return SPI_ERR_FIFO;
    case HAL_SPI_ERR_TIMEOUT:
        return SPI_ERR_TIMEOUT;
    case HAL_SPI_ERR_UNSUPPORTED:
    default:
        return SPI_ERR_UNSUPPORTED;
    }
}

/* Tác dụng: Quy đổi thời gian timeout từ mili-giây sang số tick của FreeRTOS */
static TickType_t timeout_to_ticks(uint32_t timeout_ms)
{
    uint32_t effective_ms = (timeout_ms == 0U) ? SPI_DEFAULT_TIMEOUT_MS : timeout_ms;
    uint64_t ticks = ((uint64_t)effective_ms * (uint64_t)configTICK_RATE_HZ + 999ULL) / 1000ULL;

    if (ticks == 0ULL) {
        ticks = 1ULL;
    }
    if (ticks > (uint64_t)portMAX_DELAY) {
        ticks = (uint64_t)portMAX_DELAY;
    }
    return (TickType_t)ticks;
}

/* Tác dụng: Tính toán ngân sách số vòng lặp polling an toàn dựa trên số khung và deadline */
static size_t compute_poll_budget(size_t frames, uint32_t timeout_ms)
{
    uint32_t effective_ms = (timeout_ms == 0U) ? SPI_DEFAULT_TIMEOUT_MS : timeout_ms;
    size_t budget = (size_t)effective_ms * SPI_POLL_SPINS_PER_MS + (frames * 16U);

    if (budget < SPI_MIN_POLL_BUDGET) {
        budget = SPI_MIN_POLL_BUDGET;
    }
    return budget;
}

/* Tác dụng: Trì hoãn CPU chính xác theo micro-giây bằng vòng lặp NOP */
static void delay_microseconds(uint16_t delay_us)
{
    uint32_t loops = (uint32_t)delay_us * 120U;
    for (volatile uint32_t i = 0; i < loops; ++i) {
        __asm__ volatile("nop");
    }
}

/* Tác dụng: Kiểm tra mã backend có thuộc tập hợp hợp lệ (POLL, IRQ, DMA, AUTO) hay không */
static bool is_valid_backend(spi_backend_t backend)
{
    return (backend == SPI_BACKEND_POLL) ||
           (backend == SPI_BACKEND_IRQ) ||
           (backend == SPI_BACKEND_DMA) ||
           (backend == SPI_BACKEND_AUTO);
}

/* Tác dụng: So sánh nhanh 2 cấu hình phần cứng để tránh nạp lại thanh ghi không cần thiết */
static bool hal_configs_equal(const hal_spi_config_t *a, const hal_spi_config_t *b)
{
    return (a->sck_hz == b->sck_hz) &&
           (a->mode == b->mode) &&
           (a->bits_per_word == b->bits_per_word) &&
           (a->rx_sample_dly == b->rx_sample_dly) &&
           (a->lsb_first == b->lsb_first);
}

/* Tác dụng: Kiểm tra quyền sử dụng backend dựa trên cấu hình cho phép của Bus */
static bool backend_allowed(const spi_bus_t *bus, spi_backend_t backend)
{
    if (backend == SPI_BACKEND_IRQ)
        return bus->enable_irq;
    if (backend == SPI_BACKEND_DMA)
        return bus->enable_dma && bus->dma_resources_reserved;
    return backend == SPI_BACKEND_POLL || backend == SPI_BACKEND_AUTO;
}

/* Tác dụng: Chọn cơ chế thực thi tối ưu (POLL, IRQ, hoặc DMA) cho từng phân đoạn theo kích thước */
static spi_backend_t select_segment_backend(const spi_device_t *dev, size_t seg_frames)
{
    const spi_bus_t *bus = dev->bus;

    if (dev->backend != SPI_BACKEND_AUTO) {
        return dev->backend;
    }

    if (bus->enable_dma && seg_frames >= (size_t)bus->dma_threshold_frames) {
        return SPI_BACKEND_DMA;
    }
    if (bus->enable_irq && seg_frames >= (size_t)bus->irq_threshold_frames) {
        return SPI_BACKEND_IRQ;
    }
    return SPI_BACKEND_POLL;
}

/* Tác dụng: Thực thi phân đoạn hiện tại bằng cơ chế đã chọn và cập nhật bộ đếm thống kê */
static hal_spi_status_t execute_segment_engine(spi_bus_t *bus,
                                               spi_backend_t engine,
                                               size_t frames,
                                               uint32_t timeout_ms)
{
    hal_spi_status_t status = HAL_SPI_PENDING;

    if (engine == SPI_BACKEND_DMA) {
        bus->dma_segment_count += 1U;
        return hal_spi_transfer_dma(&bus->hal, timeout_ms);
    }

    if (engine == SPI_BACKEND_IRQ) {
        bus->irq_segment_count += 1U;
        return hal_spi_transfer_irq(&bus->hal, timeout_ms);
    }

    bus->poll_segment_count += 1U;
    {
        size_t budget = compute_poll_budget(frames, timeout_ms);
        for (size_t step = 0; step < budget; ++step) {
            status = hal_spi_poll(&bus->hal);
            if (status != HAL_SPI_PENDING) {
                break;
            }
        }
        if (status == HAL_SPI_PENDING) {
            status = HAL_SPI_ERR_TIMEOUT;
        }
    }
    return status;
}

/* --------------------------------------------------------------------------
 * 2. Triển khai các hàm khởi tạo, quản lý và cấu hình Bus / Thiết bị
 * -------------------------------------------------------------------------- */

/* Tác dụng: Khởi tạo Bus SPI, mutex tĩnh bảo vệ tương tranh và liên kết với phần cứng T0/T1 */
spi_status_t spi_bus_init(spi_bus_t *bus, const spi_bus_config_t *cfg)
{
    hal_spi_status_t hal_status;

    if (bus == NULL || cfg == NULL || cfg->reg_base == 0U) {
        return SPI_ERR_INVALID;
    }

    /* Chặn thiếu quyền DMA trước cả khi khởi tạo platform SPI */
    if (cfg->enable_dma && !cfg->dma_resources_reserved)
        return SPI_ERR_UNSUPPORTED;

    bus->mutex = xSemaphoreCreateMutexStatic(&bus->mutex_storage);
    if (bus->mutex == NULL) {
        return SPI_ERR_INVALID;
    }

    hal_status = hal_spi_init(&bus->hal, cfg->reg_base, cfg->ref_clock_hz);
    if (hal_status != HAL_SPI_OK) {
        return map_hal_status(hal_status);
    }

    bus->active_dev = NULL;
    bus->last_error = SPI_OK;
    bus->enable_irq = cfg->enable_irq;
    bus->enable_dma = cfg->enable_dma;
    bus->dma_resources_reserved = cfg->dma_resources_reserved;
    bus->irq_threshold_frames = (cfg->irq_threshold_frames > 0U)
                                    ? cfg->irq_threshold_frames
                                    : SPI_DEFAULT_IRQ_THRESHOLD_FRAMES;
    bus->dma_threshold_frames = (cfg->dma_threshold_frames > 0U)
                                    ? cfg->dma_threshold_frames
                                    : SPI_DEFAULT_DMA_THRESHOLD_FRAMES;
    bus->poll_segment_count = 0U;
    bus->irq_segment_count = 0U;
    bus->dma_segment_count = 0U;
    bus->error_count = 0U;
    hal_spi_snapshot(&bus->hal, &bus->last_snapshot);
    bus->initialized = true;
    return SPI_OK;
}

/* Tác dụng: Đưa Bus SPI về trạng thái nghỉ, thu hồi tài nguyên và hủy trạng thái khởi tạo */
spi_status_t spi_bus_deinit(spi_bus_t *bus)
{
    if (bus == NULL || !bus->initialized) {
        return SPI_ERR_INVALID;
    }

    if (xSemaphoreTake(bus->mutex, timeout_to_ticks(SPI_DEFAULT_TIMEOUT_MS)) != pdTRUE) {
        return SPI_ERR_BUSY;
    }

    hal_spi_abort(&bus->hal);
    bus->active_dev = NULL;
    bus->initialized = false;
    (void)xSemaphoreGive(bus->mutex);
    return SPI_OK;
}

/* Tác dụng: Cưỡng bức thu hồi và làm mới Bus SPI an toàn đa luồng khi xảy ra sự cố ngoại vi */
spi_status_t spi_bus_recover(spi_bus_t *bus)
{
    if (bus == NULL || !bus->initialized) {
        return SPI_ERR_INVALID;
    }

    if (xSemaphoreTake(bus->mutex, timeout_to_ticks(SPI_DEFAULT_TIMEOUT_MS)) != pdTRUE) {
        return SPI_ERR_BUSY;
    }

    hal_spi_snapshot(&bus->hal, &bus->last_snapshot);
    hal_spi_abort(&bus->hal);
    bus->active_dev = NULL;
    (void)xSemaphoreGive(bus->mutex);
    return SPI_OK;
}

/* Tác dụng: Gắn và kiểm tra tính hợp lệ của một Thiết bị ngoại vi trên Bus SPI */
spi_status_t spi_device_init(spi_device_t *dev,
                             spi_bus_t *bus,
                             const spi_device_config_t *cfg)
{
    hal_spi_config_t candidate;
    hal_spi_status_t status;

    if (dev == NULL || bus == NULL || cfg == NULL || !bus->initialized) {
        return SPI_ERR_INVALID;
    }
    if (cfg->cs_id != SPI_CS0_GPIO) {
        return SPI_ERR_INVALID;
    }
    if (!is_valid_backend(cfg->backend) || !backend_allowed(bus, cfg->backend)) {
        return SPI_ERR_UNSUPPORTED;
    }

    candidate.sck_hz = cfg->sck_hz;
    candidate.mode = cfg->mode;
    candidate.bits_per_word = cfg->bits_per_word;
    candidate.rx_sample_dly = cfg->rx_sample_dly;
    candidate.lsb_first = cfg->lsb_first;

    status = hal_spi_validate_config(&bus->hal, &candidate);
    if (status != HAL_SPI_OK) {
        return map_hal_status(status);
    }

    dev->bus = bus;
    dev->hal_cfg = candidate;
    dev->cs_id = cfg->cs_id;
    dev->backend = cfg->backend;
    dev->initialized = true;
    return SPI_OK;
}

/* Tác dụng: Đổi động cơ chế truyền nhận (POLL, IRQ, DMA, AUTO) cho thiết bị ngoại vi */
spi_status_t spi_device_set_backend(spi_device_t *dev, spi_backend_t backend)
{
    if (dev == NULL || !dev->initialized || dev->bus == NULL || !dev->bus->initialized) {
        return SPI_ERR_INVALID;
    }
    if (!is_valid_backend(backend) || !backend_allowed(dev->bus, backend)) {
        return SPI_ERR_UNSUPPORTED;
    }

    if (xSemaphoreTake(dev->bus->mutex, timeout_to_ticks(SPI_DEFAULT_TIMEOUT_MS)) != pdTRUE)
        return SPI_ERR_BUSY;
    dev->backend = backend;
    (void)xSemaphoreGive(dev->bus->mutex);
    return SPI_OK;
}

/* Tác dụng: Đổi nóng tần số xung SCK và độ trễ lấy mẫu MISO của thiết bị lúc đang chạy */
spi_status_t spi_device_set_freq(spi_device_t *dev,
                                 uint32_t sck_hz,
                                 uint8_t rx_sample_dly)
{
    hal_spi_config_t candidate;
    hal_spi_status_t status;
    spi_bus_t *bus;

    if (dev == NULL || !dev->initialized || dev->bus == NULL) {
        return SPI_ERR_INVALID;
    }

    bus = dev->bus;
    candidate = dev->hal_cfg;
    candidate.sck_hz = sck_hz;
    candidate.rx_sample_dly = rx_sample_dly;

    status = hal_spi_validate_config(&bus->hal, &candidate);
    if (status != HAL_SPI_OK) {
        return map_hal_status(status);
    }

    if (xSemaphoreTake(bus->mutex, timeout_to_ticks(SPI_DEFAULT_TIMEOUT_MS)) != pdTRUE) {
        return SPI_ERR_BUSY;
    }

    dev->hal_cfg = candidate;
    if (bus->active_dev == dev) {
        /* Đánh dấu cần nạp lại BAUDR và RX_SAMPLE_DLY ở giao dịch kế tiếp */
        bus->active_dev = NULL;
    }

    (void)xSemaphoreGive(bus->mutex);
    return SPI_OK;
}

/* --------------------------------------------------------------------------
 * 3. Triển khai các hàm truyền nhận dữ liệu (spi_transfer & các hàm tiện ích)
 * -------------------------------------------------------------------------- */

/* Tác dụng: Thực thi giao dịch SPI nguyên tử đa phân đoạn, quản lý giữ/nhả CS và phục hồi lỗi */
spi_status_t spi_transfer(spi_device_t *dev,
                          const spi_segment_t *segments,
                          size_t count,
                          uint32_t timeout_ms)
{
    spi_bus_t *bus;
    spi_status_t result = SPI_OK;

    if (dev == NULL || !dev->initialized || dev->bus == NULL ||
        !dev->bus->initialized || segments == NULL || count == 0U) {
        return SPI_ERR_INVALID;
    }

    /* Kiểm tra tính hợp lệ của toàn bộ mảng phân đoạn trước khi chiếm Mutex */
    for (size_t i = 0; i < count; ++i) {
        if (segments[i].frames == 0U ||
            (segments[i].tx == NULL && segments[i].rx == NULL)) {
            return SPI_ERR_INVALID;
        }
    }

    bus = dev->bus;

    if (xSemaphoreTake(bus->mutex, timeout_to_ticks(timeout_ms)) != pdTRUE) {
        return SPI_ERR_BUSY;
    }

    /* Chuẩn bị tất cả backend cần dùng trước khi kéo CS xuống LOW */
    if (!backend_allowed(bus, dev->backend)) {
        bus->last_error = SPI_ERR_UNSUPPORTED;
        (void)xSemaphoreGive(bus->mutex);
        return SPI_ERR_UNSUPPORTED;
    }
    for (size_t i = 0; i < count; ++i) {
        spi_backend_t engine = select_segment_backend(dev, segments[i].frames);
        hal_spi_status_t status = HAL_SPI_OK;
        if (engine == SPI_BACKEND_IRQ)
            status = hal_spi_prepare_irq(&bus->hal);
        else if (engine == SPI_BACKEND_DMA)
            status = hal_spi_prepare_dma(&bus->hal, bus->dma_resources_reserved);
        if (status != HAL_SPI_OK) {
            result = map_hal_status(status);
            bus->last_error = result;
            bus->error_count += 1U;
            (void)xSemaphoreGive(bus->mutex);
            return result;
        }
    }

    /* Nạp lại cấu hình phần cứng nếu chuyển sang thiết bị mới có thông số khác (Lazy Reconfig) */
    if (bus->active_dev != dev || !bus->hal.configured) {
        if (bus->active_dev == NULL || !bus->hal.configured ||
            !hal_configs_equal(&bus->active_dev->hal_cfg, &dev->hal_cfg)) {
            hal_spi_status_t cfg_status = hal_spi_configure(&bus->hal, &dev->hal_cfg);
            if (cfg_status != HAL_SPI_OK) {
                result = map_hal_status(cfg_status);
                bus->last_error = result;
                (void)xSemaphoreGive(bus->mutex);
                return result;
            }
        }
        bus->active_dev = dev;
    }

    /* Kéo chân Chip-Select xuống mức LOW để bắt đầu phiên giao dịch */
    hal_spi_platform_cs_set(true);

    for (size_t i = 0; i < count; ++i) {
        const spi_segment_t *seg = &segments[i];
        spi_backend_t seg_engine = select_segment_backend(dev, seg->frames);
        hal_spi_status_t status;

        status = hal_spi_begin(&bus->hal,
                               seg->tx,
                               seg->rx,
                               seg->frames,
                               seg->fill_word);
        if (status == HAL_SPI_OK) {
            status = execute_segment_engine(bus, seg_engine, seg->frames, timeout_ms);
        }

        if (status != HAL_SPI_OK) {
            /* Chụp lại thanh ghi lỗi và thu hồi phần cứng ngay lập tức */
            hal_spi_snapshot(&bus->hal, &bus->last_snapshot);
            hal_spi_abort(&bus->hal);
            bus->active_dev = NULL;
            bus->error_count += 1U;
            result = map_hal_status(status);
            bus->last_error = result;
            (void)xSemaphoreGive(bus->mutex);
            return result;
        }

        hal_spi_finish(&bus->hal);

        if (seg->delay_us > 0U) {
            delay_microseconds(seg->delay_us);
        }

        if (seg->cs_change && (i + 1U) < count) {
            hal_spi_platform_cs_set(false);
            delay_microseconds(2U);
            hal_spi_platform_cs_set(true);
        }
    }

    /* Nhả chân Chip-Select lên mức HIGH khi toàn bộ các phân đoạn đã hoàn tất */
    hal_spi_platform_cs_set(false);
    hal_spi_snapshot(&bus->hal, &bus->last_snapshot);
    bus->last_error = SPI_OK;
    (void)xSemaphoreGive(bus->mutex);
    return SPI_OK;
}

/* Tác dụng: Thực thi giao dịch đơn lẻ gồm 1 phân đoạn (Full-Duplex, TX-only hoặc RX-only) */
spi_status_t spi_transfer_one(spi_device_t *dev,
                              const void *tx,
                              void *rx,
                              size_t frames,
                              uint16_t fill_word,
                              uint32_t timeout_ms)
{
    spi_segment_t seg;

    seg.tx = tx;
    seg.rx = rx;
    seg.frames = frames;
    seg.fill_word = fill_word;
    seg.delay_us = 0U;
    seg.cs_change = false;
    return spi_transfer(dev, &seg, 1U, timeout_ms);
}

/* Tác dụng: Gửi gói lệnh (TX), đợi delay_us, rồi đọc dữ liệu (RX) trong cùng 1 lần giữ CS */
spi_status_t spi_write_then_read(spi_device_t *dev,
                                 const void *tx_cmd,
                                 size_t tx_frames,
                                 void *rx_data,
                                 size_t rx_frames,
                                 uint16_t delay_us,
                                 uint32_t timeout_ms)
{
    spi_segment_t segs[2];

    if (tx_cmd == NULL || rx_data == NULL || tx_frames == 0U || rx_frames == 0U) {
        return SPI_ERR_INVALID;
    }

    segs[0].tx = tx_cmd;
    segs[0].rx = NULL;
    segs[0].frames = tx_frames;
    segs[0].fill_word = 0U;
    segs[0].delay_us = delay_us;
    segs[0].cs_change = false;

    segs[1].tx = NULL;
    segs[1].rx = rx_data;
    segs[1].frames = rx_frames;
    segs[1].fill_word = 0xFFFFU;
    segs[1].delay_us = 0U;
    segs[1].cs_change = false;

    return spi_transfer(dev, segs, 2U, timeout_ms);
}
