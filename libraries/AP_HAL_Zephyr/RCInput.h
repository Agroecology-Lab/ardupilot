#pragma once

#include <cstdint>

#include <AP_HAL/AP_HAL.h>
#include "AP_HAL_Zephyr_Namespace.h"

struct device;

class Zephyr::RCInput : public AP_HAL::RCInput {
public:
    RCInput();

    void    init() override;
    bool    new_input() override;
    uint8_t num_channels() override;
    uint16_t read(uint8_t ch) override;
    uint8_t  read(uint16_t *periods, uint8_t len) override;
    const char *protocol() const override;

    // registered as a timer proc; drains AP::RC and copies decoded
    // channels into the cache exposed to the main thread.
    void _timer_tick();

private:
    // Apply serial framing RC_CONFIGS[idx] to the UART and re-arm RX.
    void _apply_config(uint8_t idx);

    // resolved from ardupilot-rcinput DT alias; null if absent
    const struct device *_dev;
    bool                 _has_uart;
    uint8_t           _cfg_idx;
    uint32_t          _last_cfg_change_ms;
    uint32_t          _last_input_ms;
    bool              _detected;
    volatile uint32_t _current_baud;  // baud hint handed to process_byte()

    static constexpr uint8_t MAX_CH = 18u;
    uint16_t      _values[MAX_CH];
    uint8_t       _num_channels;
    volatile bool _new_input;

    static void _uart_isr(const struct device *dev, void *user_data);
};
