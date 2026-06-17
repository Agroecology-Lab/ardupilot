#include "RCInput.h"

#include <algorithm>
#include <cstring>

#include <AP_RCProtocol/AP_RCProtocol.h>

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/irq.h>
#endif

using namespace Zephyr;

constexpr uint8_t RCInput::MAX_CH;

extern const AP_HAL::HAL &hal;

RCInput::RCInput()
    : _dev(nullptr)
    , _has_uart(false)
    , _cfg_idx(0u)
    , _last_cfg_change_ms(0u)
    , _last_input_ms(0u)
    , _detected(false)
    , _current_baud(115200u)
    , _values{}
    , _num_channels(0u)
    , _new_input(false)
{
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
namespace {
struct RcSerialConfig {
    uint32_t baud;
    uint8_t  parity;     // enum uart_config_parity value
    uint8_t  stop_bits;  // enum uart_config_stop_bits value
};
const RcSerialConfig RC_CONFIGS[] = {
    // baud,   parity,               stop bits
    { 115200u, UART_CFG_PARITY_NONE, UART_CFG_STOP_BITS_1 }, // iBUS, DSM, SUMD, FPort, SRXL2, CRSF*
    { 100000u, UART_CFG_PARITY_EVEN, UART_CFG_STOP_BITS_2 }, // SBUS framing
};
const uint8_t RC_NUM_CONFIGS = sizeof(RC_CONFIGS) / sizeof(RC_CONFIGS[0]);
} // namespace

void RCInput::_apply_config(uint8_t idx)
{
    const struct uart_config cfg = {
        .baudrate  = RC_CONFIGS[idx].baud,
        .parity    = RC_CONFIGS[idx].parity,
        .stop_bits = RC_CONFIGS[idx].stop_bits,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE,
    };
    (void)uart_configure(_dev, &cfg);
    _current_baud = RC_CONFIGS[idx].baud;

    // uart_configure can reset the peripheral — (re)arm the RX path.
    uart_irq_callback_user_data_set(_dev, _uart_isr, this);
    uart_irq_rx_enable(_dev);
}
#endif

void RCInput::init()
{
    _num_channels = 0u;
    _new_input    = false;

    AP::RC().init();

    hal.scheduler->register_timer_process(
        FUNCTOR_BIND_MEMBER(&RCInput::_timer_tick, void));

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
#if DT_HAS_ALIAS(ardupilot_rcinput) && DT_NODE_HAS_STATUS(DT_ALIAS(ardupilot_rcinput), okay)
    _dev = DEVICE_DT_GET(DT_ALIAS(ardupilot_rcinput));
    if (!device_is_ready(_dev)) {
        _dev      = nullptr;
        _has_uart = false;
        return;
    }

    // Start on the first framing and let _timer_tick() cycle through the
    // rest until AP_RCProtocol detects a protocol.
    _cfg_idx            = 0u;
    _detected           = false;
    _last_cfg_change_ms = AP_HAL::millis();
    _last_input_ms      = _last_cfg_change_ms;
    _apply_config(_cfg_idx);   // sets _current_baud and arms RX IRQ
    _has_uart = true;
#endif
#endif
}

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
void RCInput::_uart_isr(const struct device *dev, void *user_data)
{
    RCInput *self = static_cast<RCInput *>(user_data);
    const uint32_t baud = self ? self->_current_baud : 115200u;
    while (uart_irq_update(dev) && uart_irq_rx_ready(dev)) {
        uint8_t byte;
        if (uart_fifo_read(dev, &byte, 1) == 1) {
            AP::RC().process_byte(byte, baud);
        }
    }
}
#endif

void RCInput::_timer_tick()
{
    // new_input() latches (resets its flag), so call it exactly once.
    const bool have_input = AP::RC().new_input();

#ifdef CONFIG_UART_INTERRUPT_DRIVEN
    if (_has_uart) {
        const uint32_t now = AP_HAL::millis();
        if (have_input) {
            // a protocol decoded a frame — hold this framing
            _last_input_ms = now;
            _detected      = true;
        } else if (!_detected && (now - _last_cfg_change_ms) > 1000u) {
            // nothing decoded within a second — try the next framing
            _cfg_idx = (uint8_t)((_cfg_idx + 1u) % RC_NUM_CONFIGS);
            _apply_config(_cfg_idx);
            _last_cfg_change_ms = now;
        } else if (_detected && (now - _last_input_ms) > 1000u) {
            // lost the link (receiver reboot / re-bind) — resume searching
            _detected           = false;
            _last_cfg_change_ms = now;
        }
    }
#endif

    if (!have_input) {
        return;
    }
    unsigned int key = irq_lock();
    _num_channels = std::min<uint8_t>(AP::RC().num_channels(), MAX_CH);
    AP::RC().read(_values, _num_channels);
    _new_input = true;
    irq_unlock(key);
}

bool RCInput::new_input()
{
    if (!_new_input) {
        return false;
    }
    unsigned int key = irq_lock();
    _new_input = false;
    irq_unlock(key);
    return true;
}

uint8_t RCInput::num_channels()
{
    return _num_channels;
}

uint16_t RCInput::read(uint8_t ch)
{
    if (ch >= _num_channels) {
        return 1500u;
    }
    unsigned int key = irq_lock();
    const uint16_t val = _values[ch];
    irq_unlock(key);
    return val;
}

uint8_t RCInput::read(uint16_t *periods, uint8_t len)
{
    const uint8_t count = std::min<uint8_t>(len, _num_channels);
    if (count == 0u) {
        return 0u;
    }
    unsigned int key = irq_lock();
    memcpy(periods, _values, count * sizeof(uint16_t));
    irq_unlock(key);
    return count;
}

const char *RCInput::protocol() const
{
    if (!_has_uart) {
        return "ZephyrStub";
    }
    const char *name = AP::RC().detected_protocol_name();
    return name ? name : "searching";
}
