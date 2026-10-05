#pragma once
#include <cstddef>
#include <cstdint>
#include "freertos/queue.h"
using uart_port_t = int;
using esp_err_t = int;
constexpr esp_err_t ESP_OK = 0;
constexpr esp_err_t ESP_FAIL = -1;
constexpr int UART_DATA_8_BITS = 8;
constexpr int UART_PARITY_EVEN = 2;
constexpr int UART_STOP_BITS_1 = 1;
constexpr int UART_HW_FLOWCTRL_DISABLE = 0;
constexpr int UART_SCLK_DEFAULT = 0;
constexpr int UART_PIN_NO_CHANGE = -1;
enum { UART_DATA, UART_FIFO_OVF, UART_BUFFER_FULL };
struct uart_event_t { int type; };
struct uart_config_t {
    int baud_rate, data_bits, parity, stop_bits, flow_ctrl, source_clk, rx_flow_ctrl_thresh;
    struct { bool allow_pd, backup_before_sleep; } flags;
};
inline const char *esp_err_to_name(esp_err_t) { return "mock error"; }
esp_err_t uart_driver_install(uart_port_t, int, int, int, QueueHandle_t *, int);
esp_err_t uart_driver_delete(uart_port_t);
esp_err_t uart_param_config(uart_port_t, const uart_config_t *);
esp_err_t uart_set_pin(uart_port_t, int, int, int, int);
esp_err_t uart_set_rx_full_threshold(uart_port_t, int);
esp_err_t uart_set_rx_timeout(uart_port_t, uint8_t);
esp_err_t uart_get_buffered_data_len(uart_port_t, size_t *);
esp_err_t uart_get_tx_buffer_free_size(uart_port_t, size_t *);
int uart_write_bytes(uart_port_t, const void *, size_t);
int uart_read_bytes(uart_port_t, void *, uint32_t, TickType_t);
esp_err_t uart_flush_input(uart_port_t);
