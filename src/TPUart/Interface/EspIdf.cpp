#include "TPUart/Interface/EspIdf.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char* TAG = "tpuart_espidf";

#ifndef TPUART_ESPIDF_RX_BUFFER_SIZE
#define TPUART_ESPIDF_RX_BUFFER_SIZE 512
#endif

#ifndef TPUART_ESPIDF_TX_BUFFER_SIZE
#define TPUART_ESPIDF_TX_BUFFER_SIZE 512
#endif

#ifndef TPUART_ESPIDF_EVENT_QUEUE_SIZE
#define TPUART_ESPIDF_EVENT_QUEUE_SIZE 32
#endif

namespace TPUart
{
    namespace Interface
    {
        EspIdf::EspIdf(uart_port_t uart_num, int rx_pin, int tx_pin, int baud)
            : _uart_num(uart_num), _rx_pin(rx_pin), _tx_pin(tx_pin), _baud(baud)
        {
        }

        EspIdf::~EspIdf()
        {
            end();
        }

        void EspIdf::begin(int baud)
        {
            if (_installed)
            {
                return;
            }

            _baud = baud;
            _event_queue = nullptr;
            _overflow = false;

            uart_config_t uart_config = {};
            uart_config.baud_rate = _baud;
            uart_config.data_bits = UART_DATA_8_BITS;
            uart_config.parity = UART_PARITY_EVEN;
            uart_config.stop_bits = UART_STOP_BITS_1;
            uart_config.flow_ctrl = UART_HW_FLOWCTRL_DISABLE;
            uart_config.source_clk = UART_SCLK_DEFAULT;
            uart_config.rx_flow_ctrl_thresh = 0;
            uart_config.flags.allow_pd = 0;
            uart_config.flags.backup_before_sleep = 0;

            esp_err_t err = uart_driver_install(_uart_num,
                                                TPUART_ESPIDF_RX_BUFFER_SIZE,
                                                TPUART_ESPIDF_TX_BUFFER_SIZE,
                                                TPUART_ESPIDF_EVENT_QUEUE_SIZE,
                                                &_event_queue,
                                                0);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "uart_driver_install(%d) failed: %s", (int)_uart_num, esp_err_to_name(err));
                return;
            }

            err = uart_param_config(_uart_num, &uart_config);
            if (err == ESP_OK)
                err = uart_set_pin(_uart_num, _tx_pin, _rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
            if (err != ESP_OK)
            {
                ESP_LOGE(TAG, "UART %d configuration failed: %s", (int)_uart_num, esp_err_to_name(err));
                uart_driver_delete(_uart_num);
                _event_queue = nullptr;
                return;
            }
            _installed = true;
            _running = true;
        }

        void EspIdf::end()
        {
            if (!_installed)
            {
                return;
            }

            _running = false;
            const esp_err_t err = uart_driver_delete(_uart_num);
            if (err != ESP_OK)
                ESP_LOGW(TAG, "uart_driver_delete(%d) failed: %s", (int)_uart_num, esp_err_to_name(err));
            _event_queue = nullptr;
            _overflow = false;
            _installed = false;
        }

        void EspIdf::drainEvents()
        {
            if (!_installed || _event_queue == nullptr)
                return;

            // A full event queue means the ISR may already have dropped the
            // notification that tells us which bytes are no longer reliable.
            // Conservatively invalidate the current stream before draining it.
            if (uxQueueSpacesAvailable(_event_queue) == 0)
                _overflow = true;

            uart_event_t event = {};
            while (xQueueReceive(_event_queue, &event, 0) == pdTRUE)
            {
                // Match the Arduino ESP32 backend: either condition means that
                // the byte stream can no longer be assumed to be contiguous.
                if (event.type == UART_FIFO_OVF || event.type == UART_BUFFER_FULL)
                    _overflow = true;
            }
        }

        bool EspIdf::available()
        {
            if (!_installed)
            {
                return false;
            }

            drainEvents();
            size_t length = 0;
            if (uart_get_buffered_data_len(_uart_num, &length) != ESP_OK)
                return false;
            return length > 0;
        }

        bool EspIdf::availableForWrite()
        {
            if (!_installed)
            {
                return false;
            }

            size_t free_size = 0;
            if (uart_get_tx_buffer_free_size(_uart_num, &free_size) != ESP_OK)
                return false;
            return free_size > 0;
        }

        bool EspIdf::write(char value)
        {
            if (!_installed)
            {
                return false;
            }

            return uart_write_bytes(_uart_num, &value, 1) == 1;
        }

        int EspIdf::read()
        {
            if (!_installed)
            {
                return -1;
            }

            drainEvents();
            uint8_t data = 0;
            int len = uart_read_bytes(_uart_num, &data, 1, pdMS_TO_TICKS(10));
            if (len > 0)
            {
                return data;
            }

            return -1;
        }

        bool EspIdf::overflow()
        {
            drainEvents();
            bool ov = _overflow;
            _overflow = false;
            return ov;
        }

        void EspIdf::flush()
        {
            if (!_installed)
            {
                return;
            }

            const esp_err_t err = uart_flush_input(_uart_num);
            if (err != ESP_OK)
            {
                ESP_LOGW(TAG, "uart_flush_input(%d) failed: %s", (int)_uart_num, esp_err_to_name(err));
                return;
            }

            // uart_flush_input() clears the RX FIFO/ring buffer but not the
            // driver's event queue. Drop events that refer to the discarded
            // bytes so a reset cannot report a stale overflow afterwards.
            if (_event_queue != nullptr)
                xQueueReset(_event_queue);
            _overflow = false;
        }

        bool EspIdf::hasCallback()
        {
            return _callback != nullptr;
        }

        void EspIdf::registerCallback(std::function<bool()> callback)
        {
            _callback = callback;
        }
    }
}
