#include "TPUart/Interface/EspIdf.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

            uart_driver_install(_uart_num, 256 * 2, 0, 0, NULL, 0);
            uart_param_config(_uart_num, &uart_config);
            uart_set_pin(_uart_num, _tx_pin, _rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
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
            uart_driver_delete(_uart_num);
            _installed = false;
        }

        bool EspIdf::available()
        {
            if (!_installed)
            {
                return false;
            }

            size_t length = 0;
            uart_get_buffered_data_len(_uart_num, &length);
            return length > 0;
        }

        bool EspIdf::availableForWrite()
        {
            if (!_installed)
            {
                return false;
            }

            size_t free_size = 0;
            uart_get_tx_buffer_free_size(_uart_num, &free_size);
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

            uart_flush(_uart_num);
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
