#ifndef TPUART_INTERFACE_ESPIDF_H
#define TPUART_INTERFACE_ESPIDF_H

#include "TPUart/Interface/Abstract.h"
#include "driver/uart.h"
#include "freertos/queue.h"

#include <functional>

namespace TPUart
{
    namespace Interface
    {
        class EspIdf : public Abstract
        {
          public:
            EspIdf(uart_port_t uart_num, int rx_pin, int tx_pin, int baud = 19200);
            ~EspIdf();

            void begin(int baud) override;
            void end() override;
            bool available() override;
            bool availableForWrite() override;
            bool write(char value) override;
            int read() override;
            bool overflow() override;
            void flush() override;
            bool hasCallback() override;
            void registerCallback(std::function<bool()> callback) override;

          private:
            void drainEvents();

            uart_port_t _uart_num;
            int _rx_pin;
            int _tx_pin;
            int _baud;
            bool _installed = false;
            volatile bool _overflow = false;
            QueueHandle_t _event_queue = nullptr;
            std::function<bool()> _callback;
        };
    }
}

#endif /* TPUART_INTERFACE_ESPIDF_H */
