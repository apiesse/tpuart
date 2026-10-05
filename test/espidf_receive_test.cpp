#include "TPUart/DataLinkLayer.h"
#include "TPUart/Interface/EspIdf.h"
#include <algorithm>
#include <cassert>
#include <deque>
#include <vector>

// Model the IDF boundary that matters to Receiver: available() sees only the
// ring buffer, never the hardware FIFO. TP bytes arrive 13 bits apart at 9600
// bit/s; a UART timeout is measured in 11-bit 8E1 characters at the UART baud.
static uint64_t nowUs = 0;
unsigned long millis() { return nowUs / 1000; }
unsigned long micros() { return nowUs; }
void vTaskDelay(TickType_t ticks) { nowUs += ticks * 1000; }

struct MockUart {
    bool installed = false;
    int baud = 19200;
    int fullThreshold = 120;
    int timeoutSymbols = 10;
    int eventCapacity = 0;
    int deleteCount = 0;
    int failAt = 0;
    int configStep = 0;
    size_t largestEventCount = 0;
    uint64_t lastByteUs = 0;
    std::deque<std::pair<uint64_t, uint8_t>> wire;
    std::deque<uint8_t> fifo, ring;
    std::deque<uart_event_t> events;

    uint64_t timeoutUs() const { return (11000000ULL * timeoutSymbols + baud - 1) / baud; }
    void flushFifo() {
        if (fifo.empty()) return;
        ring.insert(ring.end(), fifo.begin(), fifo.end());
        fifo.clear();
        if (events.size() < static_cast<size_t>(eventCapacity))
            events.push_back({UART_DATA});
        largestEventCount = std::max(largestEventCount, events.size());
    }
    void pump() {
        while (!wire.empty() && wire.front().first <= nowUs) {
            auto byte = wire.front(); wire.pop_front();
            if (!fifo.empty() && byte.first - lastByteUs >= timeoutUs()) flushFifo();
            fifo.push_back(byte.second);
            lastByteUs = byte.first;
            if (fifo.size() >= static_cast<size_t>(fullThreshold)) flushFifo();
        }
        if (!fifo.empty() && nowUs - lastByteUs >= timeoutUs()) flushFifo();
    }
    esp_err_t configure() { return ++configStep == failAt ? ESP_FAIL : ESP_OK; }
} uart;

unsigned uxQueueSpacesAvailable(QueueHandle_t) {
    uart.pump();
    return uart.eventCapacity - uart.events.size();
}
int xQueueReceive(QueueHandle_t, void *event, TickType_t) {
    uart.pump();
    if (uart.events.empty()) return pdFALSE;
    *static_cast<uart_event_t *>(event) = uart.events.front(); uart.events.pop_front();
    return pdTRUE;
}
void xQueueReset(QueueHandle_t) { uart.events.clear(); }
esp_err_t uart_driver_install(uart_port_t, int, int, int capacity, QueueHandle_t *queue, int) {
    assert(!uart.installed);
    uart.installed = true;
    uart.fullThreshold = 120;
    uart.timeoutSymbols = 10;
    uart.eventCapacity = capacity;
    uart.configStep = 0;
    *queue = &uart;
    return ESP_OK;
}
esp_err_t uart_driver_delete(uart_port_t) {
    assert(uart.installed);
    uart.installed = false;
    ++uart.deleteCount;
    uart.wire.clear(); uart.fifo.clear(); uart.ring.clear(); uart.events.clear();
    return ESP_OK;
}
esp_err_t uart_param_config(uart_port_t, const uart_config_t *config) {
    uart.baud = config->baud_rate;
    return uart.configure();
}
esp_err_t uart_set_pin(uart_port_t, int, int, int, int) { return uart.configure(); }
esp_err_t uart_set_rx_full_threshold(uart_port_t, int threshold) {
    if (uart.configure() != ESP_OK) return ESP_FAIL;
    uart.fullThreshold = threshold;
    return ESP_OK;
}
esp_err_t uart_set_rx_timeout(uart_port_t, uint8_t timeout) {
    if (uart.configure() != ESP_OK) return ESP_FAIL;
    uart.timeoutSymbols = timeout;
    return ESP_OK;
}
esp_err_t uart_get_buffered_data_len(uart_port_t, size_t *length) {
    uart.pump(); *length = uart.ring.size(); return ESP_OK;
}
esp_err_t uart_get_tx_buffer_free_size(uart_port_t, size_t *length) {
    *length = 512; return ESP_OK;
}
int uart_write_bytes(uart_port_t, const void *data, size_t length) {
    const auto *bytes = static_cast<const uint8_t *>(data);
    for (size_t i = 0; i < length; ++i) {
        if (bytes[i] == U_RESET_REQ) uart.ring.push_back(U_RESET_IND);
    }
    return length;
}
int uart_read_bytes(uart_port_t, void *data, uint32_t length, TickType_t) {
    uart.pump();
    const size_t size = std::min<size_t>(length, uart.ring.size());
    for (size_t i = 0; i < size; ++i) {
        static_cast<uint8_t *>(data)[i] = uart.ring.front(); uart.ring.pop_front();
    }
    return size;
}
esp_err_t uart_flush_input(uart_port_t) {
    uart.wire.clear(); uart.fifo.clear(); uart.ring.clear(); return ESP_OK;
}

static std::vector<uint8_t> longFrame() {
    // Extended TP frame for the OTA DATA112 envelope (126 APDU bytes).
    std::vector<uint8_t> bytes(135, 0xFF);
    const uint8_t header[] = {0x30, 0x60, 0x11, 0x62, 0x11, 0x27, 126, 0x62, 0xC7};
    std::copy(std::begin(header), std::end(header), bytes.begin());
    uint8_t checksum = 0xFF;
    for (size_t i = 0; i + 1 < bytes.size(); ++i) checksum ^= bytes[i];
    bytes.back() = checksum;
    TPUart::Frame frame(reinterpret_cast<const char *>(bytes.data()), bytes.size(), false);
    assert(frame.isValid());
    return bytes;
}

static void receiveLongFrame(bool legacyThresholds, int baud, unsigned phaseMs = 4) {
    uart = MockUart{};
    nowUs = 0;
    TPUart::Interface::EspIdf interface(1, 2, 3, baud);
    TPUart::DataLinkLayer dll;
    std::vector<std::vector<uint8_t>> received;
    dll.registerReceivedFrame([&](TPUart::Frame &frame) {
        received.emplace_back(frame.data(), frame.data() + frame.size());
    });
    dll.begin(TPUart::BCU_NCN5120, &interface);
    dll.process();
    assert(uart.installed);
    // The DLL probes at 19200 first. Reopen the real backend at either supported
    // UART speed without changing the physical TP byte rate.
    interface.end();
    interface.begin(baud);
    if (legacyThresholds) { uart.fullThreshold = 120; uart.timeoutSymbols = 10; }
    const auto bytes = longFrame();
    const uint64_t base = nowUs;
    for (size_t i = 0; i < bytes.size(); ++i)
        uart.wire.emplace_back(base + 1000 + i * 1354, bytes[i]);
    // At 164 ms the legacy FIFO has exposed 120 bytes. At 184 ms its
    // remaining 15 bytes are still hidden, so the real Receiver marks a
    // timeout. When the tail finally arrives it discards the valid frame.
    for (unsigned ms = phaseMs; ms <= 240 + phaseMs; ms += 20) {
        nowUs = base + ms * 1000;
        dll.process();
    }
    if (legacyThresholds && baud == 19200) {
        assert(received.empty());
        assert(dll.getStatistics().getRxDiscardedBytes() > 0);
    }
    else assert(received == std::vector<std::vector<uint8_t>>{bytes});
    assert(!interface.overflow());
    dll.end(false);
}

static void checkConfigurationFailure() {
    for (const int failAt : {1, 2, 3, 4}) {
        uart = MockUart{};
        uart.failAt = failAt;
        TPUart::Interface::EspIdf interface(1, 2, 3);
        interface.begin(19200);
        assert(!uart.installed && uart.deleteCount == 1);
        assert(!interface.available() && !interface.write(0));
        uart.failAt = 0;
        interface.begin(19200);
        assert(uart.installed && uart.fullThreshold == 1 && uart.timeoutSymbols == 2);
        interface.end();
        assert(uart.deleteCount == 2);
    }
}

static void checkEventQueueMarginAndErrors() {
    uart = MockUart{};
    nowUs = 0;
    TPUart::Interface::EspIdf interface(1, 2, 3);
    interface.begin(19200);
    // A 20 ms poll plus a 20 ms transmitter slice at TP1 speed must not
    // exhaust the queue and falsely invalidate a continuous byte stream.
    for (unsigned i = 0; i < 31; ++i) uart.wire.emplace_back(i * 1354, 0xFF);
    nowUs = 42000;
    assert(interface.available());
    assert(!interface.overflow());
    assert(uart.largestEventCount == 31);
    assert(uart.largestEventCount < static_cast<size_t>(uart.eventCapacity));
    // Allow one additional delayed poll without the old 32-event queue's false
    // overflow; genuine queue exhaustion below remains conservatively invalid.
    for (unsigned i = 0; i < 45; ++i) uart.wire.emplace_back(nowUs + i * 1354, 0xFF);
    nowUs += 61000;
    assert(interface.available());
    assert(!interface.overflow());
    assert(uart.largestEventCount == 45);
    // Real overflow notifications must still invalidate the stream.
    uart.events.push_back({UART_FIFO_OVF});
    assert(interface.overflow());
    assert(!interface.overflow());
    for (int i = 0; i < uart.eventCapacity; ++i) uart.events.push_back({UART_DATA});
    assert(interface.overflow());
    assert(!interface.overflow());
}

int main() {
    receiveLongFrame(true, 19200); // Regression reproduction with IDF defaults.
    for (unsigned phaseMs = 0; phaseMs < 20; ++phaseMs) {
        receiveLongFrame(false, 19200, phaseMs);
        receiveLongFrame(false, 38400, phaseMs);
    }
    checkConfigurationFailure();
    checkEventQueueMarginAndErrors();
}
