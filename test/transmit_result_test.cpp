#include "TPUart/DataLinkLayer.h"
#include <cassert>
#include <deque>
#include <vector>

static unsigned long now = 0;
unsigned long millis() { return now++; }
unsigned long micros() { return now * 1000; }

class Uart : public TPUart::Interface::Abstract
{
public:
    std::deque<int> rx;
    bool fail = false;
    bool dataNext = false;
    void flush() override { rx.clear(); }
    void begin(int) override {}
    void end() override {}
    bool available() override { return !rx.empty(); }
    bool availableForWrite() override { return true; }
    bool write(char value) override
    {
        if (fail) return false;
        if (dataNext) dataNext = false;
        else if ((static_cast<unsigned char>(value) & 0x80) != 0) dataNext = true;
        else if (value == U_RESET_REQ) rx.push_back(U_RESET_IND);
        return true;
    }
    int read() override
    {
        if (rx.empty()) return -1;
        int value = rx.front(); rx.pop_front(); return value;
    }
};

static TPUart::Frame *frame()
{
    char bytes[8] = { (char)0x90, 0x11, 0x01, 0x22, 0x02, 0, 0, 0 };
    char checksum = 0;
    for (unsigned i = 0; i < 7; ++i) checksum ^= bytes[i];
    bytes[7] = ~checksum;
    return new TPUart::Frame(bytes, (unsigned short)sizeof(bytes));
}

int main()
{
    Uart uart;
    TPUart::DataLinkLayer dll;
    std::vector<bool> results;
    dll.registerTransmitResult([&](TPUart::Frame &f, bool success) {
        assert(f.isValid()); // Driver retains ownership until callback returns.
        results.push_back(success);
    });
    dll.begin(TPUart::BCU_TPUART2, &uart);
    dll.process();
    auto &tx = dll.getTransmitter();

    assert(dll.pushTransmitQueue(frame()));
    dll.process();
    assert(tx.awaitResponse());
    uart.rx.push_back(0x8b); // Positive L_DATA_CON, without RX echo.
    dll.process();
    assert(results == std::vector<bool>{true});
    uart.rx.push_back(0x8b); // Duplicate cannot complete a frame twice.
    dll.process();
    assert(results.size() == 1);

    assert(dll.pushTransmitQueue(frame()));
    dll.process();
    uart.rx.push_back(0x0b);
    dll.process();
    assert(results.size() == 2 && !results.back());

    assert(dll.pushTransmitQueue(frame()));
    dll.process();
    assert(dll.pushTransmitQueue(frame()));
    dll.reset();
    assert(results.size() == 2); // Reset does not invoke application under locks.
    dll.process();
    assert(results.size() == 4 && !results[2] && !results[3]);

    assert(dll.pushTransmitQueue(frame()));
    dll.process();
    now += 60001;
    dll.process();
    assert(results.size() == 5 && !results.back());

    tx.setQueueSize(1);
    assert(dll.pushTransmitQueue(frame()));
    auto *rejected = frame();
    assert(!dll.pushTransmitQueue(rejected));
    delete rejected; // Rejection leaves ownership with caller.
    tx.reset();
    rejected = frame();
    assert(!dll.pushTransmitQueue(rejected)); // Pending results count towards bound.
    delete rejected;
    dll.process();
    assert(results.size() == 6);

    assert(dll.pushTransmitQueue(frame()));
    uart.fail = true;
    dll.process();
    assert(results.size() == 7 && !results.back());
    uart.fail = false;
    dll.end(false);
    assert(results.size() == 7);
}
