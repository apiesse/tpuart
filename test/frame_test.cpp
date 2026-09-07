#include <cstddef>
#include <cstdlib>

static bool allocationShouldFail = false;
static std::size_t freeCallCount = 0;

static void* testAllocation(std::size_t size)
{
    return allocationShouldFail ? nullptr : std::malloc(size);
}

static void testFree(void* ptr)
{
    ++freeCallCount;
    std::free(ptr);
}

#define TPUART_FRAME_ALLOC(size) testAllocation(size)
#define TPUART_FRAME_FREE(ptr) testFree(ptr)
#include "TPUart/Frame.h"

#include <cassert>
#include <string>

static char crc8(const char* data, std::size_t length)
{
    char checksum = 0;
    for (std::size_t i = 0; i < length; ++i)
        checksum ^= data[i];
    return (char)~checksum;
}

int main()
{
    char standard[8] = { (char)L_DATA_STANDARD_IND, 0x11, 0x01, 0x22, 0x02, 0x00, 0x00, 0x00 };
    standard[7] = crc8(standard, 7);
    TPUart::Frame borrowed(standard, sizeof(standard), false);
    assert(borrowed.size() == sizeof(standard));
    assert(borrowed.isValid());
    const std::string printed = borrowed.printFrame();
    assert(printed.find("( 90 11 01") != std::string::npos);
    assert(printed.find("FFFFFF") == std::string::npos);
    assert(printed.size() < 128);

    char truncatedExtended[8] = { (char)L_DATA_EXTENDED_IND, 0, 0, 0, 0, 0, (char)0xFF, 0 };
    TPUart::Frame bounded(truncatedExtended, sizeof(truncatedExtended), false);
    assert(bounded.size() == 0);
    assert(!bounded.isValid());
    assert(bounded.data(1000) == 0);

    TPUart::Frame owned(standard, (unsigned short)sizeof(standard));
    assert(owned.hasData());
    TPUart::Frame copied(owned);
    TPUart::Frame assigned(standard, sizeof(standard), false);
    assigned = owned;
    standard[1] = 0x33;
    assert(owned.data(1) == 0x11);
    assert(copied.data(1) == 0x11);
    assert(assigned.data(1) == 0x11);

    const std::size_t freesBeforeCemi = freeCallCount;
    char* cemi = borrowed.cemiData();
    assert(cemi != nullptr);
    TPUart::Frame::freeCemiData(cemi);
    assert(freeCallCount == freesBeforeCemi + 1);

    allocationShouldFail = true;
    TPUart::Frame allocationFailure(standard, (unsigned short)sizeof(standard));
    assert(!allocationFailure.hasData());
    assert(allocationFailure.size() == 0);
    assert(!allocationFailure.isValid());
    assert(borrowed.cemiData() == nullptr);
    return 0;
}
