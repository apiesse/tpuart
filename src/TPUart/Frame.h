#pragma once
#include "TPUart/Types.h"
#include <cstddef>
#include <cstdio>
#include <cstdlib>
#include <cstring>
// #ifdef TPUART_PRINT
#include <string>
// #endif

#ifndef TPUART_FRAME_ALLOC
#define TPUART_FRAME_ALLOC(size) malloc(size)
#endif
#ifndef TPUART_FRAME_FREE
#define TPUART_FRAME_FREE(ptr) free(ptr)
#endif

// DLL services (device is transparent)
#define L_DATA_STANDARD_IND 0x90
#define L_DATA_EXTENDED_IND 0x10
#define L_DATA_MASK 0xD3

// Means that the frame is transmitted
#define TP_FRAME_FLAG_TX 0b10000000

// Means that the frame is answered with L_DATA_CON if success the TP_FRAME_FLAG_ACK is also set
#define TP_FRAME_FLAG_DATA_CON 0b01000000

// Means that the frame should be filtered by the device
#define TP_FRAME_FLAG_FILTERED 0b00100000

// // Means that the frame comes from the device itself
// #define TP_FRAME_FLAG_ECHO 0b00010000

// Means that the frame is processed by this device
#define TP_FRAME_FLAG_ADDRESSED 0b00001000

// Means that the frame has been acked with BUSY
#define TP_FRAME_FLAG_ACK_BUSY 0b00000100

// Means that the frame has been acked with NACK
#define TP_FRAME_FLAG_ACK_NACK 0b00000010

// Means that the frame has been acked
#define TP_FRAME_FLAG_ACK 0b00000001

namespace TPUart
{
    class Frame
    {
      private:
        const char *_data = nullptr;
        // Zero means that the caller owns the storage and did not provide a bound.
        // Receive-side buffers use the array constructor or the copying constructor,
        // so all lengths derived from the wire are checked against a real capacity.
        size_t _capacity = 0;
        char _flags = 0;
        bool _deleteData = false;

        bool hasBytes(size_t count) const
        {
            return _data != nullptr && (_capacity == 0 || count <= _capacity);
        }

      public:
        /**
         * Legacy unbounded borrowed view. Use only for trusted, locally built
         * storage; wire input must use Frame(data, capacity, deleteData).
         * Kept for source compatibility with existing integrations.
         */
        Frame(const char *data) : _data(data) {}
        Frame(const char *data, size_t capacity, bool deleteData)
            : _data(data), _capacity(capacity), _deleteData(deleteData) {}
        Frame(const char *data, unsigned short size)
        {
            if (data == nullptr || size == 0)
                return;

            char* copy = (char*)TPUART_FRAME_ALLOC(size);
            if (copy == nullptr)
                return;

            memcpy(copy, data, size);
            _data = copy;
            _capacity = size;
            _deleteData = true;
        }
        /**
         * Legacy unbounded ownership overload. Use only when the allocation is
         * trusted and its complete frame size is known to be present. New code
         * should use Frame(data, capacity, deleteData).
         */
        Frame(const char *data, bool deleteData) : _data(data), _deleteData(deleteData) {}

        Frame(const Frame& other) : _capacity(other._capacity), _flags(other._flags)
        {
            if (!other._deleteData || other._data == nullptr)
            {
                _data = other._data;
                return;
            }

            const size_t copySize = other._capacity != 0 ? other._capacity : other.size();
            char* copy = copySize != 0 ? (char*)TPUART_FRAME_ALLOC(copySize) : nullptr;
            if (copy != nullptr)
            {
                memcpy(copy, other._data, copySize);
                _data = copy;
                _capacity = copySize;
                _deleteData = true;
            }
        }

        Frame& operator=(const Frame& other)
        {
            if (this == &other)
                return *this;
            if (_deleteData)
                TPUART_FRAME_FREE((char*)_data);

            _data = nullptr;
            _capacity = other._capacity;
            _flags = other._flags;
            _deleteData = false;
            if (!other._deleteData || other._data == nullptr)
            {
                _data = other._data;
                return *this;
            }

            const size_t copySize = other._capacity != 0 ? other._capacity : other.size();
            char* copy = copySize != 0 ? (char*)TPUART_FRAME_ALLOC(copySize) : nullptr;
            if (copy != nullptr)
            {
                memcpy(copy, other._data, copySize);
                _data = copy;
                _capacity = copySize;
                _deleteData = true;
            }
            return *this;
        }

        ~Frame()
        {
            if (_deleteData) TPUART_FRAME_FREE((char *)_data);
        }

        // Frame(const Frame &other)
        // {
        //     const char *newData = (const char *)malloc(other.size());
        //     // memcpy((char *)newData, (const char  *)*other._data, other.size());
        // }

        void addFlags(char flags)
        {
            _flags |= flags;
        }

        bool checkCRC16CCITT()
        {
            unsigned short crc = 0xFFFF;
            unsigned short polynomial = 0x1021;

            const unsigned short s = size();
            if (s == 0 || !hasBytes((size_t)s + 2))
                return false;
            for (unsigned short i = 0; i < s; i++)
            {
                crc ^= ((unsigned char)_data[i] << 8);
                for (int j = 0; j < 8; j++)
                    if (crc & 0x8000)
                        crc = (crc << 1) ^ polynomial;
                    else
                        crc <<= 1;
            }
            return (((unsigned char)_data[s] << 8) + (unsigned char)_data[s + 1]) == crc;
        }

        bool checkCRC16SPI()
        {
            // CRC-16/SPI-FUJITSU
            unsigned short crc = 0x1D0F;
            unsigned short polynomial = 0x1021;

            const unsigned short s = size();
            if (s == 0 || !hasBytes((size_t)s + 2))
                return false;
            for (unsigned short i = 0; i < s; i++)
            {
                crc ^= ((unsigned char)_data[i] << 8);
                for (int j = 0; j < 8; j++)
                    if (crc & 0x8000)
                        crc = (crc << 1) ^ polynomial;
                    else
                        crc <<= 1;
            }
            return (((unsigned char)_data[s] << 8) + (unsigned char)_data[s + 1]) == crc;
        }

        bool hasData() const
        {
            return _data != nullptr;
        }

        const char *data()
        {
            return _data;
        }

        char data(unsigned short pos)
        {
            return hasBytes((size_t)pos + 1) ? _data[pos] : 0;
        }

        unsigned short destination()
        {
            if (!hasBytes(isExtended() ? 6 : 5))
                return 0;
            return isExtended() ? ((unsigned char)_data[4] << 8) + (unsigned char)_data[5]
                                : ((unsigned char)_data[3] << 8) + (unsigned char)_data[4];
        }

        unsigned short flags()
        {
            return _flags;
        }

        /*
         * Include the all metadata at the beginning of the frame and the checksum at the end.
         */
        unsigned char metadataSize()
        {
            return isExtended() ? 9 : 8;
        }

        bool isExtended() const
        {
            return hasBytes(1) && (_data[0] & 0xD3) == 0x10;
        }

        bool isFrame()
        {
            return hasBytes(1) && ((_data[0] & L_DATA_MASK) == L_DATA_STANDARD_IND || (_data[0] & L_DATA_MASK) == L_DATA_EXTENDED_IND);
        }

        bool isGroupAddress()
        {
            if (!hasBytes(isExtended() ? 2 : 6))
                return false;
            return isExtended() ? ((unsigned char)_data[1] >> 7) & 0b1
                                : ((unsigned char)_data[5] >> 7) & 0b1;
        }

        bool isRepeated()
        {
            return hasBytes(1) && !(_data[0] & 0b100000);
        }

        bool isFiltered()
        {
            return _flags & TP_FRAME_FLAG_FILTERED;
        }

        void setFiltered()
        {
            _flags |= TP_FRAME_FLAG_FILTERED;
        }

        /*
         * Returns the total size of the frame, including the metadata and apdu.
         */
        unsigned short size() const
        {
            if (!hasBytes(1))
                return 0;

            const bool extended = isExtended();
            if (!hasBytes(extended ? 7 : 6))
                return 0;

            const unsigned short result = (extended ? 9U : 8U)
                + (extended ? (unsigned char)_data[6] : ((unsigned char)_data[5] & 0x0F));
            return hasBytes(result) ? result : 0;
        }

        unsigned short source()
        {
            if (!hasBytes(isExtended() ? 4 : 3))
                return 0;
            return isExtended() ? ((unsigned char)_data[2] << 8) + (unsigned char)_data[3]
                                : ((unsigned char)_data[1] << 8) + (unsigned char)_data[2];
        }

        unsigned char apduSize()
        {
            if (!hasBytes(isExtended() ? 7 : 6))
                return 0;
            return isExtended() ? (unsigned char)_data[6] : (unsigned char)(_data[5] & 0x0F);
        }

        void resetFlags()
        {
            _flags = 0;
        }

        bool isValid(bool extended = false)
        {
            (void)extended;
            const unsigned short frameSize = size();
            if (frameSize == 0)
                return false;
            const unsigned short s = frameSize - 1;
            return _data[s] == calcCRC8();
        }

        bool isAddressed()
        {
            return _flags & TP_FRAME_FLAG_ADDRESSED;
        }

        // CRC-8/GSM-A Poly
        char calcCRC8()
        {
            char chksum = 0;
            const unsigned short frameSize = size();
            if (frameSize == 0)
                return 0;
            const unsigned short s = frameSize - 1;
            for (unsigned short i = 0; i < s; i++)
                chksum ^= _data[i];
            return (char)~chksum;
        }

        unsigned short awaitDestination()
        {
            // return isExtended() ? 6 : 5;
            return 6; // Extended ist 6 und Standard ist 5, aber das isGroupFlag kommt 1 Byte später
        }

        unsigned short awaitSize()
        {
            return isExtended() ? 7 : 6;
        }

        void setAcknowledge(AcknowledgeType acknowledge)
        {
            _flags &= ~(TP_FRAME_FLAG_ADDRESSED | TP_FRAME_FLAG_ACK | TP_FRAME_FLAG_ACK_BUSY | TP_FRAME_FLAG_ACK_NACK);

            if (acknowledge == ACK_Addressed) _flags |= TP_FRAME_FLAG_ADDRESSED | TP_FRAME_FLAG_ACK;
            if (acknowledge == ACK_Busy) _flags |= TP_FRAME_FLAG_ADDRESSED | TP_FRAME_FLAG_ACK | TP_FRAME_FLAG_ACK_BUSY;
            if (acknowledge == ACK_Nack) _flags |= TP_FRAME_FLAG_ADDRESSED | TP_FRAME_FLAG_ACK | TP_FRAME_FLAG_ACK_NACK;
        }

        void setAcknowledge(bool busy = false, bool nack = false)
        {
            _flags |= TP_FRAME_FLAG_ACK;
            if (busy) _flags |= TP_FRAME_FLAG_ACK_BUSY;
            if (nack) _flags |= TP_FRAME_FLAG_ACK_NACK;
        }

        bool isInvalid()
        {
            return !isValid();
        }

        bool isAck()
        {
            return _flags & TP_FRAME_FLAG_ACK;
        }

        bool isNack()
        {
            return _flags & TP_FRAME_FLAG_ACK_NACK;
        }

        bool isBusy()
        {
            return _flags & TP_FRAME_FLAG_ACK_BUSY;
        }

        bool isTransmitted()
        {
            return _flags & TP_FRAME_FLAG_TX;
        }

        void setTransmitted()
        {
            _flags |= TP_FRAME_FLAG_TX;
        }

        /*
         * Calculates the size of a CemiFrame. A CemiFrame has 2 additional bytes at the beginning.
         * An additional byte is added to a standard frame, as this still has to be converted into an extendend.
         */
        unsigned short cemiSize()
        {
            return size() + (isExtended() ? 2 : 3) - 1; // -1 without CRC
        }

        /**
         * Creates a buffer and converts the TPFrame into a CemiFrame.
         * Important: After processing (i.e. also after using the CemiFrame), the reference must be released with
         * freeCemiData().  This keeps allocation and deallocation paired when an integrator overrides the allocator.
         */
        char *cemiData()
        {
            const unsigned short outputSize = cemiSize();
            if (size() == 0 || outputSize == 0)
                return nullptr;

            char *cemiBuffer = (char *)TPUART_FRAME_ALLOC(outputSize);
            if (cemiBuffer == nullptr)
                return nullptr;

            // Das CEMI erwartet die Daten im Extended format inkl. zwei zusätzlicher Bytes am Anfang.
            cemiBuffer[0] = 0x29;
            cemiBuffer[1] = 0x0;
            cemiBuffer[2] = _data[0];
            if (isExtended())
            {
                memcpy(cemiBuffer + 2, _data, size() - 1); // -1 without CRC
            }
            else
            {
                cemiBuffer[3] = _data[5] & 0xF0;
                memcpy(cemiBuffer + 4, _data + 1, 4);
                cemiBuffer[8] = _data[5] & 0x0F;
                memcpy(cemiBuffer + 9, _data + 6, cemiBuffer[8] + 2 - 1); // -1 without CRC
            }

            return cemiBuffer;
        }

        static void freeCemiData(char* data)
        {
            if (data != nullptr)
                TPUART_FRAME_FREE(data);
        }

        // #ifdef TPUART_PRINT
        std::string humanSource()
        {
            unsigned short value = source();
            char buffer[10];
            sprintf(buffer, "%02i.%02i.%03i", (value >> 12 & 0b1111), (value >> 8 & 0b1111), (value & 0b11111111));
            return buffer;
        }

        std::string humanDestination()
        {
            unsigned short value = destination();
            char buffer[10];
            if (isGroupAddress())
                sprintf(buffer, "%02i/%02i/%03i", (value >> 11 & 0b11111), (value >> 8 & 0b111), (value & 0b11111111));
            else
                sprintf(buffer, "%02i.%02i.%03i", (value >> 12 & 0b1111), (value >> 8 & 0b1111), (value & 0b11111111));

            return buffer;
        }

        std::string printFrame()
        {
            const unsigned short resultSize = (38 + 6 + (size() * 3));
            std::string result;
            result.reserve(resultSize);
            result.append(humanSource());
            result.append(" -> ");
            result.append(humanDestination());
            result.append(" [");
            result.push_back(isTransmitted() ? 'T' : '_'); // Was transmitted by me
            result.push_back(isAddressed() ? 'D' : '_');   // Was transmitted by me
            result.push_back(isInvalid() ? 'I' : '_');     // Invalid
            result.push_back(isExtended() ? 'E' : '_');    // Extended
            result.push_back(isRepeated() ? 'R' : '_');    // Repeat
            result.push_back(isFiltered() ? 'F' : '_');    // All ready received
            result.push_back(isBusy() ? 'B' : '_');        // ACK + BUSY
            result.push_back(isNack() ? 'N' : '_');        // ACK + NACK
            result.push_back(isAck() ? 'A' : '_');         // ACK
            result.append("] ( ");

            char hexStr[3];
            for (size_t i = 0; i < size(); i++)
            {
                if (i) result.push_back(' ');
                std::sprintf(hexStr, "%02X", (unsigned int)(unsigned char)_data[i]);
                result.append(hexStr);
            }
            result.append(" ) [");
            result.append(std::to_string(size()));
            result.append("]");
            return result;
        }
        // #endif
    };
}; // namespace TPUart
