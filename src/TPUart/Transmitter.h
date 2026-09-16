#pragma once
#include "TPUart/Frame.h"
#include "TPUart/Types.h"
#include <queue>

namespace TPUart
{
    class DataLinkLayer;

    class Transmitter
    {
        DataLinkLayer &_dll;
        volatile char _cachedAcknowledge;
        size_t _transmitPos;
        unsigned char _transmitOffset; // NCN5130 keeps the data-index offset until it is changed (DS p.42)
        volatile unsigned long _time;
        unsigned long _maxQueueSize;
        volatile TxState _state; // written by finalize() (RX path), read by the main-loop TX path -> matches Receiver::_state
        Frame *_frame = nullptr;
        Frame *_resultHead = nullptr;
        Frame *_resultTail = nullptr;
        size_t _outstanding = 0;
        void completeLocked(Frame *frame, bool success);

      public:
        std::queue<Frame *> _queue;
        Transmitter(DataLinkLayer &dll);
        ~Transmitter();

        bool transmit(const char *data, size_t size);
        void finalize(bool success = true);
        // Run only from the ordinary processing task, never from UART callbacks.
        void processResults();
        void processWatchdog();

        bool pushQueue(Frame *frame);
        size_t queueSize();
        void reset();
        void sendAcknowledge(AcknowledgeType acknowledge = ACK_None);
        void setQueueSize(unsigned long size);

        void processTransmitByte();
        Frame *currentFrame();
        bool isTransmitting();
        bool awaitResponse();
        void sendCachedAcknowledge();
        void processQueue();
        void resetWatchdogTimer();
    };

} // namespace TPUart
