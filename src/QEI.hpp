#pragma once

#include "mbed.h"

#define PREV_MASK 0x1 //Mask for the previous state in determining direction
//of rotation.
#define CURR_MASK 0x2 //Mask for the current state in determining direction
//of rotation.
#define INVALID 0x3 //XORing two states where both bits have changed.

class QEI
{
protected:
    PinName Pin[3];

public:
    QEI(const QEI &q);

    typedef enum Encoding
    {

        X2_ENCODING,
        X4_ENCODING

    } Encoding;

    QEI(PinName channelA, PinName channelB, Encoding encoding = X2_ENCODING);

    void qei_reset(void);

    void set(int pul);

    int getPulses(void);

    void state(int i);

private:
    void encode(void);

    void index(void);

    Encoding encoding_;

    InterruptIn channelA_;
    InterruptIn channelB_;

    volatile int prevState_;
    volatile int pulses_;
};