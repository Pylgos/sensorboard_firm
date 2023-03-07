#include "QEI.hpp"

QEI::QEI(const QEI &q) : channelA_(q.Pin[0]), channelB_(q.Pin[1]) {
  pulses_ = 0;
  encoding_ = q.encoding_;
  // Workout what the current state is.
  int chanA = channelA_.read();
  int chanB = channelB_.read();

  // 2-bit state.
  prevState_ = (chanA << 1) | (chanB);

  channelA_.rise(callback(this, &QEI::encode));
  channelA_.fall(callback(this, &QEI::encode));

  if (q.encoding_ == X4_ENCODING) {
    channelB_.rise(callback(this, &QEI::encode));
    channelB_.fall(callback(this, &QEI::encode));
  }
}
QEI::QEI(PinName channelA, PinName channelB, Encoding encoding) : channelA_(channelA, PullDown), channelB_(channelB, PullDown) {
  Pin[0] = channelA;
  Pin[1] = channelB;
  pulses_ = 0;
  encoding_ = encoding;

  // Workout what the current state is.
  int chanA = channelA_.read();
  int chanB = channelB_.read();

  // 2-bit state.
  prevState_ = (chanA << 1) | (chanB);

  channelA_.rise(callback(this, &QEI::encode));
  channelA_.fall(callback(this, &QEI::encode));

  if (encoding == X4_ENCODING) {
    channelB_.rise(callback(this, &QEI::encode));
    channelB_.fall(callback(this, &QEI::encode));
  }
}

void QEI::state(int i) {
  if (i == 1) {
    channelA_.disable_irq();
    channelB_.disable_irq();
  } else if (i == 0) {
    channelA_.enable_irq();
    channelB_.enable_irq();
  }
}

void QEI::qei_reset(void) { pulses_ = 0; }

void QEI::set(int pul) { pulses_ = pul; }

int QEI::getPulses(void) { return pulses_; }

void QEI::encode(void) {
  int chanA = channelA_.read();
  int chanB = channelB_.read();

  int currState = (chanA << 1) | (chanB);
  int prevState = prevState_;

  if (encoding_ == X2_ENCODING) {
    if ((prevState == 0x3 && currState == 0x0) || (prevState == 0x0 && currState == 0x3)) {
      pulses_++;

    } else if ((prevState == 0x2 && currState == 0x1) || (prevState == 0x1 && currState == 0x2)) {
      pulses_--;
    }

  } else if (encoding_ == X4_ENCODING) {
    if (((currState ^ prevState) != INVALID) && (currState != prevState)) {
      int change = (prevState & PREV_MASK) ^ ((currState & CURR_MASK) >> 1);
      if (change == 0) {
        change = -1;
      }

      pulses_ -= change;
    }
  }

  prevState_ = currState;
}
