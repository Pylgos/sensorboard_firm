#include <mbed.h>


class Debouncer {
public:
  Debouncer(DigitalIn d_in, uint16_t q_max)
    : d_in_{d_in}, q_{0}, q_max_{q_max}, state{false} {

  }

  void update() {
    bool in = d_in_.read();

    if (in) {
      if (q_ == q_max_) {
        state = true;
      } else {
        q_++;
      }
    } else {
      if (q_ == 0) {
        state = false;
      } else {
        q_--;
      }
    }
  }

  bool read() {
    return state;
  }

private:
  DigitalIn& d_in_;
  uint16_t q_, q_max_;
  bool state;
};