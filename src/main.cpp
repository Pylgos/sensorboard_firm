#include <mbed.h>
#include <bitset>
#include "QEI.hpp"
#include "debouncer.hpp"


namespace {



constexpr int CAN_ID_1 = CAN_ID_1_DEF;
constexpr int CAN_ID_2 = CAN_ID_2_DEF;
constexpr chrono::microseconds period = 5ms;
constexpr uint16_t Q_MAX = 65535;

}  // namespace

CAN can{PA_11, PA_12, 1000000};


int can_write(const CANMessage& msg) {
  int ret = can.write(msg);
  if (!ret && (can.rderror() > 5 || can.tderror() > 5)) {
    NVIC_SystemReset();
  }
  return ret;
}


int main() {
  std::array<QEI, 5> encoders = {
    QEI(PA_0, PA_1, QEI::X2_ENCODING),
    QEI(PA_2, PA_3, QEI::X2_ENCODING),
    QEI(PA_4, PA_5, QEI::X2_ENCODING),
    QEI(PA_6, PA_7, QEI::X2_ENCODING),
    QEI(PA_8, PA_9, QEI::X2_ENCODING)};

  std::array<DigitalIn, 5> switches = {
    DigitalIn(PB_1),
    DigitalIn(PB_0),
    DigitalIn(PB_7),
    DigitalIn(PB_6),
    DigitalIn(PB_5)};

  std::array<Debouncer, 5> switches_deb = {
    Debouncer(switches[0], Q_MAX),
    Debouncer(switches[1], Q_MAX),
    Debouncer(switches[2], Q_MAX),
    Debouncer(switches[3], Q_MAX),
    Debouncer(switches[4], Q_MAX)
  };

  CANMessage msg1, msg2;
  msg1.id = CAN_ID_1;
  msg2.id = CAN_ID_2;
  msg1.len = 8;
  msg2.len = 8;

  Timer timer;
  timer.start();
  auto prev_write = timer.elapsed_time();

  while (true) {
    auto now = timer.elapsed_time();
    auto elapsed = now - prev_write;

    std::array<int16_t, 5> now_enc;
    std::array<bool, 5> now_sw;

    for (auto i = 0u; i < 5; i++) {
      now_enc[i] = encoders[i].getPulses();
      switches_deb[i].update();
      now_sw[i] = switches_deb[i].read();
    }

    if (elapsed > period) {
      prev_write = timer.elapsed_time();
      uint64_t time_us = chrono::duration_cast<chrono::microseconds>(now).count();

      msg1.data[0] = (time_us >> (8 * 0)) & 0xff;
      msg1.data[1] = (time_us >> (8 * 1)) & 0xff;
      msg1.data[2] = (time_us >> (8 * 2)) & 0xff;
      msg1.data[3] = (time_us >> (8 * 3)) & 0xff;
      msg1.data[4] = (time_us >> (8 * 4)) & 0xff;
      msg1.data[5] =  now_sw[0] << 0 | now_sw[1] << 1 | now_sw[2] << 2 |
                      now_sw[3] << 3 | now_sw[4] << 4;
      msg1.data[6] = (now_enc[0] >> (8 * 0)) & 0xff;
      msg1.data[7] = (now_enc[0] >> (8 * 1)) & 0xff;

      msg2.data[0] = (now_enc[1] >> (8 * 0)) & 0xff;
      msg2.data[1] = (now_enc[1] >> (8 * 1)) & 0xff;
      msg2.data[2] = (now_enc[2] >> (8 * 0)) & 0xff;
      msg2.data[3] = (now_enc[2] >> (8 * 1)) & 0xff;
      msg2.data[4] = (now_enc[3] >> (8 * 0)) & 0xff;
      msg2.data[5] = (now_enc[3] >> (8 * 1)) & 0xff;
      msg2.data[6] = (now_enc[4] >> (8 * 0)) & 0xff;
      msg2.data[7] = (now_enc[4] >> (8 * 1)) & 0xff;

      if (can_write(msg1)) {
        while (!can_write(msg2));
      }
    }
  }
}