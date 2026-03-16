#pragma once

#include "types.hpp"

namespace nes {

// manages APU and I/O registers ($4000-$4017)
// owns APU register storage (sound channels are stubs)
// owns controller shift registers for $4016/$4017
class Apu {
 public:
  // APU/IO registers
  u8 cpu_read(u16 addr);
  void cpu_write(u16 addr, u8 value);

  // update button state for controller (0 or 1)
  void set_button_state(int controller, u8 buttons);

  // button bit masks (active high)
  static constexpr u8 btn_a      = 0x01;
  static constexpr u8 btn_b      = 0x02;
  static constexpr u8 btn_select = 0x04;
  static constexpr u8 btn_start  = 0x08;
  static constexpr u8 btn_up     = 0x10;
  static constexpr u8 btn_down   = 0x20;
  static constexpr u8 btn_left   = 0x40;
  static constexpr u8 btn_right  = 0x80;

  // APU registers
  // Addresses    Channel  Units
  // $4000–$4003  Pulse 1  Timer, length counter, envelope, sweep
  // $4004–$4007  Pulse 2  Timer, length counter, envelope, sweep
  // $4008–$400B  Triangle Timer, length counter, linear counter
  // $400C–$400F  Noise    Timer, length counter, envelope, linear feedback shift register
  // $4010–$4013  DMC      Timer, memory reader, sample buffer, output unit
  // $4015        All      Channel enable and length counter status
  // $4017        All      Frame counter
 private:
  u8 regs_[0x18]{};        // $4000-$4017 register storage (sound stubs)
  u8 joy_shift_[2]{};      // controller shift registers
  bool joy_strobe_{false}; // strobe latch ($4016 bit 0)
  u8 joy_state_[2]{};      // current button state per controller

  // $4016 Write
  // 7  bit  0
  // ---- ----
  // xxxx xEES
  //       |||
  //       ||+- Controller port latch bit
  //       ++-- Expansion port latch bits

  // $4016 / $4017 Read
  // 7  bit  0
  // ---- ----
  // xxxD DDDD
  // |||+-++++- Input data lines D4 D3 D2 D1 D0
  // +++------- Open bus
};

}  // namespace nes
