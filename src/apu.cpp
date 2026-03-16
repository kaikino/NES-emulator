#include "apu.hpp"

namespace nes {

u8 Apu::cpu_read(u16 addr) {
  switch (addr) {
    case 0x4015:  // APU status (stub)
      return 0;
    case 0x4016: {  // controller 1
      if (joy_strobe_)
        joy_shift_[0] = joy_state_[0];  // continuously reloaded while strobe is high
      u8 bit = joy_shift_[0] & 0x01;
      joy_shift_[0] >>= 1;
      return bit;
    }
    case 0x4017: {  // controller 2
      if (joy_strobe_)
        joy_shift_[1] = joy_state_[1];
      u8 bit = joy_shift_[1] & 0x01;
      joy_shift_[1] >>= 1;
      return bit;
    }
    default:
      return 0;
  }
}

void Apu::cpu_write(u16 addr, u8 value) {
  if (addr >= 0x4000 && addr <= 0x4017)
    regs_[addr - 0x4000] = value;

  switch (addr) {
    case 0x4016:  // controller strobe
      if (joy_strobe_ && !(value & 0x01)) {
        // strobe 1->0: latch current button state into shift registers
        joy_shift_[0] = joy_state_[0];
        joy_shift_[1] = joy_state_[1];
      }
      joy_strobe_ = (value & 0x01) != 0;
      if (joy_strobe_) {
        // while strobe is high, shift registers continuously reload
        joy_shift_[0] = joy_state_[0];
        joy_shift_[1] = joy_state_[1];
      }
      break;
    default:
      break;
  }
}

void Apu::set_button_state(int controller, u8 buttons) {
  if (controller >= 0 && controller < 2)
    joy_state_[controller] = buttons;
}

}  // namespace nes
