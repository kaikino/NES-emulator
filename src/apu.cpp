#include "apu.hpp"

namespace nes {

// pulse mixer lookup table
// output = 95.88 / (8128.0 / (p1 + p2) + 100.0), index = p1 + p2 (0..30)
const float* Apu::pulse_mix_table() {
  static float table[pulse_mix_size] = {};
  static bool init = false;
  if (!init) {
    table[0] = 0.0f;
    for (int i = 1; i < pulse_mix_size; ++i)
      table[i] = static_cast<float>(95.88 / (8128.0 / i + 100.0));
    init = true;
  }
  return table;
}

// public interface

void Apu::set_sample_rate(int rate) {
  sample_period_ = cpu_clock_hz / rate;
}

void Apu::run_cycles(int cpu_cycles) {
  for (int i = 0; i < cpu_cycles; ++i)
    tick();
}

int Apu::drain_samples(float* buf, int max) {
  int n = static_cast<int>(sample_buf_.size());
  if (n > max) n = max;
  for (int i = 0; i < n; ++i)
    buf[i] = sample_buf_[i];
  sample_buf_.erase(sample_buf_.begin(), sample_buf_.begin() + n);
  return n;
}

// CPU register read

u8 Apu::cpu_read(u16 addr) {
  switch (addr) {
    case 0x4015: {
      // $4015 read: length counter status
      u8 status = 0;
      if (pulse_[0].length > 0) status |= 0x01;
      if (pulse_[1].length > 0) status |= 0x02;
      return status;
    }
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

// CPU register write

void Apu::cpu_write(u16 addr, u8 value) {
  switch (addr) {
    // pulse 1 ($4000-$4003) / pulse 2 ($4004-$4007)
    case 0x4000: case 0x4004: {
      // DDLC VVVV — duty, envelope loop / length halt, constant volume, volume/period
      int ch = (addr >= 0x4004) ? 1 : 0;
      auto& p = pulse_[ch];
      p.duty_mode    = (value >> 6) & 0x03;
      p.env_loop     = (value & 0x20) != 0;
      p.constant_vol = (value & 0x10) != 0;
      p.env_period   = value & 0x0F;
      break;
    }
    case 0x4001: case 0x4005: {
      // EPPP NSSS — sweep enable, period, negate, shift
      int ch = (addr >= 0x4004) ? 1 : 0;
      auto& p = pulse_[ch];
      p.sweep_enable = (value & 0x80) != 0;
      p.sweep_period = (value >> 4) & 0x07;
      p.sweep_negate = (value & 0x08) != 0;
      p.sweep_shift  = value & 0x07;
      p.sweep_reload = true;
      break;
    }
    case 0x4002: case 0x4006: {
      // TTTT TTTT — timer low 8 bits
      int ch = (addr >= 0x4004) ? 1 : 0;
      auto& p = pulse_[ch];
      p.timer_period = (p.timer_period & 0x0700) | value;
      break;
    }
    case 0x4003: case 0x4007: {
      // LLLL LTTT — length counter load, timer high 3 bits
      int ch = (addr >= 0x4004) ? 1 : 0;
      auto& p = pulse_[ch];
      p.timer_period = (p.timer_period & 0x00FF) | (static_cast<u16>(value & 0x07) << 8);
      if (p.enabled)
        p.length = length_table[(value >> 3) & 0x1F];
      p.duty_pos = 0;       // restart sequencer
      p.env_start = true;   // restart envelope
      break;
    }

    // $4015: channel enable
    case 0x4015:
      pulse_[0].enabled = (value & 0x01) != 0;
      pulse_[1].enabled = (value & 0x02) != 0;
      if (!pulse_[0].enabled) pulse_[0].length = 0;
      if (!pulse_[1].enabled) pulse_[1].length = 0;
      break;

    // $4016: controller strobe
    case 0x4016:
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

    // $4017: frame counter
    case 0x4017:
      frame_mode_5step_  = (value & 0x80) != 0;
      frame_irq_inhibit_ = (value & 0x40) != 0;
      frame_counter_ = 0;
      // 5-step mode immediately clocks all units on write
      if (frame_mode_5step_) {
        quarter_frame();
        half_frame();
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

// internal clocking

void Apu::tick() {
  // frame counter fires at specific CPU cycle counts within the sequence
  // 4-step: quarter at 3729, 7457, 11186, 14915; half at 7457, 14915
  // 5-step: quarter at 3729, 7457, 11186, 18641; half at 7457, 18641
  bool do_quarter = false;
  bool do_half = false;

  if (!frame_mode_5step_) {
    // 4-step mode
    switch (frame_counter_) {
      case 3728:  do_quarter = true; break;
      case 7456:  do_quarter = true; do_half = true; break;
      case 11185: do_quarter = true; break;
      case 14914: do_quarter = true; do_half = true; break;
      default: break;
    }
    if (frame_counter_ >= 14915)
      frame_counter_ = 0;
    else
      ++frame_counter_;
  } else {
    // 5-step mode
    switch (frame_counter_) {
      case 3728:  do_quarter = true; break;
      case 7456:  do_quarter = true; do_half = true; break;
      case 11185: do_quarter = true; break;
      case 18640: do_quarter = true; do_half = true; break;
      default: break;
    }
    if (frame_counter_ >= 18641)
      frame_counter_ = 0;
    else
      ++frame_counter_;
  }

  if (do_quarter) quarter_frame();
  if (do_half)    half_frame();

  // pulse timers are clocked every 2 CPU cycles (APU clock = CPU / 2)
  // use bit 0 of the frame counter for the divider
  if ((frame_counter_ & 1) == 0) {
    for (int ch = 0; ch < 2; ++ch) {
      auto& p = pulse_[ch];
      if (p.timer == 0) {
        p.timer = p.timer_period;
        p.duty_pos = (p.duty_pos + 1) & 0x07;
      } else {
        --p.timer;
      }
    }
  }

  // downsample: emit one audio sample every sample_period_ CPU cycles
  sample_counter_ += 1.0;
  if (sample_counter_ >= sample_period_) {
    sample_counter_ -= sample_period_;
    sample_buf_.push_back(mix());
  }
}

// frame counter sub-clocks

void Apu::quarter_frame() {
  clock_envelope(pulse_[0]);
  clock_envelope(pulse_[1]);
}

void Apu::half_frame() {
  clock_length(pulse_[0]);
  clock_length(pulse_[1]);
  clock_sweep(pulse_[0], 0);
  clock_sweep(pulse_[1], 1);
}

void Apu::clock_envelope(PulseChannel& p) {
  if (p.env_start) {
    p.env_start = false;
    p.env_decay = 15;
    p.env_divider = p.env_period;
  } else {
    if (p.env_divider == 0) {
      p.env_divider = p.env_period;
      if (p.env_decay > 0)
        --p.env_decay;
      else if (p.env_loop)
        p.env_decay = 15;  // loop: restart decay
    } else {
      --p.env_divider;
    }
  }
}

void Apu::clock_length(PulseChannel& p) {
  if (p.length > 0 && !p.env_loop)
    --p.length;
}

void Apu::clock_sweep(PulseChannel& p, int channel) {
  // compute target period for muting check
  int delta = p.timer_period >> p.sweep_shift;
  if (p.sweep_negate) {
    // pulse 1: ones' complement (subtract + extra -1)
    // pulse 2: twos' complement (just subtract)
    delta = -delta;
    if (channel == 0) --delta;
  }
  int target = static_cast<int>(p.timer_period) + delta;

  bool muting = (p.timer_period < 8) || (target > 0x7FF);

  if (p.sweep_divider == 0 && p.sweep_enable && !muting && p.sweep_shift > 0) {
    p.timer_period = static_cast<u16>(target);
  }

  if (p.sweep_divider == 0 || p.sweep_reload) {
    p.sweep_divider = p.sweep_period;
    p.sweep_reload = false;
  } else {
    --p.sweep_divider;
  }
}

// output

u8 Apu::pulse_output(const PulseChannel& p) const {
  if (!p.enabled)         return 0;
  if (p.length == 0)      return 0;
  if (p.timer_period < 8) return 0;  // high-frequency muting

  // sweep target muting
  int delta = p.timer_period >> p.sweep_shift;
  if (!p.sweep_negate) {
    if (static_cast<int>(p.timer_period) + delta > 0x7FF)
      return 0;
  }

  if (duty_table[p.duty_mode][p.duty_pos] == 0)
    return 0;

  return p.volume();
}

float Apu::mix() const {
  int p1 = pulse_output(pulse_[0]);
  int p2 = pulse_output(pulse_[1]);
  return pulse_mix_table()[p1 + p2];
}

}  // namespace nes
