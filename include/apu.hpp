#pragma once

#include "types.hpp"
#include <vector>

namespace nes {

// manages APU and I/O registers ($4000-$4017)
// owns pulse channel state, frame counter, and audio sample buffer
// owns controller shift registers for $4016/$4017
class Apu {
 public:
  // APU/IO registers
  u8 cpu_read(u16 addr);
  void cpu_write(u16 addr, u8 value);

  // advance APU by n CPU cycles (called from main loop)
  void run_cycles(int cpu_cycles);

  // configure output sample rate (call once after opening SDL audio device)
  void set_sample_rate(int rate);

  // copy up to max samples into buf, return number written
  int drain_samples(float* buf, int max);

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
  // $4000-$4003  Pulse 1  Timer, length counter, envelope, sweep
  // $4004-$4007  Pulse 2  Timer, length counter, envelope, sweep
  // $4008-$400B  Triangle Timer, length counter, linear counter
  // $400C-$400F  Noise    Timer, length counter, envelope, linear feedback shift register
  // $4010-$4013  DMC      Timer, memory reader, sample buffer, output unit
  // $4015        All      Channel enable and length counter status
  // $4017        All      Frame counter

 private:
  // pulse channel state
  struct PulseChannel {
    // timer (11-bit period, clocked every 2 CPU cycles)
    u16 timer_period{0};   // reload value (11 bits)
    u16 timer{0};          // current countdown

    // duty cycle sequencer (8-step, 4 waveforms)
    u8 duty_mode{0};       // 0-3: 12.5%, 25%, 50%, 75%
    u8 duty_pos{0};        // 0-7 position in waveform

    // envelope (quarter-frame clock)
    bool env_start{false}; // set on $4003/$4007 write to restart envelope
    u8 env_divider{0};     // divider countdown
    u8 env_decay{0};       // decay level (0-15), output when constant_volume is off
    u8 env_period{0};      // V bits (also constant volume value)
    bool env_loop{false};  // L bit (also length counter halt)
    bool constant_vol{false}; // C bit

    // length counter (half-frame clock)
    u8 length{0};

    // sweep unit (half-frame clock)
    bool sweep_enable{false};
    u8 sweep_period{0};    // P bits
    bool sweep_negate{false};
    u8 sweep_shift{0};     // S bits
    bool sweep_reload{false};
    u8 sweep_divider{0};

    bool enabled{false};   // $4015 channel enable

    // volume output (0-15)
    u8 volume() const { return constant_vol ? env_period : env_decay; }
  };

  // triangle channel state ($4008-$400B)
  struct TriangleChannel {
    u16 timer_period{0};
    u16 timer{0};
    u8 sequencer_pos{0};     // 0-31, 32-step waveform
    u8 length{0};
    u8 linear_counter{0};
    u8 linear_reload{0};     // reload value from $4008
    bool linear_reload_flag{false};
    bool linear_control{false};  // halt length when set
    bool enabled{false};
  };

  // noise channel state ($400C-$400F)
  struct NoiseChannel {
    u16 timer_period{0};     // from period table
    u16 timer{0};
    u16 lfsr{1};             // 15-bit, never 0
    bool mode{false};        // feedback bit: bit1 (mode 0) or bit6 (mode 1)
    u8 length{0};
    bool env_start{false};
    u8 env_divider{0};
    u8 env_decay{0};
    u8 env_period{0};
    bool env_loop{false};
    bool constant_vol{false};
    bool enabled{false};
    u8 volume() const { return constant_vol ? env_period : env_decay; }
  };

  void tick();
  void quarter_frame();
  void half_frame();
  void clock_envelope(PulseChannel& p);
  void clock_envelope(NoiseChannel& n);
  void clock_length(PulseChannel& p);
  void clock_length(TriangleChannel& t);
  void clock_length(NoiseChannel& n);
  void clock_linear_counter();
  void clock_sweep(PulseChannel& p, int channel);
  u8 pulse_output(const PulseChannel& p) const;
  u8 triangle_output() const;
  u8 noise_output() const;
  float mix() const;

  PulseChannel pulse_[2];
  TriangleChannel triangle_;
  NoiseChannel noise_;

  // frame counter
  // $4017 controls mode (bit 7) and IRQ inhibit (bit 6)
  // 4-step mode: quarter at 3729, 7457, 11186, 14915; half at 7457, 14915
  // 5-step mode: quarter at 3729, 7457, 11186, 18641; half at 7457, 18641
  int frame_counter_{0};       // CPU cycle counter within frame sequence
  bool frame_mode_5step_{false};
  bool frame_irq_inhibit_{true};

  // audio output
  static constexpr double cpu_clock_hz = 1789773.0;
  double sample_period_{cpu_clock_hz / 44100.0}; // CPU cycles per output sample
  double sample_counter_{0.0};                   // fractional accumulator
  std::vector<float> sample_buf_;                // buffered output samples

  // duty cycle waveforms
  // [duty_mode][duty_pos] -> 0 or 1
  static constexpr u8 duty_table[4][8] = {
    {0, 0, 0, 0, 0, 0, 0, 1},  // 12.5%
    {0, 0, 0, 0, 0, 0, 1, 1},  // 25%
    {0, 0, 0, 0, 1, 1, 1, 1},  // 50%
    {1, 1, 1, 1, 1, 1, 0, 0},  // 75% (inverted 25%)
  };

  // length counter lookup (indexed by upper 5 bits of $4003/$4007 write)
  static constexpr u8 length_table[0x20] = {
    10, 254, 20,  2, 40,  4, 80,  6, 160,  8, 60, 10, 14, 12, 26, 14,
    12,  16, 24, 18, 48, 20, 96, 22, 192, 24, 72, 26, 16, 28, 32, 30,
  };

  // pulse mixer lookup (precomputed, indexed by pulse1+pulse2 = 0..30)
  static constexpr int pulse_mix_size = 31;
  static const float* pulse_mix_table();

  // noise period table NTSC (index 0-15 from $400E)
  static constexpr u16 noise_period_table[16] = {
    4, 8, 16, 32, 64, 96, 128, 160, 202, 254, 380, 508, 762, 1016, 2034, 4068
  };

  // controller I/O
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
