#pragma once

#include "bus.hpp"
#include "types.hpp"

namespace nes {

// implements all CPU instructions
// owns CPU registers
class CPU {
 public:
  explicit CPU(Bus& bus);

  // reset, NMI, and IRQ (return CPU cycles consumed)
  void reset();
  int nmi();
  int irq();

  // Execute one instruction; return number of cycles used.
  int step();

  // Registers (public for debugging / tests)
  u8 A{0};
  u8 X{0};
  u8 Y{0};
  u8 S{0};
  u8 P{0};
  u16 PC{0};

 private:
  Bus& bus_;

  // read and write to memory using bus
  u8 read(u16 addr) const { return bus_.read(addr); }
  void write(u16 addr, u8 value) { bus_.write(addr, value); }

  // push to and pop from stack using bus
  void push(u8 value);
  u8 pop();

  // flag setters and getters
  void set_Z(bool v);
  void set_N(bool v);
  void set_NZ(u8 value);
  bool get_C() const { return (P & flag::C) != 0; }
  void set_C(bool v);
  bool get_V() const { return (P & flag::V) != 0; }
  void set_V(bool v);
  bool get_I() const { return (P & flag::I) != 0; }
};

}  // namespace nes
