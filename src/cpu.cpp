#include "cpu.hpp"

namespace nes {

namespace {

constexpr u16 stack_base = 0x0100;
constexpr u16 nmi_vector_lo = 0xFFFA;
constexpr u16 nmi_vector_hi = 0xFFFB;
constexpr u16 reset_vector_lo = 0xFFFC;
constexpr u16 reset_vector_hi = 0xFFFD;
constexpr u16 irq_vector_lo = 0xFFFE;
constexpr u16 irq_vector_hi = 0xFFFF;

}  // namespace

CPU::CPU(Bus& bus) : bus_(bus) {}

// set flags
void CPU::set_Z(bool v) {
  if (v)
    P |= flag::Z;
  else
    P &= ~flag::Z;
}

void CPU::set_N(bool v) {
  if (v)
    P |= flag::N;
  else
    P &= ~flag::N;
}

void CPU::set_NZ(u8 value) {
  set_Z(value == 0);
  set_N((value & 0x80) != 0);
}

void CPU::set_C(bool v) {
  if (v)
    P |= flag::C;
  else
    P &= ~flag::C;
}

void CPU::set_V(bool v) {
  if (v)
    P |= flag::V;
  else
    P &= ~flag::V;
}

void CPU::push(u8 value) {
  write(static_cast<u16>(stack_base + S), value);
  S--;
}

u8 CPU::pop() {
  S++;
  return read(static_cast<u16>(stack_base + S));
}

void CPU::reset() {
  S = 0xFD;  // bottom of stack
  P = flag::U | flag::I;  // unused always 1, interrupts disabled
  PC = read(reset_vector_lo) | (static_cast<u16>(read(reset_vector_hi)) << 8);
}

int CPU::nmi() {
  push(static_cast<u8>(PC >> 8));
  push(static_cast<u8>(PC));
  push((P & ~flag::B) | flag::U);  // B=0 for NMI
  P |= flag::I;
  PC = read(nmi_vector_lo) | (static_cast<u16>(read(nmi_vector_hi)) << 8);
  return 7;
}

int CPU::irq() {
  if (get_I())
    return 0;
  push(static_cast<u8>(PC >> 8));
  push(static_cast<u8>(PC));
  push((P & ~flag::B) | flag::U);  // B=0 for IRQ
  P |= flag::I;
  PC = read(irq_vector_lo) | (static_cast<u16>(read(irq_vector_hi)) << 8);
  return 7;
}

int CPU::step() {
  const u8 op = read(PC++);
  int cycles = 0;
  bool page_crossed = false;

  // addressing modes

  // #v: Immediate
  // Uses the 8-bit operand itself as the value for the operation,
  // rather than fetching a value from a memory address.
  auto imm = [this]() { return read(PC++); };
  // d: Zero page
  // Fetches the value from an 8-bit address on the zero page.
  auto zp = [this]() {
    u8 lo = read(PC++);
    return static_cast<u16>(lo);
  };
  // d,x: Zero page indexed
  // val = PEEK((arg + X) % 256)
  auto zpx = [this]() {
    u8 lo = (read(PC++) + X) & 0xFF;
    return static_cast<u16>(lo);
  };
  // d,y: Zero page indexed
  // val = PEEK((arg + Y) % 256)
  auto zpy = [this]() {
    u8 lo = (read(PC++) + Y) & 0xFF;
    return static_cast<u16>(lo);
  };
  // a: Absolute
  // Fetches the value from a 16-bit address anywhere in memory.
  auto abs = [this]() {
    u16 lo = read(PC++);
    u16 hi = read(PC++);
    return lo | (hi << 8);
  };
  // a,x: Absolute indexed
  // val = PEEK(arg + X)
  auto absx = [this, &page_crossed]() {
    u16 lo = read(PC++);
    u16 hi = read(PC++);
    u16 base = lo | (hi << 8);
    u16 addr = base + X;
    page_crossed = (base & 0xFF00) != (addr & 0xFF00);  // track "oops" cycle for cross-page read
    return addr;
  };
  // a,y: Absolute indexed
  // val = PEEK(arg + Y)
  auto absy = [this, &page_crossed]() {
    u16 lo = read(PC++);
    u16 hi = read(PC++);
    u16 base = lo | (hi << 8);
    u16 addr = base + Y;
    page_crossed = (base & 0xFF00) != (addr & 0xFF00);
    return addr;
  };
  // (a): Indirect
  // The JMP instruction has a special indirect addressing mode that can
  // jump to the address stored in a 16-bit pointer anywhere in memory.
  auto ind = [this]() {
    u16 ptr_lo = read(PC++);
    u16 ptr_hi = read(PC++);
    u16 ptr = ptr_lo | (ptr_hi << 8);
    u16 lo = read(ptr);
    u16 hi = read((ptr & 0xFF00) | static_cast<u16>((ptr + 1) & 0xFF));  // JMP ($xxFF) bug
    return lo | (hi << 8);
  };
  // (d,x): Indexed indirect
  // val = PEEK(PEEK((arg + X) % 256) + PEEK((arg + X + 1) % 256) * 256)
  auto indx = [this]() {
    u8 ptr = read(PC++) + X;
    u16 base = read(ptr & 0xFF) | (static_cast<u16>(read((ptr + 1) & 0xFF)) << 8);
    return base;
  };
  // (d),y: Indirect indexed
  // val = PEEK(PEEK(arg) + PEEK((arg + 1) % 256) * 256 + Y)
  auto indy = [this, &page_crossed]() {
    u8 ptr = read(PC++);
    u16 base = read(ptr & 0xFF) | (static_cast<u16>(read((ptr + 1) & 0xFF)) << 8);
    u16 addr = base + Y;
    page_crossed = (base & 0xFF00) != (addr & 0xFF00);
    return addr;
  };
  // label: Relative
  // Branch instructions (e.g. BEQ, BCS) have a relative addressing mode that specifies
  // an 8-bit signed offset relative to the current PC.
  auto rel = [this]() {
    s8 offset = static_cast<s8>(read(PC++));
    return static_cast<u16>(static_cast<s16>(PC) + offset);
  };

  // instruction helpers

  auto do_adc = [this](u8 m) {
    // decimal mode (optional for NES)
    if (P & flag::D) {
      u16 sum = A + m + (get_C() ? 1 : 0);
      set_V(((A ^ sum) & (m ^ sum) & 0x80) != 0);
      u8 lo = (A & 0x0F) + (m & 0x0F) + (get_C() ? 1 : 0);
      if (lo > 9) lo += 6;
      u8 hi = (A >> 4) + (m >> 4) + (lo > 15);
      if (hi > 9) hi += 6;
      set_C(hi > 15);
      A = static_cast<u8>((hi << 4) | (lo & 0x0F));
      set_NZ(A);
    } else {
      u16 sum = A + m + (get_C() ? 1 : 0);         // A = A + memory + C
      set_V(((A ^ sum) & (m ^ sum) & 0x80) != 0);  // overflow: (result ^ A) & (result ^ memory) & $80
      set_C(sum > 0xFF);                           // carry: result > $FF
      A = static_cast<u8>(sum);
      set_NZ(A);                                   // negative: result bit 7; zero: result == 0
    }
  };

  auto do_sbc = [this](u8 m) {
    // decimal mode (optional for NES)
    if (P & flag::D) {
      u16 sub = static_cast<u16>(A) - m - (get_C() ? 0 : 1);
      set_V(((A ^ static_cast<u8>(sub)) & (A ^ m) & 0x80) != 0);
      bool borrow_lo = (A & 0x0F) < (m & 0x0F) + (get_C() ? 0 : 1);
      int lo = (A & 0x0F) - (m & 0x0F) - (get_C() ? 0 : 1);
      if (lo < 0) lo += 10;
      int hi = (A >> 4) - (m >> 4) - (borrow_lo ? 1 : 0);
      bool borrow_hi = (hi < 0);
      if (hi < 0) hi += 10;
      set_C(!borrow_hi);
      A = static_cast<u8>((hi << 4) | lo);
      set_NZ(A);
    } else {
      u16 sub = static_cast<u16>(A) - m - (get_C() ? 0 : 1);      // A = A - memory - ~C
      set_V(((A ^ static_cast<u8>(sub)) & (A ^ m) & 0x80) != 0);  // overflow: (result ^ A) & (result ^ ~memory) & $80
      set_C(sub < 0x100);                                         // carry: ~(result < $00)
      A = static_cast<u8>(sub);
      set_NZ(A);                                                  // negative: result bit 7; zero: result == 0
    }
  };

  auto do_branch = [this, &cycles](bool taken, u16 target) {
    if (!taken) {
      cycles = 2;
      return;
    }
    cycles = 3;
    if ((PC & 0xFF00) != (target & 0xFF00)) cycles++; // page crossed
    PC = target;
  };

  auto do_compare = [this](u8 reg_val, u8 m) {
    set_C(reg_val >= m);                   // carry: A >= memory
    set_NZ(static_cast<u8>(reg_val - m));  // negative: result bit 7; zero: A == 0
  };

  auto do_load = [this](u8& reg, u8 value) {
    reg = value;  // A = memory
    set_NZ(reg);  // negative: result bit 7; zero: result == 0
  };

  auto do_alu = [this](u8 m, auto op) {
    A = op(A, m);  // XOR, OR, AND
    set_NZ(A);     // negative: result bit 7; zero: result == 0
  };

  auto do_rmw_mem = [this](u16 addr, auto transform) {
    u8 v = read(addr);
    v = transform(v);
    write(addr, v);
    set_NZ(v);     // negative: result bit 7; zero: result == 0
  };

  // lambda functions for RMW opeartions
  auto rmw_asl = [this](u8 v) { set_C((v & 0x80) != 0); return static_cast<u8>(v << 1); };
  auto rmw_lsr = [this](u8 v) { set_C((v & 0x01) != 0); return static_cast<u8>(v >> 1); };
  auto rmw_rol = [this](u8 v) {
    u8 c = get_C() ? 1 : 0;
    set_C((v & 0x80) != 0);
    return static_cast<u8>((v << 1) | c);
  };
  auto rmw_ror = [this](u8 v) {
    u8 c = get_C() ? 0x80 : 0;
    set_C((v & 0x01) != 0);
    return static_cast<u8>((v >> 1) | c);
  };
  auto rmw_inc = [](u8 v) { return v + 1; };
  auto rmw_dec = [](u8 v) { return v - 1; };

  // lambda helpers for ALU operations
  auto alu_and = [](u8 a, u8 b) { return static_cast<u8>(a & b); };
  auto alu_or = [](u8 a, u8 b) { return static_cast<u8>(a | b); };
  auto alu_eor = [](u8 a, u8 b) { return static_cast<u8>(a ^ b); };

  auto do_bit = [this](u8 m) {
    set_Z((A & m) == 0);     // zero: A & memory == 0
    set_N((m & 0x80) != 0);  // negative: memory bit 7
    set_V((m & 0x40) != 0);  // overflow: memory bit 6
  };

  // instructions

  switch (op) {
    // ADC - Add with Carry
    case 0x69: do_adc(imm()); cycles = 2; break;
    case 0x65: do_adc(read(zp())); cycles = 3; break;
    case 0x75: do_adc(read(zpx())); cycles = 4; break;
    case 0x6D: do_adc(read(abs())); cycles = 4; break;
    case 0x7D: do_adc(read(absx())); cycles = 4 + (page_crossed ? 1 : 0); break;
    case 0x79: do_adc(read(absy())); cycles = 4 + (page_crossed ? 1 : 0); break;
    case 0x61: do_adc(read(indx())); cycles = 6; break;
    case 0x71: do_adc(read(indy())); cycles = 5 + (page_crossed ? 1 : 0); break;
    // AND - Bitwise AND
    case 0x29: do_alu(imm(), alu_and); cycles = 2; break;
    case 0x25: do_alu(read(zp()), alu_and); cycles = 3; break;
    case 0x35: do_alu(read(zpx()), alu_and); cycles = 4; break;
    case 0x2D: do_alu(read(abs()), alu_and); cycles = 4; break;
    case 0x3D: do_alu(read(absx()), alu_and); cycles = 4 + (page_crossed ? 1 : 0); break;
    case 0x39: do_alu(read(absy()), alu_and); cycles = 4 + (page_crossed ? 1 : 0); break;
    case 0x21: do_alu(read(indx()), alu_and); cycles = 6; break;
    case 0x31: do_alu(read(indy()), alu_and); cycles = 5 + (page_crossed ? 1 : 0); break;
    // ASL - Arithmetic Shift Left
    case 0x0A: A = rmw_asl(A); set_NZ(A); cycles = 2; break;
    case 0x06: do_rmw_mem(zp(), rmw_asl); cycles = 5; break;
    case 0x16: do_rmw_mem(zpx(), rmw_asl); cycles = 6; break;
    case 0x0E: do_rmw_mem(abs(), rmw_asl); cycles = 6; break;
    case 0x1E: do_rmw_mem(absx(), rmw_asl); cycles = 7; break;
    // BCC - Branch if Carry Clear
    case 0x90: do_branch(!get_C(), rel()); break;
    // BCS - Branch if Carry Set
    case 0xB0: do_branch(get_C(), rel()); break;
    // BEQ - Branch if Equal
    case 0xF0: do_branch((P & flag::Z) != 0, rel()); break;
    // BMI - Branch if Minus
    case 0x30: do_branch((P & flag::N) != 0, rel()); break;
    // BNE - Branch if Not Equal
    case 0xD0: do_branch((P & flag::Z) == 0, rel()); break;
    // BPL - Branch if Plus
    case 0x10: do_branch((P & flag::N) == 0, rel()); break;
    // BVC - Branch if Overflow Clear
    case 0x50: do_branch((P & flag::V) == 0, rel()); break;
    // BVS - Branch if Overflow Set
    case 0x70: do_branch((P & flag::V) != 0, rel()); break;
    // BIT - Bit Test
    case 0x24: do_bit(read(zp())); cycles = 3; break;
    case 0x2C: do_bit(read(abs())); cycles = 4; break;
    // BRK - Break (software IRQ)
    case 0x00: {
      PC++;
      push(static_cast<u8>(PC >> 8));
      push(static_cast<u8>(PC));
      push(P | flag::B | flag::U);
      P |= flag::I;
      PC = read(irq_vector_lo) | (static_cast<u16>(read(irq_vector_hi)) << 8);
      cycles = 7;
      break;
    }
    // CLC - Clear Carry
    case 0x18: set_C(false); cycles = 2; break;
    // CLD - Clear Decimal
    case 0xD8: P &= ~flag::D; cycles = 2; break;
    // CLI - Clear Interrupt Disable
    case 0x58: P &= ~flag::I; cycles = 2; break;
    // CLV - Clear Overflow
    case 0xB8: set_V(false); cycles = 2; break;
    // CMP - Compare A
    case 0xC9: do_compare(A, imm()); cycles = 2; break;
    case 0xC5: do_compare(A, read(zp())); cycles = 3; break;
    case 0xD5: do_compare(A, read(zpx())); cycles = 4; break;
    case 0xCD: do_compare(A, read(abs())); cycles = 4; break;
    case 0xDD: do_compare(A, read(absx())); cycles = 4 + (page_crossed ? 1 : 0); break;
    case 0xD9: do_compare(A, read(absy())); cycles = 4 + (page_crossed ? 1 : 0); break;
    case 0xC1: do_compare(A, read(indx())); cycles = 6; break;
    case 0xD1: do_compare(A, read(indy())); cycles = 5 + (page_crossed ? 1 : 0); break;
    // CPX - Compare X
    case 0xE0: do_compare(X, imm()); cycles = 2; break;
    case 0xE4: do_compare(X, read(zp())); cycles = 3; break;
    case 0xEC: do_compare(X, read(abs())); cycles = 4; break;
    // CPY - Compare Y
    case 0xC0: do_compare(Y, imm()); cycles = 2; break;
    case 0xC4: do_compare(Y, read(zp())); cycles = 3; break;
    case 0xCC: do_compare(Y, read(abs())); cycles = 4; break;
    // DEC - Decrement Memory
    case 0xC6: do_rmw_mem(zp(), rmw_dec); cycles = 5; break;
    case 0xD6: do_rmw_mem(zpx(), rmw_dec); cycles = 6; break;
    case 0xCE: do_rmw_mem(abs(), rmw_dec); cycles = 6; break;
    case 0xDE: do_rmw_mem(absx(), rmw_dec); cycles = 7; break;
    // DEX - Decrement X
    case 0xCA: X--; set_NZ(X); cycles = 2; break;
    // DEY - Decrement Y
    case 0x88: Y--; set_NZ(Y); cycles = 2; break;
    // EOR - Bitwise Exclusive OR
    case 0x49: do_alu(imm(), alu_eor); cycles = 2; break;
    case 0x45: do_alu(read(zp()), alu_eor); cycles = 3; break;
    case 0x55: do_alu(read(zpx()), alu_eor); cycles = 4; break;
    case 0x4D: do_alu(read(abs()), alu_eor); cycles = 4; break;
    case 0x5D: do_alu(read(absx()), alu_eor); cycles = 4 + (page_crossed ? 1 : 0); break;
    case 0x59: do_alu(read(absy()), alu_eor); cycles = 4 + (page_crossed ? 1 : 0); break;
    case 0x41: do_alu(read(indx()), alu_eor); cycles = 6; break;
    case 0x51: do_alu(read(indy()), alu_eor); cycles = 5 + (page_crossed ? 1 : 0); break;
    // INC - Increment Memory
    case 0xE6: do_rmw_mem(zp(), rmw_inc); cycles = 5; break;
    case 0xF6: do_rmw_mem(zpx(), rmw_inc); cycles = 6; break;
    case 0xEE: do_rmw_mem(abs(), rmw_inc); cycles = 6; break;
    case 0xFE: do_rmw_mem(absx(), rmw_inc); cycles = 7; break;
    // INC - Increment X
    case 0xE8: X++; set_NZ(X); cycles = 2; break;
    // INC - Increment Y
    case 0xC8: Y++; set_NZ(Y); cycles = 2; break;
    // JMP - Jump
    case 0x4C: PC = abs(); cycles = 3; break;
    case 0x6C: PC = ind(); cycles = 5; break;
    // JSR - Jump to Subroutine
    case 0x20: {
      u16 target = abs();
      u16 ret = PC - 1;
      push(static_cast<u8>(ret >> 8));
      push(static_cast<u8>(ret));
      PC = target;
      cycles = 6;
      break;
    }
    // LDA - Load A
    case 0xA9: do_load(A, imm()); cycles = 2; break;
    case 0xA5: do_load(A, read(zp())); cycles = 3; break;
    case 0xB5: do_load(A, read(zpx())); cycles = 4; break;
    case 0xAD: do_load(A, read(abs())); cycles = 4; break;
    case 0xBD: do_load(A, read(absx())); cycles = 4 + (page_crossed ? 1 : 0); break;
    case 0xB9: do_load(A, read(absy())); cycles = 4 + (page_crossed ? 1 : 0); break;
    case 0xA1: do_load(A, read(indx())); cycles = 6; break;
    case 0xB1: do_load(A, read(indy())); cycles = 5 + (page_crossed ? 1 : 0); break;
    // LDX - Load X
    case 0xA2: do_load(X, imm()); cycles = 2; break;
    case 0xA6: do_load(X, read(zp())); cycles = 3; break;
    case 0xB6: do_load(X, read(zpy())); cycles = 4; break;
    case 0xAE: do_load(X, read(abs())); cycles = 4; break;
    case 0xBE: do_load(X, read(absy())); cycles = 4 + (page_crossed ? 1 : 0); break;
    // LDY - Load Y
    case 0xA0: do_load(Y, imm()); cycles = 2; break;
    case 0xA4: do_load(Y, read(zp())); cycles = 3; break;
    case 0xB4: do_load(Y, read(zpx())); cycles = 4; break;
    case 0xAC: do_load(Y, read(abs())); cycles = 4; break;
    case 0xBC: do_load(Y, read(absx())); cycles = 4 + (page_crossed ? 1 : 0); break;
    // LSR - Logical Shift Right
    case 0x4A: A = rmw_lsr(A); set_NZ(A); cycles = 2; break;
    case 0x46: do_rmw_mem(zp(), rmw_lsr); cycles = 5; break;
    case 0x56: do_rmw_mem(zpx(), rmw_lsr); cycles = 6; break;
    case 0x4E: do_rmw_mem(abs(), rmw_lsr); cycles = 6; break;
    case 0x5E: do_rmw_mem(absx(), rmw_lsr); cycles = 7; break;
    // NOP - No Operation
    case 0xEA: cycles = 2; break;
    // ORA - Bitwise OR
    case 0x09: do_alu(imm(), alu_or); cycles = 2; break;
    case 0x05: do_alu(read(zp()), alu_or); cycles = 3; break;
    case 0x15: do_alu(read(zpx()), alu_or); cycles = 4; break;
    case 0x0D: do_alu(read(abs()), alu_or); cycles = 4; break;
    case 0x1D: do_alu(read(absx()), alu_or); cycles = 4 + (page_crossed ? 1 : 0); break;
    case 0x19: do_alu(read(absy()), alu_or); cycles = 4 + (page_crossed ? 1 : 0); break;
    case 0x01: do_alu(read(indx()), alu_or); cycles = 6; break;
    case 0x11: do_alu(read(indy()), alu_or); cycles = 5 + (page_crossed ? 1 : 0); break;
    // PHA - Push A
    case 0x48: push(A); cycles = 3; break;
    // PHP - Push Processor Status
    case 0x08: push(P | flag::B | flag::U); cycles = 3; break;
    // PLA - Pull A
    case 0x68: do_load(A, pop()); cycles = 4; break;
    // PLP - Pull Processor Status
    case 0x28: P = pop() & ~flag::B; P |= flag::U; cycles = 4; break;
    // ROL - Rotate Left
    case 0x2A: A = rmw_rol(A); set_NZ(A); cycles = 2; break;
    case 0x26: do_rmw_mem(zp(), rmw_rol); cycles = 5; break;
    case 0x36: do_rmw_mem(zpx(), rmw_rol); cycles = 6; break;
    case 0x2E: do_rmw_mem(abs(), rmw_rol); cycles = 6; break;
    case 0x3E: do_rmw_mem(absx(), rmw_rol); cycles = 7; break;
    // ROR - Rotate Right
    case 0x6A: A = rmw_ror(A); set_NZ(A); cycles = 2; break;
    case 0x66: do_rmw_mem(zp(), rmw_ror); cycles = 5; break;
    case 0x76: do_rmw_mem(zpx(), rmw_ror); cycles = 6; break;
    case 0x6E: do_rmw_mem(abs(), rmw_ror); cycles = 6; break;
    case 0x7E: do_rmw_mem(absx(), rmw_ror); cycles = 7; break;
    // RTI - Return from Interrupt
    case 0x40: {
      P = pop() & ~flag::B;
      P |= flag::U;
      u8 lo = pop();
      u8 hi = pop();
      PC = lo | (static_cast<u16>(hi) << 8);
      cycles = 6;
      break;
    }
    // RTS - Return from Subroutine
    case 0x60: {
      u8 lo = pop();
      u8 hi = pop();
      PC = (lo | (static_cast<u16>(hi) << 8)) + 1;
      cycles = 6;
      break;
    }
    // SBC - Subtract with Carry
    case 0xE9: do_sbc(imm()); cycles = 2; break;
    case 0xE5: do_sbc(read(zp())); cycles = 3; break;
    case 0xF5: do_sbc(read(zpx())); cycles = 4; break;
    case 0xED: do_sbc(read(abs())); cycles = 4; break;
    case 0xFD: do_sbc(read(absx())); cycles = 4 + (page_crossed ? 1 : 0); break;
    case 0xF9: do_sbc(read(absy())); cycles = 4 + (page_crossed ? 1 : 0); break;
    case 0xE1: do_sbc(read(indx())); cycles = 6; break;
    case 0xF1: do_sbc(read(indy())); cycles = 5 + (page_crossed ? 1 : 0); break;
    // SEC - Set Carry
    case 0x38: set_C(true); cycles = 2; break;
    // SED - Set Decimal
    case 0xF8: P |= flag::D; cycles = 2; break;
    // SEI - Set Interrupt Disable
    case 0x78: P |= flag::I; cycles = 2; break;
    // STA - Store A
    case 0x85: write(zp(), A); cycles = 3; break;
    case 0x95: write(zpx(), A); cycles = 4; break;
    case 0x8D: write(abs(), A); cycles = 4; break;
    case 0x9D: write(absx(), A); cycles = 5; break;
    case 0x99: write(absy(), A); cycles = 5; break;
    case 0x81: write(indx(), A); cycles = 6; break;
    case 0x91: write(indy(), A); cycles = 6; break;
    // STX - Store X
    case 0x86: write(zp(), X); cycles = 3; break;
    case 0x96: write(zpy(), X); cycles = 4; break;
    case 0x8E: write(abs(), X); cycles = 4; break;
    // STY - Store Y
    case 0x84: write(zp(), Y); cycles = 3; break;
    case 0x94: write(zpx(), Y); cycles = 4; break;
    case 0x8C: write(abs(), Y); cycles = 4; break;
    // TAX - Transfer A to X
    case 0xAA: do_load(X, A); cycles = 2; break;
    // TAY - Transfer A to Y
    case 0xA8: do_load(Y, A); cycles = 2; break;
    // TSX - Transfer Stack Pointer to X
    case 0xBA: do_load(X, S); cycles = 2; break;
    // TXA - Transfer X to A
    case 0x8A: do_load(A, X); cycles = 2; break;
    // TXS - Transfer X to Stack Pointer
    case 0x9A: S = X; cycles = 2; break;  // TXS: no N/Z
    // TYA - Transfer Y to A
    case 0x98: do_load(A, Y); cycles = 2; break;
    // Undocumented / illegal: treat as NOP (2 cycles)
    default: cycles = 2; break;
  }

  return cycles;
}

}  // namespace nes
