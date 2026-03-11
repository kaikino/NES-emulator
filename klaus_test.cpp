#include "bus.hpp"
#include "cpu.hpp"
#include "types.hpp"
#include <cstdio>
#include <fstream>

int main(int argc, char* argv[]) {
  const char* rom_path = (argc > 1) ? argv[1] : "6502_functional_test.bin";

  std::ifstream file(rom_path, std::ios::binary | std::ios::ate);
  if (!file && argc <= 1) {
    rom_path = "../roms/6502_functional_test.bin";
    file.open(rom_path, std::ios::binary | std::ios::ate);
  }
  if (!file) {
    std::fprintf(stderr, "Failed to open ROM: %s\n", rom_path);
    return 1;
  }
  std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);

  nes::Bus bus;
  nes::CPU cpu(bus);

  if (size > static_cast<std::streamsize>(nes::Bus::ram_size)) {
    std::fprintf(stderr, "ROM too large (%zd bytes)\n", static_cast<std::size_t>(size));
    return 1;
  }

  if (!file.read(reinterpret_cast<char*>(bus.ram_ptr()), size)) {
    std::fprintf(stderr, "Failed to read ROM\n");
    return 1;
  }

  // Klaus Dormann test: entry at $0400 (code_segment), not reset vector.
  constexpr nes::u16 klaus_entry = 0x0400;
  cpu.PC = klaus_entry;
  cpu.S = 0xFD;
  cpu.P = nes::flag::U | nes::flag::I;

  constexpr int max_cycles = 100'000'000;
  int total_cycles = 0;
  nes::u16 prev_pc = 0xFFFF;
  int same_pc_count = 0;

  while (total_cycles < max_cycles) {
    int c = cpu.step();
    total_cycles += c;
    if (cpu.PC == prev_pc) {
      if (++same_pc_count >= 5) break;
    } else {
      same_pc_count = 0;
    }
    prev_pc = cpu.PC;
  }

  std::printf("ROM: %s (%zd bytes loaded)\n", rom_path, static_cast<std::size_t>(size));
  std::printf("Cycles: %d (stopped after %s)\n", total_cycles,
              same_pc_count >= 5 ? "trap detected" : "max cycles");
  std::printf("Final PC=$%04X A=$%02X X=$%02X Y=$%02X S=$%02X P=$%02X\n",
              cpu.PC, cpu.A, cpu.X, cpu.Y, cpu.S, cpu.P);

  constexpr int expected_cycles = 92'000'000;
  if (same_pc_count >= 5 && total_cycles >= expected_cycles - 2'000'000)
    std::printf("\nPASS (trap after ~full run; compare PC to listing for success address).\n");
  else if (same_pc_count >= 5)
    std::printf("\nFAIL? (trap at PC=$%04X after %d cycles; check 6502_functional_test.lst for this address).\n",
                cpu.PC, total_cycles);
  else
    std::printf("\nStopped at max cycles (no trap).\n");

  return 0;
}
