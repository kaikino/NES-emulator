#include "bus.hpp"
#include "cpu.hpp"
#include "ines.hpp"
#include "types.hpp"
#include <cstdio>
#include <string>

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::fprintf(stderr, "Usage: %s <rom.nes>\n", argv[0]);
    return 1;
  }

  std::string path(argv[1]);
  nes::Bus bus(true);
  nes::InesCart cart;
  if (!nes::load_ines(path, bus, cart)) {
    std::fprintf(stderr, "Failed to load iNES ROM: %s\n", path.c_str());
    return 1;
  }

  if (cart.mapper != 0) {
    std::fprintf(stderr, "Unsupported mapper %d (only NROM/mapper 0)\n", cart.mapper);
    return 1;
  }

  nes::CPU cpu(bus);
  nes::u16 reset_lo = bus.read(0xFFFC);
  nes::u16 reset_hi = bus.read(0xFFFD);
  cpu.PC = reset_lo | (reset_hi << 8);
  cpu.S = 0xFD;
  cpu.P = nes::flag::U | nes::flag::I;

  std::printf("Loaded %s (mapper %d, PRG %zu bytes, CHR %zu bytes)\n",
              path.c_str(), cart.mapper, cart.prg_size, cart.chr_size);
  std::printf("Reset vector: $%04X\n", cpu.PC);

  constexpr int max_cycles = 10'000'000;
  int total = 0;
  while (total < max_cycles) {
    total += cpu.step();
  }
  std::printf("Ran %d cycles\n", total);
  return 0;
}
