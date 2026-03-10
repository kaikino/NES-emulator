#include "bus.hpp"

namespace nes {

namespace {

// NES internal RAM is 2 KB ($0000-$07FF), mirrored to fill $0000-$1FFF.
constexpr u16 ram_mask = 0x07FF;
constexpr u16 ram_region_end = 0x2000;

inline u16 mirror_ram(u16 addr) {
  return addr & ram_mask;
}

}  // namespace

Bus::Bus() {
  ram_.fill(0);
}

u8 Bus::read(u16 addr) const {
  if (ram_mirroring_ && addr < ram_region_end) {
    return ram_[mirror_ram(addr)];
  }
  return ram_[addr];
}

void Bus::write(u16 addr, u8 value) {
  if (ram_mirroring_ && addr < ram_region_end) {
    ram_[mirror_ram(addr)] = value;
    return;
  }
  ram_[addr] = value;
}

}  // namespace nes
