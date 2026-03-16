#include "bus.hpp"
#include "ppu.hpp"

namespace nes {

namespace {

constexpr u16 ram_mask = 0x07FF;       // to decode mirrored internal ram addresses to $0000-$07FF range
constexpr u16 ram_end = 0x2000;        // $0000-$1FFF
constexpr u16 ppu_end = 0x4000;        // $2000-$3FFF
constexpr u16 apu_end = 0x4017;        // $4000-$4017
constexpr u16 prg_ram_start = 0x6000;  // $6000-$7FFF
constexpr u16 prg_rom_start = 0x8000;  // $8000-$FFFF

inline u16 mirror_ram(u16 addr) {
  return addr & ram_mask;
}

}  // namespace

Bus::Bus(bool nes_mapping) : nes_mapping_(nes_mapping) {
  if (!nes_mapping_) {
    ram_.resize(ram_size, 0);
  } else {
    ram_.resize(internal_ram_size, 0);
    prg_ram_.resize(prg_ram_size, 0);
  }
}

u8* Bus::ram_ptr() {
  return (nes_mapping_) ? nullptr : ram_.data();
}

const u8* Bus::ram_ptr() const {
  return (nes_mapping_) ? nullptr : ram_.data();
}

u8 Bus::read(u16 addr) const {
  if (nes_mapping_) {
    return read_nes(addr);
  } else {
    return ram_[addr];
  }
}

void Bus::write(u16 addr, u8 value) {
  if (nes_mapping_) {
    write_nes(addr, value);
  } else {
    ram_[addr] = value;
  }
}

u8 Bus::read_nes(u16 addr) const {
  if (addr < ram_end) {
    return ram_[mirror_ram(addr)];
  }
  if (addr < ppu_end) {
    if (ppu_)
      return ppu_->cpu_read(addr & 7);
    return 0;
  }
  if (addr < apu_end) {
    // APU / I/O stub: open bus (return 0)
    return 0;
  }
  if (addr < prg_ram_start) {
    // $4020-$5FFF: unmapped / open bus
    return 0;
  }
  if (addr < prg_rom_start) {
    // $6000-$7FFF: PRG RAM
    return prg_ram_[addr - prg_ram_start];
  }
  // $8000-$FFFF: PRG ROM (potentially mirrored)
  if (prg_rom_.empty())
    return 0;
  return prg_rom_[(addr - prg_rom_start) % prg_rom_.size()];
}

void Bus::write_nes(u16 addr, u8 value) {
  if (addr < ram_end) {
    ram_[mirror_ram(addr)] = value;
    return;
  }
  if (addr < ppu_end) {
    if (ppu_)
      ppu_->cpu_write(addr & 7, value);
    return;
  }
  if (addr < apu_end) {
    // APU / I/O stub: no-op
    return;
  }
  if (addr < prg_ram_start) {
    // $4020-$5FFF: unmapped, no-op
    return;
  }
  if (addr < prg_rom_start) {
    // $6000-$7FFF: PRG RAM
    prg_ram_[addr - prg_ram_start] = value;
    return;
  }
  // $8000-$FFFF: PRG ROM read-only
  (void)value;
}

void Bus::load_prg(const u8* data, std::size_t size) {
  if (data == nullptr || size == 0)
    return;
  const std::size_t load_size = (size <= prg_rom_max) ? size : prg_rom_max;
  prg_rom_.assign(data, data + load_size);
}

}  // namespace nes
