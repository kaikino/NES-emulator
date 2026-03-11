#pragma once

#include "types.hpp"
#include <vector>

namespace nes {

class Bus {
 public:
  // nes_mapping == false: legacy mode (flat 64K for Klaus test).
  // nes_mapping == true: NES address decoding (2KB RAM, PPU, APU, cartridge).
  explicit Bus(bool nes_mapping = false);

  static constexpr std::size_t ram_size = 65536;  // legacy mode flat 64K

  // read byte from memory address
  u8 read(u16 addr) const;
  // write byte to mirrored memory address
  void write(u16 addr, u8 value);

  // Legacy mode: flat 64K (returns ram_.data()). NES mode: returns nullptr (use load_prg).
  u8* ram_ptr();
  const u8* ram_ptr() const;

  // For NES mapping: load PRG into cartridge region ($8000-$FFFF). Size typically 16KB or 32KB.
  void load_prg(const u8* data, std::size_t size);

  // Address range  Size     Device
  // $0000–$07FF    $0800    2 KB internal RAM
  // $0800–$0FFF    $0800    Mirrors of $0000–$07FF
  // $1000–$17FF    $0800    ^
  // $1800–$1FFF    $0800    ^
  // $2000–$2007    $0008    NES PPU registers
  // $2008–$3FFF    $1FF8    Mirrors of $2000–$2007 (repeats every 8 bytes)
  // $4000–$4017    $0018    NES APU and I/O registers
  // $4018–$401F    $0008    APU and I/O functionality that is normally disabled. See CPU Test Mode.
  // $4020–$FFFF    $BFE0    Unmapped. Available for cartridge use.
  //   $6000–$7FFF    $2000    Usually cartridge RAM, when present.
  //   $8000–$FFFF    $8000    Usually cartridge ROM and mapper registers.

 private:
  u8 read_nes(u16 addr) const;
  void write_nes(u16 addr, u8 value);

  static constexpr std::size_t internal_ram_size = 0x0800;  // $0000-$1FFF
  static constexpr std::size_t prg_ram_size = 0x2000;       // $6000-$7FFF, 8KB
  static constexpr std::size_t prg_rom_max = 0x8000;        // $8000-$FFFF, 32KB

  bool const nes_mapping_;
  std::vector<u8> ram_{};         // legacy: 64KB flat; NES: 2KB internal ($0000-$1FFF)
  std::vector<u8> prg_ram_{};      // NES only ($6000-$7FFF)
  std::vector<u8> prg_rom_{};      // $8000-$FFFF, size set by load_prg (mirror by decode)
};

}  // namespace nes
