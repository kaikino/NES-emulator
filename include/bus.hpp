#pragma once

#include "types.hpp"
#include <array>

namespace nes {

class Bus {
 public:
  Bus();

  static constexpr std::size_t ram_size = 65536;

  // read byte from memory address
  u8 read(u16 addr) const;
  // write byte to mirrored memory address
  void write(u16 addr, u8 value);

  // direct access to memory for loading ROMs
  u8* ram_ptr() { return ram_.data(); }
  const u8* ram_ptr() const { return ram_.data(); }

  // no mirroring when enable=false for Klaus Dormann test (expects full address space)
  void set_ram_mirroring(bool enable) { ram_mirroring_ = enable; }

 private:
  std::array<u8, ram_size> ram_{};  // memory buffer
  bool ram_mirroring_ = true;  // enable 2KB mirror for $0000-$1FFF
};

}  // namespace nes
