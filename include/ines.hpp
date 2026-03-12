#pragma once

#include "types.hpp"
#include <cstddef>
#include <string>
#include <vector>

namespace nes {

class Bus;

// Parsed iNES cart metadata and CHR
struct InesCart {
  int mapper = 0;
  std::size_t prg_size = 0;      // program size in bytes
  std::size_t chr_size = 0;      // character size in bytes
  bool mirror_vertical = false;  // byte 6 bit 0: 1 = vertical, 0 = horizontal
  bool has_battery = false;
  bool has_trainer = false;
  std::vector<u8> chr;  // CHR ROM (empty if chr_size == 0, e.g. CHR RAM)
};

// Load .nes file: parse header, load PRG into bus, fill cart with CHR and metadata.
// Returns true on success. bus must be NES mode (Bus(true)).
bool load_ines(const std::string& path, Bus& bus, InesCart& cart);

}  // namespace nes
