#include "ines.hpp"
#include "bus.hpp"
#include <fstream>
#include <cstring>

namespace nes {

namespace {

constexpr unsigned char ines_magic[4] = {'N', 'E', 'S', 0x1A};
constexpr std::size_t header_size = 16;
constexpr std::size_t trainer_size = 512;
constexpr std::size_t prg_bank_size = 16384;
constexpr std::size_t chr_bank_size = 8192;

}  // namespace

bool load_ines(const std::string& path, Bus& bus, InesCart& cart) {
  std::ifstream file(path, std::ios::binary | std::ios::ate);
  if (!file) return false;
  std::streamsize file_size = file.tellg();
  file.seekg(0, std::ios::beg);

  // verify via file header that file is type iNES
  if (file_size < static_cast<std::streamsize>(header_size)) return false;

  // read header into buffer
  unsigned char header[header_size];
  if (!file.read(reinterpret_cast<char*>(header), header_size)) return false;

  // verify via header that file is type iNES
  if (std::memcmp(header, ines_magic, 4) != 0) return false;

  // parse header for program ROM and character ROM sizes
  if (header[4] == 0) return false;
  cart.prg_size = header[4] * prg_bank_size;
  cart.chr_size = header[5] * chr_bank_size;

  // parse header for mapper number and mirroring type
  cart.mapper = static_cast<int>((header[6] >> 4) | (header[7] & 0xF0));
  cart.mirror_vertical = (header[6] & 1) != 0;
  // check if battery-backed RAM is set
  cart.has_battery = (header[6] & 2) != 0;
  // check if trainer data is set
  cart.has_trainer = (header[6] & 4) != 0;

  // calculate data start position
  std::streamsize data_start = header_size;
  if (cart.has_trainer)
    data_start += trainer_size;

  // verify that file size is large enough to contain program and character ROM
  if (file_size < data_start + static_cast<std::streamsize>(cart.prg_size + cart.chr_size)) return false;

  // seek to data start position
  file.seekg(data_start, std::ios::beg);
  
  // load program ROM via bus
  std::vector<u8> prg(cart.prg_size);
  if (!file.read(reinterpret_cast<char*>(prg.data()), static_cast<std::streamsize>(cart.prg_size))) return false;
  bus.load_prg(prg.data(), prg.size());

  // load character ROM
  if (cart.chr_size > 0) {
    cart.chr.resize(cart.chr_size);
    if (!file.read(reinterpret_cast<char*>(cart.chr.data()), static_cast<std::streamsize>(cart.chr_size)))
      return false;
  }

  return true;
}

}  // namespace nes
