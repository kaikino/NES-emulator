#include "ppu.hpp"
#include <algorithm>
#include <cstring>

namespace nes {

namespace {

// NES 64-color RGB palette (approximate NTSC; index = PPU palette byte & 0x3F)
constexpr u8 kPalette[64][3] = {
    {0x62, 0x62, 0x62}, {0x00, 0x22, 0xb8}, {0x14, 0x00, 0xbc}, {0x33, 0x00, 0xa0},
    {0x60, 0x00, 0x76}, {0x8c, 0x00, 0x26}, {0xa0, 0x00, 0x12}, {0x88, 0x10, 0x00},
    {0x5c, 0x2e, 0x00}, {0x30, 0x44, 0x00}, {0x0b, 0x54, 0x00}, {0x00, 0x5c, 0x00},
    {0x00, 0x5c, 0x10}, {0x00, 0x50, 0x4e}, {0x00, 0x00, 0x00}, {0x00, 0x00, 0x00},
    {0x98, 0x98, 0x98}, {0x00, 0x68, 0xec}, {0x38, 0x38, 0xec}, {0x58, 0x00, 0xd8},
    {0xa0, 0x00, 0xb0}, {0xd4, 0x00, 0x64}, {0xe8, 0x18, 0x00}, {0xd8, 0x30, 0x00},
    {0xac, 0x54, 0x00}, {0x70, 0x6c, 0x00}, {0x24, 0x7a, 0x00}, {0x00, 0x82, 0x00},
    {0x00, 0x82, 0x3c}, {0x00, 0x76, 0x88}, {0x00, 0x00, 0x00}, {0x00, 0x00, 0x00},
    {0xbc, 0xbc, 0xbc}, {0x3c, 0x9c, 0xfc}, {0x88, 0x88, 0xfc}, {0xb0, 0x44, 0xf8},
    {0xe8, 0x48, 0xd8}, {0xfc, 0x58, 0x98}, {0xfc, 0x74, 0x58}, {0xfc, 0x90, 0x20},
    {0xe0, 0xa8, 0x00}, {0xa8, 0xc0, 0x00}, {0x68, 0xd4, 0x28}, {0x28, 0xdc, 0x28},
    {0x00, 0xdc, 0x78}, {0x00, 0xd0, 0xcc}, {0x58, 0x58, 0x58}, {0x00, 0x00, 0x00},
    {0xfc, 0xfc, 0xfc}, {0xa8, 0xe4, 0xfc}, {0xc4, 0xd4, 0xfc}, {0xd8, 0xc8, 0xfc},
    {0xfc, 0xc4, 0xfc}, {0xfc, 0xc4, 0xd8}, {0xfc, 0xbc, 0xb0}, {0xfc, 0xd8, 0xa8},
    {0xfc, 0xe8, 0x90}, {0xf0, 0xfc, 0x90}, {0xc8, 0xfc, 0x98}, {0xa8, 0xfc, 0xb0},
    {0x98, 0xfc, 0xcc}, {0x98, 0xfc, 0xf8}, {0x00, 0x00, 0x00}, {0x00, 0x00, 0x00},
};

constexpr int kCyclesPerScanline = 341;
constexpr int kScanlinesPerFrame = 262;
constexpr int kVblankScanline = 241;

}  // namespace

PPU::PPU(const u8* chr_data, std::size_t chr_size, bool mirror_vertical)
    : chr_(chr_data),
      chr_size_(chr_size),
      mirror_vertical_(mirror_vertical) {}

// read from PPU registers
u8 PPU::cpu_read(u16 addr) {
  const u8 r = addr & 7;
  switch (r) {
    case 0:  // $2000 PPUCTRL - write-only
      return 0;
    case 1:  // $2001 PPUMASK - write-only
      return 0;
    case 2: {  // $2002 PPUSTATUS
      u8 s = status_;   // Reading PPUSTATUS will return the current state of this flag
      status_ &= 0x7F;  // and then clear it
      w_ = false;       // Reading this register has the side effect of clearing the PPU's internal w register
      return s;
    }
    case 3:  // OAMADDR - write-only
      return 0;
    case 4:  // OAMDATA
      return oam_[oamaddr_];
    case 5:  // PPUSCROLL - write-only
      return 0;
    case 6:  // PPUADDR - write-only
      return 0;
    case 7: {  // PPUDATA (buffered read)
      u8 value = data_;               // returns the contents of an internal read buffer
      data_ = read(v_);               // read buffer is updated on every PPUDATA read with the value of VRAM at v
      v_ += (ctrl_ & 0x04) ? 32 : 1;  // PPUCTRL bit 2 determines increment (0: add 1, going across; 1: add 32, going down)
      return value;
    }
    default:
      return 0;
  }
}

// write to PPU registers
void PPU::cpu_write(u16 addr, u8 value) {
  const u8 r = addr & 7;
  switch (r) {
    case 0:  // PPUCTRL
      ctrl_ = value;
      t_ = (t_ & 0xF3FF) | (static_cast<u16>(value & 0x03) << 10);  // bits 0 and 1: nametable X & Y (bits 10-11 of t)
      break;
    case 1:  // PPUMASK
      mask_ = value;
      break;
    case 2:  // PPUSTATUS - read-only
      break;
    case 3:  // OAMADDR
      oamaddr_ = value;
      break;
    case 4:  // OAMDATA
      oam_[oamaddr_++] = value;  // Writes will increment OAMADDR after the write
      break;
    case 5:  // PPUSCROLL
      if (!w_) {  // first write: X scroll
        t_ = (t_ & 0xFFE0) | (static_cast<u16>(value) >> 3);  // bits 3-7: coarse X (bits 0-4 of t)
        x_ = value & 0x07;                                       // bits 0-2: fine X (bits 0-2 of x)
        w_ = true;                                            // set write toggle to true
      } else {   // second write: Y scroll
        t_ = (t_ & 0x8C1F) | (static_cast<u16>(value & 0x07) << 12);  // bits 0-2: fine Y (bits 12-14 of t)
        t_ = (t_ & 0xFBE0) | (static_cast<u16>(value & 0xF8) << 2);   // bits 3-7: coarse Y (bits 5-9 of t)
        w_ = false;                                                   // set write toggle to false
      }
      break;
    case 6:  // PPUADDR
      if (!w_) {  // first write:
        t_ = (t_ & 0x00FF) | (static_cast<u16>(value & 0x3F) << 8);  // high bits 8-13 of t
        w_ = true;                                                   // set write toggle to true
      } else {
        t_ = (t_ & 0xFF00) | value;  // low bits 0-7 of t
        v_ = t_;                     // transfer to v
        w_ = false;                  // set write toggle to false
      }
      break;
    case 7:  // PPUDATA
      write(v_, value);               // write value to VRAM at v
      v_ += (ctrl_ & 0x04) ? 32 : 1;  // PPUCTRL bit 2 determines increment (0: add 1, going across; 1: add 32, going down)
      break;
  }
}

// run n PPU cycles (3 per CPU cycle)
void PPU::run_cycles(int n) {
  for (int i = 0; i < n; ++i)
    tick();
}

// run one PPU cycle
void PPU::tick() {
  if (scanline_ == kVblankScanline && cycle_ == 1) {  // start of vblank (scanline 241, dot 1)
    status_ |= 0x80;  // update vblank bit in PPUSTATUS
    if (ctrl_ & 0x80) {  // if Vblank NMI enabled (PPUCTRL bit 7)
      if (on_nmi_)
        on_nmi_();       // call NMI handler if set
    }
  }

  if (cycle_ == 0 && scanline_ >= 0 && scanline_ < fb_height) {
    render_scanline();  // start new scanline
  }
  ++cycle_;
  if (cycle_ >= kCyclesPerScanline) {  // end of scanline
    cycle_ = 0;
    scanline_++;
    if (scanline_ >= kScanlinesPerFrame) {  // end of frame
      scanline_ = 0;
      frame_ready_ = true;
    }  // new frame is ready to display
  }
}

void PPU::render_scanline() {
  const int sy = scanline_;
  // read t for scroll base for the frame
  const u16 coarse_x = t_ & 0x1F;        // bits 0-4
  const u16 coarse_y = (t_ >> 5) & 0x1F; // bits 5-9
  const u16 nt_x = (t_ >> 10) & 0x01;    // bit  10
  const u16 nt_y = (t_ >> 11) & 0x01;    // bit  11
  const u16 fine_y = (t_ >> 12) & 0x07;  // bits 12-14
  const int chr_base = (ctrl_ & 0x10) == 0 ? 0 : 0x1000;  // background pattern table address (0: $0000; 1: $1000)
  // pointer to the start of pixel row for current scanline
  u8* row = framebuffer_.data() + static_cast<std::size_t>(sy * fb_width * fb_bytes_per_pixel);
  // render each pixel in the scanline starting from leftmost
  for (int sx = 0; sx < fb_width; ++sx) {
    const int world_x = (coarse_x * 8) + x_ + sx;      // world pixel column (0-255)
    const int world_y = (coarse_y * 8) + fine_y + sy;  // world pixel row (0-239)
    const int tx = (world_x / 8) & 0x1F;  // tile column within nametable (0-31)
    const int ty = (world_y / 8) % 30;    // tile row within nametable (0-29)
    const int atx = tx / 4;               // attribute table column within tile (0-7)
    const int aty = ty / 4;               // attribute table row within tile (0-7)
    const int px = world_x & 0x07;        // pixel column within tile (0-7)
    const int py = world_y & 0x07;        // pixel row within tile (0-7)
    // determine VRAM address offset to base row of current nametable (0 or 1024)
    const int nt_base_row = 1024 * (mirror_vertical_ ?
              (nt_x + (world_x / 256)) & 0x01 :  // vertical mirror: nametable X (0 or 1)
              (nt_y + (world_y / 240)) & 0x01);  // horizontal mirror: nametable Y (0 or 1)

    const u8 tile_index = vram_[nt_base_row + ty * 32 + tx];  // tile index (0-255) within nametable
    const u8 attr = vram_[nt_base_row + 0x3C0 + aty * 8 + atx];  // attribute byte within attribute table
    const int shift = ((ty / 2) & 0x01) * 4 + ((tx / 2) & 0x01) * 2;  // shift for palette index
    const u8 pal_bits = (attr >> shift) & 0x03;  // palette index (0-3)

    u8 color_index = 0;
    if (chr_ && chr_size_ >= 8192) {  // if CHR ROM is present
      const int tile_off = (chr_base + tile_index * 16);
      const u8 lo = chr_[tile_off + py];  // low byte of tile data
      const u8 hi = chr_[tile_off + py + 8];  // high byte of tile data
      const u8 bit = 7 - px;  // bit index within tile (0-7)
      color_index = ((lo >> bit) & 0x01) | (((hi >> bit) & 0x01) << 1);  // color index (0-3)
    }
    const u8 pal_idx = (pal_bits << 2) | color_index;  // palette index (0-15)
    const u8 ppu_color = nes_palette(pal_idx);  // color index (0-63)
    const u8* rgb = kPalette[ppu_color & 0x3F];  // RGB color from NES palette
    row[sx * 3 + 0] = rgb[0];  // red
    row[sx * 3 + 1] = rgb[1];  // green
    row[sx * 3 + 2] = rgb[2];  // blue
  }
}

u8 PPU::read(u16 addr) const {
  addr &= 0x3FFF;
  if (addr < 0x2000) {
    if (chr_ && chr_size_ > 0)
      return chr_[addr % chr_size_];
    return 0;
  }
  if (addr >= 0x3F00)
    return palette_read(addr);
  const int nt = mirror_vertical_ ? ((addr >> 10) & 0x01) : ((addr >> 11) & 0x01);
  const int off = addr & 0x3FF;
  return vram_[nt * 1024 + off];
}

void PPU::write(u16 addr, u8 value) {
  addr &= 0x3FFF;
  if (addr < 0x2000) {  // CHR ROM: no-op for NROM
    return;
  }
  if (addr >= 0x3F00) {  // palette RAM
    palette_write(addr, value);
    return;
  }
  const int nt = mirror_vertical_ ? ((addr >> 10) & 0x01) : ((addr >> 11) & 0x01);  // nametable select Y vs X
  const int off = addr & 0x3FF;  // offset into nametable
  vram_[nt * 1024 + off] = value;
}

// read from palette RAM
u8 PPU::palette_read(u16 addr) const {
  addr &= 0x1F;  // mask to 5 bits
  if (addr >= 16 && (addr & 0x03) == 0)  // account for mirrored bg bits
    addr -= 16;
  return palette_ram_[addr];
}

// read from palette RAM
void PPU::palette_write(u16 addr, u8 value) {
  addr &= 0x1F;  // mask to 5 bits
  if (addr >= 16 && (addr & 0x03) == 0)  // account for mirrored bg bits
    addr -= 16;
  palette_ram_[addr] = value;
}

u8 PPU::nes_palette(u8 index) const {
  // First color of each palette (0, 4, 8, 12) all use backdrop $3F00
  if ((index & 0x03) == 0)
    return palette_ram_[0] & 0x3F;
  return palette_ram_[index & 0x1F] & 0x3F;
}

}  // namespace nes
