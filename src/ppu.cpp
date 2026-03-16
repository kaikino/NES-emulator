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
        t_ = (t_ & 0x8C1F) | (static_cast<u16>(value & 0x07) << 12) | (static_cast<u16>(value & 0xF8) << 2);
        w_ = false;
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
  const bool rendering = (mask_ & 0x18) != 0;  // bg or sprites enabled

  // pre-render scanline (261): clear VBL, sprite 0 hit, sprite overflow at dot 1
  if (scanline_ == 261 && cycle_ == 1)
    status_ &= ~0xE0;

  // pre-render scanline: copy vertical scroll bits from T to V (dots 280-304)
  if (scanline_ == 261 && cycle_ >= 280 && cycle_ <= 304 && rendering)
    v_ = (v_ & ~0x7BE0) | (t_ & 0x7BE0);

  if (scanline_ == kVblankScanline && cycle_ == 1) {  // start of vblank (scanline 241, dot 1)
    status_ |= 0x80;  // set vblank flag in PPUSTATUS
    if (ctrl_ & 0x80)    // if Vblank NMI enabled (PPUCTRL bit 7)
      nmi_pending_ = true;
  }

  if (cycle_ == 0 && scanline_ >= 0 && scanline_ < fb_height)
    render_scanline();

  // copy horizontal scroll bits from T to V at dot 257 (visible and pre-render so scanline 0 gets correct scroll)
  if (cycle_ == 257 && rendering && (scanline_ < fb_height || scanline_ == 261))
    v_ = (v_ & ~0x041F) | (t_ & 0x041F);

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
  // pointer to the start of pixel row for current scanline
  u8* row = framebuffer_.data() + static_cast<std::size_t>(sy * fb_width * fb_bytes_per_pixel);
  bool bg_opaque[fb_width]{};  // track opaque bg pixels for sprite priority / sprite 0 hit

  // --- background ---
  if (mask_ & 0x08) {  // PPUMASK bit 3: background enable
    // read scroll state from v (maintained by tick() via T→V copies)
    const u16 coarse_x = v_ & 0x1F;        // bits 0-4
    const u16 coarse_y = (v_ >> 5) & 0x1F; // bits 5-9
    const u16 nt_x = (v_ >> 10) & 0x01;    // bit  10
    const u16 nt_y = (v_ >> 11) & 0x01;    // bit  11
    const u16 fine_y = (v_ >> 12) & 0x07;  // bits 12-14
    const int chr_base = (ctrl_ & 0x10) ? 0x1000 : 0;  // PPUCTRL bit 4: bg pattern table address (0: $0000; 1: $1000)

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
      if (chr_ && chr_size_ > 0) {  // if CHR ROM is present
        const int tile_off = chr_base + tile_index * 0x10;
        const u8 lo = chr_[tile_off % chr_size_ + py];        // low byte of tile row
        const u8 hi = chr_[(tile_off + 0x08) % chr_size_ + py];  // high byte of tile row
        const u8 bit = 7 - px;                                 // bit index within tile row
        color_index = ((lo >> bit) & 0x01) | (((hi >> bit) & 0x01) << 1);  // 2-bit color (0-3)
      }

      bg_opaque[sx] = (color_index != 0);
      const u8 pal_idx = (pal_bits << 2) | color_index;   // palette index (0-15)
      const u8 ppu_color = nes_palette(pal_idx);           // NES color (0-63)
      const u8* rgb = kPalette[ppu_color & 0x3F];          // RGB from NES palette LUT
      row[sx * 3 + 0] = rgb[0];  // red
      row[sx * 3 + 1] = rgb[1];  // green
      row[sx * 3 + 2] = rgb[2];  // blue
    }
  } else {
    // background disabled: fill with backdrop color
    const u8 ppu_color = palette_ram_[0] & 0x3F;
    const u8* rgb = kPalette[ppu_color];
    for (int sx = 0; sx < fb_width; ++sx) {
      row[sx * 3 + 0] = rgb[0];
      row[sx * 3 + 1] = rgb[1];
      row[sx * 3 + 2] = rgb[2];
    }
  }

  // --- sprites ---
  if (!(mask_ & 0x10))  // PPUMASK bit 4: sprite enable
    return;
  if (!chr_ || chr_size_ == 0)
    return;

  const int spr_height = (ctrl_ & 0x20) ? 16 : 8;        // PPUCTRL bit 5: sprite size (0: 8x8; 1: 8x16)
  const int spr_chr_base = (ctrl_ & 0x08) ? 0x1000 : 0;  // PPUCTRL bit 3: sprite pattern table (8x8 only)

  // collect up to 8 sprites that overlap this scanline (OAM order = priority order)
  // OAM: 64 sprites, 4 bytes each: [Y, tile, attr, X]
  int spr_idx[8];
  int spr_count = 0;
  for (int i = 0; i < 64 && spr_count < 8; ++i) {
    const int spr_y = oam_[i * 4];                  // byte 0: Y position (sprite appears at Y+1)
    const int row_in_spr = sy - (spr_y + 1);        // row within sprite (0 to spr_height-1)
    if (row_in_spr >= 0 && row_in_spr < spr_height)
      spr_idx[spr_count++] = i;
  }

  // render in reverse order so lower OAM index (higher priority) is drawn last
  for (int s = spr_count - 1; s >= 0; --s) {
    const int i = spr_idx[s];
    const u8 spr_y  = oam_[i * 4 + 0];              // byte 0: Y position
    const u8 tile   = oam_[i * 4 + 1];              // byte 1: tile index
    const u8 attr   = oam_[i * 4 + 2];              // byte 2: attributes
    const u8 spr_x  = oam_[i * 4 + 3];              // byte 3: X position
    const u8 spr_pal = (attr & 0x03) + 4;           // attr bits 0-1: palette (4-7 for sprites)
    const bool behind_bg = (attr & 0x20) != 0;      // attr bit 5: priority (0: front; 1: behind bg)
    const bool flip_h    = (attr & 0x40) != 0;      // attr bit 6: flip horizontally
    const bool flip_v    = (attr & 0x80) != 0;      // attr bit 7: flip vertically

    int py = sy - (spr_y + 1);                      // row within sprite
    if (flip_v) py = spr_height - 1 - py;

    // compute CHR address for this sprite row
    int tile_addr;
    if (spr_height == 16) {
      // 8x16: tile bit 0 selects pattern table ($0000 or $1000), tile & $FE is top tile
      const int bank = (tile & 0x01) ? 0x1000 : 0;
      int tile_num = tile & 0xFE;
      if (py >= 8) { tile_num++; py -= 8; }         // bottom half of 8x16 sprite
      tile_addr = bank + tile_num * 0x10 + py;
    } else {
      // 8x8: use PPUCTRL bit 3 for pattern table
      tile_addr = spr_chr_base + tile * 0x10 + py;
    }

    const u8 lo = chr_[tile_addr % chr_size_];              // low byte of tile row
    const u8 hi = chr_[(tile_addr + 0x08) % chr_size_];     // high byte of tile row

    for (int bit = 0; bit < 8; ++bit) {
      const int sx = spr_x + bit;
      if (sx >= fb_width) break;

      const int shift = flip_h ? bit : (7 - bit);           // flip selects bit order
      const u8 color_idx = ((lo >> shift) & 0x01) | (((hi >> shift) & 0x01) << 1);  // 2-bit color
      if (color_idx == 0) continue;                          // transparent pixel

      // sprite 0 hit: opaque sprite 0 pixel overlaps opaque bg pixel (not at x=255)
      if (i == 0 && bg_opaque[sx] && sx < 255)
        status_ |= 0x40;

      if (behind_bg && bg_opaque[sx]) continue;             // behind opaque background

      const u8 pal_idx = (spr_pal << 2) | color_idx;       // sprite palette index
      const u8 ppu_color = nes_palette(pal_idx);            // NES color (0-63)
      const u8* rgb = kPalette[ppu_color & 0x3F];           // RGB from NES palette LUT
      row[sx * 3 + 0] = rgb[0];  // red
      row[sx * 3 + 1] = rgb[1];  // green
      row[sx * 3 + 2] = rgb[2];  // blue
    }
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
