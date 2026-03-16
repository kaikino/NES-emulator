#pragma once

#include "types.hpp"
#include <array>
#include <cstddef>

namespace nes {

// manages PPU memory map
// implements PPU operations (PPU timing, framebuffer, rendering)
// owns PPU registers
// owns VRAM (nametables and attribute tables) and palette RAM
// owns OAM
// pointer to CHR (pattern tables) is supplied at construction
class PPU {
 public:
  // chr_data: 8 KB for NROM; mirror_vertical from iNES header.
  PPU(const u8* chr_data, std::size_t chr_size, bool mirror_vertical);
  // PPU registers
  u8 cpu_read(u16 addr);
  void cpu_write(u16 addr, u8 value);

  // Run n PPU cycles. Call from main loop (3 cycles per CPU cycle).
  // Returns true when a new frame is complete
  void run_cycles(int n);

  // ready to display new frame
  bool frame_ready() const { return frame_ready_; }
  void clear_frame_ready() { frame_ready_ = false; }

  // 256x240, RGB888 (3 bytes per pixel), row-major.
  const u8* framebuffer() const { return framebuffer_.data(); }
  static constexpr int fb_width = 256;
  static constexpr int fb_height = 240;
  static constexpr int fb_bytes_per_pixel = 3;

  // check and clear pending NMI (set at VBlank when NMI enabled)
  bool take_nmi_pending() { bool p = nmi_pending_; nmi_pending_ = false; return p; }

  // PPUCTRL   $2000  VPHB SINN            W    NMI enable (V), PPU master/slave (P), sprite height (H), background tile select (B),
  // 7  bit  0                                   sprite tile select (S), increment mode (I), nametable select (NN)
  // ---- ----
  // VPHB SINN
  // |||| ||||
  // |||| ||++- Base nametable address
  // |||| ||    (0 = $2000; 1 = $2400; 2 = $2800; 3 = $2C00)
  // |||| |+--- VRAM address increment per CPU read/write of PPUDATA
  // |||| |     (0: add 1, going across; 1: add 32, going down)
  // |||| +---- Sprite pattern table address for 8x8 sprites
  // ||||       (0: $0000; 1: $1000; ignored in 8x16 mode)
  // |||+------ Background pattern table address (0: $0000; 1: $1000)
  // ||+------- Sprite size (0: 8x8 pixels; 1: 8x16 pixels – see PPU OAM#Byte 1)
  // |+-------- PPU master/slave select
  // |          (0: read backdrop from EXT pins; 1: output color on EXT pins)
  // +--------- Vblank NMI enable (0: off, 1: on)

  // PPUMASK   $2001  BGRs bMmG            W    color emphasis (BGR), sprite enable (s),
  // 7  bit  0                                   background enable (b), sprite left column enable (M), background left column enable (m), greyscale (G)
  // ---- ----
  // BGRs bMmG
  // |||| ||||
  // |||| |||+- Greyscale (0: normal color, 1: greyscale)
  // |||| ||+-- 1: Show background in leftmost 8 pixels of screen, 0: Hide
  // |||| |+--- 1: Show sprites in leftmost 8 pixels of screen, 0: Hide
  // |||| +---- 1: Enable background rendering
  // |||+------ 1: Enable sprite rendering
  // ||+------- Emphasize red (green on PAL/Dendy)
  // |+-------- Emphasize green (red on PAL/Dendy)
  // +--------- Emphasize blueß

  // PPUSTATUS $2002  VSO- ----            R    vblank (V), sprite 0 hit (S), sprite overflow (O); read resets write pair for $2005/$2006
  // 7  bit  0
  // ---- ----
  // VSOx xxxx
  // |||| ||||
  // |||+-++++- (PPU open bus or 2C05 PPU identifier)
  // ||+------- Sprite overflow flag
  // |+-------- Sprite 0 hit flag
  // +--------- Vblank flag, cleared on read. Unreliable; see below.

  // OAMADDR   $2003  AAAA AAAA            W    OAM read/write address
  // 7  bit  0
  // ---- ----
  // AAAA AAAA
  // |||| ||||
  // ++++-++++- OAM address

  // OAMDATA   $2004  DDDD DDDD            RW   OAM data read/write
  // 7  bit  0
  // ---- ----
  // DDDD DDDD
  // |||| ||||
  // ++++-++++- OAM data

  // PPUSCROLL $2005  XXXX XXXX YYYY YYYY  Wx2  X and Y scroll bits 7-0 (two writes: X scroll, then Y scroll)
  // 1st write                                                2nd write
  // 7  bit  0                                                7  bit  0
  // ---- ----                                                ---- ----
  // XXXX XXXX                                                YYYY YYYY
  // |||| ||||                                                |||| ||||
  // ++++-++++- X scroll bits 7-0 (bit 8 in PPUCTRL bit 0)    ++++-++++- Y scroll bits 7-0 (bit 8 in PPUCTRL bit 1)

  // PPUADDR   $2006  ..AA AAAA AAAA AAAA  Wx2  VRAM address (two writes: most significant byte, then least significant byte)
  // 1st write  2nd write
  // 15 bit  8  7  bit  0
  // ---- ----  ---- ----
  // ..AA AAAA  AAAA AAAA
  //   || ||||  |||| ||||
  //   ++-++++--++++-++++- VRAM address

  // PPUDATA   $2007  DDDD DDDD            RW   VRAM data read/write
  // 7  bit  0
  // ---- ----
  // DDDD DDDD
  // |||| ||||
  // ++++-++++- VRAM data

  // OAMDMA    $4014  AAAA AAAA            W    OAM DMA high address
  // 7  bit  0
  // ---- ----
  // AAAA AAAA
  // |||| ||||
  // ++++-++++- Source page (high byte of source address)

 private:
  void tick();
  void render_scanline();

  // PPU memory operations
  u8 read(u16 addr) const;
  void write(u16 addr, u8 value);

  // palette RAM
  u8 palette_read(u16 addr) const;
  void palette_write(u16 addr, u8 value);
  // palette slot -> color index for rendering
  u8 nes_palette(u8 index) const;

  // pointer to CHR data
  const u8* chr_;
  std::size_t chr_size_;
  // Address range  Bit 11 Bit 10 (of PPU memory address)
  // 2000–23FF        0      0         ---| mirrored (vertical)    ---|
  // 2400–27FF        1      0         ---|                           |  mirrored (horizontal)
  // 2800–2BFF        0      1                                     ---|
  // 2C00–2FFF        1      1
  bool mirror_vertical_;

  bool nmi_pending_{false};  // set at VBlank start when NMI enabled

  // PPU timing
  int cycle_{0};    // 0-339 (340 cycles per scanline)
  int scanline_{0}; // 0-261 (262 scanlines per frame)
  bool frame_ready_{false}; // true when a new frame is complete
  // output
  std::array<u8, fb_width * fb_height * fb_bytes_per_pixel> framebuffer_{};  // output image (256x240, RGB888 (3 bytes per pixel), row-major)

  // PPU memory
  std::array<u8, 2048> vram_{};  // nametables (2 KB)
  u8 palette_ram_[32]{};         // palette RAM (32 bytes)

  // PPU registers
  u8 ctrl_{0};     // PPUCTRL
  u8 mask_{0};     // PPUMASK
  u8 status_{0};   // PPUSTATUS
  u8 oamaddr_{0};  // OAMADDR
  u8 oam_[256]{};  // internal OAM (256 bytes) for OAMDATA
  u8 data_{0};     // PPUDATA
  // internal registers for PPUSCROLL and PPUADDR
  bool w_{false};  // write toggle (true for first write, false for second write)
  u16 t_{0};       // temp for scroll/addr
  u16 v_{0};       // current VRAM address
  u8 x_{0};        // temp for scroll fine X

  // PPU memory map
  // Address range  Size   Description             Mapped by
  // $0000-$0FFF    $1000  Pattern table 0         Cartridge
  // $1000-$1FFF    $1000  Pattern table 1         Cartridge
  // $2000-$23BF    $03c0  Nametable 0             Cartridge
  //  $23C0-$23FF    $0040  Attribute table 0       Cartridge
  // $2400-$27BF    $03c0  Nametable 1             Cartridge
  //  $27C0-$27FF    $0040  Attribute table 1       Cartridge
  // $2800-$2BBF    $03c0  Nametable 2             Cartridge
  //  $2BC0-$2BFF    $0040  Attribute table 2       Cartridge
  // $2C00-$2FBF    $03c0  Nametable 3             Cartridge
  //  $2FC0-$2FFF    $0040  Attribute table 3       Cartridge
  // $3000-$3EFF    $0F00  Unused                  Cartridge
  // $3F00-$3F1F    $0020  Palette RAM indexes     Internal to PPU
  // $3F20-$3FFF    $00E0  Mirrors of $3F00-$3F1F  Internal to PPU

};

}  // namespace nes
