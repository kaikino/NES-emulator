# NES Emulator

A C++ NES emulator using SDL2 for video and audio. Supports iNES (.nes) ROMs that use mapper 0 (NROM).

## Requirements

- CMake 3.10+
- C++17
- SDL2

## Build

```bash
mkdir build && cd build
cmake ..
make run
```

## Usage

```bash
./run <rom.nes>
```

Example: `./run "Super Mario Bros.nes"`

Close the window or quit the application to exit.

## Controls

### Player 1


| Button | Key             |
| ------ | --------------- |
| D-pad  | `←` `↑` `↓` `→` |
| A      | `M`             |
| B      | `N`             |
| Select | `RSHIFT`        |
| Start  | `RETURN`        |


### Player 2


| Button | Key             |
| ------ | --------------- |
| D-pad  | `W` `A` `S` `D` |
| A      | `V`             |
| B      | `C`             |
| Select | `LSHIFT`        |
| Start  | `TAB`           |


---

## Implementation

Main loop: run one CPU instruction, advance PPU by 3× and APU by the same number of cycles, then check NMI and OAM DMA (each adds cycles). This keeps CPU, PPU, and APU in lockstep.

### CPU

- 6502 legal opcodes and addressing modes; NMI and IRQ with cycle cost
- OAM DMA on $4014 write (513 cycles + 1 on odd cycle; handled in bus, cycles applied in main loop)

### PPU

- Background and sprites (8×8 / 8×16, priority, flip); sprite 0 hit; scrolling with T/V and fine X
- Nametable mirroring from iNES; VBlank NMI; scroll copies at dot 257 and pre-render (280–304)
- One 256×240 RGB framebuffer; full scanline rendered at cycle 0 per scanline

### APU + I/O

- Pulse 1 & 2, triangle, noise; frame counter (4-step / 5-step); envelope, length, sweep, linear counter
- Mixer: pulse + TND group; output buffered and fed to SDL at 44.1 kHz
- Controllers via $4016/$4017 (strobe and shift). No DMC or frame counter IRQ.

### Bus

- NES map: 2 KB RAM, PPU $2000–$3FFF, APU/I/O $4000–$4017, PRG RAM $6000–$7FFF, PRG ROM $8000–$FFFF
- OAM DMA copies 256 bytes to PPU OAM and sets pending flag. Legacy 64K flat mode for Klaus test.

### Cartridge

- iNES: PRG/CHR size, mapper, mirroring. NROM (mapper 0) only.

