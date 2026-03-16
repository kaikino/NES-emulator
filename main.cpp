#include "apu.hpp"
#include "bus.hpp"
#include "cpu.hpp"
#include "ines.hpp"
#include "ppu.hpp"
#include "types.hpp"
#include <cstdio>
#include <cstring>
#include <string>

#include <SDL.h>

int main(int argc, char* argv[]) {
  if (argc < 2) {
    std::fprintf(stderr, "Usage: %s <rom.nes>\n", argv[0]);
    return 1;
  }

  std::string path(argv[1]);
  nes::Bus bus(true);  // NES memory map ($0000-$FFFF)
  nes::InesCart cart;
  if (!nes::load_ines(path, bus, cart)) {  // parse iNES header, load PRG/CHR into bus
    std::fprintf(stderr, "Failed to load iNES ROM: %s\n", path.c_str());
    return 1;
  }
  // only NROM (mapper 0) supported
  if (cart.mapper != 0) {
    std::fprintf(stderr, "Unsupported mapper %d (only NROM/mapper 0)\n", cart.mapper);
    return 1;
  }

  // create PPU with CHR and mirroring; attach to bus for $2000-$3FFF
  nes::PPU ppu(cart.chr.empty() ? nullptr : cart.chr.data(), cart.chr.size(),
              cart.mirror_vertical);
  bus.set_ppu(&ppu);

  // create APU/IO; attach to bus for $4000-$4017
  nes::Apu apu;
  bus.set_apu(&apu);

  // create CPU; set PC from reset vector ($FFFC/$FFFD), init stack and flags
  nes::CPU cpu(bus);
  nes::u16 reset_lo = bus.read(0xFFFC);
  nes::u16 reset_hi = bus.read(0xFFFD);
  cpu.PC = reset_lo | (reset_hi << 8);
  cpu.S = 0xFD;
  cpu.P = nes::flag::U | nes::flag::I;
  if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0) {
    std::fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
    return 1;
  }
  constexpr int scale = 3;
  constexpr int overscan = 8;  // hide top/bottom 8 scanlines (NTSC overscan)
  constexpr int visible_height = nes::PPU::fb_height - overscan * 2;  // 224
  SDL_Window* win = SDL_CreateWindow("NES",
                                    SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
                                    nes::PPU::fb_width * scale, visible_height * scale,
                                    0);
  if (!win) {
    std::fprintf(stderr, "SDL_CreateWindow failed: %s\n", SDL_GetError());
    SDL_Quit();
    return 1;
  }
  SDL_Renderer* ren = SDL_CreateRenderer(win, -1, SDL_RENDERER_ACCELERATED);  // window + renderer for display
  if (!ren) {
    std::fprintf(stderr, "SDL_CreateRenderer failed: %s\n", SDL_GetError());
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 1;
  }

  // streaming texture for PPU framebuffer (256x240 RGB888)
  SDL_Texture* tex = SDL_CreateTexture(ren, SDL_PIXELFORMAT_RGB24,
                                       SDL_TEXTUREACCESS_STREAMING,
                                       nes::PPU::fb_width, nes::PPU::fb_height);
  if (!tex) {
    std::fprintf(stderr, "SDL_CreateTexture failed: %s\n", SDL_GetError());
    SDL_DestroyRenderer(ren);
    SDL_DestroyWindow(win);
    SDL_Quit();
    return 1;
  }

  // open audio device for APU output (44100 Hz, mono, float32, push via SDL_QueueAudio)
  SDL_AudioSpec want{}, have{};
  want.freq = 44100;
  want.format = AUDIO_F32SYS;
  want.channels = 1;
  want.samples = 512;
  want.callback = nullptr;
  SDL_AudioDeviceID audio_dev = SDL_OpenAudioDevice(nullptr, 0, &want, &have, 0);
  if (audio_dev == 0) {
    std::fprintf(stderr, "SDL_OpenAudioDevice failed: %s\n", SDL_GetError());
  } else {
    apu.set_sample_rate(have.freq);
    SDL_PauseAudioDevice(audio_dev, 0);
  }

  std::printf("Loaded %s (mapper %d, PRG %zu bytes, CHR %zu bytes)\n",
              path.c_str(), cart.mapper, cart.prg_size, cart.chr_size);
  std::printf("Reset vector: $%04X\n", cpu.PC);

  // NTSC frame timing (~60.0988 Hz)
  constexpr double target_frame_sec = 1.0 / 60.0988;
  const Uint64 perf_freq = SDL_GetPerformanceFrequency();
  Uint64 frame_start = SDL_GetPerformanceCounter();

  bool quit = false;
  long total_cpu_cycles = 0;
  while (!quit) {
    SDL_Event e;
    while (SDL_PollEvent(&e)) {
      if (e.type == SDL_QUIT)
        quit = true;
    }

    // poll keyboard for controller 1 (arrows + M/N + Rshift/Return)
    const Uint8* keys = SDL_GetKeyboardState(nullptr);
    nes::u8 pad = 0;
    if (keys[SDL_SCANCODE_M])      pad |= nes::Apu::btn_a;
    if (keys[SDL_SCANCODE_N])      pad |= nes::Apu::btn_b;
    if (keys[SDL_SCANCODE_RSHIFT]) pad |= nes::Apu::btn_select;
    if (keys[SDL_SCANCODE_RETURN]) pad |= nes::Apu::btn_start;
    if (keys[SDL_SCANCODE_UP])     pad |= nes::Apu::btn_up;
    if (keys[SDL_SCANCODE_DOWN])   pad |= nes::Apu::btn_down;
    if (keys[SDL_SCANCODE_LEFT])   pad |= nes::Apu::btn_left;
    if (keys[SDL_SCANCODE_RIGHT])  pad |= nes::Apu::btn_right;
    apu.set_button_state(0, pad);

    // poll keyboard for controller 2 (WASD + X/Z + Lshift/Tab)
    nes::u8 pad2 = 0;
    if (keys[SDL_SCANCODE_V])      pad2 |= nes::Apu::btn_a;
    if (keys[SDL_SCANCODE_C])      pad2 |= nes::Apu::btn_b;
    if (keys[SDL_SCANCODE_LSHIFT]) pad2 |= nes::Apu::btn_select;
    if (keys[SDL_SCANCODE_TAB])    pad2 |= nes::Apu::btn_start;
    if (keys[SDL_SCANCODE_W])      pad2 |= nes::Apu::btn_up;
    if (keys[SDL_SCANCODE_S])      pad2 |= nes::Apu::btn_down;
    if (keys[SDL_SCANCODE_A])      pad2 |= nes::Apu::btn_left;
    if (keys[SDL_SCANCODE_D])      pad2 |= nes::Apu::btn_right;
    apu.set_button_state(1, pad2);

    // run one CPU instruction; advance PPU in lockstep
    int cycles = cpu.step();
    if (bus.take_dma_pending())                         // OAM DMA triggered by $4014 write
      cycles += 513 + (total_cpu_cycles & 1);           // 513 cycles (+1 on odd CPU cycle)
    total_cpu_cycles += cycles;
    ppu.run_cycles(3 * cycles);                         // 3 PPU cycles per CPU cycle
    apu.run_cycles(cycles);                             // APU runs at CPU clock

    // NMI is sampled between instructions (PPU may have set it during the ticks above)
    if (ppu.take_nmi_pending()) {
      int nmi_cycles = cpu.nmi();
      total_cpu_cycles += nmi_cycles;
      ppu.run_cycles(3 * nmi_cycles);
      apu.run_cycles(nmi_cycles);
    }

    if (ppu.frame_ready()) {                // new frame finished: upload fb and present
      void* pixels = nullptr;
      int pitch = 0;
      // load framebuffer into texture to display
      if (SDL_LockTexture(tex, nullptr, &pixels, &pitch) == 0) {
        std::memcpy(pixels, ppu.framebuffer(),
                    static_cast<std::size_t>(nes::PPU::fb_width * nes::PPU::fb_height * nes::PPU::fb_bytes_per_pixel));
        SDL_UnlockTexture(tex);
      }
      ppu.clear_frame_ready();
      SDL_Rect src = {0, overscan, nes::PPU::fb_width, visible_height};
      SDL_RenderClear(ren);
      SDL_RenderCopy(ren, tex, &src, nullptr);
      SDL_RenderPresent(ren);

      // drain APU sample buffer and queue for playback
      if (audio_dev != 0) {
        float samples[2048];
        int n;
        while ((n = apu.drain_samples(samples, 2048)) > 0)
          SDL_QueueAudio(audio_dev, samples, static_cast<Uint32>(n * sizeof(float)));
      }

      // wait until next frame boundary (~16.64ms per frame)
      const Uint64 target_ticks = static_cast<Uint64>(target_frame_sec * perf_freq);
      Uint64 elapsed = SDL_GetPerformanceCounter() - frame_start;
      if (elapsed < target_ticks) {
        double remaining = static_cast<double>(target_ticks - elapsed) / perf_freq;
        if (remaining > 0.001)
          SDL_Delay(static_cast<Uint32>(remaining * 1000.0));
        while (SDL_GetPerformanceCounter() - frame_start < target_ticks)
          ;  // spin for sub-millisecond precision
      }
      frame_start = SDL_GetPerformanceCounter();
    }
  }

  if (audio_dev != 0)
    SDL_CloseAudioDevice(audio_dev);
  SDL_DestroyTexture(tex);
  SDL_DestroyRenderer(ren);
  SDL_DestroyWindow(win);
  SDL_Quit();
  return 0;
}
