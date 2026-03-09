#pragma once

#include <cstdint>

namespace nes {

using u8 = std::uint8_t;
using u16 = std::uint16_t;
using s8 = std::int8_t;
using s16 = std::int16_t;

// 6502 status flags bits (P register)
namespace flag {
constexpr u8 N = 0x80;  // Negative
constexpr u8 V = 0x40;  // Overflow
constexpr u8 U = 0x20;  // Unused (always pushed as 1)
constexpr u8 B = 0x10;  // B flag
constexpr u8 D = 0x08;  // Decimal (unused on NES)
constexpr u8 I = 0x04;  // Interrupt disable
constexpr u8 Z = 0x02;  // Zero
constexpr u8 C = 0x01;  // Carry
}  // namespace flag

}  // namespace nes
