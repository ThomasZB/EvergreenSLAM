/**
 * @file bitmap_font.h
 * @author hang chen (chen@hang.plus)
 * @brief 5x7 bitmap font: digits, uppercase, lowercase m and a few symbols.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_BITMAP_FONT_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_BITMAP_FONT_H_

#include <array>
#include <cstdint>
#include <string>

namespace evergreenslam::agent {

constexpr int kGlyphWidth = 5;
constexpr int kGlyphHeight = 7;

// Rows top first, bit 4 is the leftmost column. Lowercase other than 'm' falls back to uppercase;
// characters without a glyph are blank.
using GlyphRows = std::array<uint8_t, kGlyphHeight>;
GlyphRows Glyph(char c);

// One blank column between glyphs, none after the last.
int TextWidth(const std::string& text, int scale);
int TextHeight(int scale);

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_BITMAP_FONT_H_
