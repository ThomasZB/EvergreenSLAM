/**
 * @file png_writer.cc
 * @author hang chen (chen@hang.plus)
 * @brief In-memory PNG encoding of an 8-bit RGB raster.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/render/png_writer.h"

#include <cstddef>

// Static: another translation unit may compile stb as well without clashing symbols.
#define STB_IMAGE_WRITE_IMPLEMENTATION
#define STB_IMAGE_WRITE_STATIC
#define STBI_WRITE_NO_STDIO
#include "stb_image_write.h"

namespace evergreenslam::agent {
namespace {

void AppendBytes(void* context, void* data, int size) {
  auto& out = *static_cast<std::vector<uint8_t>*>(context);
  const auto* bytes = static_cast<const uint8_t*>(data);
  out.insert(out.end(), bytes, bytes + size);
}

}  // namespace

std::vector<uint8_t> EncodePng(const std::vector<uint8_t>& rgb, int width, int height) {
  constexpr int kChannels = 3;
  if (width <= 0 || height <= 0 ||
      rgb.size() != static_cast<size_t>(width) * static_cast<size_t>(height) * kChannels) {
    return {};
  }
  std::vector<uint8_t> png;
  if (stbi_write_png_to_func(&AppendBytes, &png, width, height, kChannels, rgb.data(),
                             width * kChannels) == 0) {
    return {};
  }
  return png;
}

}  // namespace evergreenslam::agent
