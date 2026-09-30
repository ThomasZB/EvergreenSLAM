/**
 * @file gray_png.cc
 * @author hang chen (chen@hang.plus)
 * @brief In-memory PNG encoding of an 8-bit grayscale raster.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "service/graph/gray_png.h"

#include <cstddef>

// Static: the renderer compiles stb as well, and the two must not clash at link time.
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

std::vector<uint8_t> EncodeGrayPng(const std::vector<uint8_t>& pixels, int width, int height) {
  if (width <= 0 || height <= 0 ||
      pixels.size() != static_cast<size_t>(width) * static_cast<size_t>(height)) {
    return {};
  }
  std::vector<uint8_t> png;
  if (stbi_write_png_to_func(&AppendBytes, &png, width, height, 1, pixels.data(), width) == 0) {
    return {};
  }
  return png;
}

}  // namespace evergreenslam::agent
