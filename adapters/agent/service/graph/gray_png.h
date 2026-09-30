/**
 * @file gray_png.h
 * @author hang chen (chen@hang.plus)
 * @brief In-memory PNG encoding of an 8-bit grayscale raster.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_GRAY_PNG_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_GRAY_PNG_H_

#include <cstdint>
#include <vector>

namespace evergreenslam::agent {

// Row-major, top row first. Empty on a size mismatch or an encoder failure.
std::vector<uint8_t> EncodeGrayPng(const std::vector<uint8_t>& pixels, int width, int height);

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_GRAPH_GRAY_PNG_H_
