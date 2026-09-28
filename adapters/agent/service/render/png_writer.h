/**
 * @file png_writer.h
 * @author hang chen (chen@hang.plus)
 * @brief In-memory PNG encoding of an 8-bit RGB raster.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_PNG_WRITER_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_PNG_WRITER_H_

#include <cstdint>
#include <vector>

namespace evergreenslam::agent {

// `rgb` holds width * height * 3 bytes, row 0 first. Empty on bad dimensions or encoder failure.
std::vector<uint8_t> EncodePng(const std::vector<uint8_t>& rgb, int width, int height);

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_PNG_WRITER_H_
