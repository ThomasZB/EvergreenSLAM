/**
 * @file view_frame.h
 * @author hang chen (chen@hang.plus)
 * @brief World <-> pixel mapping of one north-up rendered view.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_VIEW_FRAME_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_VIEW_FRAME_H_

#include <Eigen/Core>

namespace evergreenslam::agent {

// Row 0 is the top (max y), as in map.json: col = floor((x - min_x) / mpp),
// row = floor((max_y - y) / mpp).
struct ViewFrame {
  double min_x = 0.0;
  double max_y = 0.0;
  double metres_per_pixel = 0.05;
  int width = 0;
  int height = 0;

  // Clamped far outside the image, so callers never overflow.
  Eigen::Array2i ToPixel(const Eigen::Vector2d& xy) const;
  Eigen::Vector2d PixelCenter(int col, int row) const;
};

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_VIEW_FRAME_H_
