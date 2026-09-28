/**
 * @file grid_overlay.h
 * @author hang chen (chen@hang.plus)
 * @brief Metric grid lines with labels and a scale bar over a rendered view.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_GRID_OVERLAY_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_GRID_OVERLAY_H_

#include "service/render/canvas.h"
#include "service/render/view_frame.h"

namespace evergreenslam::agent {

// Lines every 1/2/5 m (then 10/20/50 ...), the smallest step at least 18 px apart; labels on
// multiples of 5 m spaced for legibility. Pixels of colour `keep` (walls) stay untouched.
void DrawGridLines(const ViewFrame& frame, Rgb keep, Canvas& canvas);

// Bottom-left, a 1/2/5 length of at most a quarter of the width.
void DrawScaleBar(const ViewFrame& frame, Canvas& canvas);

}  // namespace evergreenslam::agent

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_RENDER_GRID_OVERLAY_H_
