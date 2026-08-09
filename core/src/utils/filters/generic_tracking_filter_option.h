/**
 * @file generic_tracking_filter_option.h
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_UTILS_FILTERS_GENERIC_TRACKING_FILTER_OPTION_H_
#define EVERGREENSLAM_UTILS_FILTERS_GENERIC_TRACKING_FILTER_OPTION_H_

#include <yaml-cpp/yaml.h>

#include <ostream>

namespace evergreenslam::utils::filters {

// Singer manoeuvre model: 1/alpha is how long an acceleration stays correlated, sigma is its
// standard deviation. Both alphas must be strictly positive, the constructor CHECKs it.
struct GenericTrackingFilterOption {
  double trans_alpha = 1.0;  // 1/s, a robot spends ~1 s getting up to speed or stopping
  double rot_alpha = 2.0;    // 1/s, turns start and stop faster than that
  double trans_sigma = 5.0;  // m/s^2
  double rot_sigma = 10.0;   // rad/s^2

  double measurement_translation_sigma = 0.02;  // m
  double measurement_rotation_sigma = 0.01;     // rad

  friend std::ostream& operator<<(std::ostream& os, const GenericTrackingFilterOption& option) {
    os << "GenericTrackingFilterOption:" << std::endl;
    os << "  trans_alpha: " << option.trans_alpha << std::endl;
    os << "  rot_alpha: " << option.rot_alpha << std::endl;
    os << "  trans_sigma: " << option.trans_sigma << std::endl;
    os << "  rot_sigma: " << option.rot_sigma << std::endl;
    os << "  measurement_translation_sigma: " << option.measurement_translation_sigma << std::endl;
    os << "  measurement_rotation_sigma: " << option.measurement_rotation_sigma << std::endl;
    return os;
  }
};

GenericTrackingFilterOption LoadGenericTrackingFilterOption(const YAML::Node& node);

}  // namespace evergreenslam::utils::filters

#endif  // EVERGREENSLAM_UTILS_FILTERS_GENERIC_TRACKING_FILTER_OPTION_H_
