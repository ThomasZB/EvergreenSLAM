/**
 * @file generic_tracking_filter_option.cc
 * @author hang chen (chen@hang.plus)
 * @brief
 * @version 0.1
 * @date 2026-08-03
 *
 * @copyright Copyright (c) 2026
 *
 */

#include "utils/filters/generic_tracking_filter_option.h"

#include "utils/config/yaml_utils.h"

namespace evergreenslam::utils::filters {

using config::LoadOption;

GenericTrackingFilterOption LoadGenericTrackingFilterOption(const YAML::Node& node) {
  GenericTrackingFilterOption option;
  option.trans_alpha = LoadOption(node, "trans_alpha", option.trans_alpha);
  option.rot_alpha = LoadOption(node, "rot_alpha", option.rot_alpha);
  option.trans_sigma = LoadOption(node, "trans_sigma", option.trans_sigma);
  option.rot_sigma = LoadOption(node, "rot_sigma", option.rot_sigma);
  option.measurement_translation_sigma =
      LoadOption(node, "measurement_translation_sigma", option.measurement_translation_sigma);
  option.measurement_rotation_sigma =
      LoadOption(node, "measurement_rotation_sigma", option.measurement_rotation_sigma);
  return option;
}

}  // namespace evergreenslam::utils::filters
