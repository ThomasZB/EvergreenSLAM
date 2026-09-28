/**
 * @file httplib_include.h
 * @author hang chen (chen@hang.plus)
 * @brief The only way adapters/agent includes httplib.h: undoes the macro it leaks on Linux.
 * @version 0.1
 * @date 2026-09-24
 *
 * @copyright Copyright (c) 2026
 *
 */

#ifndef EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTPLIB_INCLUDE_H_
#define EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTPLIB_INCLUDE_H_

#include "httplib.h"

// On Linux httplib.h includes <resolv.h>, whose `_res` macro breaks every Eigen header parsed
// after it (Eigen names parameters `_res`), so include order would otherwise matter.
#ifdef _res
#undef _res
#endif

#endif  // EVERGREENSLAM_ADAPTERS_AGENT_SERVICE_HTTPLIB_INCLUDE_H_
