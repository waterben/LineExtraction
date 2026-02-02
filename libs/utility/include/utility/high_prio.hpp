/// @file high_prio.hpp
/// @brief System-level process priority control.

#pragma once

namespace lsfm {

/// @brief Set current process to high priority for benchmarking.
///
/// Increases process scheduling priority to reduce timing jitter
/// during performance measurements. Requires elevated privileges.
void setHighPriority();

}  // namespace lsfm
