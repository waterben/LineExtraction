//*****************************************************************************************
/// \copyright (c) 2016-2026 Benjamin Wassermann
// ---------------------------------------------------------------------------------------
// This file is part of LineExtraction and is licensed under the MIT License.
// See the LICENSE file at the project root for more information.
//*****************************************************************************************
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
