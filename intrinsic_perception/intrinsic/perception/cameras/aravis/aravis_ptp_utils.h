// Copyright 2026 Intrinsic Innovation LLC
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     https://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef INTRINSIC_PERCEPTION_CAMERAS_ARAVIS_ARAVIS_PTP_UTILS_H_
#define INTRINSIC_PERCEPTION_CAMERAS_ARAVIS_ARAVIS_PTP_UTILS_H_

#include <arv.h>

#include "absl/base/nullability.h"
#include "absl/status/status.h"
#include "absl/time/time.h"

namespace intrinsic {
namespace perception {

inline constexpr absl::Duration kDefaultPtpLockTimeout = absl::Seconds(1);

// Configures PTP synchronization if enabled on the camera (via GevIEEE1588 or
// PtpEnable settings), or falls back to configuring the free running trigger.
absl::Status ConfigurePtpSyncAndStreamingTrigger(
    ArvCamera* absl_nonnull camera,
    absl::Duration ptp_lock_timeout = kDefaultPtpLockTimeout);

}  // namespace perception
}  // namespace intrinsic

#endif  // INTRINSIC_PERCEPTION_CAMERAS_ARAVIS_ARAVIS_PTP_UTILS_H_
