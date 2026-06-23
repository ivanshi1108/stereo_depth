#pragma once

// Host-side startup guards:
//   * single-instance lock  - the Axera card cannot be safely driven by two
//     processes at once (the second one deadlocks), so refuse to start when
//     another instance already holds the lock.
//   * AXCL driver precheck   - verify the axcl_host kernel driver is loaded and
//     that `axcl-smi` runs, before touching the card.

#include <string>

namespace stereo_depth::host_backend {

// Acquire a process-wide single-instance lock. Returns true on success (the lock
// is held for the lifetime of the process). On failure, `reason` explains why.
bool acquireSingleInstanceLock(std::string& reason);

// Verify the AXCL host driver is loaded (axcl_host module / /dev/axcl_host) and
// that `axcl-smi` executes successfully. Returns true when the card is usable.
bool checkAxclHostReady(std::string& reason);

}  // namespace stereo_depth::host_backend
