#pragma once

// Image-processing backend selection (host CPU / OpenCV vs. AXCL IVPS on the
// card), consulted by the sample_image_* helpers. Auto/Axcl prefer the card and
// fall back to the host per-op; Host forces the CPU path.

#include "BackendSelect.hpp"

namespace stereo_depth {

void setImageProcBackend(Backend backend);
void shutdownImageProcBackend();
Backend imageProcBackend();

// True when AXCL IVPS image processing is active.
bool imageProcUsesAxcl();

}  // namespace stereo_depth
