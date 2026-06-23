#include "ImageProcBackend.hpp"

#include <atomic>

#include "AxclImageProc.hpp"

#define SAMPLE_LOG_TAG "IMGPROC"
#include "sample_log.h"

namespace stereo_depth {

namespace {
std::atomic<int> g_backend{static_cast<int>(Backend::Auto)};
std::atomic<bool> g_axclReady{false};
}  // namespace

void setImageProcBackend(Backend backend) {
    g_backend.store(static_cast<int>(backend));

    if (backend == Backend::Host) {
        // Forced host: run everything on the host CPU (OpenCV).
        g_axclReady.store(false);
        ALOGN("image processing backend: host (host CPU / OpenCV)");
        return;
    }

    // Auto and Axcl both prefer the AXCL card. Bring up IVPS now so the first
    // frame does not pay the init cost and so we can report availability up
    // front. On failure we transparently fall back to the host CPU.
    const bool ok = host_backend::axclImageProcInit();
    g_axclReady.store(ok);
    if (ok) {
        ALOGN("image processing backend: %s -> axcl (AXCL IVPS on the card)", backendName(backend));
    } else if (backend == Backend::Axcl) {
        ALOGW(
            "image processing backend: axcl requested but unavailable; falling back to host CPU "
            "(OpenCV)");
    } else {
        ALOGN("image processing backend: auto -> host CPU / OpenCV (AXCL IVPS unavailable)");
    }
}

Backend imageProcBackend() { return static_cast<Backend>(g_backend.load()); }

void shutdownImageProcBackend() {
    if (g_axclReady.load()) {
        host_backend::axclImageProcShutdown();
        g_axclReady.store(false);
    }
}

bool imageProcUsesAxcl() {
    // AXCL is used whenever it was successfully brought up, which happens for
    // both the explicit "axcl" backend and the default "auto" backend.
    return g_axclReady.load();
}

}  // namespace stereo_depth
