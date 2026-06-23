#pragma once

// Host-side AXCL runtime manager: owns the one-time runtime bring-up and the
// shared device context, reference counted across subsystems.

#include <cstdint>

namespace stereo_depth::host_backend {

class AxclRuntime {
public:
    static AxclRuntime& instance();

    // Bring the AXCL runtime up (only the first call does real work).
    bool acquire();

    // Drop an acquire() reference; finalizes the runtime on the last release.
    void release();

    // Bind the shared device context to the calling thread (AXCL keeps the
    // context thread-local).
    bool ensureThreadContext();

    int32_t deviceId() const { return m_deviceId; }
    bool ready() const { return m_ready; }

    AxclRuntime(const AxclRuntime&) = delete;
    AxclRuntime& operator=(const AxclRuntime&) = delete;

private:
    AxclRuntime() = default;
    ~AxclRuntime();

    bool m_ready = false;
    int m_refCount = 0;
    int32_t m_deviceId = 0;
    void* m_context = nullptr;
    bool m_engineInited = false;
};

}  // namespace stereo_depth::host_backend
