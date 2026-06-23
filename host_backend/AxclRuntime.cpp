#include "AxclRuntime.hpp"

#include <axcl.h>

#include <cstdlib>
#include <filesystem>
#include <mutex>
#include <string>

#define SAMPLE_LOG_TAG "AXCLRT"
#include "sample_log.h"

namespace stereo_depth::host_backend {

namespace {

std::mutex g_mutex;

// AXCL needs a json describing the attached card(s). Look in the usual spots so
// the sample works without the caller exporting anything; fall back to the
// library default (NULL) when nothing is found.
std::string resolveConfigPath() {
    if (const char* env = std::getenv("AXCL_CONFIG"); env != nullptr && env[0] != '\0') {
        return env;
    }
    const char* candidates[] = {
        "./axcl.json",
        "/usr/bin/axcl/axcl.json",
        "/usr/lib/axcl/axcl.json",
        "/etc/axcl/axcl.json",
    };
    std::error_code ec;
    for (const char* candidate : candidates) {
        if (std::filesystem::exists(candidate, ec) && !ec) {
            return candidate;
        }
    }
    return {};
}

}  // namespace

AxclRuntime& AxclRuntime::instance() {
    static AxclRuntime runtime;
    return runtime;
}

AxclRuntime::~AxclRuntime() = default;

bool AxclRuntime::acquire() {
    std::lock_guard<std::mutex> lock(g_mutex);

    if (m_ready) {
        ++m_refCount;
        return true;
    }

    const std::string configPath = resolveConfigPath();
    const char* config = configPath.empty() ? nullptr : configPath.c_str();
    ALOGN("axclInit config: %s", config ? config : "(default)");

    axclError ret = axclInit(config);
    if (ret != 0) {
        ALOGE("axclInit failed: 0x%x", ret);
        return false;
    }

    axclrtDeviceList deviceList;
    ret = axclrtGetDeviceList(&deviceList);
    if (ret != 0 || deviceList.num == 0) {
        ALOGE("axclrtGetDeviceList failed: ret=0x%x num=%u (is the Axera card attached?)", ret,
              deviceList.num);
        axclFinalize();
        return false;
    }

    m_deviceId = deviceList.devices[0];
    ret = axclrtSetDevice(m_deviceId);
    if (ret != 0) {
        ALOGE("axclrtSetDevice(%d) failed: 0x%x", m_deviceId, ret);
        axclFinalize();
        return false;
    }

    ret = axclrtCreateContext(&m_context, m_deviceId);
    if (ret != 0 || m_context == nullptr) {
        ALOGE("axclrtCreateContext failed: 0x%x", ret);
        axclrtResetDevice(m_deviceId);
        axclFinalize();
        return false;
    }

    ret = axclrtEngineInit(AXCL_VNPU_DISABLE);
    if (ret != 0) {
        ALOGE("axclrtEngineInit failed: 0x%x", ret);
        axclrtDestroyContext(m_context);
        m_context = nullptr;
        axclrtResetDevice(m_deviceId);
        axclFinalize();
        return false;
    }
    m_engineInited = true;

    m_ready = true;
    m_refCount = 1;
    ALOGN("AXCL runtime ready on device %d", m_deviceId);
    return true;
}

void AxclRuntime::release() {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!m_ready) {
        return;
    }
    if (--m_refCount > 0) {
        return;
    }

    if (m_engineInited) {
        axclrtEngineFinalize();
        m_engineInited = false;
    }
    if (m_context != nullptr) {
        axclrtDestroyContext(m_context);
        m_context = nullptr;
    }
    axclrtResetDevice(m_deviceId);
    axclFinalize();
    m_ready = false;
    ALOGN("AXCL runtime finalized");
}

bool AxclRuntime::ensureThreadContext() {
    if (!m_ready || m_context == nullptr) {
        return false;
    }
    // Cheap thread-local guard: only push the context the first time a given
    // thread touches the runtime.
    thread_local void* boundContext = nullptr;
    if (boundContext == m_context) {
        return true;
    }
    axclError ret = axclrtSetCurrentContext(m_context);
    if (ret != 0) {
        ALOGE("axclrtSetCurrentContext failed: 0x%x", ret);
        return false;
    }
    boundContext = m_context;
    return true;
}

}  // namespace stereo_depth::host_backend
