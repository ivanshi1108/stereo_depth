#include "HostPrecheck.hpp"

#include <fcntl.h>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#define SAMPLE_LOG_TAG "PRECHECK"
#include "sample_log.h"

namespace stereo_depth::host_backend {

namespace {

bool pathExists(const char* path) {
    struct stat st;
    return ::stat(path, &st) == 0;
}

// Run a command in a login shell (so /etc/profile / AXCL env is sourced) and
// return its exit code, capturing a little stdout/stderr for diagnostics.
int runLoginShell(const std::string& command, std::string& output) {
    const std::string full = "source /etc/profile >/dev/null 2>&1; " + command + " 2>&1";
    FILE* pipe = ::popen(("bash -lc '" + full + "'").c_str(), "r");
    if (pipe == nullptr) {
        return -1;
    }
    char buffer[256];
    while (std::fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        if (output.size() < 4096) {
            output += buffer;
        }
    }
    const int status = ::pclose(pipe);
    if (status == -1) {
        return -1;
    }
    if (WIFEXITED(status)) {
        return WEXITSTATUS(status);
    }
    return -1;
}

}  // namespace

bool acquireSingleInstanceLock(std::string& reason) {
    // Intentionally leaked for the process lifetime: closing the fd (or exit)
    // releases the advisory lock automatically.
    static int lockFd = -1;
    const char* lockPath = "/tmp/stereo_depth.lock";

    lockFd = ::open(lockPath, O_RDWR | O_CREAT | O_CLOEXEC, 0644);
    if (lockFd < 0) {
        reason = std::string("cannot open lock file ") + lockPath + ": " + std::strerror(errno);
        return false;
    }
    if (::flock(lockFd, LOCK_EX | LOCK_NB) != 0) {
        if (errno == EWOULDBLOCK) {
            reason =
                "another sample_stereo_depth instance is already running (the Axera card cannot be "
                "shared between processes). Stop the other instance first.";
        } else {
            reason = std::string("flock failed: ") + std::strerror(errno);
        }
        ::close(lockFd);
        lockFd = -1;
        return false;
    }

    // Record our pid for diagnostics (best-effort).
    if (::ftruncate(lockFd, 0) == 0) {
        char pidText[32];
        const int n = std::snprintf(pidText, sizeof(pidText), "%d\n", static_cast<int>(::getpid()));
        if (n > 0) {
            ssize_t wr = ::write(lockFd, pidText, static_cast<size_t>(n));
            (void)wr;
        }
    }
    return true;
}

bool checkAxclHostReady(std::string& reason) {
    // 1) kernel driver present
    const bool devPresent = pathExists("/dev/axcl_host");
    const bool modPresent = pathExists("/sys/module/axcl_host");
    if (!devPresent && !modPresent) {
        reason =
            "AXCL host driver is not loaded (neither /dev/axcl_host nor /sys/module/axcl_host "
            "found). Load the axcl_host driver before running.";
        return false;
    }
    if (!devPresent) {
        reason = "AXCL host device node /dev/axcl_host is missing (driver not fully loaded).";
        return false;
    }

    // 2) axcl-smi runs successfully after sourcing the profile
    std::string output;
    int rc = runLoginShell("command -v axcl-smi >/dev/null 2>&1 && axcl-smi", output);
    if (rc != 0) {
        // Fall back to the well-known install path if it is not on PATH.
        output.clear();
        rc = runLoginShell("/usr/bin/axcl/axcl-smi", output);
    }
    if (rc != 0) {
        reason = "axcl-smi did not run successfully (exit=" + std::to_string(rc) +
                 "). Make sure the AXCL host package is installed and the card is connected. "
                 "Try: source /etc/profile && axcl-smi";
        return false;
    }

    ALOGN("AXCL host driver loaded and axcl-smi OK");
    return true;
}

}  // namespace stereo_depth::host_backend
