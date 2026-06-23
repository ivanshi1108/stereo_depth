#pragma once

// Backend selection for image processing: host CPU (OpenCV) vs. AXCL IVPS on
// the card. Auto prefers the card and falls back to host.

#include <string>

namespace stereo_depth {

enum class Backend {
    Auto = 0,  // prefer host, fall back to AXCL card
    Host = 1,  // force host (CPU / host codec)
    Axcl = 2,  // force AXCL card
};

inline const char* backendName(Backend backend) {
    switch (backend) {
        case Backend::Host:
            return "host";
        case Backend::Axcl:
            return "axcl";
        case Backend::Auto:
        default:
            return "auto";
    }
}

// Parse "host" | "axcl" | "auto" (case-insensitive). Returns false on bad input.
inline bool parseBackend(const std::string& text, Backend& out) {
    std::string lower;
    lower.reserve(text.size());
    for (char c : text) {
        lower.push_back(static_cast<char>(c >= 'A' && c <= 'Z' ? c - 'A' + 'a' : c));
    }
    if (lower == "auto") {
        out = Backend::Auto;
        return true;
    }
    if (lower == "host") {
        out = Backend::Host;
        return true;
    }
    if (lower == "axcl") {
        out = Backend::Axcl;
        return true;
    }
    return false;
}

}  // namespace stereo_depth
