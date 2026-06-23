cmake_minimum_required(VERSION 3.14)
project(sample_stereo_depth LANGUAGES C CXX)

# =============================================================================
# Native host + AXCL build (amd64 today; the same layout also targets native
# Raspberry Pi / Orange Pi builds). The Axera silicon is reached over PCIe via
# the AXCL runtime (libaxcl_rt); CPU image work uses the host OpenCV. The
# original on-board (cross aarch64) build is kept in CMakeLists.board-aarch64.txt
# =============================================================================

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_C_STANDARD 11)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(PkgConfig REQUIRED)
pkg_check_modules(OPENCV REQUIRED opencv4)
pkg_check_modules(FFMPEG REQUIRED libavcodec libavutil libswscale)

# ----------------------------------------------------------------------------
# AXCL SDK (headers + runtime)
# ----------------------------------------------------------------------------
set(AXCL_INCLUDE_DIR "/usr/include/axcl" CACHE PATH "Path to AXCL headers")
set(AXCL_LIB_DIR "/usr/lib/axcl" CACHE PATH "Path to AXCL runtime libraries")
if(NOT EXISTS "${AXCL_INCLUDE_DIR}/axcl.h")
    message(FATAL_ERROR "AXCL headers not found at ${AXCL_INCLUDE_DIR} (install the AXCL host SDK)")
endif()

# ----------------------------------------------------------------------------
# curl (the runtime .so may not have a -dev symlink; resolve the versioned lib).
# Arch-portable: works for x86_64 and aarch64 (Raspberry Pi / Orange Pi).
# ----------------------------------------------------------------------------
execute_process(COMMAND ${CMAKE_C_COMPILER} -dumpmachine
    OUTPUT_VARIABLE HOST_MULTIARCH OUTPUT_STRIP_TRAILING_WHITESPACE)
find_library(CURL_LIB NAMES curl libcurl.so.4
    PATHS /lib/${HOST_MULTIARCH} /usr/lib/${HOST_MULTIARCH} /usr/lib /lib)
if(NOT CURL_LIB)
    foreach(cand
            /lib/${HOST_MULTIARCH}/libcurl.so.4 /usr/lib/${HOST_MULTIARCH}/libcurl.so.4
            /lib/x86_64-linux-gnu/libcurl.so.4 /usr/lib/x86_64-linux-gnu/libcurl.so.4
            /lib/aarch64-linux-gnu/libcurl.so.4 /usr/lib/aarch64-linux-gnu/libcurl.so.4)
        if(EXISTS "${cand}")
            set(CURL_LIB "${cand}")
            break()
        endif()
    endforeach()
endif()
if(NOT CURL_LIB)
    message(FATAL_ERROR "libcurl not found")
endif()
message(STATUS "Using curl: ${CURL_LIB} (multiarch: ${HOST_MULTIARCH})")

# ----------------------------------------------------------------------------
# Foxglove SDK (architecture-specific prebuilt) + MCAP (header-only)
# ----------------------------------------------------------------------------
if(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64|amd64|AMD64")
    set(FOXGLOVE_ZIP "${CMAKE_SOURCE_DIR}/third_party/foxglove/foxglove-v0.21.0-cpp-x86_64-unknown-linux-gnu.zip")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm64")
    # Native aarch64 build (Raspberry Pi / Orange Pi).
    set(FOXGLOVE_ZIP "${CMAKE_SOURCE_DIR}/third_party/foxglove/foxglove-v0.21.0-cpp-aarch64-unknown-linux-gnu.zip")
else()
    set(FOXGLOVE_ZIP "${CMAKE_SOURCE_DIR}/third_party/foxglove/foxglove-v0.21.0-cpp-aarch64-unknown-linux-gnu.zip")
endif()
if(NOT EXISTS "${FOXGLOVE_ZIP}")
    message(FATAL_ERROR "Foxglove SDK zip not found for this architecture: ${FOXGLOVE_ZIP}")
endif()

set(FOXGLOVE_EXTRACT_DIR "${CMAKE_BINARY_DIR}/foxglove")
set(MCAP_ZIP "${CMAKE_SOURCE_DIR}/third_party/mcap/mcap-releases-cpp-v2.1.3.zip")
set(MCAP_EXTRACT_DIR "${CMAKE_BINARY_DIR}/mcap")

if(NOT EXISTS "${FOXGLOVE_EXTRACT_DIR}")
    message(STATUS "Extracting Foxglove SDK from ${FOXGLOVE_ZIP}")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E tar xzf "${FOXGLOVE_ZIP}"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
        RESULT_VARIABLE EXTRACT_RESULT)
    if(NOT EXTRACT_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to extract Foxglove SDK zip")
    endif()
endif()

if(NOT EXISTS "${MCAP_EXTRACT_DIR}/mcap-releases-cpp-v2.1.3")
    file(MAKE_DIRECTORY "${MCAP_EXTRACT_DIR}")
    message(STATUS "Extracting MCAP SDK from ${MCAP_ZIP}")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E tar xzf "${MCAP_ZIP}"
        WORKING_DIRECTORY "${MCAP_EXTRACT_DIR}"
        RESULT_VARIABLE EXTRACT_MCAP_RESULT)
    if(NOT EXTRACT_MCAP_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to extract MCAP SDK zip")
    endif()
endif()

set(foxglove_SOURCE_DIR "${FOXGLOVE_EXTRACT_DIR}")
set(mcap_SOURCE_DIR "${MCAP_EXTRACT_DIR}/mcap-releases-cpp-v2.1.3")

# Build the Foxglove C++ wrapper as a shared library against the prebuilt core.
file(GLOB FOXGLOVE_SOURCES CONFIGURE_DEPENDS
    "${foxglove_SOURCE_DIR}/src/*.cpp"
    "${foxglove_SOURCE_DIR}/src/server/*.cpp")
add_library(foxglove_sdk SHARED ${FOXGLOVE_SOURCES})
set_target_properties(foxglove_sdk PROPERTIES OUTPUT_NAME foxglove)
target_include_directories(foxglove_sdk PUBLIC
    ${foxglove_SOURCE_DIR}/include ${foxglove_SOURCE_DIR}/include/foxglove)
target_link_libraries(foxglove_sdk PRIVATE
    ${foxglove_SOURCE_DIR}/lib/libfoxglove.a pthread dl m rt)

# ----------------------------------------------------------------------------
# Application
# ----------------------------------------------------------------------------
set(APP_SOURCES
    ax_stereo_depth_api.cpp
    AppRuntime.cpp
    AppTopics.cpp
    main.cpp
    FoxgloveWrapper.cpp
    V4L2Capture.cpp
    FrameInputSource.cpp
    StereoDepthCalibrationMesh.cpp
    StereoDepthPipeline.cpp
    LaserRangefinder.cpp
    # Host / AXCL backend (replaces the on-chip axera_pipeline implementations)
    host_backend/AxclRuntime.cpp
    host_backend/sample_engine_axcl.cpp
    host_backend/sample_image_host.cpp
    host_backend/ImageProcBackend.cpp
    host_backend/AxclImageProc.cpp
    host_backend/HostPrecheck.cpp
    host_backend/HostVoDisplay.cpp
    host_backend/AxVencH264Encoder.cpp
    host_backend/AxVdecH264Decoder.cpp
    host_backend/H264CodecHost.cpp)

add_executable(sample_stereo_depth ${APP_SOURCES})

target_include_directories(sample_stereo_depth PRIVATE
    ${CMAKE_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/axera_pipeline/include
    ${CMAKE_SOURCE_DIR}/host_backend
    ${CMAKE_SOURCE_DIR}/third_party/curl/include
    ${AXCL_INCLUDE_DIR}
    ${foxglove_SOURCE_DIR}/include
    ${foxglove_SOURCE_DIR}/include/foxglove
    ${mcap_SOURCE_DIR}/cpp/mcap/include
    ${OPENCV_INCLUDE_DIRS}
    ${FFMPEG_INCLUDE_DIRS})

target_compile_definitions(sample_stereo_depth PRIVATE
    MCAP_COMPRESSION_NO_LZ4 MCAP_COMPRESSION_NO_ZSTD
    STEREO_DEPTH_APP_MODEL_DIR="${CMAKE_SOURCE_DIR}/models")

target_link_directories(sample_stereo_depth PRIVATE
    ${AXCL_LIB_DIR} ${OPENCV_LIBRARY_DIRS} ${FFMPEG_LIBRARY_DIRS})

target_link_libraries(sample_stereo_depth PRIVATE
    foxglove_sdk
    ${OPENCV_LIBRARIES}
    ${FFMPEG_LIBRARIES}
    ${CURL_LIB}
    axcl_rt
    axcl_sys
    axcl_ivps
    pthread
    dl
    m)

# Make sure libaxcl_rt and the bundled foxglove shared lib are found at runtime.
set_target_properties(sample_stereo_depth PROPERTIES
    BUILD_RPATH "$ORIGIN;$ORIGIN/lib;${AXCL_LIB_DIR}"
    INSTALL_RPATH "$ORIGIN;$ORIGIN/lib;${AXCL_LIB_DIR}")

add_custom_command(TARGET sample_stereo_depth POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:foxglove_sdk>"
            "$<TARGET_FILE_DIR:sample_stereo_depth>/$<TARGET_FILE_NAME:foxglove_sdk>")
