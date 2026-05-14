cmake_minimum_required(VERSION 3.14)
project(sample_stereo_depth LANGUAGES C CXX)

# Set C++ standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Use LLVM's libc++ instead of GCC's libstdc++
if(CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-stdlib=libc++)
    add_link_options(-stdlib=libc++)
endif()

# Pre-downloaded Foxglove SDK zip
set(FOXGLOVE_ZIP "${CMAKE_SOURCE_DIR}/third_party/foxglove/foxglove-v0.21.0-cpp-aarch64-unknown-linux-gnu.zip")
set(FOXGLOVE_EXTRACT_DIR "${CMAKE_BINARY_DIR}/foxglove")
set(MCAP_ZIP "${CMAKE_SOURCE_DIR}/third_party/mcap/mcap-releases-cpp-v2.1.3.zip")
set(MCAP_EXTRACT_DIR "${CMAKE_BINARY_DIR}/mcap")

# Extract the zip file if not already extracted
if(NOT EXISTS "${FOXGLOVE_EXTRACT_DIR}")
    message(STATUS "Extracting Foxglove SDK from ${FOXGLOVE_ZIP}")
    execute_process(
        COMMAND ${CMAKE_COMMAND} -E tar xzf "${FOXGLOVE_ZIP}"
        WORKING_DIRECTORY "${CMAKE_BINARY_DIR}"
        RESULT_VARIABLE EXTRACT_RESULT
    )
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
        RESULT_VARIABLE EXTRACT_MCAP_RESULT
    )
    if(NOT EXTRACT_MCAP_RESULT EQUAL 0)
        message(FATAL_ERROR "Failed to extract MCAP SDK zip")
    endif()
endif()

# Set foxglove source dir
set(foxglove_SOURCE_DIR "${FOXGLOVE_EXTRACT_DIR}")
set(mcap_SOURCE_DIR "${MCAP_EXTRACT_DIR}/mcap-releases-cpp-v2.1.3")
set(CURL_SOURCE_DIR "${CMAKE_SOURCE_DIR}/third_party/curl")
set(CURL_INCLUDE_DIR "${CURL_SOURCE_DIR}/include")
set(CURL_RUNTIME_LIB_DIR "${CURL_SOURCE_DIR}/runtime")
set(DSP_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/third_party/dsp" CACHE PATH "Path to vendored DSP headers")
set(DSP_LIB_DIR "${CMAKE_SOURCE_DIR}/third_party/dsp" CACHE PATH "Path to vendored DSP shared libraries")
set(DSP_FIRMWARE_DIR "${CMAKE_SOURCE_DIR}/third_party/dsp" CACHE PATH "Path to vendored DSP firmware binaries")
set(OPENCV_SOURCE_DIR "${CMAKE_SOURCE_DIR}/third_party/opencv")
set(OPENCV_SHARED_LIB_DIR "${OPENCV_SOURCE_DIR}/lib")
set(LIBEXIF_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/third_party/libexif/include" CACHE PATH "Path to libexif headers")
set(LIBEXIF_LIB_DIR "${CMAKE_SOURCE_DIR}/third_party/libexif/lib" CACHE PATH "Path to libexif shared libraries")
set(DRM_INCLUDE_DIR "${CMAKE_SOURCE_DIR}/third_party/drm/include" CACHE PATH "Path to libdrm headers")
set(DRM_LIB_DIR "${CMAKE_SOURCE_DIR}/third_party/drm/lib" CACHE PATH "Path to libdrm shared libraries")

set(CURL_SHARED_LIBS
    ${CURL_RUNTIME_LIB_DIR}/libcurl.so.4.8.0
    ${CURL_RUNTIME_LIB_DIR}/libnghttp2.so.14.24.1
    ${CURL_RUNTIME_LIB_DIR}/libssl.so.3
    ${CURL_RUNTIME_LIB_DIR}/libcrypto.so.3
    ${CURL_RUNTIME_LIB_DIR}/libzstd.so.1.4.5
    ${CURL_RUNTIME_LIB_DIR}/libz.so.1.2.13
)

set(LIBEXIF_SHARED_LIBS
    ${LIBEXIF_LIB_DIR}/libexif.so.12.3.4
)

set(OPENCV_SHARED_LIBS
    ${OPENCV_SHARED_LIB_DIR}/libopencv_calib3d.so.4.5.5
    ${OPENCV_SHARED_LIB_DIR}/libopencv_features2d.so.4.5.5
    ${OPENCV_SHARED_LIB_DIR}/libopencv_flann.so.4.5.5
    ${OPENCV_SHARED_LIB_DIR}/libopencv_imgproc.so.4.5.5
    ${OPENCV_SHARED_LIB_DIR}/libopencv_core.so.4.5.5
)

set(DRM_SHARED_LIBS
    ${DRM_LIB_DIR}/libdrm.so.2.4.112
)

set(DSP_SHARED_LIBS
    ${DSP_LIB_DIR}/libax_dsp.so
    ${DSP_LIB_DIR}/libax_dsp_cv.so
)

# Optional external sources passed from Makefile
set(SAMPLE_ENGINE_CPP "" CACHE STRING "Path to sample_engine.cpp")
set(SAMPLE_ENGINE_H "" CACHE STRING "Path to sample_engine.h")
set(SAMPLE_DSP_C "" CACHE STRING "Path to sample_dsp.c")
set(SAMPLE_DSP_H "" CACHE STRING "Path to sample_dsp.h")
set(SAMPLE_GDC_C "" CACHE STRING "Path to sample_gdc.c")
set(SAMPLE_GDC_H "" CACHE STRING "Path to sample_gdc.h")
set(SAMPLE_IMAGE_C "" CACHE STRING "Path to sample_image.c")
set(SAMPLE_IMAGE_H "" CACHE STRING "Path to sample_image.h")
set(SAMPLE_ENGINE_INCLUDE_DIR "" CACHE STRING "Path to sample_engine helper headers")
set(AX_INCLUDE_DIR "" CACHE PATH "Path to AX SDK headers, e.g. /soc/include")
set(AX_LIB_DIR "" CACHE PATH "Path to AX SDK libraries, e.g. /soc/lib")
set(AX_DEP_LIBS "" CACHE STRING "Comma-separated AX libraries")

# Add executable
set(APP_SOURCES
    ax_stereo_depth_api.cpp
    AppRuntime.cpp
    AppTopics.cpp
    main.cpp
    FoxgloveWrapper.cpp
    V4L2Capture.cpp
    FrameInputSource.cpp
    axera_pipeline/AxVdecH264Decoder.cpp
    StereoDepthCalibrationMesh.cpp
    StereoDepthPipeline.cpp
    LaserRangefinder.cpp
    axera_pipeline/AxVencH264Encoder.cpp
    axera_pipeline/AxVoDisplay.cpp
)
if(SAMPLE_ENGINE_CPP)
    list(APPEND APP_SOURCES ${SAMPLE_ENGINE_CPP})
endif()
if(SAMPLE_DSP_C)
    list(APPEND APP_SOURCES ${SAMPLE_DSP_C})
endif()
if(SAMPLE_GDC_C)
    list(APPEND APP_SOURCES ${SAMPLE_GDC_C})
endif()
if(SAMPLE_IMAGE_C)
    list(APPEND APP_SOURCES ${SAMPLE_IMAGE_C})
endif()
add_executable(sample_stereo_depth ${APP_SOURCES})

if(SAMPLE_ENGINE_H)
    target_sources(sample_stereo_depth PRIVATE ${SAMPLE_ENGINE_H})
endif()
if(SAMPLE_DSP_H)
    target_sources(sample_stereo_depth PRIVATE ${SAMPLE_DSP_H})
endif()
if(SAMPLE_GDC_H)
    target_sources(sample_stereo_depth PRIVATE ${SAMPLE_GDC_H})
endif()
if(SAMPLE_IMAGE_H)
    target_sources(sample_stereo_depth PRIVATE ${SAMPLE_IMAGE_H})
endif()

if(SAMPLE_ENGINE_CPP)
    get_filename_component(SAMPLE_ENGINE_DIR "${SAMPLE_ENGINE_CPP}" DIRECTORY)
    target_include_directories(sample_stereo_depth PRIVATE ${SAMPLE_ENGINE_DIR})
endif()
if(SAMPLE_ENGINE_INCLUDE_DIR)
    target_include_directories(sample_stereo_depth PRIVATE ${SAMPLE_ENGINE_INCLUDE_DIR})
endif()
if(DSP_INCLUDE_DIR)
    target_include_directories(sample_stereo_depth PRIVATE ${DSP_INCLUDE_DIR})
endif()
if((SAMPLE_ENGINE_CPP OR SAMPLE_DSP_C OR SAMPLE_GDC_C OR SAMPLE_IMAGE_C) AND AX_INCLUDE_DIR)
    target_include_directories(sample_stereo_depth PRIVATE ${AX_INCLUDE_DIR})
endif()

if(SAMPLE_DSP_C)
    if(NOT DSP_INCLUDE_DIR)
        message(FATAL_ERROR "DSP backend enabled but DSP_INCLUDE_DIR is empty")
    endif()
    if(NOT EXISTS "${DSP_INCLUDE_DIR}/ax_dsp_slam_api.h")
        message(FATAL_ERROR "Missing AX DSP SLAM header at ${DSP_INCLUDE_DIR}/ax_dsp_slam_api.h")
    endif()
endif()

target_include_directories(sample_stereo_depth PRIVATE ${CURL_INCLUDE_DIR})
target_include_directories(sample_stereo_depth PRIVATE ${LIBEXIF_INCLUDE_DIR})
target_include_directories(sample_stereo_depth PRIVATE ${DRM_INCLUDE_DIR})
target_include_directories(sample_stereo_depth PRIVATE ${CMAKE_SOURCE_DIR})
target_include_directories(sample_stereo_depth PRIVATE ${CMAKE_SOURCE_DIR}/axera_pipeline/include)

# Add include directory for Foxglove SDK
target_include_directories(sample_stereo_depth PRIVATE ${foxglove_SOURCE_DIR}/include)
target_include_directories(sample_stereo_depth PRIVATE ${foxglove_SOURCE_DIR}/include/foxglove)
target_include_directories(sample_stereo_depth PRIVATE ${mcap_SOURCE_DIR}/cpp/mcap/include)
target_include_directories(sample_stereo_depth PRIVATE ${OPENCV_SOURCE_DIR}/include/opencv4)
target_compile_definitions(sample_stereo_depth PRIVATE MCAP_COMPRESSION_NO_LZ4 MCAP_COMPRESSION_NO_ZSTD)
target_compile_definitions(sample_stereo_depth PRIVATE STEREO_DEPTH_APP_DSP_DIR="${DSP_FIRMWARE_DIR}")
target_compile_definitions(sample_stereo_depth PRIVATE STEREO_DEPTH_APP_MODEL_DIR="${CMAKE_SOURCE_DIR}/models")

# Build Foxglove as a shared library with the current toolchain to keep ABI compatibility.
file(GLOB FOXGLOVE_SOURCES CONFIGURE_DEPENDS
    "${foxglove_SOURCE_DIR}/src/*.cpp"
    "${foxglove_SOURCE_DIR}/src/server/*.cpp"
)

add_library(foxglove_sdk SHARED ${FOXGLOVE_SOURCES})
set_target_properties(foxglove_sdk PROPERTIES OUTPUT_NAME foxglove)
target_include_directories(foxglove_sdk PUBLIC ${foxglove_SOURCE_DIR}/include ${foxglove_SOURCE_DIR}/include/foxglove)
target_link_libraries(foxglove_sdk PRIVATE ${foxglove_SOURCE_DIR}/lib/libfoxglove.a pthread dl m)

foreach(curl_lib ${CURL_SHARED_LIBS})
    if(NOT EXISTS "${curl_lib}")
        message(FATAL_ERROR "Missing vendored curl runtime library at ${curl_lib}")
    endif()
endforeach()
foreach(libexif_lib ${LIBEXIF_SHARED_LIBS})
    if(NOT EXISTS "${libexif_lib}")
        message(FATAL_ERROR "Missing libexif runtime library at ${libexif_lib}")
    endif()
endforeach()
foreach(dsp_lib ${DSP_SHARED_LIBS})
    if(NOT EXISTS "${dsp_lib}")
        message(FATAL_ERROR "Missing vendored DSP runtime library at ${dsp_lib}")
    endif()
endforeach()
if(NOT EXISTS "${OPENCV_SHARED_LIB_DIR}/libopencv_core.so.4.5.5")
    message(FATAL_ERROR "Missing shared OpenCV core at ${OPENCV_SHARED_LIB_DIR}/libopencv_core.so.4.5.5")
endif()
if(NOT EXISTS "${OPENCV_SHARED_LIB_DIR}/libopencv_calib3d.so.4.5.5")
    message(FATAL_ERROR "Missing shared OpenCV calib3d at ${OPENCV_SHARED_LIB_DIR}/libopencv_calib3d.so.4.5.5")
endif()
if(NOT EXISTS "${OPENCV_SHARED_LIB_DIR}/libopencv_imgproc.so.4.5.5")
    message(FATAL_ERROR "Missing shared OpenCV imgproc at ${OPENCV_SHARED_LIB_DIR}/libopencv_imgproc.so.4.5.5")
endif()
if(NOT EXISTS "${OPENCV_SHARED_LIB_DIR}/libopencv_flann.so.4.5.5")
    message(FATAL_ERROR "Missing shared OpenCV flann at ${OPENCV_SHARED_LIB_DIR}/libopencv_flann.so.4.5.5")
endif()
if(NOT EXISTS "${OPENCV_SHARED_LIB_DIR}/libopencv_features2d.so.4.5.5")
    message(FATAL_ERROR "Missing shared OpenCV features2d at ${OPENCV_SHARED_LIB_DIR}/libopencv_features2d.so.4.5.5")
endif()
if(NOT EXISTS "${DRM_LIB_DIR}/libdrm.so.2.4.112")
    message(FATAL_ERROR "Missing shared DRM library at ${DRM_LIB_DIR}/libdrm.so.2.4.112")
endif()

if(SAMPLE_ENGINE_CPP OR SAMPLE_DSP_C OR SAMPLE_GDC_C OR SAMPLE_IMAGE_C)
    if(DSP_LIB_DIR)
        target_link_directories(sample_stereo_depth PRIVATE ${DSP_LIB_DIR})
        target_link_options(sample_stereo_depth PRIVATE "-Wl,-rpath-link=${DSP_LIB_DIR}")
    endif()

    if(AX_LIB_DIR)
        target_link_directories(sample_stereo_depth PRIVATE ${AX_LIB_DIR})
        target_link_options(sample_stereo_depth PRIVATE "-Wl,-rpath-link=${AX_LIB_DIR}")
    endif()

    if(AX_DEP_LIBS)
        string(REPLACE "," ";" AX_DEP_LIB_LIST "${AX_DEP_LIBS}")
    else()
        set(AX_DEP_LIB_LIST ax_sys ax_ivps ax_dsp ax_dsp_cv ax_dmadim ax_engine ax_venc ax_vdec ax_vo)
    endif()

    if(NOT "ax_vdec" IN_LIST AX_DEP_LIB_LIST)
        list(APPEND AX_DEP_LIB_LIST ax_vdec)
    endif()

    list(REMOVE_ITEM AX_DEP_LIB_LIST ax_dsp ax_dsp_cv)

    target_link_libraries(sample_stereo_depth PRIVATE ${DSP_SHARED_LIBS} ${AX_DEP_LIB_LIST} m stdc++)
endif()

# Link against shared third-party libraries and required system libraries
target_link_directories(sample_stereo_depth PRIVATE ${CURL_RUNTIME_LIB_DIR} ${LIBEXIF_LIB_DIR} ${OPENCV_SHARED_LIB_DIR} ${DRM_LIB_DIR})
target_link_options(sample_stereo_depth PRIVATE "-Wl,-rpath-link=${CURL_RUNTIME_LIB_DIR}" "-Wl,-rpath-link=${LIBEXIF_LIB_DIR}" "-Wl,-rpath-link=${OPENCV_SHARED_LIB_DIR}" "-Wl,-rpath-link=${DRM_LIB_DIR}")
set_target_properties(sample_stereo_depth PROPERTIES BUILD_RPATH "$ORIGIN;$ORIGIN/lib" INSTALL_RPATH "$ORIGIN/lib")
target_link_libraries(sample_stereo_depth PRIVATE
    foxglove_sdk
    ${CURL_SHARED_LIBS}
    ${LIBEXIF_SHARED_LIBS}
    ${OPENCV_SHARED_LIBS}
    ${DRM_SHARED_LIBS}
    pthread
    dl)

add_custom_command(TARGET sample_stereo_depth POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
            "$<TARGET_FILE:foxglove_sdk>"
            "$<TARGET_FILE_DIR:sample_stereo_depth>/$<TARGET_FILE_NAME:foxglove_sdk>")

foreach(runtime_lib ${DSP_SHARED_LIBS})
    if(EXISTS "${runtime_lib}")
        add_custom_command(TARGET sample_stereo_depth POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${runtime_lib}"
                    "$<TARGET_FILE_DIR:sample_stereo_depth>/")
    endif()
endforeach()

set(CURL_RUNTIME_LIBS
    libcurl.so.4.8.0
    libcurl.so.4
    libssl.so.3
    libcrypto.so.3
    libnghttp2.so.14.24.1
    libnghttp2.so.14
    libz.so.1.2.13
    libz.so.1
    libzstd.so.1.4.5
    libzstd.so.1
)

set(LIBEXIF_RUNTIME_LIBS
    libexif.so.12.3.4
    libexif.so.12
    libexif.so
)

foreach(runtime_lib ${CURL_RUNTIME_LIBS})
    if(EXISTS "${CURL_RUNTIME_LIB_DIR}/${runtime_lib}")
        add_custom_command(TARGET sample_stereo_depth POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${CURL_RUNTIME_LIB_DIR}/${runtime_lib}"
                    "$<TARGET_FILE_DIR:sample_stereo_depth>/${runtime_lib}")
    endif()
endforeach()

foreach(runtime_lib ${LIBEXIF_RUNTIME_LIBS})
    if(EXISTS "${LIBEXIF_LIB_DIR}/${runtime_lib}")
        add_custom_command(TARGET sample_stereo_depth POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${LIBEXIF_LIB_DIR}/${runtime_lib}"
                    "$<TARGET_FILE_DIR:sample_stereo_depth>/${runtime_lib}")
    endif()
endforeach()

set(OPENCV_RUNTIME_LIBS
    libopencv_calib3d.so.4.5.5
    libopencv_calib3d.so.405
    libopencv_core.so.4.5.5
    libopencv_core.so.405
    libopencv_features2d.so.4.5.5
    libopencv_features2d.so.405
    libopencv_flann.so.4.5.5
    libopencv_flann.so.405
    libopencv_imgproc.so.4.5.5
    libopencv_imgproc.so.405
)

set(DRM_RUNTIME_LIBS
    libdrm.so.2.4.112
    libdrm.so.2
    libdrm.so
)

foreach(runtime_lib ${OPENCV_RUNTIME_LIBS})
    if(EXISTS "${OPENCV_SHARED_LIB_DIR}/${runtime_lib}")
        add_custom_command(TARGET sample_stereo_depth POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${OPENCV_SHARED_LIB_DIR}/${runtime_lib}"
                    "$<TARGET_FILE_DIR:sample_stereo_depth>/${runtime_lib}")
    endif()
endforeach()

foreach(runtime_lib ${DRM_RUNTIME_LIBS})
    if(EXISTS "${DRM_LIB_DIR}/${runtime_lib}")
        add_custom_command(TARGET sample_stereo_depth POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E copy_if_different
                    "${DRM_LIB_DIR}/${runtime_lib}"
                    "$<TARGET_FILE_DIR:sample_stereo_depth>/${runtime_lib}")
    endif()
endforeach()
