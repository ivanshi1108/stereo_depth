
.NOTPARALLEL: clean

CUR_PATH  := $(shell pwd)
TOOLCHAIN_FILE := $(CUR_PATH)/toolchain-aarch64.cmake
HOST_ARCH      := $(shell uname -m)
SOC_PATH       ?= /soc
THIRD_PARTY_DIR := $(CUR_PATH)/third_party
SOC_INCLUDE_DIR ?= $(SOC_PATH)/include
SOC_LIB_DIR    ?= $(SOC_PATH)/lib
DSP_THIRD_PARTY_DIR ?= $(THIRD_PARTY_DIR)/dsp
LIBEXIF_INCLUDE_DIR ?= $(THIRD_PARTY_DIR)/libexif/include
LIBEXIF_LIB_DIR ?= $(THIRD_PARTY_DIR)/libexif/lib
DRM_INCLUDE_DIR ?= $(THIRD_PARTY_DIR)/drm/include
DRM_LIB_DIR    ?= $(THIRD_PARTY_DIR)/drm/lib

MOD_NAME                  := sample_stereo_depth
INSTALL_ROOT              ?= $(CUR_PATH)/output
INSTALL_DIR               := $(INSTALL_ROOT)/$(MOD_NAME)
INSTALL_LIB_DIR           := $(INSTALL_DIR)/lib
INSTALL_MODEL_DIR         := $(INSTALL_DIR)/models
INSTALL_DSP_DIR           := $(INSTALL_DIR)/dsp
INSTALL_MODEL_FILES       := axstereo_pro.axmodel axstereo_lite.axmodel
INSTALL_DSP_FILES         := itcm.bin sram.bin dtcm.bin dtcm2.bin
BUILD_ROOT                ?= $(CUR_PATH)/build
AXERA_PIPELINE_DIR        := $(CUR_PATH)/axera_pipeline
SAMPLE_ENGINE_CPP         := $(AXERA_PIPELINE_DIR)/sample_engine.cpp
SAMPLE_ENGINE_H           := $(AXERA_PIPELINE_DIR)/sample_engine.h
SAMPLE_DSP_C              := $(AXERA_PIPELINE_DIR)/sample_dsp.c
SAMPLE_DSP_H              := $(AXERA_PIPELINE_DIR)/include/sample_dsp.h
SAMPLE_GDC_C              := $(AXERA_PIPELINE_DIR)/sample_gdc.c
SAMPLE_GDC_H              := $(AXERA_PIPELINE_DIR)/include/sample_gdc.h
SAMPLE_IMAGE_C            := $(AXERA_PIPELINE_DIR)/sample_image.c
SAMPLE_IMAGE_H            := $(AXERA_PIPELINE_DIR)/include/sample_image.h
SAMPLE_ENGINE_INCLUDE_DIR := $(AXERA_PIPELINE_DIR)/include
AX_INCLUDE_DIR            ?= $(SOC_INCLUDE_DIR)
AX_LIB_DIR                ?= $(SOC_LIB_DIR)
AX_DEP_LIBS               := ax_sys,ax_ivps,ax_dsp,ax_dsp_cv,ax_dmadim,ax_engine,ax_venc,ax_vo
EXTRA_CMAKE_DEFS          := -DSAMPLE_ENGINE_CPP=$(SAMPLE_ENGINE_CPP) \
	-DSAMPLE_ENGINE_H=$(SAMPLE_ENGINE_H) \
	-DSAMPLE_DSP_C=$(SAMPLE_DSP_C) \
	-DSAMPLE_DSP_H=$(SAMPLE_DSP_H) \
	-DSAMPLE_GDC_C=$(SAMPLE_GDC_C) \
	-DSAMPLE_GDC_H=$(SAMPLE_GDC_H) \
	-DSAMPLE_IMAGE_C=$(SAMPLE_IMAGE_C) \
	-DSAMPLE_IMAGE_H=$(SAMPLE_IMAGE_H) \
	-DSAMPLE_ENGINE_INCLUDE_DIR=$(SAMPLE_ENGINE_INCLUDE_DIR) \
	-DAX_INCLUDE_DIR=$(AX_INCLUDE_DIR) \
	-DAX_LIB_DIR=$(AX_LIB_DIR) \
	-DDSP_INCLUDE_DIR=$(DSP_THIRD_PARTY_DIR) \
	-DDSP_LIB_DIR=$(DSP_THIRD_PARTY_DIR) \
	-DDSP_FIRMWARE_DIR=$(DSP_THIRD_PARTY_DIR) \
	-DLIBEXIF_INCLUDE_DIR=$(LIBEXIF_INCLUDE_DIR) \
	-DLIBEXIF_LIB_DIR=$(LIBEXIF_LIB_DIR) \
	-DDRM_INCLUDE_DIR=$(DRM_INCLUDE_DIR) \
	-DDRM_LIB_DIR=$(DRM_LIB_DIR) \
	-DAX_DEP_LIBS=$(AX_DEP_LIBS)

INSTALL_RUNTIME_LIBS     := libfoxglove.so \
	libax_dsp.so \
	libax_dsp_cv.so \
	libcurl.so.4.8.0 \
	libdrm.so.2.4.112 \
	libexif.so.12.3.4 \
	libssl.so.3 \
	libcrypto.so.3 \
	libnghttp2.so.14.24.1 \
	libz.so.1.2.13 \
	libzstd.so.1.4.5 \
	libopencv_calib3d.so.4.5.5 \
	libopencv_core.so.4.5.5 \
	libopencv_features2d.so.4.5.5 \
	libopencv_flann.so.4.5.5 \
	libopencv_imgproc.so.4.5.5

INSTALL_RUNTIME_SYMLINKS := libcurl.so.4:libcurl.so.4.8.0 \
	libdrm.so.2:libdrm.so.2.4.112 \
	libdrm.so:libdrm.so.2.4.112 \
	libexif.so.12:libexif.so.12.3.4 \
	libexif.so:libexif.so.12.3.4 \
	libnghttp2.so.14:libnghttp2.so.14.24.1 \
	libz.so.1:libz.so.1.2.13 \
	libzstd.so.1:libzstd.so.1.4.5 \
	libopencv_calib3d.so.405:libopencv_calib3d.so.4.5.5 \
	libopencv_core.so.405:libopencv_core.so.4.5.5 \
	libopencv_features2d.so.405:libopencv_features2d.so.4.5.5 \
	libopencv_flann.so.405:libopencv_flann.so.4.5.5 \
	libopencv_imgproc.so.405:libopencv_imgproc.so.4.5.5

# Check if cross-compiling
ifeq ($(filter aarch64 arm64,$(HOST_ARCH)),)
CMAKE_TOOLCHAIN := -DCMAKE_TOOLCHAIN_FILE=$(TOOLCHAIN_FILE)
BUILD_SUFFIX    := -aarch64
STRIP_TOOL      := $(shell command -v aarch64-none-linux-gnu-strip 2>/dev/null || command -v aarch64-linux-gnu-strip 2>/dev/null || command -v strip 2>/dev/null)
else
CMAKE_TOOLCHAIN :=
BUILD_SUFFIX    :=
STRIP_TOOL      := $(shell command -v strip 2>/dev/null)
endif

BUILD_DIR := $(BUILD_ROOT)$(BUILD_SUFFIX)

.PHONY: all clean build install uninstall distclean

all: build

clean:
	@echo "[Cleaning build directory...]"
	rm -rf $(BUILD_DIR)

uninstall:
	@echo "[Removing installed $(MOD_NAME)...]"
	rm -rf $(INSTALL_DIR)
	@echo "[Removed install directory] $(INSTALL_DIR)"

distclean: clean uninstall
	@echo "[Distclean completed.]"

build: $(BUILD_DIR)
	@echo "[Configuring $(MOD_NAME)...]"
	cmake -S $(CUR_PATH) -B $(BUILD_DIR) $(CMAKE_TOOLCHAIN) $(EXTRA_CMAKE_DEFS)
	@echo "[Building $(MOD_NAME)...]"
	cmake --build $(BUILD_DIR)
	@echo "[Build completed.]"

$(BUILD_DIR):
	mkdir -p $(BUILD_DIR)

install:
	@echo "[Installing $(MOD_NAME)...]"
	@rm -rf $(INSTALL_DIR)
	@mkdir -p $(INSTALL_LIB_DIR) $(INSTALL_MODEL_DIR) $(INSTALL_DSP_DIR)
	@cp $(BUILD_DIR)/$(MOD_NAME) $(INSTALL_DIR)/
	@set -e; \
	for model_file in $(INSTALL_MODEL_FILES); do \
		model_source="$(CUR_PATH)/models/$$model_file"; \
		if [ -f "$$model_source" ]; then \
			cp "$$model_source" "$(INSTALL_MODEL_DIR)/"; \
		else \
			echo "[Missing model] $$model_source"; \
			exit 1; \
		fi; \
	done
	@set -e; \
	for dsp_file in $(INSTALL_DSP_FILES); do \
		dsp_source="$(DSP_THIRD_PARTY_DIR)/$$dsp_file"; \
		if [ -f "$$dsp_source" ]; then \
			cp "$$dsp_source" "$(INSTALL_DSP_DIR)/"; \
		else \
			echo "[Missing DSP firmware] $$dsp_source"; \
			exit 1; \
		fi; \
	done
	@set -e; \
	for runtime_lib in $(INSTALL_RUNTIME_LIBS); do \
		if [ -f "$(BUILD_DIR)/$$runtime_lib" ]; then \
			cp "$(BUILD_DIR)/$$runtime_lib" "$(INSTALL_LIB_DIR)/"; \
		fi; \
	done
	@if [ -n "$(STRIP_TOOL)" ]; then \
		$(STRIP_TOOL) --strip-unneeded "$(INSTALL_DIR)/$(MOD_NAME)"; \
		set -e; \
		for runtime_lib in $(INSTALL_RUNTIME_LIBS); do \
			if [ -f "$(INSTALL_LIB_DIR)/$$runtime_lib" ]; then \
				$(STRIP_TOOL) --strip-unneeded "$(INSTALL_LIB_DIR)/$$runtime_lib"; \
			fi; \
		done; \
		echo "[Stripped binaries] $(STRIP_TOOL)"; \
	else \
		echo "[Skip strip] no cross strip tool found"; \
	fi
	@set -e; \
	for runtime_link in $(INSTALL_RUNTIME_SYMLINKS); do \
		link_name=$${runtime_link%%:*}; \
		target_name=$${runtime_link##*:}; \
		if [ -f "$(INSTALL_LIB_DIR)/$$target_name" ]; then \
			ln -sf "$$target_name" "$(INSTALL_LIB_DIR)/$$link_name"; \
		fi; \
	done
	@echo "[Installed executable] $(INSTALL_DIR)/$(MOD_NAME)"
	@echo "[Installed runtime libs] $(INSTALL_LIB_DIR)"
	@echo "[Installed models] $(INSTALL_MODEL_DIR): $(INSTALL_MODEL_FILES)"
	@echo "[Installed DSP firmware] $(INSTALL_DSP_DIR): $(INSTALL_DSP_FILES)"
	@echo "[Installation completed.]"
