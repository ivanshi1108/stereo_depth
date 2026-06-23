# Native host + AXCL build (amd64 / Raspberry Pi / Orange Pi).
# The original cross-compiled on-board build lives in Makefile.board-aarch64.

CUR_PATH   := $(shell pwd)
MOD_NAME   := sample_stereo_depth
BUILD_DIR  ?= $(CUR_PATH)/build
INSTALL_ROOT ?= $(CUR_PATH)/output
INSTALL_DIR  := $(INSTALL_ROOT)/$(MOD_NAME)
INSTALL_LIB_DIR   := $(INSTALL_DIR)/lib
INSTALL_MODEL_DIR := $(INSTALL_DIR)/models
INSTALL_MODEL_FILES := axstereo_pro.axmodel axstereo_lite.axmodel \
	mesh_hd_l_32x16.txt mesh_hd_r_32x16.txt
JOBS ?= $(shell nproc 2>/dev/null || echo 4)

.PHONY: all build install uninstall clean distclean

all: build

$(BUILD_DIR):
	@mkdir -p $(BUILD_DIR)

build: $(BUILD_DIR)
	@echo "[Configuring $(MOD_NAME) (native host + AXCL)...]"
	@cd $(BUILD_DIR) && cmake $(CUR_PATH) -DCMAKE_BUILD_TYPE=Release
	@echo "[Building $(MOD_NAME)...]"
	@cmake --build $(BUILD_DIR) -j$(JOBS)
	@echo "[Build done] $(BUILD_DIR)/$(MOD_NAME)"

install: build
	@echo "[Installing $(MOD_NAME) -> $(INSTALL_DIR)]"
	@mkdir -p $(INSTALL_DIR) $(INSTALL_LIB_DIR) $(INSTALL_MODEL_DIR)
	@cp -f $(BUILD_DIR)/$(MOD_NAME) $(INSTALL_DIR)/
	@cp -f $(BUILD_DIR)/libfoxglove.so $(INSTALL_LIB_DIR)/ 2>/dev/null || true
	@for m in $(INSTALL_MODEL_FILES); do \
		if [ -f "$(CUR_PATH)/models/$$m" ]; then cp -f "$(CUR_PATH)/models/$$m" "$(INSTALL_MODEL_DIR)/"; fi; \
	done
	@echo "[Installed] $(INSTALL_DIR)"

uninstall:
	@rm -rf $(INSTALL_DIR)
	@echo "[Removed] $(INSTALL_DIR)"

clean:
	@echo "[Cleaning build directory...]"
	@rm -rf $(BUILD_DIR)

distclean: clean uninstall
	@echo "[Distclean completed.]"
