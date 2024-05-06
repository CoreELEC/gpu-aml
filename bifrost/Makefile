#
# Copyright (C) 2015 The Android Open Source Project
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#      http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#


GPU_TYPE:=dvalin
GPU_ARCH:=bifrost
GPU_DRV_VERSION?=r25p0
GPU_LOW_MEM?=0

EXTRA_INCLUDE := -I$(KERNEL_SRC)/$(M)/../bifrost/$(GPU_DRV_VERSION)/kernel/drivers/gpu/arm/midgard/backend/gpu \
                 -I$(KERNEL_SRC)/$(M)/../bifrost/$(GPU_DRV_VERSION)/kernel/drivers/gpu/arm/midgard/platform/devicetree \
                 -I$(KERNEL_SRC)/$(M)/../bifrost/$(GPU_DRV_VERSION)/kernel/drivers/gpu/arm/midgard/ipa \
                 -I$(KERNEL_SRC)/$(M)/../bifrost/$(GPU_DRV_VERSION)/kernel/drivers/gpu/arm/midgard \
                 -I$(KERNEL_SRC)/$(M)/../bifrost/$(GPU_DRV_VERSION)/kernel/include

KBUILD_CFLAGS_MODULE += $(GKI_EXT_MODULE_PREDEFINE)
$(warning "CC:"$(CC) )
$(warning "HOSTCC:"$(HOSTCC) )
$(warning "CROSS_COMPILE:"$(CROSS_COMPILE) )

ifeq ($(O),)
out_dir := .
else
out_dir := $(O)
endif
include $(out_dir)/include/config/auto.conf

modules:
	$(MAKE) -C $(KERNEL_SRC) M=$(M)/$(GPU_DRV_VERSION)/kernel/drivers/gpu/arm/midgard  \
	EXTRA_CFLAGS="-DCONFIG_MALI_PLATFORM_DEVICETREE -DCONFIG_MALI_MIDGARD_DVFS -DCONFIG_MALI_BACKEND=gpu -DCONFIG_MALI_GATOR_SUPPORT -DCONFIG_MALI_REAL_HW" \
	EXTRA_CFLAGS+="-DCONFIG_MALI_DMA_BUF_MAP_ON_DEMAND=1 -DCONFIG_MALI_DMA_BUF_LEGACY_COMPAT=0 -DCONFIG_MALI_LOW_MEM=$(GPU_LOW_MEM) -DCONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD=1 -DCONFIG_LARGE_PAGE_SUPPORT=0" \
	EXTRA_CFLAGS+="-Wno-error -Wno-pointer-sign -Wno-error=frame-larger-than= $(EXTRA_INCLUDE) $(KBUILD_CFLAGS_MODULE)" \
	EXTRA_LDFLAGS+="--strip-debug" \
	CONFIG_MALI_MIDGARD=m CONFIG_MALI_PLATFORM_DEVICETREE=y CONFIG_MALI_MIDGARD_DVFS=y CONFIG_MALI_BACKEND=gpu CONFIG_MALI_GATOR_SUPPORT=y CONFIG_LARGE_PAGE_SUPPORT=0 CONFIG_MALI_PLATFORM_NAME="devicetree" \
	    CONFIG_MALI_REAL_HW=y CONFIG_MALI_TRACE_POWER_GPU_WORK_PERIOD=y \

modules_install:
	$(MAKE) INSTALL_MOD_STRIP=1 M=$(M)/$(GPU_DRV_VERSION)/kernel/drivers/gpu/arm/midgard -C $(KERNEL_SRC) modules_install
	$(Q)mkdir -p ${out_dir}/../vendor_lib/modules
	$(Q)if [ -z "$(CONFIG_AMLOGIC_KERNEL_VERSION)" ]; then \
		cd ${out_dir}/$(M)/; find -name "*.ko" -exec cp {} ${out_dir}/../vendor_lib/modules/mali.ko \; ; \
	else \
		find $(INSTALL_MOD_PATH)/lib/modules/*/$(INSTALL_MOD_DIR) -name "*.ko" -exec cp {} ${out_dir}/../vendor_lib/modules/mali.ko \; ; \
	fi


clean:
	$(MAKE) -C $(KERNEL_SRC) M=$(M)/$(GPU_DRV_VERSION)/kernel/drivers/gpu/arm/midgard clean
