// SPDX-License-Identifier: GPL-2.0 WITH Linux-syscall-note
/*
 *
 * (C) COPYRIGHT 2015-2022 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms
 * of such GNU license.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, you can access it online at
 * http://www.gnu.org/licenses/gpl-2.0.html.
 *
 */

#include <mali_kbase.h>
#include <mali_kbase_defs.h>
#include <linux/pm_runtime.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/regulator/consumer.h>
#include <linux/delay.h>
#include <linux/io.h>

#include "mali_kbase_config_platform.h"
#include "mali_scaling.h"
#include "mali_clock.h"

void *reg_base_reset = NULL;
static int first = 1;

#define Rd(r)                           readl((reg_base_reset) + ((r)<<2))
#define Wr(r, v)                        writel((v), ((reg_base_reset) + ((r)<<2)))
#define Mali_WrReg(regnum, value)       writel((value), kbdev->reg + (regnum))
#define Mali_RdReg(regnum)              readl(kbdev->reg + (regnum))

extern u64 kbase_pm_get_ready_cores(struct kbase_device *kbdev, enum kbase_pm_core_type type);

//[0]:CG [1]:SC0 [2]:SC2
static void  Mali_pwr_on_with_kdev( struct kbase_device *kbdev, uint32_t  mask)
{
	uint32_t    part1_done;
	uint32_t    shader_present;
	uint32_t    tiler_present;
	uint32_t    l2_present;

	part1_done = 0;
	Mali_WrReg(0x0000024, 0xffffffff); // clear interrupts

	shader_present = Mali_RdReg(0x100);
	tiler_present  = Mali_RdReg(0x110);
	l2_present     = Mali_RdReg(0x120);
	dev_dbg(kbdev->dev, "shader_present=%d, tiler_present=%d, l2_present=%d\n",
	shader_present, tiler_present, l2_present);

	if (  mask == 0 ) {
		Mali_WrReg(0x00000180, 0xffffffff);   // Power on all cores (shader low)
		Mali_WrReg(0x00000184, 0xffffffff);   // Power on all cores (shader high)
		Mali_WrReg(0x00000190, 0xffffffff);   // Power on all cores (tiler low)
		Mali_WrReg(0x00000194, 0xffffffff);   // Power on all cores (tiler high)
		Mali_WrReg(0x000001a0, 0xffffffff);   // Power on all cores (l2 low)
		Mali_WrReg(0x000001a4, 0xffffffff);   // Power on all cores (l2 high)
	} else {
		Mali_WrReg(0x00000180, mask);    // Power on all cores (shader low)
		Mali_WrReg(0x00000184, 0);       // Power on all cores (shader high)
		Mali_WrReg(0x00000190, mask);    // Power on all cores (tiler low)
		Mali_WrReg(0x00000194, 0);       // Power on all cores (tiler high)
		Mali_WrReg(0x000001a0, mask);    // Power on all cores (l2 low)
		Mali_WrReg(0x000001a4, 0);       // Power on all cores (l2 high)
	}

	part1_done = Mali_RdReg(0x0000020);
	while (0 == part1_done) { part1_done = Mali_RdReg(0x00000020); }
	Mali_WrReg(0x0000024, 0xffffffff); // clear interrupts
}

/*
 * reset note:
 * there have mali reset and mali_capb bus reset.
 * reset step:[vlsi suggestion]
 * step1. mali-reset and mali-capb-reset level and mask config to 0
 * step2. mali-reset and mali-capb-reset level config to 1
 * Todo: sync old ic similar with t7,reset reg config by dts
 */
static void mali_reset(void)
{
	mali_plat_info_t* pmali_plat = get_mali_plat_data();
	u32 value = 0;

	/*
	 * different ic may have different reset bit
	 */
	if (pmali_plat->reset_flag & MALI_RESET_MODULE) {
		value = Rd(pmali_plat->module_reset.reg_mask);
		value = value & (~(0x1<<pmali_plat->module_reset.reg_bit));
		Wr(pmali_plat->module_reset.reg_mask, value);

		value = Rd(pmali_plat->module_reset.reg_level);
		value = value & (~(0x1<<pmali_plat->module_reset.reg_bit));
		Wr(pmali_plat->module_reset.reg_level, value);
	}

	if (pmali_plat->reset_flag & MALI_RESET_APB_BUS) {
		value = Rd(pmali_plat->apb_reset.reg_mask);
		value = value & (~(0x1<<pmali_plat->apb_reset.reg_bit));
		Wr(pmali_plat->apb_reset.reg_mask, value);

		value = Rd(pmali_plat->apb_reset.reg_level);
		value = value & (~(0x1<<pmali_plat->apb_reset.reg_bit));
		Wr(pmali_plat->apb_reset.reg_level, value);

		value = Rd(pmali_plat->apb_reset.reg_level);
		value = value | ((0x1<<pmali_plat->apb_reset.reg_bit));
		Wr(pmali_plat->apb_reset.reg_level, value);
	}

	if (pmali_plat->reset_flag & MALI_RESET_MODULE) {
		value = Rd(pmali_plat->module_reset.reg_level);
		value = value | ((0x1<<pmali_plat->module_reset.reg_bit));
		Wr(pmali_plat->module_reset.reg_level, value);
	}
}

/*
 * 1.mali-reset;2.init pwr_override1;3.pwr_on one core manually
 */
static void mali_hw_init(struct kbase_device *kbdev)
{
	mali_reset();
	udelay(10); // OR POLL for reset done
	Mali_WrReg(GPU_CONTROL_REG(PWR_KEY), 0x2968A819);
	Mali_WrReg(GPU_CONTROL_REG(PWR_OVERRIDE1), 0xfff | (0x20<<16));
	Mali_pwr_on_with_kdev(kbdev, 0x1);
}

static int pm_callback_power_on(struct kbase_device *kbdev)
{
	int ret = 0;

	struct mali_plat_info_t *mpdata  = (struct mali_plat_info_t *) kbdev->platform_context;
	reg_base_reset = mpdata->reg_base_reset;


	if (first == 0) goto out;
	if (!pm_runtime_enabled(kbdev->dev)) {
		pm_runtime_enable(kbdev->dev);
		dev_dbg(kbdev->dev, "pm_runtime not enabled, enable it here\n");
		ret = pm_runtime_get_sync(kbdev->dev);
		udelay(100);
		dev_dbg(kbdev->dev, "pm_runtime_get_sync returned %d\n", ret);
	} else {
		dev_dbg(kbdev->dev, "pm_runtime enabled\n");
	}

	first = 0;
	mali_hw_init(kbdev);
out:
	return ret;
}

/*
 * pm_callback_power_off will be called when gpu have no job todo.
 * the pm_runtime_put_autosuspend will trigger gpu power reset
 * which will affect gpu work on T7,so we can't do put here
 * the out power of gpu on t7 will be power off by platform when suspend
 */
static void pm_callback_power_off(struct kbase_device *kbdev)
{
	dev_dbg(kbdev->dev, "pm_callback_power_off\n");
}

#if MALI_USE_CSF && defined(KBASE_PM_RUNTIME)
static void pm_callback_runtime_gpu_active(struct kbase_device *kbdev)
{
	unsigned long flags;
	int error;

	lockdep_assert_held(&kbdev->pm.lock);

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	WARN_ON(!kbdev->pm.backend.gpu_powered);
	WARN_ON(!kbdev->pm.active_count);
	WARN_ON(kbdev->pm.runtime_active);
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	if (pm_runtime_status_suspended(kbdev->dev)) {
		error = pm_runtime_get_sync(kbdev->dev);
		dev_dbg(kbdev->dev, "pm_runtime_get_sync returned %d", error);
	} else {
		/* Call the async version here, otherwise there could be
		 * a deadlock if the runtime suspend operation is ongoing.
		 * Caller would have taken the kbdev->pm.lock and/or the
		 * scheduler lock, and the runtime suspend callback function
		 * will also try to acquire the same lock(s).
		 */
		error = pm_runtime_get(kbdev->dev);
		dev_dbg(kbdev->dev, "pm_runtime_get returned %d", error);
	}

	kbdev->pm.runtime_active = true;
}

static void pm_callback_runtime_gpu_idle(struct kbase_device *kbdev)
{
	unsigned long flags;

	lockdep_assert_held(&kbdev->pm.lock);

	dev_dbg(kbdev->dev, "%s", __func__);

	spin_lock_irqsave(&kbdev->hwaccess_lock, flags);
	WARN_ON(!kbdev->pm.backend.gpu_powered);
	WARN_ON(kbdev->pm.backend.l2_state != KBASE_L2_OFF);
	WARN_ON(kbdev->pm.active_count);
	WARN_ON(!kbdev->pm.runtime_active);
	spin_unlock_irqrestore(&kbdev->hwaccess_lock, flags);

	pm_runtime_mark_last_busy(kbdev->dev);
	pm_runtime_put_autosuspend(kbdev->dev);
	kbdev->pm.runtime_active = false;
}
#endif

#ifdef KBASE_PM_RUNTIME
static int kbase_device_runtime_init(struct kbase_device *kbdev)
{
	int ret = 0;

	dev_dbg(kbdev->dev, "kbase_device_runtime_init\n");
	pm_runtime_enable(kbdev->dev);
	ret = pm_runtime_get_sync(kbdev->dev);
	dev_dbg(kbdev->dev, "pm_runtime_get_sync ret=%d\n", ret);

	return ret;
}

static void kbase_device_runtime_disable(struct kbase_device *kbdev)
{
	dev_dbg(kbdev->dev, "kbase_device_runtime_disable\n");
	pm_runtime_disable(kbdev->dev);
}
#endif

static int pm_callback_runtime_on(struct kbase_device *kbdev)
{
	dev_dbg(kbdev->dev, "pm_callback_runtime_on\n");

	return 0;
}

static void pm_callback_runtime_off(struct kbase_device *kbdev)
{
	dev_dbg(kbdev->dev, "pm_callback_runtime_off\n");
}

/*
 * the out power of gpu on t7[vdd_gpu] will be power off by platform when suspend,
 * which will result in the gpu internal register reset to default.
 * all of the ic before t7 use the vdd_ee,which will never power down even suspend,
 * so we need do once init and check by pwr_override1.
 */
static void pm_callback_resume(struct kbase_device *kbdev)
{
	int ret;
	u32 pwr_override1;
	struct mali_plat_info_t *mpdata;
	struct clk *clk_mali;

	dev_dbg(kbdev->dev, "pm_callback_resume in\n");
	/* clock resume avoid clk be changed by system */
	mpdata  = (struct mali_plat_info_t *) kbdev->platform_context;
	clk_mali = mpdata->clk_mali;
	dev_dbg(kbdev->dev, "clk_mali = %lu\n", clk_get_rate(clk_mali));
	if (__clk_is_enabled(clk_mali)) {
		clk_disable_unprepare(clk_mali);
		dev_dbg(kbdev->dev, "reset gpu clock\n");
	}
	mali_clock_init_clk_tree(mpdata->pdev);
	dev_dbg(kbdev->dev, "mali clock resume done\n");
	if (!pm_runtime_enabled(kbdev->dev)) {
		pm_runtime_enable(kbdev->dev);
		dev_dbg(kbdev->dev, "pm_runtime not enable, enable here\n");
	} else {
		dev_dbg(kbdev->dev, "pm_runtime enabled already\n");
	}
	ret = pm_runtime_get_sync(kbdev->dev);
	dev_dbg(kbdev->dev, "pm_runtime_get_sync ret=%d\n", ret);
	Mali_WrReg(GPU_CONTROL_REG(PWR_KEY), 0x2968A819);
	pwr_override1 = Mali_RdReg(GPU_CONTROL_REG(PWR_OVERRIDE1));
	if (!pwr_override1) {
		dev_dbg(kbdev->dev, "pwr_override1=0,need do once init\n");
		mali_hw_init(kbdev);
	}
	ret = pm_callback_runtime_on(kbdev);
	dev_dbg(kbdev->dev, "pm_callback_resume out\n");
}

/* the out power of gpu on t7 will be power off by platform when suspend */
static void pm_callback_suspend(struct kbase_device *kbdev)
{
	struct mali_plat_info_t *mpdata;
	struct clk *clk_mali;

	mpdata  = (struct mali_plat_info_t *) kbdev->platform_context;
	clk_mali = mpdata->clk_mali;
	dev_dbg(kbdev->dev, "pm_callback_suspend in\n");
	pm_callback_runtime_off(kbdev);
	pm_runtime_put_sync(kbdev->dev);
	pm_runtime_disable(kbdev->dev);
	/* for power save,disable gpu clock */
	if (__clk_is_enabled(clk_mali)) {
		clk_disable_unprepare(clk_mali);
		dev_dbg(kbdev->dev, "disable gpu clk done\n");
	} else {
		dev_dbg(kbdev->dev, "gpu clk have disable before\n");
	}
	dev_dbg(kbdev->dev, "pm_callback_suspend out\n");
}

struct kbase_pm_callback_conf pm_callbacks = {
	.power_on_callback = pm_callback_power_on,
	.power_off_callback = pm_callback_power_off,
	.power_suspend_callback = pm_callback_suspend,
	.power_resume_callback = pm_callback_resume,
#ifdef KBASE_PM_RUNTIME
	.power_runtime_init_callback = kbase_device_runtime_init,
	.power_runtime_term_callback = kbase_device_runtime_disable,
	.power_runtime_on_callback = pm_callback_runtime_on,
	.power_runtime_off_callback = pm_callback_runtime_off,
#else				/* KBASE_PM_RUNTIME */
	.power_runtime_init_callback = NULL,
	.power_runtime_term_callback = NULL,
	.power_runtime_on_callback = NULL,
	.power_runtime_off_callback = NULL,
#endif				/* KBASE_PM_RUNTIME */

#if MALI_USE_CSF && defined(KBASE_PM_RUNTIME)
	.power_runtime_gpu_idle_callback = pm_callback_runtime_gpu_idle,
	.power_runtime_gpu_active_callback = pm_callback_runtime_gpu_active,
#else
	.power_runtime_gpu_idle_callback = NULL,
	.power_runtime_gpu_active_callback = NULL,
#endif
};
