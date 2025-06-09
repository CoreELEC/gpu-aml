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
void *reg_base_dmc = NULL;
static int first = 1;

//remove this if sc2 worked fine.
//g12a
#define RESET0_MASK    0x10
#define RESET1_MASK    0x11
#define RESET2_MASK    0x12
#define RESET0_LEVEL   0x20
#define RESET1_LEVEL   0x21
#define RESET2_LEVEL   0x22
//sc2
#define RESETCTRL_RESET1_LEVEL  0x11
#define RESETCTRL_RESET1_MASK   0x21

#define Rd(r)                           readl((reg_base_reset) + ((r)<<2))
#define Wr(r, v)                        writel((v), ((reg_base_reset) + ((r)<<2)))
#define Mali_WrReg(regnum, value)       writel((value), kbdev->reg + (regnum))
#define Mali_RdReg(regnum)              readl(kbdev->reg + (regnum))

extern u64 kbase_pm_get_ready_cores(struct kbase_device *kbdev, enum kbase_pm_core_type type);

void get_dmc_gpu_status(void)
{
	u32 dmc_chan_sts, rst1_value, rst2_value;
	mali_plat_info_t* pmali_plat = get_mali_plat_data();
	struct platform_device* ptr_plt_dev = pmali_plat->pdev;
	struct kbase_device *kbdev = dev_get_drvdata(&ptr_plt_dev->dev);

	if (reg_base_dmc && pmali_plat->dmc_reset_flag) {
		dmc_chan_sts = readl(reg_base_dmc + (pmali_plat->dmc_reset.reg_sts << 2));
		rst1_value = readl(reg_base_dmc + (pmali_plat->dmc_reset.reg_rst << 2));
		rst2_value = readl(reg_base_dmc + (pmali_plat->dmc_reset.reg_rst1 << 2));
		dev_info(kbdev->dev, "gpu dmc reset vale: 0x%x,0x%x, dmc chan sts: 0x%x\n",
			rst1_value, rst2_value, dmc_chan_sts);
	}
}

void reset_dmc_gpu(void)
{
	u32 rst1_gpu, rst2_gpu, rst1_value, rst2_value;
	mali_plat_info_t* pmali_plat = get_mali_plat_data();
	struct platform_device* ptr_plt_dev = pmali_plat->pdev;
	struct kbase_device *kbdev = dev_get_drvdata(&ptr_plt_dev->dev);

	if (reg_base_dmc && pmali_plat->dmc_reset_flag) {
		dev_info(kbdev->dev, "gpu dmc reset start\n");
		rst1_value = readl(reg_base_dmc + (pmali_plat->dmc_reset.reg_rst << 2));
		rst2_value = readl(reg_base_dmc + (pmali_plat->dmc_reset.reg_rst1 << 2));
		rst1_gpu = rst1_value & (~(1 << pmali_plat->dmc_reset.reg_bit));
		rst2_gpu = rst2_value & (~(1 << pmali_plat->dmc_reset.reg_bit));
		dev_info(kbdev->dev, "gpu dmc reset vale 0x%x,0x%x,0x%x,0x%x\n",
			rst1_value, rst2_value, rst1_gpu, rst2_gpu);
		writel(rst1_gpu, reg_base_dmc + (pmali_plat->dmc_reset.reg_rst << 2));
		writel(rst2_gpu, reg_base_dmc + (pmali_plat->dmc_reset.reg_rst1 << 2));
		mdelay(1);
		writel(rst1_value, reg_base_dmc + (pmali_plat->dmc_reset.reg_rst << 2));
		writel(rst2_value, reg_base_dmc + (pmali_plat->dmc_reset.reg_rst1 << 2));
		dev_info(kbdev->dev, "gpu dmc reset end\n");
	}
}

//[0]:CG [1]:SC0 [2]:SC2
static void  Mali_pwr_on_with_kdev( struct kbase_device *kbdev, uint32_t  mask)
{
	uint32_t part1_done;
	uint32_t shader_present;
	uint32_t tiler_present;
	uint32_t l2_present;
	uint32_t timeout_ms;

	part1_done = 0;
	timeout_ms = 500;
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
	while (!part1_done && timeout_ms) {
		part1_done = Mali_RdReg(0x00000020);
		mdelay(10);
		timeout_ms -= 10;
	}
	if (!timeout_ms)
		dev_info(kbdev->dev, "%s wait power change timeout\n", __func__);
	Mali_WrReg(0x0000024, 0xffffffff); // clear interrupts
}

/*reset function before t7 by register*/
static void mali_reset_v0(int reset_g12a)
{
	u32 value;

	if (reset_g12a) {
		value = Rd(RESET0_MASK);
		value = value & (~(0x1<<20));
		Wr(RESET0_MASK, value);

		value = Rd(RESET0_LEVEL);
		value = value & (~(0x1<<20));
		Wr(RESET0_LEVEL, value);

		value = Rd(RESET2_MASK);
		value = value & (~(0x1<<14));
		Wr(RESET2_MASK, value);

		value = Rd(RESET2_LEVEL);
		value = value & (~(0x1<<14));
		Wr(RESET2_LEVEL, value);

		value = Rd(RESET0_LEVEL);
		value = value | ((0x1<<20));
		Wr(RESET0_LEVEL, value);

		value = Rd(RESET2_LEVEL);
		value = value | ((0x1<<14));
		Wr(RESET2_LEVEL, value);
	} else {
		/* JOHNT: remove this if sc2 worked fine. */
		value = ~(1 << 2);
		Wr(RESETCTRL_RESET1_MASK, value);
		Wr(RESETCTRL_RESET1_LEVEL, value);
		Wr(RESETCTRL_RESET1_LEVEL, 0xFFFFFFFF);
	}
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
	int reset_g12a = pmali_plat->reset_g12a;

	if (!pmali_plat->reset_flag) {
		mali_reset_v0(reset_g12a);
		return;
	}

	/*
	 * for t7 , if old ic also sync with the follow method,
	 * need sync the dts similar with T7
	 */
	value = Rd(pmali_plat->module_reset.reg_mask);
	value = value & (~(0x1<<pmali_plat->module_reset.reg_bit));
	Wr(pmali_plat->module_reset.reg_mask, value);

	value = Rd(pmali_plat->module_reset.reg_level);
	value = value & (~(0x1<<pmali_plat->module_reset.reg_bit));
	Wr(pmali_plat->module_reset.reg_level, value);

	value = Rd(pmali_plat->apb_reset.reg_mask);
	value = value & (~(0x1<<pmali_plat->apb_reset.reg_bit));
	Wr(pmali_plat->apb_reset.reg_mask, value);

	value = Rd(pmali_plat->apb_reset.reg_level);
	value = value & (~(0x1<<pmali_plat->apb_reset.reg_bit));
	Wr(pmali_plat->apb_reset.reg_level, value);

	value = Rd(pmali_plat->apb_reset.reg_level);
	value = value | ((0x1<<pmali_plat->apb_reset.reg_bit));
	Wr(pmali_plat->apb_reset.reg_level, value);

	value = Rd(pmali_plat->module_reset.reg_level);
	value = value | ((0x1<<pmali_plat->module_reset.reg_bit));
	Wr(pmali_plat->module_reset.reg_level, value);
}

void mali_reset_top(void)
{
	mali_plat_info_t* pmali_plat = get_mali_plat_data();
	struct platform_device* ptr_plt_dev = pmali_plat->pdev;
	struct kbase_device *kbdev = dev_get_drvdata(&ptr_plt_dev->dev);

	dev_info(kbdev->dev, "gpu reset top start\n");
	mali_reset();
	dev_info(kbdev->dev, "gpu reset top end\n");
}

static void mali_reset_low(void)
{
	mali_plat_info_t* pmali_plat = get_mali_plat_data();
	u32 value = 0;
	int reset_g12a = pmali_plat->reset_g12a;

	if (!pmali_plat->reset_flag) {
		if (reset_g12a) {
			value = Rd(RESET0_MASK);
			value = value & (~(0x1<<20));
			Wr(RESET0_MASK, value);

			value = Rd(RESET0_LEVEL);
			value = value & (~(0x1<<20));
			Wr(RESET0_LEVEL, value);

			value = Rd(RESET2_MASK);
			value = value & (~(0x1<<14));
			Wr(RESET2_MASK, value);

			value = Rd(RESET2_LEVEL);
			value = value & (~(0x1<<14));
			Wr(RESET2_LEVEL, value);
		} else {
			value = ~(1 << 2);
			Wr(RESETCTRL_RESET1_MASK, value);
			Wr(RESETCTRL_RESET1_LEVEL, value);
		}
	} else {
		value = Rd(pmali_plat->module_reset.reg_mask);
		value = value & (~(0x1<<pmali_plat->module_reset.reg_bit));
		Wr(pmali_plat->module_reset.reg_mask, value);

		value = Rd(pmali_plat->module_reset.reg_level);
		value = value & (~(0x1<<pmali_plat->module_reset.reg_bit));
		Wr(pmali_plat->module_reset.reg_level, value);

		value = Rd(pmali_plat->apb_reset.reg_mask);
		value = value & (~(0x1<<pmali_plat->apb_reset.reg_bit));
		Wr(pmali_plat->apb_reset.reg_mask, value);

		value = Rd(pmali_plat->apb_reset.reg_level);
		value = value & (~(0x1<<pmali_plat->apb_reset.reg_bit));
		Wr(pmali_plat->apb_reset.reg_level, value);
	}
}

static void mali_reset_high(void)
{
	mali_plat_info_t* pmali_plat = get_mali_plat_data();
	u32 value = 0;
	int reset_g12a = pmali_plat->reset_g12a;

	if (!pmali_plat->reset_flag) {
		if (reset_g12a) {
			value = Rd(RESET0_LEVEL);
			value = value | ((0x1<<20));
			Wr(RESET0_LEVEL, value);

			value = Rd(RESET2_LEVEL);
			value = value | ((0x1<<14));
			Wr(RESET2_LEVEL, value);
		} else {
			Wr(RESETCTRL_RESET1_LEVEL, 0xFFFFFFFF);
		}
	} else {
		value = Rd(pmali_plat->apb_reset.reg_level);
		value = value | ((0x1<<pmali_plat->apb_reset.reg_bit));
		Wr(pmali_plat->apb_reset.reg_level, value);

		value = Rd(pmali_plat->module_reset.reg_level);
		value = value | ((0x1<<pmali_plat->module_reset.reg_bit));
		Wr(pmali_plat->module_reset.reg_level, value);
	}
}

/* 1:skip dmc reset, 0:dmc reset done*/
int mali_reset_top_dmc(struct kbase_device *kbdev)
{
	u32 value;
	mali_plat_info_t* pmali_plat = get_mali_plat_data();

	if (!pmali_plat->dmc_reset_flag) {
		dev_info(kbdev->dev, "no dmc reset config,skip dmc reset");
		return 1;
	}
	get_dmc_gpu_status();
	dev_info(kbdev->dev, "gpu reset top and dmc start");
	dev_info(kbdev->dev, "gpu reset top to 0");
	value = ~(1 << 2);
	mali_reset_low();
	mdelay(1);
	reset_dmc_gpu();
	dev_info(kbdev->dev, "gpu reset top level to 1");
	mali_reset_high();
	dev_info(kbdev->dev, "gpu hw init start");
	udelay(10); // OR POLL for reset done
	Mali_WrReg(GPU_CONTROL_REG(PWR_KEY), 0x2968A819);
	Mali_WrReg(GPU_CONTROL_REG(PWR_OVERRIDE1), 0xfff | (0x20<<16));
	Mali_pwr_on_with_kdev(kbdev, 0x1);
	dev_info(kbdev->dev, "gpu hw init end");
	dev_info(kbdev->dev, "gpu reset top and dmc end");
	return 0;
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

void mali_hw_init_ext(struct kbase_device *kbdev)
{
	dev_info(kbdev->dev, "gpu hw init start\n");
	mali_reset_top();
	udelay(10); // OR POLL for reset done
	Mali_WrReg(GPU_CONTROL_REG(PWR_KEY), 0x2968A819);
	Mali_WrReg(GPU_CONTROL_REG(PWR_OVERRIDE1), 0xfff | (0x20<<16));
	Mali_pwr_on_with_kdev(kbdev, 0x1);
	dev_info(kbdev->dev,"gpu hw init end\n");
}

static int pm_callback_power_on(struct kbase_device *kbdev)
{
	int ret = 0;

	struct mali_plat_info_t *mpdata  = (struct mali_plat_info_t *) kbdev->platform_context;
	reg_base_reset = mpdata->reg_base_reset;
	reg_base_dmc = mpdata->dmc_reset.reg_base_dmc;

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
		dev_info(kbdev->dev, "reset gpu clock\n");
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

	dev_info(kbdev->dev, "pm_callback_suspend in\n");
	pm_callback_runtime_off(kbdev);
	pm_runtime_put_sync(kbdev->dev);
	pm_runtime_disable(kbdev->dev);
	/* for power save,disable gpu clock */
	mpdata  = (struct mali_plat_info_t *) kbdev->platform_context;
	clk_mali = mpdata->clk_mali;
	if (__clk_is_enabled(clk_mali)) {
		clk_disable_unprepare(clk_mali);
		dev_info(kbdev->dev, "disable gpu clk done\n");
	} else {
		dev_info(kbdev->dev, "gpu clk have disable before\n");
	}
	dev_info(kbdev->dev, "pm_callback_suspend out\n");
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
};
