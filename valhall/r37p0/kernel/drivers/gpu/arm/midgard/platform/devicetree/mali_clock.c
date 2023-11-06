/*
 * mali_clock.c
 *
 * Copyright (C) 2017 Amlogic, Inc. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include "mali_scaling.h"
#include "mali_clock.h"

#ifndef AML_CLK_LOCK_ERROR
#define AML_CLK_LOCK_ERROR 1
#endif

static unsigned gpu_dbg_level = 0;
module_param(gpu_dbg_level, uint, 0644);
MODULE_PARM_DESC(gpu_dbg_level, "gpu debug level");

#define gpu_dbg(level, fmt, arg...)			\
	do {			\
		if (gpu_dbg_level >= (level))		\
		printk("gpu_debug"fmt , ## arg); 	\
	} while (0)

#define GPU_CLK_DBG(fmt, arg...)

static mali_plat_info_t* pmali_plat = NULL;
int mali_pm_statue = 0;

int mali_clock_init_clk_tree(struct platform_device* pdev)
{
	mali_dvfs_threshold_table *dvfs_tbl = &pmali_plat->dvfs_table[pmali_plat->def_clock];
	struct clk *clk_mali = pmali_plat->clk_mali;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
	if ((0 == strcmp(dvfs_tbl->clk_parent, "gp0_pll")) &&
			!IS_ERR(dvfs_tbl->clkp_handle) &&
			(0 != dvfs_tbl->clkp_freq)) {
		if (__clk_is_enabled(dvfs_tbl->clkp_handle))
			pr_info("clk gp0_pll have enabled\n");
		else
			clk_prepare_enable(dvfs_tbl->clkp_handle);
		clk_set_rate(dvfs_tbl->clkp_handle, dvfs_tbl->clkp_freq);
	}
	if (__clk_is_enabled(clk_mali))
		pr_info("clk mali have enabled\n");
	else
		clk_prepare_enable(clk_mali);
	clk_set_rate(clk_mali, dvfs_tbl->clk_freq);
#else
	/* pay attetion of the sequence,
	 * if the set clock first,then enable clock
	 * the clock may not be ready really.
	 * which is relaated with our double gpu clk.
	 */
	dev_dbg(&pdev->dev, "kernel version >= 5.4\n");
	if (__clk_is_enabled(clk_mali))
		dev_dbg(&pdev->dev, "kernel version >= 5.4\n");
	else
		clk_prepare_enable(clk_mali);
	clk_set_rate(clk_mali, dvfs_tbl->clk_freq);
#endif

	return 0;
}

int mali_clock_init(mali_plat_info_t *pdev)
{
	*pdev = *pdev;
	return 0;
}

int mali_clock_critical(critical_t critical, size_t param)
{
	int ret = 0;

	ret = critical(param);

	return ret;
}

static int critical_clock_set(size_t param)
{
	int ret = 0;
	unsigned int idx = param;
	mali_dvfs_threshold_table *dvfs_tbl = &pmali_plat->dvfs_table[idx];

	struct clk *clk_mali   = pmali_plat->clk_mali;

	GPU_CLK_DBG();
	ret = clk_set_rate(clk_mali, dvfs_tbl->clk_freq);
	GPU_CLK_DBG();

#ifndef AML_CLK_LOCK_ERROR
	clk_disable_unprepare(clk_mali_x_old);
#endif

	return 0;
}

int mali_clock_set(unsigned int clock)
{
	return mali_clock_critical(critical_clock_set, (size_t)clock);
}

void disable_clock(void)
{
#ifndef AML_CLK_LOCK_ERROR
	struct clk *clk_mali = pmali_plat->clk_mali;

	GPU_CLK_DBG();
	clk_disable_unprepare(clk_mali);
#endif
	GPU_CLK_DBG();
}

void enable_clock(void)
{
#ifndef AML_CLK_LOCK_ERROR
	struct clk *clk_mali = pmali_plat->clk_mali;

	clk_prepare_enable(clk_mali);
#endif
	GPU_CLK_DBG();
}

u32 get_mali_freq(u32 idx)
{
	if (!mali_pm_statue) {
		return pmali_plat->clk_sample[idx];
	} else {
		return 0;
	}
}

void set_str_src(u32 data)
{
	printk("gpu: %s, %s, %d\n", __FILE__, __func__, __LINE__);
}

int mali_reset_info(struct platform_device *pdev, struct device_node *gpu_dn,
		    struct mali_plat_info_t *mpdata)
{
	struct device_node *reset_dn, *apb_reset_dn;
	int ret = 0;

	mpdata->reset_flag = 0;
	reset_dn = of_get_child_by_name(gpu_dn, "reset_cfg");
	if (!reset_dn) {
		mpdata->reset_flag &= ~(1 << MALI_RESET_MODULE);
		dev_dbg(&pdev->dev, "no reset_cfg\n");
	} else {
		ret = of_property_read_u32(reset_dn,"reg_level",
			&mpdata->module_reset.reg_level);
		if (ret) {
			dev_err(&pdev->dev, "no reg_level for reset\n");
			return -ENOMEM;
		}
		ret = of_property_read_u32(reset_dn,"reg_mask",
			&mpdata->module_reset.reg_mask);
		if (ret) {
			dev_err(&pdev->dev, "no reg_mask for reset\n");
			return -ENOMEM;
		}
		ret = of_property_read_u32(reset_dn,"reg_bit",
			&mpdata->module_reset.reg_bit);
		if (ret) {
			dev_err(&pdev->dev, "no reg_bit for reset\n");
			return -ENOMEM;
		}
		mpdata->reset_flag |= 1 << MALI_RESET_MODULE;
	}
	apb_reset_dn = of_get_child_by_name(gpu_dn, "capb_reset");
	if (!apb_reset_dn) {
		mpdata->reset_flag &= ~(1 << MALI_RESET_APB_BUS);
		dev_dbg(&pdev->dev, "no apb_reset\n");
	} else {
		ret = of_property_read_u32(apb_reset_dn,"reg_level",
			&mpdata->apb_reset.reg_level);
		if (ret) {
			dev_err(&pdev->dev, "no reg_level for apb_reset\n");
			return -ENOMEM;
		}
		ret = of_property_read_u32(apb_reset_dn,"reg_mask",
			&mpdata->apb_reset.reg_mask);
		if (ret) {
			dev_err(&pdev->dev, "no reg_mask for apb_reset\n");
			return -ENOMEM;
		}
		ret = of_property_read_u32(apb_reset_dn,"reg_bit",
			&mpdata->apb_reset.reg_bit);
		if (ret) {
			dev_err(&pdev->dev, "no reg_bit for apb_reset\n");
			return -ENOMEM;
		}
		mpdata->reset_flag |= 1 << MALI_RESET_APB_BUS;
	}
	return ret;
}

int mali_dt_info(struct platform_device *pdev, struct mali_plat_info_t *mpdata)
{
	struct device_node *gpu_dn = pdev->dev.of_node;
	struct device_node *gpu_clk_dn;
	phandle dvfs_clk_hdl;
	mali_dvfs_threshold_table *dvfs_tbl = NULL;
	uint32_t *clk_sample = NULL;

	struct property *prop;
	const __be32 *p;
	int length = 0, i = 0;
	u32 u;

	int ret = 0;
	if (!gpu_dn) {
		dev_err(&pdev->dev, "gpu device node not right\n");
		return -ENODEV;
	}

	ret = of_property_read_u32(gpu_dn,"num_of_pp",
			&mpdata->cfg_pp);
	if (ret) {
		dev_dbg(&pdev->dev, "set max pp to default 6\n");
		mpdata->cfg_pp = 6;
	}
	mpdata->scale_info.maxpp = mpdata->cfg_pp;
	mpdata->maxpp_sysfs = mpdata->cfg_pp;
	dev_dbg(&pdev->dev, "max pp is %d\n", mpdata->scale_info.maxpp);

	ret = of_property_read_u32(gpu_dn,"min_pp",
			&mpdata->cfg_min_pp);
	if (ret) {
		dev_dbg(&pdev->dev, "set min pp to default 1\n");
		mpdata->cfg_min_pp = 1;
	}
	mpdata->scale_info.minpp = mpdata->cfg_min_pp;
	dev_dbg(&pdev->dev, "min pp is %d\n", mpdata->scale_info.minpp);

	ret = of_property_read_u32(gpu_dn,"min_clk",
			&mpdata->cfg_min_clock);
	if (ret) {
		dev_dbg(&pdev->dev, "set min clk default to 0\n");
		mpdata->cfg_min_clock = 0;
	}
	mpdata->scale_info.minclk = mpdata->cfg_min_clock;
	dev_dbg(&pdev->dev, "min clk  is %d\n", mpdata->scale_info.minclk);

	mpdata->reg_base_reset = of_iomap(gpu_dn, 1);
	dev_dbg(&pdev->dev, "reset bus 0x%p\n", mpdata->reg_base_reset);

	ret = of_property_read_u32(gpu_dn,"sc_mpp",
			&mpdata->sc_mpp);
	if (ret) {
		dev_dbg(&pdev->dev, "set pp used most of time default to %d\n", mpdata->cfg_pp);
		mpdata->sc_mpp = mpdata->cfg_pp;
	}
	dev_dbg(&pdev->dev, "num of pp used most of time %d\n", mpdata->sc_mpp);

	of_get_property(gpu_dn, "tbl", &length);

	length = length /sizeof(u32);
	dev_dbg(&pdev->dev, "clock dvfs cfg table size is %d\n", length);

	mpdata->dvfs_table = devm_kzalloc(&pdev->dev,
								  sizeof(struct mali_dvfs_threshold_table)*length,
								  GFP_KERNEL);
	dvfs_tbl = mpdata->dvfs_table;
	if (mpdata->dvfs_table == NULL) {
		dev_err(&pdev->dev, "failed to alloc dvfs table\n");
		return -ENOMEM;
	}
	mpdata->clk_sample = devm_kzalloc(&pdev->dev, sizeof(u32)*length, GFP_KERNEL);
	if (mpdata->clk_sample == NULL) {
		dev_err(&pdev->dev, "failed to alloc clk_sample table\n");
		return -ENOMEM;
	}
	clk_sample = mpdata->clk_sample;
	/* mali external reset reg */
	ret = mali_reset_info(pdev, gpu_dn, mpdata);
	if (ret)
		return ret;
	/* dvfs clk table */
	of_property_for_each_u32(gpu_dn, "tbl", prop, p, u) {
		dvfs_clk_hdl = (phandle) u;
		gpu_clk_dn = of_find_node_by_phandle(dvfs_clk_hdl);
		ret = of_property_read_u32(gpu_clk_dn,"clk_freq", &dvfs_tbl->clk_freq);
		if (ret) {
			dev_err(&pdev->dev, "read clk_freq failed\n");
		}

		ret = of_property_read_string(gpu_clk_dn,"clk_parent",
				&dvfs_tbl->clk_parent);
		if (ret) {
			dev_err(&pdev->dev, "read clk_parent failed\n");
#if (LINUX_VERSION_CODE < KERNEL_VERSION(5, 4, 0))
		} else if (0 == strcmp(dvfs_tbl->clk_parent, "gp0_pll")) {
			dvfs_tbl->clkp_handle = devm_clk_get(&pdev->dev, dvfs_tbl->clk_parent);
			if (IS_ERR(dvfs_tbl->clkp_handle)) {
				dev_err(&pdev->dev, "failed to get %s's clock pointer\n", dvfs_tbl->clk_parent);
			}
			ret = of_property_read_u32(gpu_clk_dn,"clkp_freq", &dvfs_tbl->clkp_freq);
			if (ret) {
				dev_err(&pdev->dev, "read clk_parent freq failed\n");
			}
#endif
		}

		ret = of_property_read_u32(gpu_clk_dn,"voltage", &dvfs_tbl->voltage);
		if (ret) {
			dev_err(&pdev->dev, "read voltage failed\n");
		}
		ret = of_property_read_u32(gpu_clk_dn,"keep_count", &dvfs_tbl->keep_count);
		if (ret) {
			dev_err(&pdev->dev, "read keep_count failed\n");
		}
		//downthreshold and upthreshold shall be u32
		ret = of_property_read_u32_array(gpu_clk_dn,"threshold",
		&dvfs_tbl->downthreshold, 2);
		if (ret) {
			dev_err(&pdev->dev, "read threshold failed\n");
		}
		dvfs_tbl->freq_index = i;

		*clk_sample = dvfs_tbl->clk_freq / 1000000;

		dvfs_tbl ++;
		clk_sample ++;
		i++;
		mpdata->dvfs_table_size ++;
	}

	ret = of_property_read_u32(gpu_dn,"max_clk",
			&mpdata->cfg_clock);
	if (ret) {
		dev_dbg(&pdev->dev, "max clk set %d\n", mpdata->dvfs_table_size-2);
		mpdata->cfg_clock = mpdata->dvfs_table_size-2;
	}

	mpdata->cfg_clock_bkup = mpdata->cfg_clock;
	mpdata->maxclk_sysfs = mpdata->cfg_clock;
	mpdata->scale_info.maxclk = mpdata->cfg_clock;
	dev_dbg(&pdev->dev, "max clk  is %d\n", mpdata->scale_info.maxclk);

	ret = of_property_read_u32(gpu_dn,"turbo_clk",
			&mpdata->turbo_clock);
	if (ret) {
		dev_dbg(&pdev->dev, "turbo clk set to %d\n", mpdata->dvfs_table_size-1);
		mpdata->turbo_clock = mpdata->dvfs_table_size-1;
	}
	dev_dbg(&pdev->dev, "turbo clk  is %d\n", mpdata->turbo_clock);

	ret = of_property_read_u32(gpu_dn,"def_clk",
			&mpdata->def_clock);
	if (ret) {
		mpdata->def_clock = mpdata->scale_info.maxclk;
		dev_dbg(&pdev->dev, "default clk set to %d\n", mpdata->def_clock);
	}
	if (mpdata->def_clock > mpdata->scale_info.maxclk)
		mpdata->def_clock = mpdata->scale_info.maxclk;
	dev_dbg(&pdev->dev, "default clk  is %d\n", mpdata->def_clock);

	dvfs_tbl = mpdata->dvfs_table;
	clk_sample = mpdata->clk_sample;
	for (i = 0; i< mpdata->dvfs_table_size; i++) {
		dev_dbg(&pdev->dev, "====================%d====================\n"
		            "clk_freq=%10d, clk_parent=%9s, voltage=%d, keep_count=%d, threshold=<%d %d>, clk_sample=%d\n",
					i,
					dvfs_tbl->clk_freq, dvfs_tbl->clk_parent,
					dvfs_tbl->voltage,  dvfs_tbl->keep_count,
					dvfs_tbl->downthreshold, dvfs_tbl->upthreshold, *clk_sample);
		dvfs_tbl ++;
		clk_sample ++;
	}
	dev_dbg(&pdev->dev, "clock dvfs table size is %d\n", mpdata->dvfs_table_size);

	mpdata->clk_mali = devm_clk_get(&pdev->dev, "gpu_mux");
	if (IS_ERR(mpdata->clk_mali)) {
		dev_err(&pdev->dev, "failed to get clock pointer\n");
		return -EFAULT;
	}

	pmali_plat = mpdata;
	mpdata->pdev = pdev;
	return 0;
}
