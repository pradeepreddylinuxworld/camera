/*
 * csiphy.c
 *
 * Qualcomm MSM Camera Subsystem - CSIPHY Module
 *
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2016 Linaro Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "csiphy.h"
#include "camss.h"

#define MSM_CSIPHY_NAME "msm_csiphy"

#define ULPM_WAKE_UP_TIMER_MODE                   2
#define GLITCH_ELIMINATION_NUM                    0x12 /* bit [6:4] */

struct csiphy_reg_parms_t csiphy_v3_5 = {
	.mipi_csiphy_interrupt_status0_addr = 0x8B0,
	.mipi_csiphy_interrupt_clear0_addr = 0x858,
	.mipi_csiphy_glbl_irq_cmd_addr = 0x828,
	.combo_clk_mask = 0x10,
};

struct csiphy_reg_3ph_parms_t csiphy_v3_5_3ph = {
	/*MIPI CSI PHY registers*/
	{0x814, 0x0},
	{0x818, 0x1},
	{0x188, 0x7F},
	{0x18C, 0x7F},
	{0x190, 0x0},
	{0x104, 0x6},
	{0x108, 0x0},
	{0x10c, 0x0},
	{0x114, 0x20},
	{0x118, 0x3E},
	{0x11c, 0x41},
	{0x120, 0x41},
	{0x124, 0x7F},
	{0x128, 0x0},
	{0x12c, 0x0},
	{0x130, 0x1},
	{0x134, 0x0},
	{0x138, 0x0},
	{0x13C, 0x10},
	{0x140, 0x1},
	{0x144, GLITCH_ELIMINATION_NUM},
	{0x148, 0xFE},
	{0x14C, 0x1},
	{0x154, 0x0},
	{0x15C, 0x33},
	{0x160, ULPM_WAKE_UP_TIMER_MODE},
	{0x164, 0x48},
	{0x168, 0xA0},
	{0x16C, 0x17},
	{0x170, 0x41},
	{0x174, 0x41},
	{0x178, 0x3E},
	{0x17C, 0x0},
	{0x180, 0x0},
	{0x184, 0x7F},
	{0x1cc, 0x10},
	{0x81c, 0x6},
	{0x82c, 0xFF},
	{0x830, 0xFF},
	{0x834, 0xFB},
	{0x838, 0xFF},
	{0x83c, 0x7F},
	{0x840, 0xFF},
	{0x844, 0xFF},
	{0x848, 0xEF},
	{0x84c, 0xFF},
	{0x850, 0xFF},
	{0x854, 0xFF},
	{0x28, 0x0},
	{0x800, 0x0},
	{0x0, 0xD7},
	{0x4, 0x8},
	{0x8, 0x0},
	{0xC, 0xA5},
	{0x10, 0x50},
	{0x2C, 0x1},
	{0x30, 0x2},
	{0x34, 0x3},
	{0x38, 0x1},
	{0x3C, 0xB8},
	{0x1C, 0xA},
	{0x14, 0x0},
	{0x0, 0x0},
	{0x700, 0xC0},
};



static const u32 csiphy_formats[] = {
	MEDIA_BUS_FMT_UYVY8_2X8,
	MEDIA_BUS_FMT_VYUY8_2X8,
	MEDIA_BUS_FMT_YUYV8_2X8,
	MEDIA_BUS_FMT_YVYU8_2X8,
	MEDIA_BUS_FMT_SBGGR8_1X8,
	MEDIA_BUS_FMT_SGBRG8_1X8,
	MEDIA_BUS_FMT_SGRBG8_1X8,
	MEDIA_BUS_FMT_SRGGB8_1X8,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR12_1X12,
	MEDIA_BUS_FMT_SGBRG12_1X12,
	MEDIA_BUS_FMT_SGRBG12_1X12,
	MEDIA_BUS_FMT_SRGGB12_1X12,
};

/*
 * csiphy_isr - CSIPHY module interrupt handler
 * @irq: Interrupt line
 * @dev: CSIPHY device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t csiphy_isr(int irq_num, void *dev)
{
#if 0
	uint32_t irq;
	int i;
	struct csiphy_device *csiphy_dev = dev;

	for (i = 0; i < csiphy_dev->num_irq_registers; i++) {
		irq = readl_relaxed(
			csiphy_dev->base +
			csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_interrupt_status0_addr + 0x4*i);
		writel_relaxed(irq,
			csiphy_dev->base +
			csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_interrupt_clear0_addr + 0x4*i);

		writel_relaxed(0x0,
			csiphy_dev->base +
			csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_interrupt_clear0_addr + 0x4*i);
	}
	writel_relaxed(0x1, csiphy_dev->base +
		csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_glbl_irq_cmd_addr);
	writel_relaxed(0x0, csiphy_dev->base +
		csiphy_dev->ctrl_reg->csiphy_reg.mipi_csiphy_glbl_irq_cmd_addr);
#endif
	return IRQ_HANDLED;
}

/*
 * csiphy_reset - Perform software reset on CSIPHY module
 * @csiphy: CSIPHY device
 */
static void csiphy_reset(struct csiphy_device *csiphy)
{
	writel_relaxed(0x1, csiphy->base +
		csiphy->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl0.addr);
	usleep_range(5000, 8000);
	writel_relaxed(0x0, csiphy->base +
		csiphy->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl0.addr);
}

/*
 * csiphy_set_power - Power on/off CSIPHY module
 * @sd: CSIPHY V4L2 subdevice
 * @on: Requested power state
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csiphy_set_power(struct v4l2_subdev *sd, int on)
{
	struct csiphy_device *csiphy = v4l2_get_subdevdata(sd);
	struct device *dev = to_device_index(csiphy, csiphy->id);
	int ret;
	pr_err("[camera] : %s, on = %d\n", __func__, on);
	if (on) {
//		u8 hw_version;
//		ret = cam_config_ahb_clk(NULL, 0, CAM_AHB_CLIENT_CSIPHY, CAM_AHB_SVS_VOTE);
//		if (ret < 0) {
//			pr_err("%s: failed to vote for AHB\n", __func__);
//			return ret;
//		}

		ret = camss_enable_clocks(csiphy->nclocks, csiphy->clock, dev);
		if (ret < 0)
			return ret;

		enable_irq(csiphy->irq);

		csiphy_reset(csiphy);

//		hw_version = readl_relaxed(csiphy->base +
//					   CAMSS_CSI_PHY_HW_VERSION);
//		dev_dbg(dev, "CSIPHY HW Version = 0x%02x\n", hw_version);
	} else {

		writel_relaxed(0x0,
			csiphy->base + csiphy->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl5.addr);
		writel_relaxed(0x0,
			csiphy->base + csiphy->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl6.addr);
		disable_irq(csiphy->irq);

		camss_disable_clocks(csiphy->nclocks, csiphy->clock);

//		if (cam_config_ahb_clk(NULL, 0, CAM_AHB_CLIENT_CSIPHY, CAM_AHB_SUSPEND_VOTE) < 0)
//			pr_err("%s: failed to remove vote for AHB\n", __func__);
	}

	return 0;
}

/*
 * csiphy_get_lane_mask - Calculate CSI2 lane mask configuration parameter
 * @lane_cfg - CSI2 lane configuration
 *
 * Return lane mask
 */
static u8 csiphy_get_lane_mask(struct csiphy_lanes_cfg *lane_cfg)
{
	u8 lane_mask;
	int i;

	lane_mask = 1 << lane_cfg->clk.pos;

	for (i = 0; i < lane_cfg->num_data; i++)
		lane_mask |= 1 << lane_cfg->data[i].pos;

	return lane_mask;
}

static void msm_csiphy_cphy_irq_config(
	struct csiphy_device *csiphy_dev,
	struct msm_camera_csiphy_params *csiphy_params)
{
	void __iomem *csiphybase;

	csiphybase = csiphy_dev->base;
	writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl11.data,
		csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl11.addr);
	writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl12.data,
		csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl12.addr);
	writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl13.data,
		csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl13.addr);
	writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl14.data,
		csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl14.addr);
	writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl15.data,
		csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl15.addr);
	writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl16.data,
		csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl16.addr);
	writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl17.data,
		csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl17.addr);
	writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl18.data,
		csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl18.addr);
	writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl19.data,
		csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl19.addr);
	writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl20.data,
		csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl20.addr);
	writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl21.data,
		csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl21.addr);
}


#define MAX_LANES                                   4
#define CLOCK_OFFSET                              0x700

static int msm_csiphy_2phase_lane_config(
	struct csiphy_device *csiphy_dev,
	struct msm_camera_csiphy_params *csiphy_params)
{
	uint32_t val = 0, lane_enable = 0, clk_lane, mask = 1;
	uint16_t lane_mask = 0, i = 0, offset;
	void __iomem *csiphybase;

	csiphybase = csiphy_dev->base;
	lane_mask = csiphy_params->lane_mask & 0x1f;
	for (i = 0; i < MAX_LANES; i++) {
		if (mask == 0x2) {
			if (lane_mask & mask)
				lane_enable |= 0x80;
			i--;
		} else if (lane_mask & mask)
			lane_enable |= 0x1 << (i<<1);
		mask <<= 1;
	}
	pr_err("%s:%d lane_enable: %d\n", __func__, __LINE__, lane_enable);

	writel_relaxed(lane_enable,
		csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl5.addr);
	writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl6.data,
		csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl6.addr);

	for (i = 0, mask = 0x1; i < MAX_LANES; i++) {
		if (!(lane_mask & mask)) {
			if (mask == 0x2)
				i--;
			mask <<= 0x1;
			continue;
		}
		if (mask == 0x2) {
			val = 4;
			offset = CLOCK_OFFSET;
			clk_lane = 1;
			i--;
		} else {
			offset = 0x200*i;
			val = 0;
			clk_lane = 0;
		}

		if (csiphy_params->combo_mode == 1) {
			val |= 0xA;
			if (mask == csiphy_dev->ctrl_reg->csiphy_reg.combo_clk_mask) {
				val |= 0x4;
				clk_lane = 1;
			}
		}
		writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg7.data,
			csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg7.addr + offset);
		writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg6.data,
			csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg6.addr + offset);
		writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg8.data,
			csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg8.addr + offset);
		writel_relaxed(val, csiphybase +
			csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_misc1.addr + offset);
		writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_ctrl15.data,
			csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_ctrl15.addr + offset);
		writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg2.data,
			csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg2.addr + offset);

		writel_relaxed((csiphy_params->settle_cnt & 0xFF),
			csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg3.addr + offset);

		if (clk_lane == 1) {
			writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_lnck_cfg1.data, csiphybase +
				csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_lnck_cfg1.addr);

			writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg4.data, csiphybase +
				csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg4.addr + offset);
		} else {
			writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg1.data,
				csiphybase +
				csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg1.addr + offset);
		}

		writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg5.data,
			csiphybase +
			csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg5.addr + offset);

		writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg9.data,
			csiphybase +
			csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_cfg9.addr + offset);

		writel_relaxed(csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_test_imp.data,
			csiphybase + csiphy_dev->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_2ph_lnn_test_imp.addr + offset);
		mask <<= 1;
	}

	msm_csiphy_cphy_irq_config(csiphy_dev, csiphy_params);
	return 0;
}


static int msm_csiphy_lane_config(struct csiphy_device *csiphy_dev,
	struct msm_camera_csiphy_params *csiphy_params)
{
	int rc = 0;
//	uint8_t lane_cnt = 0;
//	uint16_t lane_mask = 0;
	void __iomem *csiphybase;
//	uint8_t csiphy_id = csiphy_dev->pdev->id;
//	struct clk **csid_phy_clk_ptr;
	uint32_t val = 0;



	csiphybase = csiphy_dev->base;
	if (!csiphybase) {
		pr_err("%s: csiphybase NULL\n", __func__);
		return -EINVAL;
	}

//	csiphy_dev->lane_mask[csiphy_id] |= csiphy_params->lane_mask;
//	lane_mask = csiphy_dev->lane_mask[csiphy_id];
//	lane_cnt = csiphy_params->lane_cnt;
	if (csiphy_params->lane_cnt < 1 || csiphy_params->lane_cnt > 4) {
		pr_err("%s: unsupported lane cnt %d\n",
			__func__, csiphy_params->lane_cnt);
		return rc;
	}
#if 0
	csid_phy_clk_ptr = csiphy_dev->csiphy_clk;
	if (!csid_phy_clk_ptr) {
		pr_err("csiphy_timer_src_clk get failed\n");
		return -EINVAL;
	}

	clk_rate = (csiphy_params->csiphy_clk > 0)
			? csiphy_params->csiphy_clk :
			csiphy_dev->csiphy_max_clk;
	round_rate = clk_round_rate(
			csid_phy_clk_ptr[csiphy_dev->csiphy_clk_index],
			clk_rate);
	if (round_rate >= csiphy_dev->csiphy_max_clk)
		round_rate = csiphy_dev->csiphy_max_clk;
	else {
		ratio = csiphy_dev->csiphy_max_clk/round_rate;
		csiphy_params->settle_cnt = csiphy_params->settle_cnt/ratio;
	}

	pr_err("set from usr csiphy_clk clk_rate = %u round_rate = %u\n",
			clk_rate, round_rate);
	rc = clk_set_rate(
		csid_phy_clk_ptr[csiphy_dev->csiphy_clk_index],
		round_rate);
	if (rc < 0) {
		pr_err("csiphy_timer_src_clk set failed\n");
		return rc;
	}
#endif
	pr_err("%s csiphy_params, mask = 0x%x cnt = %d\n",
		__func__,
		csiphy_params->lane_mask,
		csiphy_params->lane_cnt);
	pr_err("%s csiphy_params, settle cnt = 0x%x csid %d\n",
		__func__, csiphy_params->settle_cnt,
		csiphy_params->csid_core);

	val = readl_relaxed(csiphy_dev->base_clk_mux);
	if (csiphy_params->combo_mode &&
		(csiphy_params->lane_mask & 0x18) == 0x18) {
		val &= ~0xf0;
		val |= csiphy_params->csid_core << 4;
	} else {
		val &= ~0xf;
		val |= csiphy_params->csid_core;
	}
	writel_relaxed(val, csiphy_dev->base_clk_mux);
	pr_err("%s clk mux val 0x%x\n", __func__, val);
	/* ensure write is done */
	mb();



	rc = msm_csiphy_2phase_lane_config(csiphy_dev, csiphy_params);
	csiphy_dev->num_irq_registers = 11;

	if (rc < 0) {
		pr_err("%s:%d: Error in setting lane configuration\n",
			__func__, __LINE__);
	}
	return rc;

}



/*
 * csiphy_stream_on - Enable streaming on CSIPHY module
 * @csiphy: CSIPHY device
 *
 * Helper function to enable streaming on CSIPHY module.
 * Main configuration of CSIPHY module is also done here.
 */
static void csiphy_stream_on(struct csiphy_device *csiphy)
{
	struct csiphy_config *cfg = &csiphy->cfg;
	struct msm_camera_csiphy_params csiphy_params;
	csiphy_params.lane_cnt = 2;
	csiphy_params.lane_mask = csiphy_get_lane_mask(&cfg->csi2->lane_cfg);
	csiphy_params.combo_mode = cfg->combo_mode;
	csiphy_params.csid_core = cfg->csid_id;
	csiphy_params.settle_cnt = cfg->csi2->settle_cnt;

	msm_csiphy_lane_config(csiphy, &csiphy_params);
}

/*
 * csiphy_stream_off - Disable streaming on CSIPHY module
 * @csiphy: CSIPHY device
 *
 * Helper function to disable streaming on CSIPHY module
 */
static void csiphy_stream_off(struct csiphy_device *csiphy)
{
	writel_relaxed(0x0,
		csiphy->base + csiphy->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl5.addr);
	writel_relaxed(0x0,
		csiphy->base + csiphy->ctrl_reg->csiphy_3ph_reg.mipi_csiphy_3ph_cmn_ctrl6.addr);

//	if (cam_config_ahb_clk(NULL, 0, CAM_AHB_CLIENT_CSIPHY,
//	 CAM_AHB_SUSPEND_VOTE) < 0)
//		pr_err("%s: failed to remove vote for AHB\n", __func__);
}


/*
 * csiphy_set_stream - Enable/disable streaming on CSIPHY module
 * @sd: CSIPHY V4L2 subdevice
 * @enable: Requested streaming state
 *
 * Return 0 (awlays succeeds)
 */
static int csiphy_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct csiphy_device *csiphy = v4l2_get_subdevdata(sd);
	pr_err("[camera] : %s, enable = %d\n", __func__, enable);

	if (enable)
		csiphy_stream_on(csiphy);
	else
		csiphy_stream_off(csiphy);

	return 0;
}

/*
 * __csiphy_get_format - Get pointer to format structure
 * @csiphy: CSIPHY device
 * @cfg: V4L2 subdev pad configuration
 * @pad: pad from which format is requested
 * @which: TRY or ACTIVE format
 *
 * Return pointer to TRY or ACTIVE format structure
 */
static struct v4l2_mbus_framefmt *
__csiphy_get_format(struct csiphy_device *csiphy,
		    struct v4l2_subdev_fh *fh,
		    unsigned int pad,
		    enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);

	return &csiphy->fmt[pad];
}

/*
 * csiphy_try_format - Handle try format by pad subdev method
 * @csiphy: CSIPHY device
 * @cfg: V4L2 subdev pad configuration
 * @pad: pad on which format is requested
 * @fmt: pointer to v4l2 format structure
 * @which: wanted subdev format
 */
static void csiphy_try_format(struct csiphy_device *csiphy,
			      struct v4l2_subdev_fh *fh,
			      unsigned int pad,
			      struct v4l2_mbus_framefmt *fmt,
			      enum v4l2_subdev_format_whence which)
{
	unsigned int i;

	switch (pad) {
	case MSM_CSIPHY_PAD_SINK:
		/* Set format on sink pad */

		for (i = 0; i < ARRAY_SIZE(csiphy_formats); i++)
			if (fmt->code == csiphy_formats[i])
				break;

		/* If not found, use UYVY as default */
		if (i >= ARRAY_SIZE(csiphy_formats))
			fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;

		fmt->width = clamp_t(u32, fmt->width, 1, 8191);
		fmt->height = clamp_t(u32, fmt->height, 1, 8191);

		if (fmt->field == V4L2_FIELD_ANY)
			fmt->field = V4L2_FIELD_NONE;

		fmt->colorspace = V4L2_COLORSPACE_SRGB;

		break;

	case MSM_CSIPHY_PAD_SRC:
		/* Set and return a format same as sink pad */

		*fmt = *__csiphy_get_format(csiphy, fh, MSM_CSID_PAD_SINK,
					    which);

		break;
	}
}

/*
 * csiphy_enum_mbus_code - Handle pixel format enumeration
 * @sd: CSIPHY V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @code: pointer to v4l2_subdev_mbus_code_enum structure
 * return -EINVAL or zero on success
 */
static int csiphy_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	struct csiphy_device *csiphy = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;
	pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
	if (code->pad == MSM_CSIPHY_PAD_SINK) {
		if (code->index >= ARRAY_SIZE(csiphy_formats))
			return -EINVAL;

		code->code = csiphy_formats[code->index];
	} else {
		if (code->index > 0)
			return -EINVAL;

		format = __csiphy_get_format(csiphy, fh, MSM_CSIPHY_PAD_SINK,
					     code->which);

		code->code = format->code;
	}

	return 0;
}

/*
 * csiphy_enum_frame_size - Handle frame size enumeration
 * @sd: CSIPHY V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fse: pointer to v4l2_subdev_frame_size_enum structure
 * return -EINVAL or zero on success
 */
static int csiphy_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	struct csiphy_device *csiphy = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt format;

	if (fse->index != 0)
		return -EINVAL;
	pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
	format.code = fse->code;
	format.width = 1;
	format.height = 1;
	csiphy_try_format(csiphy, fh, fse->pad, &format, fse->which);
	fse->min_width = format.width;
	fse->min_height = format.height;

	if (format.code != fse->code)
		return -EINVAL;

	format.code = fse->code;
	format.width = -1;
	format.height = -1;
	csiphy_try_format(csiphy, fh, fse->pad, &format, fse->which);
	fse->max_width = format.width;
	fse->max_height = format.height;

	return 0;
}

/*
 * csiphy_get_format - Handle get format by pads subdev method
 * @sd: CSIPHY V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int csiphy_get_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_fh *fh,
			     struct v4l2_subdev_format *fmt)
{
	struct csiphy_device *csiphy = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;
	pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
	format = __csiphy_get_format(csiphy, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;

	return 0;
}

/*
 * csiphy_set_format - Handle set format by pads subdev method
 * @sd: CSIPHY V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int csiphy_set_format(struct v4l2_subdev *sd,
			     struct v4l2_subdev_fh *fh,
			     struct v4l2_subdev_format *fmt)
{
	struct csiphy_device *csiphy = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;
	pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
	format = __csiphy_get_format(csiphy, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	csiphy_try_format(csiphy, fh, fmt->pad, &fmt->format, fmt->which);
	*format = fmt->format;

	/* Propagate the format from sink to source */
	if (fmt->pad == MSM_CSIPHY_PAD_SINK) {
		format = __csiphy_get_format(csiphy, fh, MSM_CSIPHY_PAD_SRC,
					     fmt->which);

		*format = fmt->format;
		csiphy_try_format(csiphy, fh, MSM_CSIPHY_PAD_SRC, format,
				  fmt->which);
	}

	return 0;
}

/*
 * csiphy_init_formats - Initialize formats on all pads
 * @sd: CSIPHY V4L2 subdevice
 * @fh: V4L2 subdev file handle
 *
 * Initialize all pad formats with default values.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csiphy_init_formats(struct v4l2_subdev *sd,
			       struct v4l2_subdev_fh *fh)
{
	struct v4l2_subdev_format format;
	pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
	memset(&format, 0, sizeof(format));
	format.pad = MSM_CSIPHY_PAD_SINK;
	format.which = fh ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	format.format.code = MEDIA_BUS_FMT_UYVY8_2X8;
	format.format.width = 1920;
	format.format.height = 1080;

	return csiphy_set_format(sd, fh ? fh : NULL, &format);
}

/*
 * msm_csiphy_subdev_init - Initialize CSIPHY device structure and resources
 * @csiphy: CSIPHY device
 * @res: CSIPHY module resources table
 * @id: CSIPHY module id
 *
 * Return 0 on success or a negative error code otherwise
 */
int msm_csiphy_subdev_init(struct csiphy_device *csiphy,
			   struct resources *res, u8 id)
{
	struct device *dev = to_device_index(csiphy, id);
	struct platform_device *pdev = container_of(dev,
						struct platform_device, dev);
	struct resource *r;
	int i;
	int ret;

	csiphy->id = id;
	csiphy->cfg.combo_mode = 0;

	/* Memory */
	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, res->reg[0]);
	csiphy->base = devm_ioremap_resource(dev, r);
	if (IS_ERR(csiphy->base)) {
		dev_err(dev, "could not map memory\n");
		return PTR_ERR(csiphy->base);
	}

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, res->reg[1]);
	csiphy->base_clk_mux = devm_ioremap_resource(dev, r);
	if (IS_ERR(csiphy->base_clk_mux)) {
		dev_err(dev, "could not map memory\n");
		return PTR_ERR(csiphy->base_clk_mux);
	}

	/* Interrupt */

	r = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
					 res->interrupt[0]);
	if (!r) {
		dev_err(dev, "missing IRQ\n");
		return -EINVAL;
	}

	csiphy->irq = r->start;
	snprintf(csiphy->irq_name, sizeof(csiphy->irq_name), "%s_%s%d",
		 dev_name(dev), MSM_CSIPHY_NAME, csiphy->id);
	ret = devm_request_irq(dev, csiphy->irq, csiphy_isr,
			       IRQF_TRIGGER_RISING, csiphy->irq_name, csiphy);
	if (ret < 0) {
		dev_err(dev, "request_irq failed\n");
		return ret;
	}

	disable_irq(csiphy->irq);

	/* Clocks */

	csiphy->nclocks = 0;
	while (res->clock[csiphy->nclocks])
		csiphy->nclocks++;

	csiphy->clock = devm_kzalloc(dev, csiphy->nclocks *
				     sizeof(*csiphy->clock), GFP_KERNEL);
	if (!csiphy->clock)
		return -ENOMEM;

	for (i = 0; i < csiphy->nclocks; i++) {
		csiphy->clock[i] = devm_clk_get(dev, res->clock[i]);
		if (IS_ERR(csiphy->clock[i]))
			return PTR_ERR(csiphy->clock[i]);

		if (res->clock_rate[i]) {
			long clk_rate = clk_round_rate(csiphy->clock[i],
						       res->clock_rate[i]);
			if (clk_rate < 0) {
				dev_err(to_device_index(csiphy, csiphy->id),
					"clk round rate failed\n");
				return -EINVAL;
			}
			ret = clk_set_rate(csiphy->clock[i], clk_rate);
			if (ret < 0) {
				dev_err(to_device_index(csiphy, csiphy->id),
					"clk set rate failed\n");
				return ret;
			}
		}
	}

	csiphy->ctrl_reg = NULL;
	csiphy->ctrl_reg = kzalloc(sizeof(struct csiphy_ctrl_t),
		GFP_KERNEL);
	if (!csiphy->ctrl_reg) {
		pr_err("%s:%d kzalloc failed\n", __func__, __LINE__);
		return -ENOMEM;
	}

	csiphy->ctrl_reg->csiphy_3ph_reg = csiphy_v3_5_3ph;
	csiphy->ctrl_reg->csiphy_reg = csiphy_v3_5;

	return 0;
}

/*
 * csiphy_link_setup - Setup CSIPHY connections
 * @entity: Pointer to media entity structure
 * @local: Pointer to local pad
 * @remote: Pointer to remote pad
 * @flags: Link flags
 *
 * Rreturn 0 on success
 */
static int csiphy_link_setup(struct media_entity *entity,
			     const struct media_pad *local,
			     const struct media_pad *remote, u32 flags)
{
	if ((local->flags & MEDIA_PAD_FL_SOURCE) &&
	    (flags & MEDIA_LNK_FL_ENABLED)) {
		struct v4l2_subdev *sd;
		struct csiphy_device *csiphy;
		struct csid_device *csid;
		pr_err("[camera] : %s, local entity = %s, remote entity = %s\n", __func__, local->entity->name, remote->entity->name);
		if (media_entity_remote_pad((struct media_pad *)local))
			return -EBUSY;

		sd = container_of(entity, struct v4l2_subdev, entity);
		csiphy = v4l2_get_subdevdata(sd);

		sd = container_of(remote->entity, struct v4l2_subdev, entity);
		csid = v4l2_get_subdevdata(sd);

		csiphy->cfg.csid_id = csid->id;
	}

	return 0;
}

static const struct v4l2_subdev_core_ops csiphy_core_ops = {
	.s_power = csiphy_set_power,
};

static const struct v4l2_subdev_video_ops csiphy_video_ops = {
	.s_stream = csiphy_set_stream,
};

static const struct v4l2_subdev_pad_ops csiphy_pad_ops = {
	.enum_mbus_code = csiphy_enum_mbus_code,
	.enum_frame_size = csiphy_enum_frame_size,
	.get_fmt = csiphy_get_format,
	.set_fmt = csiphy_set_format,
};

static const struct v4l2_subdev_ops csiphy_v4l2_ops = {
	.core = &csiphy_core_ops,
	.video = &csiphy_video_ops,
	.pad = &csiphy_pad_ops,
};

static const struct v4l2_subdev_internal_ops csiphy_v4l2_internal_ops = {
	.open = csiphy_init_formats,
};

static const struct media_entity_operations csiphy_media_ops = {
	.link_setup = csiphy_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

/*
 * msm_csiphy_register_entity - Register subdev node for CSIPHY module
 * @csiphy: CSIPHY device
 * @v4l2_dev: V4L2 device
 *
 * Return 0 on success or a negative error code otherwise
 */
int msm_csiphy_register_entity(struct csiphy_device *csiphy,
			       struct v4l2_device *v4l2_dev)
{
	struct v4l2_subdev *sd = &csiphy->subdev;
	struct media_pad *pads = csiphy->pads;
	struct device *dev = to_device_index(csiphy, csiphy->id);
	int ret;

	v4l2_subdev_init(sd, &csiphy_v4l2_ops);
	sd->internal_ops = &csiphy_v4l2_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, ARRAY_SIZE(sd->name), "%s%d",
		 MSM_CSIPHY_NAME, csiphy->id);
	v4l2_set_subdevdata(sd, csiphy);

	ret = csiphy_init_formats(sd, NULL);
	if (ret < 0) {
		dev_err(dev, "Failed to init format\n");
		return ret;
	}

	pads[MSM_CSIPHY_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[MSM_CSIPHY_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;

	sd->entity.ops = &csiphy_media_ops;
	ret = media_entity_init(&sd->entity, MSM_CSIPHY_PADS_NUM, pads, 0);
	if (ret < 0) {
		dev_err(dev, "Failed to init media entity\n");
		return ret;
	}

	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0) {
		dev_err(dev, "Failed to register subdev\n");
		media_entity_cleanup(&sd->entity);
	}

	return ret;
}

/*
 * msm_csiphy_unregister_entity - Unregister CSIPHY module subdev node
 * @csiphy: CSIPHY device
 */
void msm_csiphy_unregister_entity(struct csiphy_device *csiphy)
{
	v4l2_device_unregister_subdev(&csiphy->subdev);
}
