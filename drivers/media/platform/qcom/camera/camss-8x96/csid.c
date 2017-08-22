/*
 * csid.c
 *
 * Qualcomm MSM Camera Subsystem - CSID Module
 *
 * Copyright (c) 2011-2015, The Linux Foundation. All rights reserved.
 * Copyright (C) 2015-2016 Linaro Ltd.
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
#include <linux/completion.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "csid.h"
#include "camss.h"

#define MSM_CSID_NAME "msm_csid"


#define DATA_TYPE_EMBEDDED_DATA_8BIT	0x12
#define DATA_TYPE_YUV422_8BIT		0x1e
#define DATA_TYPE_RAW_6BIT		0x28
#define DATA_TYPE_RAW_8BIT		0x2a
#define DATA_TYPE_RAW_10BIT		0x2b
#define DATA_TYPE_RAW_12BIT		0x2c

#define DECODE_FORMAT_UNCOMPRESSED_6_BIT	0x0
#define DECODE_FORMAT_UNCOMPRESSED_8_BIT	0x1
#define DECODE_FORMAT_UNCOMPRESSED_10_BIT	0x2
#define DECODE_FORMAT_UNCOMPRESSED_12_BIT	0x3

#define CSID_RESET_TIMEOUT_MS 500



uint8_t csid_lane_assign_v3_5[PHY_LANE_MAX] = {0, 4, 1, 2, 3};

struct csid_reg_parms_t csid_v3_5 = {
	/* MIPI	CSID registers */
	0x0,
	0x4,
	0x8,
	0x10,
	0x14,
	0x18,
	0x1C,
	0x20,
	0x24,
	0x64,
	0x68,
	0x6C,
	0x70,
	0x74,
	0x78,
	0x7C,
	0x80,
	0x88,
	0x8C,
	0x90,
	0x94,
	0x98,
	0x9C,
	0xA0,
	0xA8,
	0xAC,
	0xB4,
	0xB8,
	0xBC,
	11,
	0x7FFF,
	0x4,
	17,
	0x30050000,
	0xC,
	0x84,
	0xA4,
	0x7f010800,
	20,
	17,
	16,
};



static const struct {
	u32 code;
	u32 uncompressed;
	u8 data_type;
	u8 decode_format;
	u8 uncompr_bpp;
} csid_input_fmts[] = {
	{
		MEDIA_BUS_FMT_UYVY8_2X8,
		MEDIA_BUS_FMT_UYVY8_2X8,
		DATA_TYPE_YUV422_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		16
	},
	{
		MEDIA_BUS_FMT_VYUY8_2X8,
		MEDIA_BUS_FMT_VYUY8_2X8,
		DATA_TYPE_YUV422_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		16
	},
	{
		MEDIA_BUS_FMT_YUYV8_2X8,
		MEDIA_BUS_FMT_YUYV8_2X8,
		DATA_TYPE_YUV422_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		16
	},
	{
		MEDIA_BUS_FMT_YVYU8_2X8,
		MEDIA_BUS_FMT_YVYU8_2X8,
		DATA_TYPE_YUV422_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		16
	},
	{
		MEDIA_BUS_FMT_SBGGR8_1X8,
		MEDIA_BUS_FMT_SBGGR8_1X8,
		DATA_TYPE_RAW_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8
	},
	{
		MEDIA_BUS_FMT_SGBRG8_1X8,
		MEDIA_BUS_FMT_SGBRG8_1X8,
		DATA_TYPE_RAW_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8
	},
	{
		MEDIA_BUS_FMT_SGRBG8_1X8,
		MEDIA_BUS_FMT_SGRBG8_1X8,
		DATA_TYPE_RAW_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8
	},
	{
		MEDIA_BUS_FMT_SRGGB8_1X8,
		MEDIA_BUS_FMT_SRGGB8_1X8,
		DATA_TYPE_RAW_8BIT,
		DECODE_FORMAT_UNCOMPRESSED_8_BIT,
		8
	},
	{
		MEDIA_BUS_FMT_SBGGR10_1X10,
		MEDIA_BUS_FMT_SBGGR10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10
	},
	{
		MEDIA_BUS_FMT_SGBRG10_1X10,
		MEDIA_BUS_FMT_SGBRG10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10
	},
	{
		MEDIA_BUS_FMT_SGRBG10_1X10,
		MEDIA_BUS_FMT_SGRBG10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10
	},
	{
		MEDIA_BUS_FMT_SRGGB10_1X10,
		MEDIA_BUS_FMT_SRGGB10_1X10,
		DATA_TYPE_RAW_10BIT,
		DECODE_FORMAT_UNCOMPRESSED_10_BIT,
		10
	},
	{
		MEDIA_BUS_FMT_SBGGR12_1X12,
		MEDIA_BUS_FMT_SBGGR12_1X12,
		DATA_TYPE_RAW_12BIT,
		DECODE_FORMAT_UNCOMPRESSED_12_BIT,
		12
	},
	{
		MEDIA_BUS_FMT_SGBRG12_1X12,
		MEDIA_BUS_FMT_SGBRG12_1X12,
		DATA_TYPE_RAW_12BIT,
		DECODE_FORMAT_UNCOMPRESSED_12_BIT,
		12
	},
	{
		MEDIA_BUS_FMT_SGRBG12_1X12,
		MEDIA_BUS_FMT_SGRBG12_1X12,
		DATA_TYPE_RAW_12BIT,
		DECODE_FORMAT_UNCOMPRESSED_12_BIT,
		12
	},
	{
		MEDIA_BUS_FMT_SRGGB12_1X12,
		MEDIA_BUS_FMT_SRGGB12_1X12,
		DATA_TYPE_RAW_12BIT,
		DECODE_FORMAT_UNCOMPRESSED_12_BIT,
		12
	}
};

/*
 * csid_isr - CSID module interrupt handler
 * @irq: Interrupt line
 * @dev: CSID device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t csid_isr(int irq_num, void *dev)
{
	uint32_t irq;
	struct csid_device *csid_dev = dev;

	if (!csid_dev) {
		pr_err("%s:%d csid_dev NULL\n", __func__, __LINE__);
		return IRQ_HANDLED;
	}
/*
	if (csid_dev->csid_sof_debug == SOF_DEBUG_ENABLE) {
		if (csid_dev->csid_sof_debug_count < CSID_SOF_DEBUG_COUNT)
			csid_dev->csid_sof_debug_count++;
		else {
			msm_csid_set_sof_freeze_debug_reg(csid_dev, false);
			return IRQ_HANDLED;
		}
	}
*/
	irq = readl_relaxed(csid_dev->base +
		csid_dev->ctrl_reg->csid_reg.csid_irq_status_addr);

	if (irq & (0x1 <<
		csid_dev->ctrl_reg->csid_reg.csid_rst_done_irq_bitshift))
		complete(&csid_dev->reset_complete);
	writel_relaxed(irq, csid_dev->base +
		csid_dev->ctrl_reg->csid_reg.csid_irq_clear_cmd_addr);

	return IRQ_HANDLED;
}

/*
 * csid_reset - Trigger reset on CSID module and wait to complete
 * @csid: CSID device
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_reset(struct csid_device *csid)
{
	unsigned long time;

	reinit_completion(&csid->reset_complete);

	writel_relaxed(csid->ctrl_reg->csid_reg.csid_rst_stb_all,
	csid->base + csid->ctrl_reg->csid_reg.csid_rst_cmd_addr);

	time = wait_for_completion_timeout(&csid->reset_complete,
		msecs_to_jiffies(CSID_RESET_TIMEOUT_MS));
	if (!time) {
		dev_err(to_device_index(csid, csid->id),
			"CSID reset timeout\n");
		return -EIO;
	}

	return 0;
}

/*
 * csid_set_power - Power on/off CSID module
 * @sd: CSID V4L2 subdevice
 * @on: Requested power state
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_set_power(struct v4l2_subdev *sd, int on)
{
	struct csid_device *csid = v4l2_get_subdevdata(sd);
	struct device *dev = to_device_index(csid, csid->id);
	int ret;
	pr_err("[camera] : %s, on = %d\n", __func__, on);
	if (on) {
	//	u32 hw_version;

		ret = regulator_enable(csid->vdda);
		if (ret < 0){
			dev_err(dev, "Can not enable vdda regulator\n");
			return ret;
		}

		ret = regulator_enable(csid->gdscr);
		if (ret < 0){
			dev_err(dev, "Can not enable camss regulator\n");
			return ret;
		}
		ret = regulator_enable(csid->mmagic);
		if (ret < 0){
			dev_err(dev, "Can not enable mmagic regulator\n");
			return ret;
		}

		ret = camss_enable_clocks(csid->nclocks, csid->clock, dev);
		if (ret < 0) {
			regulator_disable(csid->vdda);
			regulator_disable(csid->gdscr);
			regulator_disable(csid->mmagic);
			return ret;
		}

		enable_irq(csid->irq);

		ret = csid_reset(csid);
		if (ret < 0) {
			disable_irq(csid->irq);
			camss_disable_clocks(csid->nclocks, csid->clock);
			regulator_disable(csid->vdda);
			regulator_disable(csid->gdscr);
			regulator_disable(csid->mmagic);
			return ret;
		}

//		hw_version = readl_relaxed(csid->base + CAMSS_CSID_HW_VERSION);
//		dev_dbg(dev, "CSID HW Version = 0x%08x\n", hw_version);
	} else {
		disable_irq(csid->irq);
		camss_disable_clocks(csid->nclocks, csid->clock);

		ret = regulator_disable(csid->vdda);
		if (ret < 0){
			dev_err(dev, "Can not disable vdda regulator\n");
		}

		ret = regulator_disable(csid->gdscr);
		if (ret < 0){
			dev_err(dev, "Can not disable camss regulator\n");
		}
		ret = regulator_disable(csid->mmagic);
		if (ret < 0){
			dev_err(dev, "Can not disable mmagic regulator\n");
		}
		//TODO
	}

	return ret;
}

/*
 * csid_get_uncompressed - map media bus format to uncompressed media bus format
 * @code: media bus format code
 *
 * Return uncompressed media bus format code
 */
static u32 csid_get_uncompressed(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(csid_input_fmts); i++)
		if (code == csid_input_fmts[i].code)
			break;

	return csid_input_fmts[i].uncompressed;
}
#if 0
/*
 * csid_get_data_type - map media bus format to data type
 * @code: media bus format code
 *
 * Return data type code
 */
static u8 csid_get_data_type(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(csid_input_fmts); i++)
		if (code == csid_input_fmts[i].code)
			break;

	return csid_input_fmts[i].data_type;
}

/*
 * csid_get_decode_format - map media bus format to decode format
 * @code: media bus format code
 *
 * Return decode format code
 */
static u8 csid_get_decode_format(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(csid_input_fmts); i++)
		if (code == csid_input_fmts[i].code)
			break;

	return csid_input_fmts[i].decode_format;
}

/*
 * csid_get_bpp - map media bus format to bits per pixel
 * @code: media bus format code
 *
 * Return number of bits per pixel
 */
static u8 csid_get_bpp(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(csid_input_fmts); i++)
		if (code == csid_input_fmts[i].uncompressed)
			break;

	return csid_input_fmts[i].uncompr_bpp;
}
#endif

static int msm_csid_cid_lut(
	struct csid_lut_params *csid_lut_params,
	struct csid_device *csid_dev)
{
	int rc = 0, i = 0;
	uint32_t val = 0;

	if (!csid_lut_params) {
		pr_err("%s:%d csid_lut_params NULL\n", __func__, __LINE__);
		return -EINVAL;
	}
	if (csid_lut_params->num_cid > MAX_CID) {
		pr_err("%s:%d num_cid exceeded limit num_cid = %d max = %d\n",
			__func__, __LINE__, csid_lut_params->num_cid, MAX_CID);
		return -EINVAL;
	}
	for (i = 0; i < csid_lut_params->num_cid; i++) {
		if (csid_lut_params->vc_cfg[i]->cid >= MAX_CID) {
			pr_err("%s: cid outside range %d\n",
				 __func__, csid_lut_params->vc_cfg[i]->cid);
			return -EINVAL;
		}
		pr_err("%s lut params num_cid = %d, cid = %d\n",
			__func__,
			csid_lut_params->num_cid,
			csid_lut_params->vc_cfg[i]->cid);
		pr_err("%s lut params dt = 0x%x, df = %d\n", __func__,
			csid_lut_params->vc_cfg[i]->dt,
			csid_lut_params->vc_cfg[i]->decode_format);
		if (csid_lut_params->vc_cfg[i]->dt < 0x12 ||
			csid_lut_params->vc_cfg[i]->dt > 0x37) {
			pr_err("%s: unsupported data type 0x%x\n",
				 __func__, csid_lut_params->vc_cfg[i]->dt);
			return rc;
		}
		val = readl_relaxed(csid_dev->base +
			csid_dev->ctrl_reg->csid_reg.csid_cid_lut_vc_0_addr +
			(csid_lut_params->vc_cfg[i]->cid >> 2) * 4)
			& ~(0xFF << ((csid_lut_params->vc_cfg[i]->cid % 4) *
			8));
		val |= (csid_lut_params->vc_cfg[i]->dt <<
			((csid_lut_params->vc_cfg[i]->cid % 4) * 8));
		writel_relaxed(val, csid_dev->base +
			csid_dev->ctrl_reg->csid_reg.csid_cid_lut_vc_0_addr +
			(csid_lut_params->vc_cfg[i]->cid >> 2) * 4);

		val = (csid_lut_params->vc_cfg[i]->decode_format << 4) | 0x3;
		writel_relaxed(val, csid_dev->base +
			csid_dev->ctrl_reg->csid_reg.csid_cid_n_cfg_addr +
			(csid_lut_params->vc_cfg[i]->cid * 4));
	}
	return rc;
}

static int msm_csid_config(struct csid_device *csid_dev,
	struct csid_params *csid_params)
{
	int rc = 0;
	uint32_t val = 0; //clk_rate = 0;
//	uint32_t round_rate = 0, input_sel;
	uint32_t lane_assign = 0;
	uint8_t  lane_num = 0;
	uint8_t  i, j;
//	struct clk **csid_clk_ptr;
	void __iomem *csidbase;

	csidbase = csid_dev->base;
	if (!csidbase || !csid_params) {
		pr_err("%s:%d csidbase %pK, csid params %pK\n", __func__,
			__LINE__, csidbase, csid_params);
		return -EINVAL;
	}

	pr_err("%s csid_params, lane_cnt = %d, lane_assign = 0x%x\n",
		__func__,
		csid_params->lane_cnt,
		csid_params->lane_assign);
	pr_err("%s csid_params phy_sel = %d\n", __func__,
		csid_params->phy_sel);

//	csid_dev->csid_lane_cnt = csid_params->lane_cnt;
	rc = csid_reset(csid_dev);
	if (rc < 0) {
		pr_err("%s:%d csid_reset failed\n", __func__, __LINE__);
		return rc;
	}
#if 0
	csid_clk_ptr = csid_dev->csid_clk;
	if (!csid_clk_ptr) {
		pr_err("csi_src_clk get failed\n");
		return -EINVAL;
	}

	clk_rate = (csid_params->csi_clk > 0) ?
				(csid_params->csi_clk) : csid_dev->csid_max_clk;
	round_rate = clk_round_rate(csid_clk_ptr[csid_dev->csid_clk_index],
					clk_rate);
	if (round_rate > csid_dev->csid_max_clk)
		round_rate = csid_dev->csid_max_clk;
	pr_debug("usr set rate csi_clk clk_rate = %u round_rate = %u\n",
					clk_rate, round_rate);
	rc = clk_set_rate(csid_clk_ptr[csid_dev->csid_clk_index],
				round_rate);
	if (rc < 0) {
		pr_err("csi_src_clk set failed\n");
		return rc;
	}
#endif

	val = csid_params->lane_cnt - 1;
	pr_err("%s:%d called\n", __func__, __LINE__);
	for (i = 0, j = 0; i < PHY_LANE_MAX; i++) {
		if (i == PHY_LANE_CLK)
			continue;
		lane_num = (csid_params->lane_assign >> j) & 0xF;
		if (lane_num >= PHY_LANE_MAX) {
			pr_err("%s:%d invalid lane number %d\n",
				__func__, __LINE__, lane_num);
			return -EINVAL;
		}
		if (csid_dev->ctrl_reg->csid_lane_assign[lane_num] >=
			PHY_LANE_MAX){
			pr_err("%s:%d invalid lane map %d\n",
				__func__, __LINE__,
				csid_dev->ctrl_reg->csid_lane_assign[lane_num]);
			return -EINVAL;
		}
		lane_assign |=
			csid_dev->ctrl_reg->csid_lane_assign[lane_num]
			<< j;
		j += 4;
	}

	pr_err("%s csid_params calculated lane_assign = 0x%X\n",
		__func__, lane_assign);

	val |= lane_assign <<
		csid_dev->ctrl_reg->csid_reg.csid_dl_input_sel_shift;

	writel_relaxed(val, csidbase + csid_dev->ctrl_reg->csid_reg.csid_core_ctrl_0_addr);
	val = csid_params->phy_sel <<
	    csid_dev->ctrl_reg->csid_reg.csid_phy_sel_shift;
	val |= 0xF;
	writel_relaxed(val, csidbase + csid_dev->ctrl_reg->csid_reg.csid_core_ctrl_1_addr);

	rc = msm_csid_cid_lut(&csid_params->lut_params, csid_dev);
	if (rc < 0) {
		pr_err("%s:%d config cid lut failed\n", __func__, __LINE__);
		return rc;
	}
//	msm_csid_set_debug_reg(csid_dev, csid_params);

//	if (csid_dev->is_testmode == 1)
//		writel_relaxed(0x00A06437, csidbase + csid_dev->ctrl_reg->csid_reg.csid_tg_ctrl_addr);

	return rc;
}

/*
 * csid_set_stream - Enable/disable streaming on CSID module
 * @sd: CSID V4L2 subdevice
 * @enable: Requested streaming state
 *
 * Main configuration of CSID module is also done here.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct csid_device *csid = v4l2_get_subdevdata(sd);
	struct csid_testgen_config *tg = &csid->testgen;
	struct csid_params csid_params;
	struct csid_vc_cfg *vc_cfg = NULL;
	struct csid_phy_config *phy = &csid->phy;
	int i = 0;
	pr_err("[camera] : %s, enable = %d\n", __func__, enable);
	if (enable) {
		int ret;

		ret = v4l2_ctrl_handler_setup(&csid->ctrls);
		if (ret < 0) {
			dev_err(to_device_index(csid, csid->id),
				"could not sync v4l2 controls\n");
			return ret;
		}

		if (!tg->enabled &&
		    !media_entity_remote_pad(&csid->pads[MSM_CSID_PAD_SINK])) {
			return -ENOLINK;
		}

	//	dt = csid_get_data_type(csid->fmt[MSM_CSID_PAD_SRC].code);


		csid_params.lane_cnt = phy->lane_cnt;
		csid_params.lane_assign = phy->lane_assign;
		csid_params.phy_sel = phy->csiphy_id;

		csid_params.lut_params.num_cid = 2;

		if (csid_params.lut_params.num_cid < 1 ||
			csid_params.lut_params.num_cid > MAX_CID) {
			pr_err("%s: %d num_cid outside range %d\n", __func__,
				__LINE__, csid_params.lut_params.num_cid);
			ret = -EINVAL;
		}

		for (i = 0; i < csid_params.lut_params.num_cid; i++) {
			vc_cfg = kzalloc(sizeof(struct csid_vc_cfg),
				GFP_KERNEL);
			if (!vc_cfg) {
				pr_err("%s: %d failed\n", __func__, __LINE__);
				ret = -ENOMEM;
				goto MEM_CLEAN32;
			}
			/* msm_camera_csid_vc_cfg size
			 * does not change in COMPAT MODE
			 */
			 if(i == 0)
			 {
				vc_cfg->cid = 0;
				vc_cfg->dt = 0x1e;
				vc_cfg->decode_format = 1;
			 }
			 else
			 {
				vc_cfg->cid = 1;
				vc_cfg->dt = 0x12;
				vc_cfg->decode_format = 1;
			 }
			csid_params.lut_params.vc_cfg[i] = vc_cfg;
		}
		ret= msm_csid_config(csid, &csid_params);
	//	csid->current_csid_params = csid_params;

MEM_CLEAN32:
		for (i--; i >= 0; i--) {
			kfree(csid_params.lut_params.vc_cfg[i]);
			csid_params.lut_params.vc_cfg[i] = NULL;
		}
	} else {
		uint32_t irq;
		irq = readl_relaxed(csid->base +
		csid->ctrl_reg->csid_reg.csid_irq_status_addr);
		writel_relaxed(irq, csid->base +
		csid->ctrl_reg->csid_reg.csid_irq_clear_cmd_addr);
		writel_relaxed(0, csid->base +
		csid->ctrl_reg->csid_reg.csid_irq_mask_addr);
//		if (tg->enabled) {
//			val = 0x00a06436;
//			writel_relaxed(val, csid->base + CAMSS_CSID_TG_CTRL);
//		}
	}

	return 0;
}

/*
 * __csid_get_format - Get pointer to format structure
 * @csid: CSID device
 * @cfg: V4L2 subdev pad configuration
 * @pad: pad from which format is requested
 * @which: TRY or ACTIVE format
 *
 * Return pointer to TRY or ACTIVE format structure
 */
static struct v4l2_mbus_framefmt *
__csid_get_format(struct csid_device *csid,
		  struct v4l2_subdev_fh *fh,
		  unsigned int pad,
		  enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);

	return &csid->fmt[pad];
}

/*
 * csid_try_format - Handle try format by pad subdev method
 * @csid: CSID device
 * @cfg: V4L2 subdev pad configuration
 * @pad: pad on which format is requested
 * @fmt: pointer to v4l2 format structure
 * @which: wanted subdev format
 */
static void csid_try_format(struct csid_device *csid,
			    struct v4l2_subdev_fh *fh,
			    unsigned int pad,
			    struct v4l2_mbus_framefmt *fmt,
			    enum v4l2_subdev_format_whence which)
{
	unsigned int i;

	switch (pad) {
	case MSM_CSID_PAD_SINK:
		/* Set format on sink pad */

		for (i = 0; i < ARRAY_SIZE(csid_input_fmts); i++)
			if (fmt->code == csid_input_fmts[i].code)
				break;

		/* If not found, use UYVY as default */
		if (i >= ARRAY_SIZE(csid_input_fmts))
			fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;

		fmt->width = clamp_t(u32, fmt->width, 1, 8191);
		fmt->height = clamp_t(u32, fmt->height, 1, 8191);

		if (fmt->field == V4L2_FIELD_ANY)
			fmt->field = V4L2_FIELD_NONE;

		break;

	case MSM_CSID_PAD_SRC:
		if (csid->testgen_mode->cur.val == 0) {
			/* Test generator is disabled, keep pad formats */
			/* in sync - set and return a format same as sink pad */
			struct v4l2_mbus_framefmt format;

			format = *__csid_get_format(csid, fh,
						    MSM_CSID_PAD_SINK, which);
			format.code = csid_get_uncompressed(format.code);
			*fmt = format;
		} else {
			/* Test generator is enabled, set format on source*/
			/* pad to allow test generator usage */

			for (i = 0; i < ARRAY_SIZE(csid_input_fmts); i++)
				if (csid_input_fmts[i].uncompressed ==
								fmt->code)
					break;

			/* If not found, use UYVY as default */
			if (i >= ARRAY_SIZE(csid_input_fmts))
				fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;

			fmt->width = clamp_t(u32, fmt->width, 1, 8191);
			fmt->height = clamp_t(u32, fmt->height, 1, 8191);

			fmt->field = V4L2_FIELD_NONE;
		}
		break;
	}

	fmt->colorspace = V4L2_COLORSPACE_SRGB;
}

/*
 * csid_enum_mbus_code - Handle pixel format enumeration
 * @sd: CSID V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @code: pointer to v4l2_subdev_mbus_code_enum structure
 * return -EINVAL or zero on success
 */
static int csid_enum_mbus_code(struct v4l2_subdev *sd,
			       struct v4l2_subdev_fh *fh,
			       struct v4l2_subdev_mbus_code_enum *code)
{
	struct csid_device *csid = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;
	pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
	if (code->pad == MSM_CSID_PAD_SINK) {
		if (code->index >= ARRAY_SIZE(csid_input_fmts))
			return -EINVAL;

		code->code = csid_input_fmts[code->index].code;
	} else {
		if (csid->testgen_mode->cur.val == 0) {
			if (code->index > 0)
				return -EINVAL;

			format = __csid_get_format(csid, fh, MSM_CSID_PAD_SINK,
						   code->which);

			code->code = csid_get_uncompressed(format->code);
		} else {
			if (code->index >= ARRAY_SIZE(csid_input_fmts))
				return -EINVAL;

			code->code = csid_input_fmts[code->index].uncompressed;
		}
	}

	return 0;
}

/*
 * csid_enum_frame_size - Handle frame size enumeration
 * @sd: CSID V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fse: pointer to v4l2_subdev_frame_size_enum structure
 * return -EINVAL or zero on success
 */
static int csid_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_frame_size_enum *fse)
{
	struct csid_device *csid = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt format;

	if (fse->index != 0)
		return -EINVAL;
	pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
	format.code = fse->code;
	format.width = 1;
	format.height = 1;
	csid_try_format(csid, fh, fse->pad, &format, fse->which);
	fse->min_width = format.width;
	fse->min_height = format.height;

	if (format.code != fse->code)
		return -EINVAL;

	format.code = fse->code;
	format.width = -1;
	format.height = -1;
	csid_try_format(csid, fh, fse->pad, &format, fse->which);
	fse->max_width = format.width;
	fse->max_height = format.height;

	return 0;
}

/*
 * csid_get_format - Handle get format by pads subdev method
 * @sd: CSID V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int csid_get_format(struct v4l2_subdev *sd,
			   struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_format *fmt)
{
	struct csid_device *csid = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;
	pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
	format = __csid_get_format(csid, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;

	return 0;
}

/*
 * csid_set_format - Handle set format by pads subdev method
 * @sd: CSID V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int csid_set_format(struct v4l2_subdev *sd,
			   struct v4l2_subdev_fh *fh,
			   struct v4l2_subdev_format *fmt)
{
	struct csid_device *csid = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;
	pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
	format = __csid_get_format(csid, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	csid_try_format(csid, fh, fmt->pad, &fmt->format, fmt->which);
	*format = fmt->format;

	/* Propagate the format from sink to source */
	if (fmt->pad == MSM_CSID_PAD_SINK) {
		format = __csid_get_format(csid, fh, MSM_CSID_PAD_SRC,
					   fmt->which);

		*format = fmt->format;
		csid_try_format(csid, fh, MSM_CSID_PAD_SRC, format,
				fmt->which);
	}

	return 0;
}

/*
 * csid_init_formats - Initialize formats on all pads
 * @sd: CSID V4L2 subdevice
 * @fh: V4L2 subdev file handle
 *
 * Initialize all pad formats with default values.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_init_formats(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_subdev_format format;
	pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
	memset(&format, 0, sizeof(format));
	format.pad = MSM_CSID_PAD_SINK;
	format.which = fh ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	format.format.code = MEDIA_BUS_FMT_UYVY8_2X8;
	format.format.width = 1920;
	format.format.height = 1080;

	return csid_set_format(sd, fh ? fh : NULL, &format);
}

static const char * const csid_test_pattern_menu[] = {
	"Disabled",
	"Incrementing",
	"Alternating 55/AA",
	"All Zeros",
	"All Ones",
	"Random Data",
};

/*
 * csid_set_test_pattern - Set test generator's pattern mode
 * @csid: CSID device
 * @value: desired test pattern mode
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_set_test_pattern(struct csid_device *csid, s32 value)
{
	struct csid_testgen_config *tg = &csid->testgen;

	/* If CSID is linked to CSIPHY, do not allow to enable test generator */
	if (value && media_entity_remote_pad(&csid->pads[MSM_CSID_PAD_SINK]))
		return -EBUSY;

	tg->enabled = !!value;

	switch (value) {
	case 1:
		tg->payload_mode = CSID_PAYLOAD_MODE_INCREMENTING;
		break;
	case 2:
		tg->payload_mode = CSID_PAYLOAD_MODE_ALTERNATING_55_AA;
		break;
	case 3:
		tg->payload_mode = CSID_PAYLOAD_MODE_ALL_ZEROES;
		break;
	case 4:
		tg->payload_mode = CSID_PAYLOAD_MODE_ALL_ONES;
		break;
	case 5:
		tg->payload_mode = CSID_PAYLOAD_MODE_RANDOM;
		break;
	}

	return 0;
}

/*
 * csid_s_ctrl - Handle set control subdev method
 * @ctrl: pointer to v4l2 control structure
 *
 * Return 0 on success or a negative error code otherwise
 */
static int csid_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct csid_device *csid = container_of(ctrl->handler,
						struct csid_device, ctrls);
	int ret = -EINVAL;

	switch (ctrl->id) {
	case V4L2_CID_TEST_PATTERN:
		ret = csid_set_test_pattern(csid, ctrl->val);
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops csid_ctrl_ops = {
	.s_ctrl = csid_s_ctrl,
};

/*
 * msm_csid_subdev_init - Initialize CSID device structure and resources
 * @csid: CSID device
 * @res: CSID module resources table
 * @id: CSID module id
 *
 * Return 0 on success or a negative error code otherwise
 */
int msm_csid_subdev_init(struct csid_device *csid,
			 struct resources *res, u8 id)
{
	struct device *dev = to_device_index(csid, id);
	struct platform_device *pdev = container_of(dev, struct platform_device,
						    dev);
	struct resource *r;
	int i;
	int ret;

	csid->id = id;

	/* Memory */

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, res->reg[0]);
	csid->base = devm_ioremap_resource(dev, r);
	if (IS_ERR(csid->base)) {
		dev_err(dev, "could not map memory\n");
		return PTR_ERR(csid->base);
	}

	/* Interrupt */

	r = platform_get_resource_byname(pdev, IORESOURCE_IRQ,
					 res->interrupt[0]);
	if (!r) {
		dev_err(dev, "missing IRQ\n");
		return -EINVAL;
	}

	csid->irq = r->start;
	snprintf(csid->irq_name, sizeof(csid->irq_name), "%s_%s%d",
		 dev_name(dev), MSM_CSID_NAME, csid->id);
	ret = devm_request_irq(dev, csid->irq, csid_isr,
		IRQF_TRIGGER_RISING, csid->irq_name, csid);
	if (ret < 0) {
		dev_err(dev, "request_irq failed\n");
		return ret;
	}

	disable_irq(csid->irq);

	/* Clocks */

	csid->nclocks = 0;
	while (res->clock[csid->nclocks])
		csid->nclocks++;

	csid->clock = devm_kzalloc(dev, csid->nclocks * sizeof(*csid->clock),
				    GFP_KERNEL);
	if (!csid->clock)
		return -ENOMEM;

	for (i = 0; i < csid->nclocks; i++) {
		csid->clock[i] = devm_clk_get(dev, res->clock[i]);
		if (IS_ERR(csid->clock[i]))
			return PTR_ERR(csid->clock[i]);

		if (res->clock_rate[i]) {
			long clk_rate = clk_round_rate(csid->clock[i],
						       res->clock_rate[i]);
			if (clk_rate < 0) {
				dev_err(to_device_index(csid, csid->id),
					"clk round rate failed\n");
				return -EINVAL;
			}
			ret = clk_set_rate(csid->clock[i], clk_rate);
			if (ret < 0) {
				dev_err(to_device_index(csid, csid->id),
					"clk set rate failed\n");
				return ret;
			}
		}
	}

	/* Regulator */

	csid->vdda = devm_regulator_get(dev, res->regulator[0]);
	if (IS_ERR(csid->vdda)) {
		dev_err(dev, "could not get regulator\n");
		return PTR_ERR(csid->vdda);
	}

	csid->mmagic = devm_regulator_get(dev, res->regulator[1]);
	if (IS_ERR(csid->mmagic)) {
		dev_err(dev, "could not get regulator\n");
		return PTR_ERR(csid->mmagic);
	}
	csid->gdscr = devm_regulator_get(dev, res->regulator[2]);
	if (IS_ERR(csid->gdscr)) {
		dev_err(dev, "could not get regulator\n");
		return PTR_ERR(csid->gdscr);
	}


	init_completion(&csid->reset_complete);

	csid->ctrl_reg = NULL;
	csid->ctrl_reg = kzalloc(sizeof(struct csid_ctrl_t),
		GFP_KERNEL);
	if (!csid->ctrl_reg) {
		pr_err("%s:%d kzalloc failed\n", __func__, __LINE__);
		return -ENOMEM;
	}
	csid->ctrl_reg->csid_reg = csid_v3_5;
	csid->ctrl_reg->csid_lane_assign = csid_lane_assign_v3_5;

	return 0;
}

/*
 * msm_csid_get_csid_id - Get CSID HW module id
 * @entity: Pointer to CSID media entity structure
 * @id: Return CSID HW module id here
 */
void msm_csid_get_csid_id(struct media_entity *entity, u8 *id)
{
	struct v4l2_subdev *sd;
	struct csid_device *csid;

	sd = container_of(entity, struct v4l2_subdev, entity);
	csid = v4l2_get_subdevdata(sd);

	*id = csid->id;
}

/*
 * csid_get_lane_assign - Calculate CSI2 lane assign configuration parameter
 * @lane_cfg - CSI2 lane configuration
 *
 * Return lane assign
 */
static u32 csid_get_lane_assign(struct csiphy_lanes_cfg *lane_cfg)
{
	u32 lane_assign = 0;
	int i;

	for (i = 0; i < lane_cfg->num_data; i++)
		lane_assign |= lane_cfg->data[i].pos << (i * 4);

	return lane_assign;
}

/*
 * csid_link_setup - Setup CSID connections
 * @entity: Pointer to media entity structure
 * @local: Pointer to local pad
 * @remote: Pointer to remote pad
 * @flags: Link flags
 *
 * Return 0 on success
 */
static int csid_link_setup(struct media_entity *entity,
			   const struct media_pad *local,
			   const struct media_pad *remote, u32 flags)
{
	pr_err("[camera] : %s, local entity = %s, remote entity = %s\n", __func__, local->entity->name, remote->entity->name);
	if (flags & MEDIA_LNK_FL_ENABLED)
		if (media_entity_remote_pad((struct media_pad *)local))
			return -EBUSY;

	if ((local->flags & MEDIA_PAD_FL_SINK) &&
	    (flags & MEDIA_LNK_FL_ENABLED)) {
		struct v4l2_subdev *sd;
		struct csid_device *csid;
		struct csiphy_device *csiphy;
		struct csiphy_lanes_cfg *lane_cfg;
		struct v4l2_subdev_format format;

		sd = container_of(entity, struct v4l2_subdev, entity);
		csid = v4l2_get_subdevdata(sd);

		/* If test generator is enabled */
		/* do not allow a link from CSIPHY to CSID */
		if (csid->testgen_mode->cur.val != 0)
			return -EBUSY;

		sd = container_of(remote->entity, struct v4l2_subdev, entity);
		csiphy = v4l2_get_subdevdata(sd);

		/* If a sensor is not linked to CSIPHY */
		/* do no allow a link from CSIPHY to CSID */
		if (!csiphy->cfg.csi2)
			return -EPERM;

		csid->phy.csiphy_id = csiphy->id;

		lane_cfg = &csiphy->cfg.csi2->lane_cfg;
		csid->phy.lane_cnt = lane_cfg->num_data;
		csid->phy.lane_assign = csid_get_lane_assign(lane_cfg);
		pr_err("[camera] %s : %d, : %d : 0x%x\n", __func__, csid->phy.csiphy_id, csid->phy.lane_cnt, csid->phy.lane_assign);
		/* Reset format on source pad to sink pad format */
		memset(&format, 0, sizeof(format));
		format.pad = MSM_CSID_PAD_SRC;
		format.which = V4L2_SUBDEV_FORMAT_ACTIVE;
		csid_set_format(&csid->subdev, NULL, &format);
	}

	return 0;
}

static const struct v4l2_subdev_core_ops csid_core_ops = {
	.s_power = csid_set_power,
};

static const struct v4l2_subdev_video_ops csid_video_ops = {
	.s_stream = csid_set_stream,
};

static const struct v4l2_subdev_pad_ops csid_pad_ops = {
	.enum_mbus_code = csid_enum_mbus_code,
	.enum_frame_size = csid_enum_frame_size,
	.get_fmt = csid_get_format,
	.set_fmt = csid_set_format,
};

static const struct v4l2_subdev_ops csid_v4l2_ops = {
	.core = &csid_core_ops,
	.video = &csid_video_ops,
	.pad = &csid_pad_ops,
};

static const struct v4l2_subdev_internal_ops csid_v4l2_internal_ops = {
	.open = csid_init_formats,
};

static const struct media_entity_operations csid_media_ops = {
	.link_setup = csid_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

/*
 * msm_csid_register_entity - Register subdev node for CSID module
 * @csid: CSID device
 * @v4l2_dev: V4L2 device
 *
 * Return 0 on success or a negative error code otherwise
 */
int msm_csid_register_entity(struct csid_device *csid,
			     struct v4l2_device *v4l2_dev)
{
	struct v4l2_subdev *sd = &csid->subdev;
	struct media_pad *pads = csid->pads;
	struct device *dev = to_device_index(csid, csid->id);
	int ret;

	v4l2_subdev_init(sd, &csid_v4l2_ops);
	sd->internal_ops = &csid_v4l2_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(sd->name, ARRAY_SIZE(sd->name), "%s%d",
		 MSM_CSID_NAME, csid->id);
	v4l2_set_subdevdata(sd, csid);

	ret = v4l2_ctrl_handler_init(&csid->ctrls, 1);
	if (ret < 0) {
		dev_err(dev, "Failed to init ctrl handler\n");
		return ret;
	}

	csid->testgen_mode = v4l2_ctrl_new_std_menu_items(&csid->ctrls,
				&csid_ctrl_ops, V4L2_CID_TEST_PATTERN,
				ARRAY_SIZE(csid_test_pattern_menu) - 1, 0, 0,
				csid_test_pattern_menu);

	if (csid->ctrls.error) {
		dev_err(dev, "Failed to init ctrl: %d\n", csid->ctrls.error);
		ret = csid->ctrls.error;
		goto free_ctrl;
	}

	csid->subdev.ctrl_handler = &csid->ctrls;

	ret = csid_init_formats(sd, NULL);
	if (ret < 0) {
		dev_err(dev, "Failed to init format\n");
		goto free_ctrl;
	}

	pads[MSM_CSID_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
	pads[MSM_CSID_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;

	sd->entity.ops = &csid_media_ops;
	ret = media_entity_init(&sd->entity, MSM_CSID_PADS_NUM, pads, 0);
	if (ret < 0) {
		dev_err(dev, "Failed to init media entity\n");
		goto free_ctrl;
	}

	ret = v4l2_device_register_subdev(v4l2_dev, sd);
	if (ret < 0) {
		dev_err(dev, "Failed to register subdev\n");
		goto media_cleanup;
	}

	return 0;

media_cleanup:
	media_entity_cleanup(&sd->entity);
free_ctrl:
	v4l2_ctrl_handler_free(&csid->ctrls);

	return ret;
}

/*
 * msm_csid_unregister_entity - Unregister CSID module subdev node
 * @csid: CSID device
 */
void msm_csid_unregister_entity(struct csid_device *csid)
{
	v4l2_device_unregister_subdev(&csid->subdev);
	v4l2_ctrl_handler_free(&csid->ctrls);
}
