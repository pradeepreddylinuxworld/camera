/*
 * ispif.c
 *
 * Qualcomm MSM Camera Subsystem - ISPIF Module
 *
 * Copyright (c) 2013-2015, The Linux Foundation. All rights reserved.
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
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

#include "ispif.h"
#include "camss.h"

#define MSM_ISPIF_NAME "msm_ispif"

#define ispif_line_array(ptr_line)	\
	((const struct ispif_line (*)[]) &(ptr_line[-(ptr_line->id)]))

#define to_ispif(ptr_line)	\
	container_of(ispif_line_array(ptr_line), struct ispif_device, ptr_line)

#define ISPIF_RST_CMD_0			0x008
#define ISPIF_IRQ_GLOBAL_CLEAR_CMD_ADDR	0x01c
#define ISPIF_VFE_m_CTRL_0(m)		(0x200 + 0x200 * (m))
#define ISPIF_VFE_m_CTRL_0_PIX0_LINE_BUF_EN	(1 << 6)
#define ISPIF_VFE_m_IRQ_MASK_0(m)	(0x208 + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_MASK_0_PIX0_ENABLE	0x00001249
#define ISPIF_VFE_m_IRQ_MASK_0_PIX0_MASK	0x00001fff
#define ISPIF_VFE_m_IRQ_MASK_0_RDI0_ENABLE	0x02492000
#define ISPIF_VFE_m_IRQ_MASK_0_RDI0_MASK	0x03ffe000
#define ISPIF_VFE_m_IRQ_MASK_1(m)	(0x20c + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_MASK_1_PIX1_ENABLE	0x00001249
#define ISPIF_VFE_m_IRQ_MASK_1_PIX1_MASK	0x00001fff
#define ISPIF_VFE_m_IRQ_MASK_1_RDI1_ENABLE	0x02492000
#define ISPIF_VFE_m_IRQ_MASK_1_RDI1_MASK	0x03ffe000
#define ISPIF_VFE_m_IRQ_MASK_2(m)	(0x210 + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_MASK_2_RDI2_ENABLE	0x00001249
#define ISPIF_VFE_m_IRQ_MASK_2_RDI2_MASK	0x00001fff
#define ISPIF_VFE_m_IRQ_STATUS_0(m)	(0x21c + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_STATUS_0_PIX0_OVERFLOW	(1 << 12)
#define ISPIF_VFE_m_IRQ_STATUS_0_RDI0_OVERFLOW	(1 << 25)
#define ISPIF_VFE_m_IRQ_STATUS_1(m)	(0x220 + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_STATUS_1_PIX1_OVERFLOW	(1 << 12)
#define ISPIF_VFE_m_IRQ_STATUS_1_RDI1_OVERFLOW	(1 << 25)
#define ISPIF_VFE_m_IRQ_STATUS_2(m)	(0x224 + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_STATUS_2_RDI2_OVERFLOW	(1 << 12)
#define ISPIF_VFE_m_IRQ_CLEAR_0(m)	(0x230 + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_CLEAR_1(m)	(0x234 + 0x200 * (m))
#define ISPIF_VFE_m_IRQ_CLEAR_2(m)	(0x238 + 0x200 * (m))
#define ISPIF_VFE_m_INTF_INPUT_SEL(m)	(0x244 + 0x200 * (m))
#define ISPIF_VFE_m_INTF_CMD_0(m)	(0x248 + 0x200 * (m))
#define ISPIF_VFE_m_INTF_CMD_1(m)	(0x24c + 0x200 * (m))
#define ISPIF_VFE_m_PIX_INTF_n_CID_MASK(m, n)	\
					(0x254 + 0x200 * (m) + 0x4 * (n))
#define ISPIF_VFE_m_RDI_INTF_n_CID_MASK(m, n)	\
					(0x264 + 0x200 * (m) + 0x4 * (n))
#define ISPIF_VFE_m_PIX_INTF_n_CROP(m, n)   \
					(0x288 + 0x200 * (m) + 0x4 * (n))
#define ISPIF_VFE_m_PIX_INTF_n_STATUS(m, n)	\
					(0x2c0 + 0x200 * (m) + 0x4 * (n))
#define ISPIF_VFE_m_RDI_INTF_n_STATUS(m, n)	\
					(0x2d0 + 0x200 * (m) + 0x4 * (n))

#define CSI_PIX_CLK_MUX_SEL		0x000
#define CSI_RDI_CLK_MUX_SEL		0x008

#define ISPIF_TIMEOUT_SLEEP_US		1000
#define ISPIF_TIMEOUT_ALL_US		1000000
#define ISPIF_RESET_TIMEOUT_MS		500

#define ISPIF_IRQ_GLOBAL_CLEAR_CMD               0x1

#define ISPIF_STOP_INTF_IMMEDIATELY              0xAAAAAAAA
#define PIX0_LINE_BUF_EN_BIT                     6
#define ISPIF_IRQ_STATUS_MASK                    0x0A493249
#define ISPIF_IRQ_STATUS_1_MASK                  0x02493249
#define ISPIF_IRQ_STATUS_2_MASK                  0x00001249


enum ispif_intf_cmd {
	CMD_DISABLE_FRAME_BOUNDARY = 0x0,
	CMD_ENABLE_FRAME_BOUNDARY = 0x1,
	CMD_DISABLE_IMMEDIATELY = 0x2,
	CMD_ALL_DISABLE_IMMEDIATELY = 0xaaaaaaaa,
	CMD_ALL_NO_CHANGE = 0xffffffff,
};

static const u32 ispif_formats[] = {
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


#define ISPIF_IRQ_STATUS_PIX_SOF_MASK            0x249
#define ISPIF_IRQ_STATUS_RDI0_SOF_MASK           0x492000
#define ISPIF_IRQ_STATUS_RDI1_SOF_MASK           0x492000
#define ISPIF_IRQ_STATUS_RDI2_SOF_MASK           0x249


/*
 * ispif_isr - ISPIF module interrupt handler
 * @irq: Interrupt line
 * @dev: ISPIF device
 *
 * Return IRQ_HANDLED on success
 */
static irqreturn_t ispif_isr(int irq, void *dev)
{
	struct ispif_device *ispif = dev;
	u32 value0, value1, value2;

	value0 = readl_relaxed(ispif->base + ISPIF_VFE_m_IRQ_STATUS_0(0));
	value1 = readl_relaxed(ispif->base + ISPIF_VFE_m_IRQ_STATUS_1(0));
	value2 = readl_relaxed(ispif->base + ISPIF_VFE_m_IRQ_STATUS_2(0));

	writel_relaxed(value0, ispif->base + ISPIF_VFE_m_IRQ_CLEAR_0(0));
	writel_relaxed(value1, ispif->base + ISPIF_VFE_m_IRQ_CLEAR_1(0));
	writel_relaxed(value2, ispif->base + ISPIF_VFE_m_IRQ_CLEAR_2(0));

	writel(ISPIF_IRQ_GLOBAL_CLEAR_CMD, ispif->base + ISPIF_IRQ_GLOBAL_CLEAR_CMD_ADDR);

	if ((value0 >> 27) & 0x1)
		complete(&ispif->reset_complete);

	if (unlikely(value0 & ISPIF_VFE_m_IRQ_STATUS_0_PIX0_OVERFLOW))
		dev_err_ratelimited(to_device(ispif), "VFE0 pix0 overflow\n");

	if (unlikely(value0 & ISPIF_VFE_m_IRQ_STATUS_0_RDI0_OVERFLOW))
		dev_err_ratelimited(to_device(ispif), "VFE0 rdi0 overflow\n");

	if (unlikely(value1 & ISPIF_VFE_m_IRQ_STATUS_1_PIX1_OVERFLOW))
		dev_err_ratelimited(to_device(ispif), "VFE0 pix1 overflow\n");

	if (unlikely(value1 & ISPIF_VFE_m_IRQ_STATUS_1_RDI1_OVERFLOW))
		dev_err_ratelimited(to_device(ispif), "VFE0 rdi1 overflow\n");

	if (unlikely(value2 & ISPIF_VFE_m_IRQ_STATUS_2_RDI2_OVERFLOW))
		dev_err_ratelimited(to_device(ispif), "VFE0 rdi2 overflow\n");

	if (value0 & ISPIF_IRQ_STATUS_PIX_SOF_MASK)
		dev_dbg_ratelimited(to_device(ispif), "VFE0 pix0 SOF\n");

	if (value0 & ISPIF_IRQ_STATUS_RDI0_SOF_MASK)
		dev_dbg_ratelimited(to_device(ispif), "VFE0 rdi0 SOF\n");

	if (value1 & ISPIF_IRQ_STATUS_RDI1_SOF_MASK)
		dev_dbg_ratelimited(to_device(ispif), "VFE0 rdi1 SOF\n");

	if (value2 & ISPIF_IRQ_STATUS_RDI2_SOF_MASK)
		dev_dbg_ratelimited(to_device(ispif), "VFE0 rdi2 SOF\n");

	return IRQ_HANDLED;
}

#if 0
/*
 * ispif_reset_hw - Trigger reset on ISPIF module and wait to complete
 * @ispif: ISPIF device
 *
 * Return 0 on success or a negative error code otherwise
 */
static int ispif_reset_hw(struct ispif_device *ispif)
{
	unsigned long time;
/*	int ret;

	ret = camss_enable_clocks(ispif->nclocks_for_reset,
				  ispif->clock_for_reset,
				  to_device(ispif));
	if (ret < 0)
		return ret;
*/
	reinit_completion(&ispif->reset_complete);

//	writel_relaxed(0x000f1fff, ispif->base + ISPIF_RST_CMD_0);
	writel_relaxed(0xFE7F1FFF, ispif->base + ISPIF_RST_CMD_0);

	time = wait_for_completion_timeout(&ispif->reset_complete,
		msecs_to_jiffies(ISPIF_RESET_TIMEOUT_MS));
	if (!time) {
		dev_err(to_device(ispif), "ISPIF reset timeout\n");
		return -EIO;
	}

//	camss_disable_clocks(ispif->nclocks_for_reset, ispif->clock_for_reset);

	return 0;
}
#endif

static int ispif_reset(struct ispif_device *ispif)
{
	int rc = 0;
	int i = 0;

	BUG_ON(!ispif);

//	memset(ispif->sof_count, 0, sizeof(ispif->sof_count));
//	for (i = 0; i < ispif->vfe_info.num_vfe; i++) {

		writel_relaxed(1 << PIX0_LINE_BUF_EN_BIT,
			ispif->base + ISPIF_VFE_m_CTRL_0(i));
		writel_relaxed(0, ispif->base + ISPIF_VFE_m_IRQ_MASK_0(i));
		writel_relaxed(0, ispif->base + ISPIF_VFE_m_IRQ_MASK_1(i));
		writel_relaxed(0, ispif->base + ISPIF_VFE_m_IRQ_MASK_2(i));
		writel_relaxed(0xFFFFFFFF, ispif->base +
			ISPIF_VFE_m_IRQ_CLEAR_0(i));
		writel_relaxed(0xFFFFFFFF, ispif->base +
			ISPIF_VFE_m_IRQ_CLEAR_1(i));
		writel_relaxed(0xFFFFFFFF, ispif->base +
			ISPIF_VFE_m_IRQ_CLEAR_2(i));

		writel_relaxed(0, ispif->base + ISPIF_VFE_m_INTF_INPUT_SEL(i));

		writel_relaxed(ISPIF_STOP_INTF_IMMEDIATELY,
			ispif->base + ISPIF_VFE_m_INTF_CMD_0(i));
		writel_relaxed(ISPIF_STOP_INTF_IMMEDIATELY,
			ispif->base + ISPIF_VFE_m_INTF_CMD_1(i));
		pr_debug("%s: base %lx", __func__, (unsigned long)ispif->base);
		writel_relaxed(0, ispif->base +
			ISPIF_VFE_m_PIX_INTF_n_CID_MASK(i, 0));
		writel_relaxed(0, ispif->base +
			ISPIF_VFE_m_PIX_INTF_n_CID_MASK(i, 1));
		writel_relaxed(0, ispif->base +
			ISPIF_VFE_m_RDI_INTF_n_CID_MASK(i, 0));
		writel_relaxed(0, ispif->base +
			ISPIF_VFE_m_RDI_INTF_n_CID_MASK(i, 1));
		writel_relaxed(0, ispif->base +
			ISPIF_VFE_m_RDI_INTF_n_CID_MASK(i, 2));

		writel_relaxed(0, ispif->base +
			ISPIF_VFE_m_PIX_INTF_n_CROP(i, 0));
		writel_relaxed(0, ispif->base +
			ISPIF_VFE_m_PIX_INTF_n_CROP(i, 1));
//	}

	wmb();
	writel_relaxed(ISPIF_IRQ_GLOBAL_CLEAR_CMD, ispif->base +
		ISPIF_IRQ_GLOBAL_CLEAR_CMD_ADDR);
	wmb();
	return rc;
}


/*
 * ispif_set_power - Power on/off ISPIF module
 * @sd: ISPIF V4L2 subdevice
 * @on: Requested power state
 *
 * Return 0 on success or a negative error code otherwise
 */
static int ispif_set_power(struct v4l2_subdev *sd, int on)
{
	struct ispif_line *line = v4l2_get_subdevdata(sd);
	struct ispif_device *ispif = to_ispif(line);
	struct device *dev = to_device(ispif);
	int ret = 0;
	pr_err("[camera] : %s, on = %d\n", __func__, on);
	mutex_lock(&ispif->power_lock);

	if (on) {
		if (ispif->power_count) {
			/* Power is already on */
			ispif->power_count++;
			goto exit;
		}

		ret = regulator_enable(ispif->vfe0);
		if (ret < 0){
			dev_err(to_device(ispif), "Can not enable vfe0 regulator\n");
			return ret;
		}
//		ret = regulator_enable(ispif->vfe1);
//		if (ret < 0){
//			dev_err(to_device(ispif), "Can not enable vfe1 regulator\n");
//			return ret;
//		}
		ret = regulator_enable(ispif->gdscr);
		if (ret < 0){
			dev_err(to_device(ispif), "Can not enable gdscr regulator\n");
			return ret;
		}
		ret = regulator_enable(ispif->mmagic);
		if (ret < 0){
			dev_err(to_device(ispif), "Can not enable mmagic regulator\n");
			return ret;
		}

		ret = camss_enable_clocks(ispif->nclocks, ispif->clock, dev);
		if (ret < 0)
			goto exit;
		pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
#if 0
		ret = ispif_reset_hw(ispif);
		if (ret < 0) {
			camss_disable_clocks(ispif->nclocks, ispif->clock);
			regulator_disable(ispif->vfe0);
		//	regulator_disable(ispif->vfe1);
			regulator_disable(ispif->gdscr);
			regulator_disable(ispif->mmagic);
			goto exit;
		}
		pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
#endif
		ispif_reset(ispif);
		ispif->intf_cmd[line->vfe_id].cmd_0 = CMD_ALL_NO_CHANGE;
		ispif->intf_cmd[line->vfe_id].cmd_1 = CMD_ALL_NO_CHANGE;

		ispif->power_count++;
	} else {
		if (ispif->power_count == 0) {
			dev_err(dev, "ispif power off on power_count == 0\n");
			goto exit;
		} else if (ispif->power_count == 1) {
			camss_disable_clocks(ispif->nclocks, ispif->clock);
			regulator_disable(ispif->vfe0);
		//	regulator_disable(ispif->vfe1);
			regulator_disable(ispif->gdscr);
			regulator_disable(ispif->mmagic);
		}

		ispif->power_count--;
	}

exit:
	mutex_unlock(&ispif->power_lock);

	return ret;
}

/*
 * ispif_select_clk_mux - Select clock for PIX/RDI interface
 * @ispif: ISPIF device
 * @intf: VFE interface
 * @csid: CSID HW module id
 * @vfe: VFE HW module id
 * @enable: enable or disable the selected clock
 */
static void ispif_select_clk_mux(struct ispif_device *ispif,
				 enum ispif_intf intf, u8 csid,
				 u8 vfe, u8 enable)
{
	u32 val;

	switch (intf) {
	case PIX0:
		val = readl_relaxed(ispif->base_clk_mux + CSI_PIX_CLK_MUX_SEL);
		val &= ~(0xf << (vfe * 8));
		if (enable)
			val |= (csid << (vfe * 8));
		writel_relaxed(val, ispif->base_clk_mux + CSI_PIX_CLK_MUX_SEL);
		break;

	case RDI0:
		val = readl_relaxed(ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		val &= ~(0xf << (vfe * 12));
		if (enable)
			val |= (csid << (vfe * 12));
		writel_relaxed(val, ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		break;

	case PIX1:
		val = readl_relaxed(ispif->base_clk_mux + CSI_PIX_CLK_MUX_SEL);
		val &= ~(0xf << (4 + (vfe * 8)));
		if (enable)
			val |= (csid << (4 + (vfe * 8)));
		writel_relaxed(val, ispif->base_clk_mux + CSI_PIX_CLK_MUX_SEL);
		break;

	case RDI1:
		val = readl_relaxed(ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		val &= ~(0xf << (4 + (vfe * 12)));
		if (enable)
			val |= (csid << (4 + (vfe * 12)));
		writel_relaxed(val, ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		break;

	case RDI2:
		val = readl_relaxed(ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		val &= ~(0xf << (8 + (vfe * 12)));
		if (enable)
			val |= (csid << (8 + (vfe * 12)));
		writel_relaxed(val, ispif->base_clk_mux + CSI_RDI_CLK_MUX_SEL);
		break;
	}

	mb();
}

/*
 * ispif_validate_intf_status - Validate current status of PIX/RDI interface
 * @ispif: ISPIF device
 * @intf: VFE interface
 * @vfe: VFE HW module id
 *
 * Return 0 when interface is idle or -EBUSY otherwise
 */
static int ispif_validate_intf_status(struct ispif_device *ispif,
				      enum ispif_intf intf, u8 vfe)
{
	int ret = 0;
	u32 val = 0;

	switch (intf) {
	case PIX0:
		val = readl_relaxed(ispif->base +
			ISPIF_VFE_m_PIX_INTF_n_STATUS(vfe, 0));
		break;
	case RDI0:
		val = readl_relaxed(ispif->base +
			ISPIF_VFE_m_RDI_INTF_n_STATUS(vfe, 0));
		break;
	case PIX1:
		val = readl_relaxed(ispif->base +
			ISPIF_VFE_m_PIX_INTF_n_STATUS(vfe, 1));
		break;
	case RDI1:
		val = readl_relaxed(ispif->base +
			ISPIF_VFE_m_RDI_INTF_n_STATUS(vfe, 1));
		break;
	case RDI2:
		val = readl_relaxed(ispif->base +
			ISPIF_VFE_m_RDI_INTF_n_STATUS(vfe, 2));
		break;
	}

	if ((val & 0xf) != 0xf) {
		dev_err(to_device(ispif), "%s: ispif is busy: 0x%x\n",
			__func__, val);
		ret = -EBUSY;
	}

	return ret;
}

/*
 * ispif_wait_for_stop - Wait for PIX/RDI interface to stop
 * @ispif: ISPIF device
 * @intf: VFE interface
 * @vfe: VFE HW module id
 *
 * Return 0 on success or a negative error code otherwise
 */
static int ispif_wait_for_stop(struct ispif_device *ispif,
			       enum ispif_intf intf, u8 vfe)
{
	u32 addr = 0;
	u32 stop_flag = 0;
	int ret;

	switch (intf) {
	case PIX0:
		addr = ISPIF_VFE_m_PIX_INTF_n_STATUS(vfe, 0);
		break;
	case RDI0:
		addr = ISPIF_VFE_m_RDI_INTF_n_STATUS(vfe, 0);
		break;
	case PIX1:
		addr = ISPIF_VFE_m_PIX_INTF_n_STATUS(vfe, 1);
		break;
	case RDI1:
		addr = ISPIF_VFE_m_RDI_INTF_n_STATUS(vfe, 1);
		break;
	case RDI2:
		addr = ISPIF_VFE_m_RDI_INTF_n_STATUS(vfe, 2);
		break;
	}

	ret = readl_poll_timeout(ispif->base + addr,
				 stop_flag,
				 (stop_flag & 0xf) == 0xf,
				 ISPIF_TIMEOUT_SLEEP_US,
				 ISPIF_TIMEOUT_ALL_US);
	if (ret < 0)
		dev_err(to_device(ispif), "%s: ispif stop timeout\n",
			__func__);

	return ret;
}

/*
 * ispif_select_csid - Select CSID HW module for input from
 * @ispif: ISPIF device
 * @intf: VFE interface
 * @csid: CSID HW module id
 * @vfe: VFE HW module id
 * @enable: enable or disable the selected input
 */
static void ispif_select_csid(struct ispif_device *ispif, enum ispif_intf intf,
			      u8 csid, u8 vfe, u8 enable)
{
	u32 val;

	val = readl_relaxed(ispif->base + ISPIF_VFE_m_INTF_INPUT_SEL(vfe));
	switch (intf) {
	case PIX0:
		val &= ~(BIT(1) | BIT(0));
		if (enable)
			val |= csid;
		break;
	case RDI0:
		val &= ~(BIT(5) | BIT(4));
		if (enable)
			val |= (csid << 4);
		break;
	case PIX1:
		val &= ~(BIT(9) | BIT(8));
		if (enable)
			val |= (csid << 8);
		break;
	case RDI1:
		val &= ~(BIT(13) | BIT(12));
		if (enable)
			val |= (csid << 12);
		break;
	case RDI2:
		val &= ~(BIT(21) | BIT(20));
		if (enable)
			val |= (csid << 20);
		break;
	}

	writel(val, ispif->base + ISPIF_VFE_m_INTF_INPUT_SEL(vfe));
}

/*
 * ispif_select_cid - Enable/disable desired CID
 * @ispif: ISPIF device
 * @intf: VFE interface
 * @cid: desired CID to enable/disable
 * @vfe: VFE HW module id
 * @enable: enable or disable the desired CID
 */
static void ispif_select_cid(struct ispif_device *ispif, enum ispif_intf intf,
			     u8 cid, u8 vfe, u8 enable)
{
	u32 cid_mask = 1 << cid;
	u32 addr = 0;
	u32 val;

	switch (intf) {
	case PIX0:
		addr = ISPIF_VFE_m_PIX_INTF_n_CID_MASK(vfe, 0);
		break;
	case RDI0:
		addr = ISPIF_VFE_m_RDI_INTF_n_CID_MASK(vfe, 0);
		break;
	case PIX1:
		addr = ISPIF_VFE_m_PIX_INTF_n_CID_MASK(vfe, 1);
		break;
	case RDI1:
		addr = ISPIF_VFE_m_RDI_INTF_n_CID_MASK(vfe, 1);
		break;
	case RDI2:
		addr = ISPIF_VFE_m_RDI_INTF_n_CID_MASK(vfe, 2);
		break;
	}

	val = readl_relaxed(ispif->base + addr);
	if (enable)
		val |= cid_mask;
	else
		val &= ~cid_mask;

	writel(val, ispif->base + addr);
}

/*
 * ispif_config_irq - Enable/disable interrupts for PIX/RDI interface
 * @ispif: ISPIF device
 * @intf: VFE interface
 * @vfe: VFE HW module id
 * @enable: enable or disable
 */
static void ispif_config_irq(struct ispif_device *ispif, enum ispif_intf intf,
			     u8 vfe, u8 enable)
{
	u32 val;

	switch (intf) {
	case PIX0:
		val = readl_relaxed(ispif->base + ISPIF_VFE_m_IRQ_MASK_0(vfe));
		val &= ~ISPIF_VFE_m_IRQ_MASK_0_PIX0_MASK;
		if (enable)
			val |= ISPIF_VFE_m_IRQ_MASK_0_PIX0_ENABLE;
		writel_relaxed(val, ispif->base + ISPIF_VFE_m_IRQ_MASK_0(vfe));
		writel_relaxed(ISPIF_VFE_m_IRQ_MASK_0_PIX0_ENABLE,
			       ispif->base + ISPIF_VFE_m_IRQ_CLEAR_0(vfe));
		break;
	case RDI0:
		val = readl_relaxed(ispif->base + ISPIF_VFE_m_IRQ_MASK_0(vfe));
		val &= ~ISPIF_VFE_m_IRQ_MASK_0_RDI0_MASK;
		if (enable)
			val |= ISPIF_VFE_m_IRQ_MASK_0_RDI0_ENABLE;
		writel_relaxed(val, ispif->base + ISPIF_VFE_m_IRQ_MASK_0(vfe));
		writel_relaxed(ISPIF_VFE_m_IRQ_MASK_0_RDI0_ENABLE,
			       ispif->base + ISPIF_VFE_m_IRQ_CLEAR_0(vfe));
		break;
	case PIX1:
		val = readl_relaxed(ispif->base + ISPIF_VFE_m_IRQ_MASK_1(vfe));
		val &= ~ISPIF_VFE_m_IRQ_MASK_1_PIX1_MASK;
		if (enable)
			val |= ISPIF_VFE_m_IRQ_MASK_1_PIX1_ENABLE;
		writel_relaxed(val, ispif->base + ISPIF_VFE_m_IRQ_MASK_1(vfe));
		writel_relaxed(ISPIF_VFE_m_IRQ_MASK_1_PIX1_ENABLE,
			       ispif->base + ISPIF_VFE_m_IRQ_CLEAR_1(vfe));
		break;
	case RDI1:
		val = readl_relaxed(ispif->base + ISPIF_VFE_m_IRQ_MASK_1(vfe));
		val &= ~ISPIF_VFE_m_IRQ_MASK_1_RDI1_MASK;
		if (enable)
			val |= ISPIF_VFE_m_IRQ_MASK_1_RDI1_ENABLE;
		writel_relaxed(val, ispif->base + ISPIF_VFE_m_IRQ_MASK_1(vfe));
		writel_relaxed(ISPIF_VFE_m_IRQ_MASK_1_RDI1_ENABLE,
			       ispif->base + ISPIF_VFE_m_IRQ_CLEAR_1(vfe));
		break;
	case RDI2:
		val = readl_relaxed(ispif->base + ISPIF_VFE_m_IRQ_MASK_2(vfe));
		val &= ~ISPIF_VFE_m_IRQ_MASK_2_RDI2_MASK;
		if (enable)
			val |= ISPIF_VFE_m_IRQ_MASK_2_RDI2_ENABLE;
		writel_relaxed(val, ispif->base + ISPIF_VFE_m_IRQ_MASK_2(vfe));
		writel_relaxed(ISPIF_VFE_m_IRQ_MASK_2_RDI2_ENABLE,
			       ispif->base + ISPIF_VFE_m_IRQ_CLEAR_2(vfe));
		break;
	}

	writel(ISPIF_IRQ_GLOBAL_CLEAR_CMD, ispif->base + ISPIF_IRQ_GLOBAL_CLEAR_CMD_ADDR);
}

/*
 * ispif_set_intf_cmd - Set command to enable/disable interface
 * @ispif: ISPIF device
 * @cmd: interface command
 * @intf: VFE interface
 * @vfe: VFE HW module id
 * @vc: virtual channel
 */
static void ispif_set_intf_cmd(struct ispif_device *ispif, u8 cmd,
			       enum ispif_intf intf, u8 vfe, u8 vc)
{
	u32 *val;

	if (intf == RDI2) {
		val = &ispif->intf_cmd[vfe].cmd_1;
		*val &= ~(0x3 << (vc * 2 + 8));
		*val |= (cmd << (vc * 2 + 8));
		wmb();
		writel_relaxed(*val, ispif->base + ISPIF_VFE_m_INTF_CMD_1(vfe));
		wmb();
	} else {
		val = &ispif->intf_cmd[vfe].cmd_0;
		*val &= ~(0x3 << (vc * 2 + intf * 8));
		*val |= (cmd << (vc * 2 + intf * 8));
		wmb();
		writel_relaxed(*val, ispif->base + ISPIF_VFE_m_INTF_CMD_0(vfe));
		wmb();
	}
}

/*
 * ispif_set_stream - Enable/disable streaming on ISPIF module
 * @sd: ISPIF V4L2 subdevice
 * @enable: Requested streaming state
 *
 * Main configuration of ISPIF module is also done here.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int ispif_set_stream(struct v4l2_subdev *sd, int enable)
{
	struct ispif_line *line = v4l2_get_subdevdata(sd);
	struct ispif_device *ispif = to_ispif(line);
	enum ispif_intf intf = line->interface;
	u8 csid = line->csid_id;
	u8 vfe = line->vfe_id;
	u8 vc = 0; /* Virtual Channel 0 */
	u8 cid = vc * 4;
	int ret;
	pr_err("[camera] : %s, enable = %d\n", __func__, enable);
	if (enable) {
		if (!media_entity_remote_pad(
					&line->pads[MSM_ISPIF_PAD_SINK])) {
			return -ENOLINK;
		}

		/* Config */

		mutex_lock(&ispif->config_lock);
		ispif_select_clk_mux(ispif, intf, csid, vfe, 1);

		ret = ispif_validate_intf_status(ispif, intf, vfe);
		if (ret < 0) {
			mutex_unlock(&ispif->config_lock);
			return ret;
		}
		pr_err("[camera] : %s, intf = %d, csid = %d, vfe = %d, cid = %d\n", __func__, intf, csid, vfe,cid);
		ispif_select_csid(ispif, intf, csid, vfe, 1);
		ispif_select_cid(ispif, intf, cid, vfe, 1);
		ispif_config_irq(ispif, intf, vfe, 1);
		ispif_set_intf_cmd(ispif, CMD_ENABLE_FRAME_BOUNDARY,
				   intf, vfe, vc);


		writel_relaxed(ISPIF_IRQ_STATUS_MASK, ispif->base +
			ISPIF_VFE_m_IRQ_MASK_0(vfe));

		writel_relaxed(ISPIF_IRQ_STATUS_MASK, ispif->base +
			ISPIF_VFE_m_IRQ_CLEAR_0(vfe));

		writel_relaxed(ISPIF_IRQ_STATUS_1_MASK, ispif->base +
			ISPIF_VFE_m_IRQ_MASK_1(vfe));

		writel_relaxed(ISPIF_IRQ_STATUS_1_MASK, ispif->base +
			ISPIF_VFE_m_IRQ_CLEAR_1(vfe));

		writel_relaxed(ISPIF_IRQ_STATUS_2_MASK, ispif->base +
			ISPIF_VFE_m_IRQ_MASK_2(vfe));

		writel_relaxed(ISPIF_IRQ_STATUS_2_MASK, ispif->base +
			ISPIF_VFE_m_IRQ_CLEAR_2(vfe));

		wmb();
		writel_relaxed(ISPIF_IRQ_GLOBAL_CLEAR_CMD, ispif->base +
			ISPIF_IRQ_GLOBAL_CLEAR_CMD_ADDR);
		wmb();
	} else {
		mutex_lock(&ispif->config_lock);
		ispif_set_intf_cmd(ispif, CMD_DISABLE_FRAME_BOUNDARY,
				   intf, vfe, vc);
		mutex_unlock(&ispif->config_lock);

		ret = ispif_wait_for_stop(ispif, intf, vfe);
		if (ret < 0)
			return ret;

		mutex_lock(&ispif->config_lock);
		ispif_config_irq(ispif, intf, vfe, 0);
		ispif_select_cid(ispif, intf, cid, vfe, 0);
		ispif_select_csid(ispif, intf, csid, vfe, 0);
		ispif_select_clk_mux(ispif, intf, csid, vfe, 0);
	}

	mutex_unlock(&ispif->config_lock);

	return 0;
}

/*
 * __ispif_get_format - Get pointer to format structure
 * @ispif: ISPIF line
 * @cfg: V4L2 subdev pad configuration
 * @pad: pad from which format is requested
 * @which: TRY or ACTIVE format
 *
 * Return pointer to TRY or ACTIVE format structure
 */
static struct v4l2_mbus_framefmt *
__ispif_get_format(struct ispif_line *line,
		   struct v4l2_subdev_fh *fh,
		   unsigned int pad,
		   enum v4l2_subdev_format_whence which)
{
	if (which == V4L2_SUBDEV_FORMAT_TRY)
		return v4l2_subdev_get_try_format(fh, pad);

	return &line->fmt[pad];
}

/*
 * ispif_try_format - Handle try format by pad subdev method
 * @ispif: ISPIF line
 * @cfg: V4L2 subdev pad configuration
 * @pad: pad on which format is requested
 * @fmt: pointer to v4l2 format structure
 * @which: wanted subdev format
 */
static void ispif_try_format(struct ispif_line *line,
			     struct v4l2_subdev_fh *fh,
			     unsigned int pad,
			     struct v4l2_mbus_framefmt *fmt,
			     enum v4l2_subdev_format_whence which)
{
	unsigned int i;

	switch (pad) {
	case MSM_ISPIF_PAD_SINK:
		/* Set format on sink pad */

		for (i = 0; i < ARRAY_SIZE(ispif_formats); i++)
			if (fmt->code == ispif_formats[i])
				break;

		/* If not found, use UYVY as default */
		if (i >= ARRAY_SIZE(ispif_formats))
			fmt->code = MEDIA_BUS_FMT_UYVY8_2X8;

		fmt->width = clamp_t(u32, fmt->width, 1, 8191);
		fmt->height = clamp_t(u32, fmt->height, 1, 8191);

		if (fmt->field == V4L2_FIELD_ANY)
			fmt->field = V4L2_FIELD_NONE;

		break;

	case MSM_ISPIF_PAD_SRC:
		/* Set and return a format same as sink pad */

		*fmt = *__ispif_get_format(line, fh, MSM_ISPIF_PAD_SINK,
					   which);

		break;
	}

	fmt->colorspace = V4L2_COLORSPACE_SRGB;
}

/*
 * ispif_enum_mbus_code - Handle pixel format enumeration
 * @sd: ISPIF V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @code: pointer to v4l2_subdev_mbus_code_enum structure
 * return -EINVAL or zero on success
 */
static int ispif_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_mbus_code_enum *code)
{
	struct ispif_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;
	pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
	if (code->pad == MSM_ISPIF_PAD_SINK) {
		if (code->index >= ARRAY_SIZE(ispif_formats))
			return -EINVAL;

		code->code = ispif_formats[code->index];
	} else {
		if (code->index > 0)
			return -EINVAL;

		format = __ispif_get_format(line, fh, MSM_ISPIF_PAD_SINK,
					    code->which);

		code->code = format->code;
	}

	return 0;
}

/*
 * ispif_enum_frame_size - Handle frame size enumeration
 * @sd: ISPIF V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fse: pointer to v4l2_subdev_frame_size_enum structure
 * return -EINVAL or zero on success
 */
static int ispif_enum_frame_size(struct v4l2_subdev *sd,
				 struct v4l2_subdev_fh *fh,
				 struct v4l2_subdev_frame_size_enum *fse)
{
	struct ispif_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt format;

	if (fse->index != 0)
		return -EINVAL;
	pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
	format.code = fse->code;
	format.width = 1;
	format.height = 1;
	ispif_try_format(line, fh, fse->pad, &format, fse->which);
	fse->min_width = format.width;
	fse->min_height = format.height;

	if (format.code != fse->code)
		return -EINVAL;

	format.code = fse->code;
	format.width = -1;
	format.height = -1;
	ispif_try_format(line, fh, fse->pad, &format, fse->which);
	fse->max_width = format.width;
	fse->max_height = format.height;

	return 0;
}

/*
 * ispif_get_format - Handle get format by pads subdev method
 * @sd: ISPIF V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int ispif_get_format(struct v4l2_subdev *sd,
			    struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_format *fmt)
{
	struct ispif_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;
	pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
	format = __ispif_get_format(line, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;

	return 0;
}

/*
 * ispif_set_format - Handle set format by pads subdev method
 * @sd: ISPIF V4L2 subdevice
 * @cfg: V4L2 subdev pad configuration
 * @fmt: pointer to v4l2 subdev format structure
 *
 * Return -EINVAL or zero on success
 */
static int ispif_set_format(struct v4l2_subdev *sd,
			    struct v4l2_subdev_fh *fh,
			    struct v4l2_subdev_format *fmt)
{
	struct ispif_line *line = v4l2_get_subdevdata(sd);
	struct v4l2_mbus_framefmt *format;
	pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
	format = __ispif_get_format(line, fh, fmt->pad, fmt->which);
	if (format == NULL)
		return -EINVAL;

	ispif_try_format(line, fh, fmt->pad, &fmt->format, fmt->which);
	*format = fmt->format;

	/* Propagate the format from sink to source */
	if (fmt->pad == MSM_ISPIF_PAD_SINK) {
		format = __ispif_get_format(line, fh, MSM_ISPIF_PAD_SRC,
					    fmt->which);

		*format = fmt->format;
		ispif_try_format(line, fh, MSM_ISPIF_PAD_SRC, format,
				 fmt->which);
	}

	return 0;
}

/*
 * ispif_init_formats - Initialize formats on all pads
 * @sd: ISPIF V4L2 subdevice
 * @fh: V4L2 subdev file handle
 *
 * Initialize all pad formats with default values.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int ispif_init_formats(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct v4l2_subdev_format format;
	pr_err("[camera] : %s, line = %d\n", __func__, __LINE__);
	memset(&format, 0, sizeof(format));
	format.pad = MSM_ISPIF_PAD_SINK;
	format.which = fh ? V4L2_SUBDEV_FORMAT_TRY : V4L2_SUBDEV_FORMAT_ACTIVE;
	format.format.code = MEDIA_BUS_FMT_UYVY8_2X8;
	format.format.width = 1920;
	format.format.height = 1080;

	return ispif_set_format(sd, fh ? fh : NULL, &format);
}

/*
 * msm_ispif_subdev_init - Initialize ISPIF device structure and resources
 * @ispif: ISPIF device
 * @res: ISPIF module resources table
 *
 * Return 0 on success or a negative error code otherwise
 */
int msm_ispif_subdev_init(struct ispif_device *ispif,
			  struct resources_ispif *res)
{
	struct device *dev = to_device(ispif);
	struct platform_device *pdev = container_of(dev, struct platform_device,
						    dev);
	struct resource *r;
	int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(ispif->line); i++)
		ispif->line[i].id = i;

	mutex_init(&ispif->power_lock);
	ispif->power_count = 0;

	mutex_init(&ispif->config_lock);

	/* Memory */

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, res->reg[0]);
	ispif->base = devm_ioremap_resource(dev, r);
	if (IS_ERR(ispif->base)) {
		dev_err(dev, "could not map memory\n");
		return PTR_ERR(ispif->base);
	}

	r = platform_get_resource_byname(pdev, IORESOURCE_MEM, res->reg[1]);
	ispif->base_clk_mux = devm_ioremap_resource(dev, r);
	if (IS_ERR(ispif->base_clk_mux)) {
		dev_err(dev, "could not map memory\n");
		return PTR_ERR(ispif->base_clk_mux);
	}

	/* Interrupt */

	r = platform_get_resource_byname(pdev, IORESOURCE_IRQ, res->interrupt);

	if (!r) {
		dev_err(dev, "missing IRQ\n");
		return -EINVAL;
	}

	ispif->irq = r->start;
	snprintf(ispif->irq_name, sizeof(ispif->irq_name), "%s_%s",
		 dev_name(dev), MSM_ISPIF_NAME);
	ret = devm_request_irq(dev, ispif->irq, ispif_isr,
			       IRQF_TRIGGER_RISING, ispif->irq_name, ispif);
	if (ret < 0) {
		dev_err(dev, "request_irq failed\n");
		return ret;
	}

	/*regulators/gdscs */
	ispif->vfe0 = devm_regulator_get(dev, res->regulator[0]);
	if (IS_ERR(ispif->vfe0)) {
		dev_err(dev, "could not get regulator\n");
		return PTR_ERR(ispif->vfe0);
	}
	ispif->vfe1 = devm_regulator_get(dev, res->regulator[1]);
	if (IS_ERR(ispif->vfe1)) {
		dev_err(dev, "could not get regulator\n");
		return PTR_ERR(ispif->vfe1);
	}
	ispif->gdscr = devm_regulator_get(dev, res->regulator[2]);
	if (IS_ERR(ispif->gdscr)) {
		dev_err(dev, "could not get regulator\n");
		return PTR_ERR(ispif->gdscr);
	}
	ispif->mmagic = devm_regulator_get(dev, res->regulator[3]);
	if (IS_ERR(ispif->mmagic)) {
		dev_err(dev, "could not get regulator\n");
		return PTR_ERR(ispif->mmagic);
	}

	/* Clocks */

	ispif->nclocks = 0;
	while (res->clock[ispif->nclocks])
		ispif->nclocks++;

	ispif->clock = devm_kzalloc(dev, ispif->nclocks * sizeof(*ispif->clock),
				    GFP_KERNEL);
	if (!ispif->clock)
		return -ENOMEM;

	for (i = 0; i < ispif->nclocks; i++) {
		long clk_rate;
		ispif->clock[i] = devm_clk_get(dev, res->clock[i]);
		if (IS_ERR(ispif->clock[i]))
			return PTR_ERR(ispif->clock[i]);

		if (res->clock_rate[i] && res->clock_init[i] == SET_RATE) {
			clk_rate = clk_round_rate(ispif->clock[i],
						       res->clock_rate[i]);
			if (clk_rate < 0) {
				dev_err(dev, "clk round rate failed\n");
				return -EINVAL;
			}
			ret = clk_set_rate(ispif->clock[i], clk_rate);
			if (ret < 0) {
				dev_err(dev, "clk set rate failed\n");
				return ret;
			}
		} else if (res->clock_init[i] == INIT_RATE) {
			clk_rate = clk_get_rate(ispif->clock[i]);
			if (clk_rate == 0) {
				clk_rate =
					  clk_round_rate(ispif->clock[i], 0);
				if (clk_rate < 0) {
					dev_err(dev, "clk round rate failed\n");
					return -EINVAL;

				}
				ret = clk_set_rate(ispif->clock[i],
							clk_rate);
				if (ret < 0) {
					dev_err(dev, "clk set rate failed\n");
					return ret;
				}
			}
		}
	}

	ispif->nclocks_for_reset = 0;
	while (res->clock_for_reset[ispif->nclocks_for_reset])
		ispif->nclocks_for_reset++;

	ispif->clock_for_reset = devm_kzalloc(dev, ispif->nclocks_for_reset *
			sizeof(*ispif->clock_for_reset), GFP_KERNEL);
	if (!ispif->clock_for_reset)
		return -ENOMEM;

	for (i = 0; i < ispif->nclocks_for_reset; i++) {
		ispif->clock_for_reset[i] = devm_clk_get(dev,
						res->clock_for_reset[i]);
		if (IS_ERR(ispif->clock_for_reset[i]))
			return PTR_ERR(ispif->clock_for_reset[i]);
	}

	init_completion(&ispif->reset_complete);

	return 0;
}

/*
 * ispif_get_intf - Get ISPIF interface to use by VFE line id
 * @line_id: VFE line id that the ISPIF line is connected to
 *
 * Return ISPIF interface to use
 */
static enum ispif_intf ispif_get_intf(enum vfe_line_id line_id)
{
	switch (line_id) {
	case (VFE_LINE_RDI0):
		return RDI0;
	case (VFE_LINE_RDI1):
		return RDI1;
	case (VFE_LINE_RDI2):
		return RDI2;
	case (VFE_LINE_PIX):
		return PIX0;
	default:
		return RDI0;
	}
}

/*
 * ispif_link_setup - Setup ISPIF connections
 * @entity: Pointer to media entity structure
 * @local: Pointer to local pad
 * @remote: Pointer to remote pad
 * @flags: Link flags
 *
 * Return 0 on success
 */
static int ispif_link_setup(struct media_entity *entity,
			    const struct media_pad *local,
			    const struct media_pad *remote, u32 flags)
{
	pr_err("[camera] : %s, local entity = %s, remote entity = %s\n", __func__, local->entity->name, remote->entity->name);
	if (flags & MEDIA_LNK_FL_ENABLED) {
		if (media_entity_remote_pad((struct media_pad *)local))
			return -EBUSY;

		if (local->flags & MEDIA_PAD_FL_SINK) {
			struct v4l2_subdev *sd;
			struct ispif_line *line;

			sd = container_of(entity, struct v4l2_subdev, entity);
			line = v4l2_get_subdevdata(sd);

			msm_csid_get_csid_id(remote->entity, &line->csid_id);
		} else { /* MEDIA_PAD_FL_SOURCE */
			struct v4l2_subdev *sd;
			struct ispif_line *line;
			enum vfe_line_id id;

			sd = container_of(entity, struct v4l2_subdev, entity);
			line = v4l2_get_subdevdata(sd);

			msm_vfe_get_vfe_id(remote->entity, &line->vfe_id);
			msm_vfe_get_vfe_line_id(remote->entity, &id);
			line->interface = ispif_get_intf(id);
		}
	}

	return 0;
}

static const struct v4l2_subdev_core_ops ispif_core_ops = {
	.s_power = ispif_set_power,
};

static const struct v4l2_subdev_video_ops ispif_video_ops = {
	.s_stream = ispif_set_stream,
};

static const struct v4l2_subdev_pad_ops ispif_pad_ops = {
	.enum_mbus_code = ispif_enum_mbus_code,
	.enum_frame_size = ispif_enum_frame_size,
	.get_fmt = ispif_get_format,
	.set_fmt = ispif_set_format,
};

static const struct v4l2_subdev_ops ispif_v4l2_ops = {
	.core = &ispif_core_ops,
	.video = &ispif_video_ops,
	.pad = &ispif_pad_ops,
};

static const struct v4l2_subdev_internal_ops ispif_v4l2_internal_ops = {
	.open = ispif_init_formats,
};

static const struct media_entity_operations ispif_media_ops = {
	.link_setup = ispif_link_setup,
	.link_validate = v4l2_subdev_link_validate,
};

/*
 * msm_ispif_register_entities - Register subdev node for ISPIF module
 * @ispif: ISPIF device
 * @v4l2_dev: V4L2 device
 *
 * Return 0 on success or a negative error code otherwise
 */
int msm_ispif_register_entities(struct ispif_device *ispif,
				struct v4l2_device *v4l2_dev)
{
	struct device *dev = to_device(ispif);
	int ret;
	int i;

	for (i = 0; i < ARRAY_SIZE(ispif->line); i++) {
		struct v4l2_subdev *sd = &ispif->line[i].subdev;
		struct media_pad *pads = ispif->line[i].pads;

		v4l2_subdev_init(sd, &ispif_v4l2_ops);
		sd->internal_ops = &ispif_v4l2_internal_ops;
		sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
		snprintf(sd->name, ARRAY_SIZE(sd->name), "%s%d",
			 MSM_ISPIF_NAME, i);
		v4l2_set_subdevdata(sd, &ispif->line[i]);

		ret = ispif_init_formats(sd, NULL);
		if (ret < 0) {
			dev_err(dev, "Failed to init format\n");
			goto error;
		}

		pads[MSM_ISPIF_PAD_SINK].flags = MEDIA_PAD_FL_SINK;
		pads[MSM_ISPIF_PAD_SRC].flags = MEDIA_PAD_FL_SOURCE;

		sd->entity.ops = &ispif_media_ops;
		ret = media_entity_init(&sd->entity, MSM_ISPIF_PADS_NUM,
					     pads, 0);
		if (ret < 0) {
			dev_err(dev, "Failed to init media entity\n");
			goto error;
		}

		ret = v4l2_device_register_subdev(v4l2_dev, sd);
		if (ret < 0) {
			dev_err(dev, "Failed to register subdev\n");
			media_entity_cleanup(&sd->entity);
			goto error;
		}
	}

	return 0;

error:
	for (i--; i >= 0; i--) {
		struct v4l2_subdev *sd = &ispif->line[i].subdev;

		v4l2_device_unregister_subdev(sd);
		media_entity_cleanup(&sd->entity);
	}

	return ret;
}

/*
 * msm_ispif_unregister_entities - Unregister ISPIF module subdev node
 * @ispif: ISPIF device
 */
void msm_ispif_unregister_entities(struct ispif_device *ispif)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ispif->line); i++) {
		struct v4l2_subdev *sd = &ispif->line[i].subdev;

		v4l2_device_unregister_subdev(sd);
		media_entity_cleanup(&sd->entity);
	}
}
