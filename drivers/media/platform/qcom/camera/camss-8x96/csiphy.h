/*
 * csiphy.h
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
#ifndef QC_MSM_CAMSS_CSIPHY_H
#define QC_MSM_CAMSS_CSIPHY_H

#include <linux/clk.h>
#include <media/media-entity.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#define MSM_CSIPHY_PAD_SINK 0
#define MSM_CSIPHY_PAD_SRC 1
#define MSM_CSIPHY_PADS_NUM 2

struct csiphy_lane {
	u8 pos;
	u8 pol;
};

struct csiphy_lanes_cfg {
	int num_data;
	struct csiphy_lane *data;
	struct csiphy_lane clk;
};

struct csiphy_csi2_cfg {
	int settle_cnt;
	struct csiphy_lanes_cfg lane_cfg;
};

struct csiphy_config {
	u8 combo_mode;
	u8 csid_id;
	struct csiphy_csi2_cfg *csi2;
};

struct msm_camera_csiphy_params {
	uint8_t lane_cnt;
	uint8_t settle_cnt;
	uint16_t lane_mask;
	uint8_t combo_mode;
	uint8_t csid_core;
};

struct csiphy_reg_t {
	uint32_t addr;
	uint32_t data;
};

struct csiphy_reg_parms_t {
/*MIPI CSI PHY registers*/
	uint32_t mipi_csiphy_lnn_cfg1_addr;
	uint32_t mipi_csiphy_lnn_cfg2_addr;
	uint32_t mipi_csiphy_lnn_cfg3_addr;
	uint32_t mipi_csiphy_lnn_cfg4_addr;
	uint32_t mipi_csiphy_lnn_cfg5_addr;
	uint32_t mipi_csiphy_lnck_cfg1_addr;
	uint32_t mipi_csiphy_lnck_cfg2_addr;
	uint32_t mipi_csiphy_lnck_cfg3_addr;
	uint32_t mipi_csiphy_lnck_cfg4_addr;
	uint32_t mipi_csiphy_lnn_test_imp;
	uint32_t mipi_csiphy_lnn_misc1_addr;
	uint32_t mipi_csiphy_glbl_reset_addr;
	uint32_t mipi_csiphy_glbl_pwr_cfg_addr;
	uint32_t mipi_csiphy_glbl_irq_cmd_addr;
	uint32_t mipi_csiphy_hw_version_addr;
	uint32_t mipi_csiphy_interrupt_status0_addr;
	uint32_t mipi_csiphy_interrupt_mask0_addr;
	uint32_t mipi_csiphy_interrupt_mask_val;
	uint32_t mipi_csiphy_interrupt_mask_addr;
	uint32_t mipi_csiphy_interrupt_clear0_addr;
	uint32_t mipi_csiphy_interrupt_clear_addr;
	uint32_t mipi_csiphy_mode_config_shift;
	uint32_t mipi_csiphy_glbl_t_init_cfg0_addr;
	uint32_t mipi_csiphy_t_wakeup_cfg0_addr;
	uint32_t csiphy_version;
	uint32_t combo_clk_mask;
};

struct csiphy_reg_3ph_parms_t {
/*MIPI CSI PHY registers*/
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl5;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl6;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl34;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl35;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl36;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl1;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl2;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl3;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl5;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl6;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl7;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl8;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl9;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl10;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl11;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl12;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl13;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl14;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl15;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl16;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl17;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl18;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl19;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl21;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl23;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl24;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl25;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl26;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl27;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl28;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl29;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl30;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl31;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl32;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl33;
	struct csiphy_reg_t mipi_csiphy_3ph_lnn_ctrl51;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl7;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl11;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl12;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl13;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl14;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl15;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl16;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl17;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl18;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl19;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl20;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl21;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_misc1;
	struct csiphy_reg_t mipi_csiphy_3ph_cmn_ctrl0;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg1;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg2;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg3;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg4;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg5;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg6;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg7;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg8;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_cfg9;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_ctrl15;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_test_imp;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_test_force;
	struct csiphy_reg_t mipi_csiphy_2ph_lnn_ctrl5;
	struct csiphy_reg_t mipi_csiphy_3ph_lnck_cfg1;
};

struct csiphy_ctrl_t {
	struct csiphy_reg_parms_t csiphy_reg;
	struct csiphy_reg_3ph_parms_t csiphy_3ph_reg;
};

struct csiphy_device {
	u8 id;
	struct v4l2_subdev subdev;
	struct media_pad pads[MSM_CSIPHY_PADS_NUM];
	void __iomem *base;
	void __iomem *base_clk_mux;
	u32 irq;
	char irq_name[30];
	struct csiphy_ctrl_t *ctrl_reg;
	uint8_t num_irq_registers;
	struct clk **clock;
	int nclocks;
	struct csiphy_config cfg;
	struct v4l2_mbus_framefmt fmt[MSM_CSIPHY_PADS_NUM];
};

struct resources;

int msm_csiphy_subdev_init(struct csiphy_device *csiphy,
			   struct resources *res, u8 id);

int msm_csiphy_register_entity(struct csiphy_device *csiphy,
			       struct v4l2_device *v4l2_dev);

void msm_csiphy_unregister_entity(struct csiphy_device *csiphy);

#endif /* QC_MSM_CAMSS_CSIPHY_H */
