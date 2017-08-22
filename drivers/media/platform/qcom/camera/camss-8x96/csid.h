/*
 * csid.h
 *
 * Qualcomm MSM Camera Subsystem - CSID Module
 *
 * Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
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
#ifndef QC_MSM_CAMSS_CSID_H
#define QC_MSM_CAMSS_CSID_H

#include <linux/clk.h>
#include <media/media-entity.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mediabus.h>
#include <media/v4l2-subdev.h>

#define MSM_CSID_PAD_SINK 0
#define MSM_CSID_PAD_SRC 1
#define MSM_CSID_PADS_NUM 2
#define MAX_CID                 16

enum csid_payload_mode {
	CSID_PAYLOAD_MODE_INCREMENTING = 0,
	CSID_PAYLOAD_MODE_ALTERNATING_55_AA = 1,
	CSID_PAYLOAD_MODE_ALL_ZEROES = 2,
	CSID_PAYLOAD_MODE_ALL_ONES = 3,
	CSID_PAYLOAD_MODE_RANDOM = 4,
	CSID_PAYLOAD_MODE_USER_SPECIFIED = 5,
};

struct csid_testgen_config {
	u8 enabled;
	enum csid_payload_mode payload_mode;
};

struct csid_phy_config {
	u8 csiphy_id;
	u8 lane_cnt;
	u32 lane_assign;
};

struct csid_vc_cfg {
	uint8_t cid;
	uint8_t dt;
	uint8_t decode_format;
};

struct csid_lut_params {
	uint8_t num_cid;
	struct csid_vc_cfg *vc_cfg[MAX_CID];
};

struct csid_params {
	uint8_t lane_cnt;
	uint16_t lane_assign;
	uint8_t phy_sel;
	struct csid_lut_params lut_params;
};

enum csiphy_lane_assign {
	PHY_LANE_D0,
	PHY_LANE_CLK,
	PHY_LANE_D1,
	PHY_LANE_D2,
	PHY_LANE_D3,
	PHY_LANE_MAX,
};


struct csid_reg_parms_t {
/* MIPI	CSID registers */
	uint32_t csid_hw_version_addr;
	uint32_t csid_core_ctrl_0_addr;
	uint32_t csid_core_ctrl_1_addr;
	uint32_t csid_rst_cmd_addr;
	uint32_t csid_cid_lut_vc_0_addr;
	uint32_t csid_cid_lut_vc_1_addr;
	uint32_t csid_cid_lut_vc_2_addr;
	uint32_t csid_cid_lut_vc_3_addr;
	uint32_t csid_cid_n_cfg_addr;
	uint32_t csid_irq_clear_cmd_addr;
	uint32_t csid_irq_mask_addr;
	uint32_t csid_irq_status_addr;
	uint32_t csid_captured_unmapped_long_pkt_hdr_addr;
	uint32_t csid_captured_mmaped_long_pkt_hdr_addr;
	uint32_t csid_captured_short_pkt_addr;
	uint32_t csid_captured_long_pkt_hdr_addr;
	uint32_t csid_captured_long_pkt_ftr_addr;
	uint32_t csid_pif_misr_dl0_addr;
	uint32_t csid_pif_misr_dl1_addr;
	uint32_t csid_pif_misr_dl2_addr;
	uint32_t csid_pif_misr_dl3_addr;
	uint32_t csid_stats_total_pkts_rcvd_addr;
	uint32_t csid_stats_ecc_addr;
	uint32_t csid_stats_crc_addr;
	uint32_t csid_tg_ctrl_addr;
	uint32_t csid_tg_vc_cfg_addr;
	uint32_t csid_tg_dt_n_cfg_0_addr;
	uint32_t csid_tg_dt_n_cfg_1_addr;
	uint32_t csid_tg_dt_n_cfg_2_addr;
	uint32_t csid_rst_done_irq_bitshift;
	uint32_t csid_rst_stb_all;
	uint32_t csid_dl_input_sel_shift;
	uint32_t csid_phy_sel_shift;
	uint32_t csid_version;
	uint32_t csid_3p_ctrl_0_addr;
	uint32_t csid_3p_pkt_hdr_addr;
	uint32_t csid_test_bus_ctrl;
	uint32_t csid_irq_mask_val;
	uint32_t csid_err_lane_overflow_offset_2p;
	uint32_t csid_err_lane_overflow_offset_3p;
	uint32_t csid_phy_sel_shift_3p;
};

struct csid_ctrl_t {
	struct csid_reg_parms_t csid_reg;
	uint8_t *csid_lane_assign;
};


struct csid_device {
	u8 id;
	struct v4l2_subdev subdev;
	struct media_pad pads[MSM_CSID_PADS_NUM];
	void __iomem *base;
	struct csid_ctrl_t *ctrl_reg;
	u32 irq;
	char irq_name[30];
	struct clk **clock;
	int nclocks;
	struct regulator *vdda;
	struct regulator *gdscr;
	struct regulator *mmagic;
	struct completion reset_complete;
	struct csid_testgen_config testgen;
	struct csid_phy_config phy;
	struct v4l2_mbus_framefmt fmt[MSM_CSID_PADS_NUM];
	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *testgen_mode;
};

struct resources;

int msm_csid_subdev_init(struct csid_device *csid,
			 struct resources *res, u8 id);

int msm_csid_register_entity(struct csid_device *csid,
			     struct v4l2_device *v4l2_dev);

void msm_csid_unregister_entity(struct csid_device *csid);

void msm_csid_get_csid_id(struct media_entity *entity, u8 *id);

#endif /* QC_MSM_CAMSS_CSID_H */
