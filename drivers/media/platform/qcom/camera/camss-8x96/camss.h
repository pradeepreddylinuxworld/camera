/*
 * camss.h
 *
 * Qualcomm MSM Camera Subsystem - Core
 *
 * Copyright (c) 2015, The Linux Foundation. All rights reserved.
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
#ifndef QC_MSM_CAMSS_H
#define QC_MSM_CAMSS_H

#include <linux/types.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <linux/device.h>

#include "csid.h"
#include "csiphy.h"
#include "ispif.h"
#include "vfe.h"

#define CAMSS_CSID_NUM 4
#define CAMSS_CSIPHY_NUM 3

#define NO_SET_RATE -1
#define INIT_RATE -2
#define SET_RATE 1


#define to_camss(ptr_module)	\
	container_of(ptr_module, struct camss, ptr_module)

#define to_device(ptr_module)	\
	(to_camss(ptr_module)->dev)

#define module_pointer(ptr_module, index)	\
	((const struct ptr_module##_device (*)[]) &(ptr_module[-(index)]))

#define to_camss_index(ptr_module, index)	\
	container_of(module_pointer(ptr_module, index),	\
		     struct camss, ptr_module)

#define to_device_index(ptr_module, index)	\
	(to_camss_index(ptr_module, index)->dev)

#define CAMSS_RES_MAX 30

struct resources {
	char *regulator[CAMSS_RES_MAX];
	char *clock[CAMSS_RES_MAX];
	s32 clock_rate[CAMSS_RES_MAX];
	char *reg[CAMSS_RES_MAX];
	char *interrupt[CAMSS_RES_MAX];
};

struct resources_ispif {
	char *regulator[CAMSS_RES_MAX];
	char *clock[CAMSS_RES_MAX];
	s32 clock_rate[CAMSS_RES_MAX];
	int clock_init[CAMSS_RES_MAX];
	char *clock_for_reset[CAMSS_RES_MAX];
	char *reg[CAMSS_RES_MAX];
	char *interrupt;
};

struct camss {
	struct v4l2_device v4l2_dev;
	struct v4l2_async_notifier notifier;
	struct media_device media_dev;
	struct device *dev;
	struct csiphy_device csiphy[CAMSS_CSIPHY_NUM];
	struct csid_device csid[CAMSS_CSID_NUM];
	struct ispif_device ispif;
	struct vfe_device vfe;
		/*IOMMU driver*/
	int iommu_hdl;
    struct device *iommu_dev;
	struct dma_iommu_mapping *mapping;
	atomic_t ref_count;
};

struct camss_camera_interface {
	u8 csiphy_id;
	struct csiphy_csi2_cfg csi2;
};

struct camss_async_subdev {
	struct camss_camera_interface interface;
	struct v4l2_async_subdev asd;
};

int camss_enable_clocks(int nclocks, struct clk **clock, struct device *dev);
void camss_disable_clocks(int nclocks, struct clk **clock);
void camss_delete(struct camss *camss);
int msm_camss_pipeline_pm_use(struct media_entity *entity, int use);


#endif /* QC_MSM_CAMSS_H */
