/*
 * camss.c
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
#include <linux/clk.h>
#include <linux/media-bus-format.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <asm/dma-iommu.h>
#include <linux/qcom_iommu.h>
#include <linux/dma-mapping.h>
#include <linux/msm_dma_iommu_mapping.h>
#include <linux/iommu.h>

#include <media/media-device.h>
#include <media/v4l2-async.h>
#include <media/v4l2-device.h>
//#include <media/v4l2-mc.h>
#include <media/v4l2-of.h>

#include "camss.h"
#include <../drivers/media/platform/qcom/camera/common/cam_hw_ops.h>
#include <../drivers/media/platform/qcom/camera/common/cam_smmu_api.h>

#if 0
static struct resources csiphy_res[] = {
	/* CSIPHY0 */
	{
		.regulator = { NULL },
		.clock = { "camss_top_ahb_clk", "ispif_ahb_clk",
			   "camss_ahb_clk", "csiphy0_timer_clk" },
		.clock_rate = { 0, 0, 0, 200000000 },
		.reg = { "csiphy0", "csiphy0_clk_mux" },
		.interrupt = { "csiphy0" }
	},

	/* CSIPHY1 */
	{
		.regulator = { NULL },
		.clock = { "camss_top_ahb_clk", "ispif_ahb_clk",
			   "camss_ahb_clk", "csiphy1_timer_clk" },
		.clock_rate = { 0, 0, 0, 200000000 },
		.reg = { "csiphy1", "csiphy1_clk_mux" },
		.interrupt = { "csiphy1" }
	}
};

static struct resources csid_res[] = {
	/* CSID0 */
	{
		.regulator = { "vdda" },
		.clock = { "camss_top_ahb_clk", "ispif_ahb_clk",
			   "csi0_ahb_clk", "camss_ahb_clk",
			   "csi0_clk", "csi0_phy_clk",
			   "csi0_pix_clk", "csi0_rdi_clk" },
		.clock_rate = { 0, 0, 0, 0, 200000000, 0, 0, 0 },
		.reg = { "csid0" },
		.interrupt = { "csid0" }
	},

	/* CSID1 */
	{
		.regulator = { "vdda" },
		.clock = { "camss_top_ahb_clk", "ispif_ahb_clk",
			   "csi1_ahb_clk", "camss_ahb_clk",
			   "csi1_clk", "csi1_phy_clk",
			   "csi1_pix_clk", "csi1_rdi_clk" },
		.clock_rate = { 0, 0, 0, 0, 200000000, 0, 0, 0 },
		.reg = { "csid1" },
		.interrupt = { "csid1" }
	},
};

static struct resources_ispif ispif_res = {
	/* ISPIF */
	.clock = { "camss_top_ahb_clk", "camss_ahb_clk", "ispif_ahb_clk",
		   "csi0_clk", "csi0_pix_clk", "csi0_rdi_clk",
		   "csi1_clk", "csi1_pix_clk", "csi1_rdi_clk" },
	.clock_for_reset = { "camss_vfe_vfe_clk", "camss_csi_vfe_clk" },
	.reg = { "ispif", "csi_clk_mux" },
	.interrupt = "ispif"

};

static struct resources vfe_res = {
	/* VFE0 */
	.regulator = { NULL },
	.clock = { "camss_top_ahb_clk", "camss_vfe_vfe_clk",
		   "camss_csi_vfe_clk", "iface_clk",
		   "bus_clk", "camss_ahb_clk" },
	.clock_rate = { 0, 320000000, 0, 0, 0, 0, 0, 0 },
	.reg = { "vfe0" },
	.interrupt = { "vfe0" }
};
#else
static struct resources csiphy_res[] = {
	/* CSIPHY0 */
	{
		.regulator = { NULL },
		.clock = { "camss_top_ahb_clk", "ispif_ahb_clk",
			   "csiphy0_timer_src_clk", "csiphy0_timer_clk",
			   "camss_ahb_clk", "csiphy0_3p_clk_src",
			   "csi_phy0_3p_clk"},
		.clock_rate = { 0, 0,
					200000000, 0,
					0, 100000000,
					0},
		.reg = { "csiphy0", "csiphy0_clk_mux" },
		.interrupt = { "csiphy0" }
	},

	/* CSIPHY1 */
	{
		.regulator = { NULL },
		.clock = { "camss_top_ahb_clk", "ispif_ahb_clk",
			   "csiphy1_timer_src_clk", "csiphy1_timer_clk",
			   "camss_ahb_clk", "csiphy1_3p_clk_src",
			   "csi_phy1_3p_clk"},
		.clock_rate = { 0, 0,
					200000000, 0,
					0, 100000000,
					0},
		.reg = { "csiphy1", "csiphy1_clk_mux" },
		.interrupt = { "csiphy1" }
	},

		/* CSIPHY2 */
	{
		.regulator = { NULL },
		.clock = { "camss_top_ahb_clk", "ispif_ahb_clk",
			   "csiphy2_timer_src_clk", "csiphy2_timer_clk",
			   "camss_ahb_clk", "csiphy2_3p_clk_src",
			   "csi_phy2_3p_clk"},
		.clock_rate = { 0, 0,
					200000000, 0,
					0, 100000000,
					0},
		.reg = { "csiphy2", "csiphy2_clk_mux" },
		.interrupt = { "csiphy2" }
	}

};

static struct resources csid_res[] = {
	/* CSID0 */
	{
		.regulator = { "mipi-csi-vdd", "mmagic-vdd", "gdscr" },
		.clock = { "mmagic_camss_ahb_clk", "camss_top_ahb_clk",
			   "ispif_ahb_clk", "csi0_src_clk",
			   "csi0_clk", "csi0_phy_clk",
			   "csi0_ahb_clk", "csi0_rdi_clk",
			   "csi0_pix_clk", "camss_ahb_clk"},
		.clock_rate = { 0, 0,
					0, 200000000,
					0, 0,
					0, 0,
					0, 0},
		.reg = { "csid0" },
		.interrupt = { "csid0" }
	},

		/* CSID1 */
	{
		.regulator = { "mipi-csi-vdd", "mmagic-vdd", "gdscr" },
		.clock = { "mmagic_camss_ahb_clk", "camss_top_ahb_clk",
			   "ispif_ahb_clk", "csi1_src_clk",
			   "csi1_clk", "csi1_phy_clk",
			   "csi1_ahb_clk", "csi1_rdi_clk",
			   "csi1_pix_clk", "camss_ahb_clk"},
		.clock_rate = { 0, 0,
					0, 200000000,
					0, 0,
					0, 0,
					0, 0},
		.reg = { "csid1" },
		.interrupt = { "csid1" }
	},

		/* CSID2 */
	{
		.regulator = { "mipi-csi-vdd", "mmagic-vdd", "gdscr" },
		.clock = { "mmagic_camss_ahb_clk", "camss_top_ahb_clk",
			   "ispif_ahb_clk", "csi2_src_clk",
			   "csi2_clk", "csi2_phy_clk",
			   "csi2_ahb_clk", "csi2_rdi_clk",
			   "csi2_pix_clk", "camss_ahb_clk"},
		.clock_rate = { 0, 0,
					0, 200000000,
					0, 0,
					0, 0,
					0, 0},
		.reg = { "csid2" },
		.interrupt = { "csid2" }
	},

		/* CSID3 */
	{
		.regulator = { "mipi-csi-vdd", "mmagic-vdd", "gdscr" },
		.clock = { "mmagic_camss_ahb_clk", "camss_top_ahb_clk",
			   "ispif_ahb_clk", "csi3_src_clk",
			   "csi3_clk", "csi3_phy_clk",
			   "csi3_ahb_clk", "csi3_rdi_clk",
			   "csi3_pix_clk", "camss_ahb_clk"},
		.clock_rate = { 0, 0,
					0, 200000000,
					0, 0,
					0, 0,
					0, 0},
		.reg = { "csid3" },
		.interrupt = { "csid3" }
	},
};

static struct resources_ispif ispif_res = {
	/* ISPIF */
	.regulator = { "vfe0-vdd", "vfe1-vdd", "gdscr", "mmagic-vdd"  },
	.clock = { "mmagic_camss_ahb_clk", "camss_top_ahb_clk", "camss_ahb_clk", "ispif_ahb_clk",
			"csi0_src_clk", "csi0_clk", "csi0_rdi_clk", "csi0_pix_clk",
			"csi1_src_clk", "csi1_clk", "csi1_rdi_clk", "csi1_pix_clk",
			"csi2_src_clk", "csi2_clk", "csi2_rdi_clk", "csi2_pix_clk",
			"csi3_src_clk", "csi3_clk", "csi3_rdi_clk", "csi3_pix_clk",
			"vfe0_clk_src", "camss_vfe0_clk", "camss_csi_vfe0_clk"},
	//		"vfe1_clk_src", "camss_vfe1_clk", "camss_csi_vfe1_clk"},
	.clock_rate = { 0, 0, 0, 0,
				200000000, 0, 0, 0,
				200000000, 0, 0, 0,
				200000000, 0, 0, 0,
				200000000, 0, 0, 0,
				0, 0, 0},
	//			0, 0, 0},
	.clock_init = { NO_SET_RATE, NO_SET_RATE, NO_SET_RATE, NO_SET_RATE,
				SET_RATE, NO_SET_RATE, NO_SET_RATE, NO_SET_RATE,
				SET_RATE, NO_SET_RATE, NO_SET_RATE, NO_SET_RATE,
				SET_RATE, NO_SET_RATE, NO_SET_RATE, NO_SET_RATE,
				SET_RATE, NO_SET_RATE, NO_SET_RATE, NO_SET_RATE,
				INIT_RATE, NO_SET_RATE, NO_SET_RATE},
	//			INIT_RATE, NO_SET_RATE, NO_SET_RATE},
	.clock_for_reset = { "camss_vfe0_clk", "camss_csi_vfe0_clk" },
	.reg = { "ispif", "csi_clk_mux" },
	.interrupt = "ispif"
};

static struct resources vfe_res = {
	/* VFE0 */
	.regulator = { "vfe0-vdd", "gdscr", "mmagic-vdd"  },
	.clock = { "mmagic_camss_ahb_clk", "camss_axi_clk","camss_top_ahb_clk",
		   "camss_ahb_clk", "vfe0_clk_src", "camss_vfe0_clk",
		   "camss_csi_vfe0_clk", "vfe_vbif_ahb_clk", "vfe0_ahb_clk",
		   "bus_clk", "vfe0_stream_clk", "smmu_vfe_axi_clk"},
	.clock_rate = { 0, 0, 0,
				0, 320000000, 0,
				0, 0, 0,
				0, 0, 0 },
	.reg = { "vfe0", "vfe_vbif" },
	.interrupt = { "vfe0" }
};

#endif

#if 0
static int camss_smmu_setup(struct camss *camss)
{
	int rc = 0;
	int disable_htw = 1;

	/* create a virtual mapping */
	camss->mapping = arm_iommu_create_mapping(msm_iommu_get_bus(camss->dev),
		SZ_128K, SZ_2G - SZ_128K);
	if (IS_ERR(camss->mapping)) {
		pr_err("%s: create mapping Failed\n", __func__);
		rc = -ENODEV;
		goto end;
	}

	/*
	 * Set the domain attributes
	 * disable L2 redirect since it decreases
	 * performance
	 */
	if (iommu_domain_set_attr(camss->mapping->domain,
		DOMAIN_ATTR_COHERENT_HTW_DISABLE,
		&disable_htw)) {
		pr_err("%s: couldn't disable coherent HTW\n", __func__);
		rc = -ENODEV;
		goto err_set_attr;
	}
	return 0;
err_set_attr:
	arm_iommu_release_mapping(camss->mapping);
end:
	return rc;
}

static int cam_smmu_attach(struct camss *camss)
{
	int rc;

	/* attach the mapping to device */
	rc = arm_iommu_attach_device(camss->dev, camss->mapping);
	if (rc < 0) {
		pr_err("%s: ARM IOMMU attach failed. ret = %d\n", __func__, rc);
		return -ENODEV;
	}
	return rc;
}
#else


#endif

/*
 * camss_enable_clocks - Enable multiple clocks
 * @nclocks: Number of clocks in clock array
 * @clock: Clock array
 * @dev: Device
 *
 * Return 0 on success or a negative error code otherwise
 */
int camss_enable_clocks(int nclocks, struct clk **clock, struct device *dev)
{
	int ret;
	int i;

	for (i = 0; i < nclocks; i++) {
		ret = clk_prepare_enable(clock[i]);
		if (ret) {
			dev_err(dev, "clock enable failed\n");
			goto error;
		}
	}

	return 0;

error:
	for (i--; i >= 0; i--)
		clk_disable_unprepare(clock[i]);

	return ret;
}

/*
 * camss_disable_clocks - Disable multiple clocks
 * @nclocks: Number of clocks in clock array
 * @clock: Clock array
 */
void camss_disable_clocks(int nclocks, struct clk **clock)
{
	int i;

	for (i = nclocks - 1; i >= 0; i--)
		clk_disable_unprepare(clock[i]);
}

/*
 * camss_of_parse_endpoint_node - Parse port endpoint node
 * @dev: Device
 * @node: Device node to be parsed
 * @csd: Parsed data from port endpoint node
 *
 * Return 0 on success or a negative error code on failure
 */
static int camss_of_parse_endpoint_node(struct device *dev,
					struct device_node *node,
					struct camss_async_subdev *csd)
{
	struct csiphy_lanes_cfg *lncfg = &csd->interface.csi2.lane_cfg;
	int *settle_cnt = &csd->interface.csi2.settle_cnt;
	struct v4l2_of_bus_mipi_csi2 *mipi_csi2;
	struct v4l2_of_endpoint vep;
	unsigned int i;

	v4l2_of_parse_endpoint(node, &vep);

	csd->interface.csiphy_id = vep.base.port;

	mipi_csi2 = &vep.bus.mipi_csi2;
	lncfg->clk.pos = mipi_csi2->clock_lane;
	lncfg->clk.pol = mipi_csi2->lane_polarities[0];
	lncfg->num_data = mipi_csi2->num_data_lanes;

	lncfg->data = devm_kzalloc(dev, lncfg->num_data * sizeof(*lncfg->data),
				   GFP_KERNEL);
	if (!lncfg->data)
		return -ENOMEM;

	for (i = 0; i < lncfg->num_data; i++) {
		lncfg->data[i].pos = mipi_csi2->data_lanes[i];
		lncfg->data[i].pol = mipi_csi2->lane_polarities[i + 1];
	}

	of_property_read_u32(node, "qcom,settle-cnt", settle_cnt);

	return 0;
}

/*
 * camss_of_parse_ports - Parse ports node
 * @dev: Device
 * @notifier: v4l2_device notifier data
 *
 * Return number of "port" nodes found in "ports" node
 */
static int camss_of_parse_ports(struct device *dev,
				struct v4l2_async_notifier *notifier)
{
	struct device_node *node = NULL;
	unsigned int size, i;
	int ret;

	while ((node = of_graph_get_next_endpoint(dev->of_node, node)))
		if (of_device_is_available(node))
			notifier->num_subdevs++;

	size = sizeof(*notifier->subdevs) * notifier->num_subdevs;
	notifier->subdevs = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!notifier->subdevs) {
		dev_err(dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	i = 0;
	while ((node = of_graph_get_next_endpoint(dev->of_node, node))) {
		struct camss_async_subdev *csd;
		pr_err("[camera] : %s, %d\n", __func__, __LINE__);
		csd = devm_kzalloc(dev, sizeof(*csd), GFP_KERNEL);
		if (!csd) {
			of_node_put(node);
			dev_err(dev, "Failed to allocate memory\n");
			return -ENOMEM;
		}

		notifier->subdevs[i++] = &csd->asd;

		ret = camss_of_parse_endpoint_node(dev, node, csd);
		if (ret < 0) {
			of_node_put(node);
			return ret;
		}

		csd->asd.match.of.node = of_graph_get_remote_port_parent(node);
		of_node_put(node);
		if (!csd->asd.match.of.node) {
			dev_err(dev, "Bad remote port parent\n");
			return -EINVAL;
		}

		csd->asd.match_type = V4L2_ASYNC_MATCH_OF;
	}
	pr_err("[camera] : %s, notifier->num_subdevs = %d\n", __func__, notifier->num_subdevs);
	return notifier->num_subdevs;
}

/*
 * camss_init_subdevices - Initialize subdev structures and resources
 * @camss: CAMSS device
 *
 * Return 0 on success or a negative error code on failure
 */
static int camss_init_subdevices(struct camss *camss)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(camss->csiphy); i++) {
		ret = msm_csiphy_subdev_init(&camss->csiphy[i],
					     &csiphy_res[i], i);
		if (ret < 0) {
			dev_err(camss->dev,
				"Failed to init csiphy%d sub-device\n", i);
			return ret;
		}
	}

	for (i = 0; i < ARRAY_SIZE(camss->csid); i++) {
		ret = msm_csid_subdev_init(&camss->csid[i],
					   &csid_res[i], i);
		if (ret < 0) {
			dev_err(camss->dev,
				"Failed to init csid%d sub-device\n", i);
			return ret;
		}
	}

	ret = msm_ispif_subdev_init(&camss->ispif, &ispif_res);
	if (ret < 0) {
		dev_err(camss->dev, "Failed to init ispif sub-device\n");
		return ret;
	}

	ret = msm_vfe_subdev_init(&camss->vfe, &vfe_res);
	if (ret < 0) {
		dev_err(camss->dev, "Fail to init vfe sub-device\n");
		return ret;
	}

	return 0;
}

/*
 * camss_register_entities - Register subdev nodes and create links
 * @camss: CAMSS device
 *
 * Return 0 on success or a negative error code on failure
 */
static int camss_register_entities(struct camss *camss)
{
	int i, j;
	int ret;

	for (i = 0; i < ARRAY_SIZE(camss->csiphy); i++) {
		ret = msm_csiphy_register_entity(&camss->csiphy[i],
						 &camss->v4l2_dev);
		if (ret < 0) {
			dev_err(camss->dev,
				"Failed to register csiphy%d entity\n", i);
			goto err_reg_csiphy;
		}
	}

	for (i = 0; i < ARRAY_SIZE(camss->csid); i++) {
		ret = msm_csid_register_entity(&camss->csid[i],
					       &camss->v4l2_dev);
		if (ret < 0) {
			dev_err(camss->dev,
				"Failed to register csid%d entity\n", i);
			goto err_reg_csid;
		}
	}

	ret = msm_ispif_register_entities(&camss->ispif, &camss->v4l2_dev);
	if (ret < 0) {
		dev_err(camss->dev, "Failed to register ispif entities\n");
		goto err_reg_ispif;
	}

	ret = msm_vfe_register_entities(&camss->vfe, &camss->v4l2_dev);
	if (ret < 0) {
		dev_err(camss->dev, "Failed to register vfe entities\n");
		goto err_reg_vfe;
	}

	for (i = 0; i < ARRAY_SIZE(camss->csiphy); i++) {
		for (j = 0; j < ARRAY_SIZE(camss->csid); j++) {
			ret = media_entity_create_link(
				&camss->csiphy[i].subdev.entity,
				MSM_CSIPHY_PAD_SRC,
				&camss->csid[j].subdev.entity,
				MSM_CSID_PAD_SINK,
				0);
			if (ret < 0) {
				dev_err(camss->dev,
					"Failed to link %s->%s entities\n",
					camss->csiphy[i].subdev.entity.name,
					camss->csid[j].subdev.entity.name);
				goto err_link;
			}
		}
	}

	for (i = 0; i < ARRAY_SIZE(camss->csid); i++) {
		for (j = 0; j < ARRAY_SIZE(camss->ispif.line); j++) {
			ret = media_entity_create_link(
				&camss->csid[i].subdev.entity,
				MSM_CSID_PAD_SRC,
				&camss->ispif.line[j].subdev.entity,
				MSM_ISPIF_PAD_SINK,
				0);
			if (ret < 0) {
				dev_err(camss->dev,
					"Failed to link %s->%s entities\n",
					camss->csid[i].subdev.entity.name,
					camss->ispif.line[j].subdev.entity.name
					);
				goto err_link;
			}
		}
	}

	for (i = 0; i < ARRAY_SIZE(camss->ispif.line); i++) {
		for (j = 0; j < ARRAY_SIZE(camss->vfe.line); j++) {
			ret = media_entity_create_link(
				&camss->ispif.line[i].subdev.entity,
				MSM_ISPIF_PAD_SRC,
				&camss->vfe.line[j].subdev.entity,
				MSM_VFE_PAD_SINK,
				0);
			if (ret < 0) {
				dev_err(camss->dev,
					"Failed to link %s->%s entities\n",
					camss->ispif.line[i].subdev.entity.name,
					camss->vfe.line[j].subdev.entity.name);
				goto err_link;
			}
		}
	}

	return 0;

err_link:
	msm_vfe_unregister_entities(&camss->vfe);
err_reg_vfe:
	msm_ispif_unregister_entities(&camss->ispif);
err_reg_ispif:

	i = ARRAY_SIZE(camss->csid);
err_reg_csid:
	for (i--; i >= 0; i--)
		msm_csid_unregister_entity(&camss->csid[i]);

	i = ARRAY_SIZE(camss->csiphy);
err_reg_csiphy:
	for (i--; i >= 0; i--)
		msm_csiphy_unregister_entity(&camss->csiphy[i]);

	return ret;
}
/*
 * camss_pipeline_pm_use_count - Count the number of users of a pipeline
 * @entity: The entity
 *
 * Return the total number of users of all video device nodes in the pipeline.
 */
static int camss_pipeline_pm_use_count(struct media_entity *entity)
{
	struct media_entity_graph graph;
	int use = 0;

	media_entity_graph_walk_start(&graph, entity);

	while ((entity = media_entity_graph_walk_next(&graph))) {
		if (media_entity_type(entity) == MEDIA_ENT_T_DEVNODE)
			use += entity->use_count;
	}

	return use;
}

/*
 * camss_pipeline_pm_power_one - Apply power change to an entity
 * @entity: The entity
 * @change: Use count change
 *
 * Change the entity use count by @change. If the entity is a subdev update its
 * power state by calling the core::s_power operation when the use count goes
 * from 0 to != 0 or from != 0 to 0.
 *
 * Return 0 on success or a negative error code on failure.
 */
static int camss_pipeline_pm_power_one(struct media_entity *entity, int change)
{
	struct v4l2_subdev *subdev;
	int ret;

	subdev = media_entity_type(entity) == MEDIA_ENT_T_V4L2_SUBDEV
	       ? media_entity_to_v4l2_subdev(entity) : NULL;

	if (entity->use_count == 0 && change > 0 && subdev != NULL) {
		ret = v4l2_subdev_call(subdev, core, s_power, 1);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			return ret;
	}

	entity->use_count += change;
	WARN_ON(entity->use_count < 0);

	if (entity->use_count == 0 && change < 0 && subdev != NULL)
		v4l2_subdev_call(subdev, core, s_power, 0);

	return 0;
}

/*
 * camss_pipeline_pm_power - Apply power change to all entities in a pipeline
 * @entity: The entity
 * @change: Use count change
 *
 * Walk the pipeline to update the use count and the power state of all non-node
 * entities.
 *
 * Return 0 on success or a negative error code on failure.
 */
static int camss_pipeline_pm_power(struct media_entity *entity, int change)
{
	struct media_entity_graph graph;
	struct media_entity *first = entity;
	int ret = 0;

	if (!change)
		return 0;

	media_entity_graph_walk_start(&graph, entity);

	while (!ret && (entity = media_entity_graph_walk_next(&graph)))
		if (media_entity_type(entity) != MEDIA_ENT_T_DEVNODE)
			ret = camss_pipeline_pm_power_one(entity, change);

	if (!ret)
		return 0;

	media_entity_graph_walk_start(&graph, first);

	while ((first = media_entity_graph_walk_next(&graph))
	       && first != entity)
		if (media_entity_type(first) != MEDIA_ENT_T_DEVNODE)
			camss_pipeline_pm_power_one(first, -change);

	return ret;
}

/*
 * msm_camss_pipeline_pm_use - Update the use count of an entity
 * @entity: The entity
 * @use: Use (1) or stop using (0) the entity
 *
 * Update the use count of all entities in the pipeline and power entities on or
 * off accordingly.
 *
 * Return 0 on success or a negative error code on failure. Powering entities
 * off is assumed to never fail. No failure can occur when the use parameter is
 * set to 0.
 */
int msm_camss_pipeline_pm_use(struct media_entity *entity, int use)
{
	int change = use ? 1 : -1;
	int ret;

	mutex_lock(&entity->parent->graph_mutex);

	/* Apply use count to node. */
	entity->use_count += change;
	WARN_ON(entity->use_count < 0);

	/* Apply power change to connected non-nodes. */
	ret = camss_pipeline_pm_power(entity, change);
	if (ret < 0)
		entity->use_count -= change;

	mutex_unlock(&entity->parent->graph_mutex);

	return ret;
}

/*
 * camss_pipeline_link_notify - Link management notification callback
 * @link: The link
 * @flags: New link flags that will be applied
 * @notification: The link's state change notification type (MEDIA_DEV_NOTIFY_*)
 *
 * React to link management on powered pipelines by updating the use count of
 * all entities in the source and sink sides of the link. Entities are powered
 * on or off accordingly.
 *
 * Return 0 on success or a negative error code on failure. Powering entities
 * off is assumed to never fail. This function will not fail for disconnection
 * events.
 */
static int camss_pipeline_link_notify(struct media_link *link, u32 flags,
				    unsigned int notification)
{
	struct media_entity *source = link->source->entity;
	struct media_entity *sink = link->sink->entity;
	int source_use = camss_pipeline_pm_use_count(source);
	int sink_use = camss_pipeline_pm_use_count(sink);
	int ret;

	if (notification == MEDIA_DEV_NOTIFY_POST_LINK_CH &&
	    !(flags & MEDIA_LNK_FL_ENABLED)) {
		/* Powering off entities is assumed to never fail. */
		camss_pipeline_pm_power(source, -sink_use);
		camss_pipeline_pm_power(sink, -source_use);
		return 0;
	}

	if (notification == MEDIA_DEV_NOTIFY_PRE_LINK_CH &&
		(flags & MEDIA_LNK_FL_ENABLED)) {

		ret = camss_pipeline_pm_power(source, sink_use);
		if (ret < 0)
			return ret;

		ret = camss_pipeline_pm_power(sink, source_use);
		if (ret < 0)
			camss_pipeline_pm_power(source, -sink_use);

		return ret;
	}

	return 0;
}

/*
 * camss_unregister_entities - Unregister subdev nodes
 * @camss: CAMSS device
 *
 * Return 0 on success or a negative error code on failure
 */
static void camss_unregister_entities(struct camss *camss)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(camss->csiphy); i++)
		msm_csiphy_unregister_entity(&camss->csiphy[i]);

	for (i = 0; i < ARRAY_SIZE(camss->csid); i++)
		msm_csid_unregister_entity(&camss->csid[i]);

	msm_ispif_unregister_entities(&camss->ispif);
	msm_vfe_unregister_entities(&camss->vfe);
}

static int camss_subdev_notifier_bound(struct v4l2_async_notifier *async,
				       struct v4l2_subdev *subdev,
				       struct v4l2_async_subdev *asd)
{
	struct camss *camss = container_of(async, struct camss, notifier);
	struct camss_async_subdev *csd =
		container_of(asd, struct camss_async_subdev, asd);
	u8 id = csd->interface.csiphy_id;
	struct csiphy_device *csiphy = &camss->csiphy[id];

	csiphy->cfg.csi2 = &csd->interface.csi2;
	subdev->host_priv = csiphy;

	return 0;
}

static int camss_subdev_notifier_complete(struct v4l2_async_notifier *async)
{
	struct camss *camss = container_of(async, struct camss, notifier);
	struct v4l2_device *v4l2_dev = &camss->v4l2_dev;
	struct v4l2_subdev *sd;
	int ret;

	list_for_each_entry(sd, &v4l2_dev->subdevs, list) {
		if (sd->host_priv) {
			struct media_entity *sensor = &sd->entity;
			struct csiphy_device *csiphy =
					(struct csiphy_device *) sd->host_priv;
			struct media_entity *input = &csiphy->subdev.entity;
			unsigned int i;

			for (i = 0; i < sensor->num_pads; i++) {
				if (sensor->pads[i].flags & MEDIA_PAD_FL_SOURCE)
					break;
			}
			if (i == sensor->num_pads) {
				dev_err(camss->dev,
					"No source pad in external entity\n");
				return -EINVAL;
			}

			ret = media_entity_create_link(sensor, i,
				input, MSM_CSIPHY_PAD_SINK,
				MEDIA_LNK_FL_IMMUTABLE | MEDIA_LNK_FL_ENABLED);
			if (ret < 0) {
				dev_err(camss->dev,
					"Failed to link %s->%s entities\n",
					sensor->name, input->name);
				return ret;
			}
		}
	}

	ret = v4l2_device_register_subdev_nodes(&camss->v4l2_dev);
	if (ret < 0)
		return ret;

	//return media_device_register(&camss->media_dev);
	return 0;
}

#if 0
static const struct media_device_ops camss_media_ops = {
	.link_notify = v4l2_pipeline_link_notify,
};
#endif
/*
 * camss_probe - Probe CAMSS platform device
 * @pdev: Pointer to CAMSS platform device
 *
 * Return 0 on success or a negative error code on failure
 */
static int camss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct camss *camss;
	int ret;
	pr_err("[camera] : %s+, %d\n", __func__, __LINE__);
	camss = kzalloc(sizeof(*camss), GFP_KERNEL);
	if (!camss)
		return -ENOMEM;

	atomic_set(&camss->ref_count, 0);
	camss->dev = dev;
	platform_set_drvdata(pdev, camss);
	pr_err("[camera] : %s, %d\n", __func__, __LINE__);
	ret = camss_of_parse_ports(dev, &camss->notifier);
	if (ret < 0)
		return ret;
	else if (ret == 0)
		return -ENODEV;
	pr_err("[camera] : %s, %d\n", __func__, __LINE__);
	ret = camss_init_subdevices(camss);
	if (ret < 0)
		return ret;
	pr_err("[camera] : %s, %d\n", __func__, __LINE__);
	ret = dma_set_mask_and_coherent(dev, 0xffffffff);
	if (ret)
		return ret;
#if 0
	camss_smmu_setup(camss);
	if (ret)
		return ret;
	pr_err("[camera] : %s, %d\n", __func__, __LINE__);

	cam_smmu_attach(camss);
	if (ret)
		return ret;
#else
	ret = cam_smmu_get_handle("vfe", &camss->iommu_hdl);
	if (ret < 0) {
		pr_err("%s , vfe get handle failed\n", __func__);
		return ret;
	}

	ret= cam_smmu_ops(camss->iommu_hdl,
		CAM_SMMU_ATTACH);
	if (ret < 0) {
		pr_err("%s: img smmu attach error, ret :%d\n",
			__func__, ret);
		return ret;
	}
#endif
	pr_err("[camera] : %s, %d\n", __func__, __LINE__);

	camss->media_dev.dev = camss->dev;
	strlcpy(camss->media_dev.model, "Qualcomm Camera Subsystem",
		sizeof(camss->media_dev.model));
//	camss->media_dev.ops = &camss_media_ops;
//	media_device_init(&camss->media_dev);
	camss->media_dev.link_notify = camss_pipeline_link_notify;
	ret = media_device_register(&camss->media_dev);
	if (ret < 0) {
		dev_err(dev, "Failed to register media device");
		goto err_register_subdevs;
	}
	camss->v4l2_dev.mdev = &camss->media_dev;
	ret = v4l2_device_register(camss->dev, &camss->v4l2_dev);
	if (ret < 0) {
		dev_err(dev, "Failed to register V4L2 device\n");
		return ret;
	}
	pr_err("[camera] : %s, %d\n", __func__, __LINE__);

	ret = camss_register_entities(camss);
	if (ret < 0)
		goto err_register_entities;
	pr_err("[camera] : %s, %d\n", __func__, __LINE__);
	if (camss->notifier.num_subdevs) {
		pr_err("[camera] : %s, %d\n", __func__, __LINE__);
		camss->notifier.bound = camss_subdev_notifier_bound;
		camss->notifier.complete = camss_subdev_notifier_complete;

		ret = v4l2_async_notifier_register(&camss->v4l2_dev,
						   &camss->notifier);
		if (ret) {
			dev_err(dev, "Failed to register async subdev nodes");
			goto err_register_subdevs;
		}
	} else {
		pr_err("[camera] : %s, %d\n", __func__, __LINE__);
		ret = v4l2_device_register_subdev_nodes(&camss->v4l2_dev);
		if (ret < 0) {
			dev_err(dev, "Failed to register subdev nodes");
			goto err_register_subdevs;
		}
	}

	ret = cam_ahb_clk_init(pdev);
	if (ret < 0) {
		pr_err("%s: failed to register ahb clocks\n", __func__);
		return ret;
	}

	pr_err("[camera] : %s-, %d\n", __func__, __LINE__);
	return 0;

err_register_subdevs:
	camss_unregister_entities(camss);
err_register_entities:
	v4l2_device_unregister(&camss->v4l2_dev);

	return ret;
}

void camss_delete(struct camss *camss)
{
	v4l2_device_unregister(&camss->v4l2_dev);
	media_device_unregister(&camss->media_dev);
//	media_device_cleanup(&camss->media_dev);

	kfree(camss);
}

/*
 * camss_remove - Remove CAMSS platform device
 * @pdev: Pointer to CAMSS platform device
 *
 * Always returns 0.
 */
static int camss_remove(struct platform_device *pdev)
{
	struct camss *camss = platform_get_drvdata(pdev);

	msm_vfe_stop_streaming(&camss->vfe);

	v4l2_async_notifier_unregister(&camss->notifier);
	camss_unregister_entities(camss);

	if (atomic_read(&camss->ref_count) == 0)
		camss_delete(camss);

	return 0;
}

static const struct of_device_id camss_dt_match[] = {
	{ .compatible = "qcom,msm8996-camss" },
	{ }
};

MODULE_DEVICE_TABLE(of, camss_dt_match);

static struct platform_driver qcom_camss_driver = {
	.probe = camss_probe,
	.remove = camss_remove,
	.driver = {
		.name = "qcom-camss",
		.of_match_table = camss_dt_match,
	},
};

module_platform_driver(qcom_camss_driver);

MODULE_ALIAS("platform:qcom-camss");
MODULE_DESCRIPTION("Qualcomm Camera Subsystem driver");
MODULE_LICENSE("GPL");
