/*
 * video.c
 *
 * Qualcomm MSM Camera Subsystem - V4L2 device node
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
#include <media/media-entity.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
//#include <media/v4l2-mc.h>
#include <media/videobuf-core.h>
//#include <media/videobuf2-dma-sg.h>
#include <media/videobuf2-dma-contig.h>
#include <linux/msm_dma_iommu_mapping.h>
#include <../drivers/media/platform/qcom/camera/common/cam_smmu_api.h>


#include "video.h"
#include "camss.h"

struct fract {
	u8 numerator;
	u8 denominator;
};

/*
 * struct format_info - ISP media bus format information
 * @code: V4L2 media bus format code
 * @pixelformat: V4L2 pixel format FCC identifier
 * @bpp: Bits per pixel when stored in memory
 */
static const struct format_info {
	u32 code;
	u32 pixelformat;
	u8 planes;
	struct fract hsub[3];
	struct fract vsub[3];
	unsigned int bpp[3];
} formats[] = {
	{ MEDIA_BUS_FMT_UYVY8_2X8, V4L2_PIX_FMT_UYVY, 1,
	  { { 1, 1 } }, { { 1, 1 } }, { 16 } },
	{ MEDIA_BUS_FMT_VYUY8_2X8, V4L2_PIX_FMT_VYUY, 1,
	  { { 1, 1 } }, { { 1, 1 } }, { 16 } },
	{ MEDIA_BUS_FMT_YUYV8_2X8, V4L2_PIX_FMT_YUYV, 1,
	  { { 1, 1 } }, { { 1, 1 } }, { 16 } },
	{ MEDIA_BUS_FMT_YVYU8_2X8, V4L2_PIX_FMT_YVYU, 1,
	  { { 1, 1 } }, { { 1, 1 } }, { 16 } },
	{ MEDIA_BUS_FMT_SBGGR8_1X8, V4L2_PIX_FMT_SBGGR8, 1,
	  { { 1, 1 } }, { { 1, 1 } }, { 8 } },
	{ MEDIA_BUS_FMT_SGBRG8_1X8, V4L2_PIX_FMT_SGBRG8, 1,
	  { { 1, 1 } }, { { 1, 1 } }, { 8 } },
	{ MEDIA_BUS_FMT_SGRBG8_1X8, V4L2_PIX_FMT_SGRBG8, 1,
	  { { 1, 1 } }, { { 1, 1 } }, { 8 } },
	{ MEDIA_BUS_FMT_SRGGB8_1X8, V4L2_PIX_FMT_SRGGB8, 1,
	  { { 1, 1 } }, { { 1, 1 } }, { 8 } },
	{ MEDIA_BUS_FMT_SBGGR10_1X10, V4L2_PIX_FMT_SBGGR10P, 1,
	  { { 1, 1 } }, { { 1, 1 } }, { 10 } },
	{ MEDIA_BUS_FMT_SGBRG10_1X10, V4L2_PIX_FMT_SGBRG10P, 1,
	  { { 1, 1 } }, { { 1, 1 } }, { 10 } },
	{ MEDIA_BUS_FMT_SGRBG10_1X10, V4L2_PIX_FMT_SGRBG10P, 1,
	  { { 1, 1 } }, { { 1, 1 } }, { 10 } },
	{ MEDIA_BUS_FMT_SRGGB10_1X10, V4L2_PIX_FMT_SRGGB10P, 1,
	  { { 1, 1 } }, { { 1, 1 } }, { 10 } },
	{ MEDIA_BUS_FMT_SBGGR12_1X12, V4L2_PIX_FMT_SRGGB12P, 1,
	  { { 1, 1 } }, { { 1, 1 } }, { 12 } },
	{ MEDIA_BUS_FMT_SGBRG12_1X12, V4L2_PIX_FMT_SGBRG12P, 1,
	  { { 1, 1 } }, { { 1, 1 } }, { 12 } },
	{ MEDIA_BUS_FMT_SGRBG12_1X12, V4L2_PIX_FMT_SGRBG12P, 1,
	  { { 1, 1 } }, { { 1, 1 } }, { 12 } },
	{ MEDIA_BUS_FMT_SRGGB12_1X12, V4L2_PIX_FMT_SRGGB12P, 1,
	  { { 1, 1 } }, { { 1, 1 } }, { 12 } },
	{ MEDIA_BUS_FMT_YUYV8_1_5X8, V4L2_PIX_FMT_NV12M, 2,
	  { { 1, 1 }, { 1, 1 } }, { { 1, 1 }, { 2, 1 } }, { 8, 8 } },
	{ MEDIA_BUS_FMT_YVYU8_1_5X8, V4L2_PIX_FMT_NV12M, 2,
	  { { 1, 1 }, { 1, 1 } }, { { 1, 1 }, { 2, 1 } }, { 8, 8 } },
	{ MEDIA_BUS_FMT_UYVY8_1_5X8, V4L2_PIX_FMT_NV12M, 2,
	  { { 1, 1 }, { 1, 1 } }, { { 1, 1 }, { 2, 1 } }, { 8, 8 } },
	{ MEDIA_BUS_FMT_VYUY8_1_5X8, V4L2_PIX_FMT_NV12M, 2,
	  { { 1, 1 }, { 1, 1 } }, { { 1, 1 }, { 2, 1 } }, { 8, 8 } },
	{ MEDIA_BUS_FMT_YUYV8_1_5X8, V4L2_PIX_FMT_NV21M, 2,
	  { { 1, 1 }, { 1, 1 } }, { { 1, 1 }, { 2, 1 } }, { 8, 8 } },
	{ MEDIA_BUS_FMT_YVYU8_1_5X8, V4L2_PIX_FMT_NV21M, 2,
	  { { 1, 1 }, { 1, 1 } }, { { 1, 1 }, { 2, 1 } }, { 8, 8 } },
	{ MEDIA_BUS_FMT_UYVY8_1_5X8, V4L2_PIX_FMT_NV21M, 2,
	  { { 1, 1 }, { 1, 1 } }, { { 1, 1 }, { 2, 1 } }, { 8, 8 } },
	{ MEDIA_BUS_FMT_VYUY8_1_5X8, V4L2_PIX_FMT_NV21M, 2,
	  { { 1, 1 }, { 1, 1 } }, { { 1, 1 }, { 2, 1 } }, { 8, 8 } },
	{ MEDIA_BUS_FMT_YUYV8_1_5X8, V4L2_PIX_FMT_NV12, 1,
	  { { 1, 1 } }, { { 2, 3 } }, { 8 } },
	{ MEDIA_BUS_FMT_YVYU8_1_5X8, V4L2_PIX_FMT_NV12, 1,
	  { { 1, 1 } }, { { 2, 3 } }, { 8 } },
	{ MEDIA_BUS_FMT_UYVY8_1_5X8, V4L2_PIX_FMT_NV12, 1,
	  { { 1, 1 } }, { { 2, 3 } }, { 8 } },
	{ MEDIA_BUS_FMT_VYUY8_1_5X8, V4L2_PIX_FMT_NV12, 1,
	  { { 1, 1 } }, { { 2, 3 } }, { 8 } },
	{ MEDIA_BUS_FMT_YUYV8_1_5X8, V4L2_PIX_FMT_NV21, 1,
	  { { 1, 1 } }, { { 2, 3 } }, { 8 } },
	{ MEDIA_BUS_FMT_YVYU8_1_5X8, V4L2_PIX_FMT_NV21, 1,
	  { { 1, 1 } }, { { 2, 3 } }, { 8 } },
	{ MEDIA_BUS_FMT_UYVY8_1_5X8, V4L2_PIX_FMT_NV21, 1,
	  { { 1, 1 } }, { { 2, 3 } }, { 8 } },
	{ MEDIA_BUS_FMT_VYUY8_1_5X8, V4L2_PIX_FMT_NV21, 1,
	  { { 1, 1 } }, { { 2, 3 } }, { 8 } },
};

/* -----------------------------------------------------------------------------
 * Helper functions
 */

static int video_find_format(u32 code, u32 pixelformat)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(formats); i++) {
		if (formats[i].code == code &&
		    formats[i].pixelformat == pixelformat)
			return i;
	}

	for (i = 0; i < ARRAY_SIZE(formats); i++)
		if (formats[i].code == code)
			return i;

	WARN_ON(1);

	return -EINVAL;
}

static int video_find_format_n(u32 code, u32 index)
{
	int i;
	u32 n = 0;

	for (i = 0; i < ARRAY_SIZE(formats); i++)
		if (formats[i].code == code) {
			if (n == index)
				return i;
			n++;
		}

	return -EINVAL;
}

/*
 * video_mbus_to_pix_mp - Convert v4l2_mbus_framefmt to v4l2_pix_format_mplane
 * @mbus: v4l2_mbus_framefmt format
 * @pix: v4l2_pix_format_mplane format (output)
 * @index: index of an entry in formats array to be used for the conversion
 * @alignment: bytesperline alignment value
 *
 * Fill the output pix structure with information from the input mbus format.
 *
 * Return 0 on success or a negative error code otherwise
 */
static int video_mbus_to_pix_mp(const struct v4l2_mbus_framefmt *mbus,
				struct v4l2_pix_format_mplane *pix, int index,
				unsigned int alignment)
{
	const struct format_info *f;
	unsigned int i;
	u32 bytesperline;

	f = &formats[index];
	memset(pix, 0, sizeof(*pix));
	pix->width = mbus->width;
	pix->height = mbus->height;
	pix->pixelformat = f->pixelformat;
	pix->num_planes = f->planes;
	for (i = 0; i < pix->num_planes; i++) {
		bytesperline = pix->width / f->hsub[i].numerator *
			f->hsub[i].denominator * f->bpp[i] / 8;
		bytesperline = ALIGN(bytesperline, alignment);
		pix->plane_fmt[i].bytesperline = bytesperline;
		pix->plane_fmt[i].sizeimage = pix->height /
				f->vsub[i].numerator * f->vsub[i].denominator *
				bytesperline;
	}
	pix->colorspace = mbus->colorspace;
	pix->field = mbus->field;

	return 0;
}

static struct v4l2_subdev *video_remote_subdev(struct camss_video *video,
					       u32 *pad)
{
	struct media_pad *remote;

	remote = media_entity_remote_pad(&video->pad);

	if (!remote /*|| !is_media_entity_v4l2_subdev(remote->entity)*/)
		return NULL;

	if (pad)
		*pad = remote->index;

	return media_entity_to_v4l2_subdev(remote->entity);
}

static int video_get_subdev_format(struct camss_video *video,
				   struct v4l2_format *format)
{
	struct v4l2_subdev_format fmt;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = video_remote_subdev(video, &pad);
	if (subdev == NULL)
		return -EINVAL;

	fmt.pad = pad;
	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;

	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt);
	if (ret)
		return ret;

	format->type = video->type;

	ret = video_find_format(fmt.format.code,
				format->fmt.pix_mp.pixelformat);
	if (ret < 0)
		return ret;

	return video_mbus_to_pix_mp(&fmt.format, &format->fmt.pix_mp, ret,
				    video->bpl_alignment);
}

static int video_get_pixelformat(struct camss_video *video, u32 *pixelformat,
				 u32 index)
{
	struct v4l2_subdev_format fmt;
	struct v4l2_subdev *subdev;
	u32 pad;
	int ret;

	subdev = video_remote_subdev(video, &pad);
	if (subdev == NULL)
		return -EINVAL;

	fmt.pad = pad;
	fmt.which = V4L2_SUBDEV_FORMAT_ACTIVE;

	ret = v4l2_subdev_call(subdev, pad, get_fmt, NULL, &fmt);
	if (ret)
		return ret;

	ret = video_find_format_n(fmt.format.code, index);
	if (ret < 0)
		return ret;

	*pixelformat = formats[ret].pixelformat;

	return 0;
}

/* -----------------------------------------------------------------------------
 * Video queue operations
 */

static int video_queue_setup(struct vb2_queue *q, const struct v4l2_format *fmt,
	unsigned int *num_buffers, unsigned int *num_planes,
	unsigned int sizes[], void *alloc_ctxs[])
{
	struct camss_video *video = vb2_get_drv_priv(q);
	const struct v4l2_pix_format_mplane *format =
						&video->active_fmt.fmt.pix_mp;
	unsigned int i;
	pr_err("%s: %d\n", __func__, __LINE__);
	if (*num_planes) {
		if (*num_planes != format->num_planes)
			return -EINVAL;

		for (i = 0; i < *num_planes; i++)
			if (sizes[i] < format->plane_fmt[i].sizeimage)
				return -EINVAL;

		return 0;
	}

	*num_planes = format->num_planes;

	for (i = 0; i < *num_planes; i++)
		sizes[i] = format->plane_fmt[i].sizeimage;

	return 0;
}

static int video_buf_init(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct camss_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct camss_buffer *buffer = container_of(vbuf, struct camss_buffer,
						   vb);
	const struct v4l2_pix_format_mplane *format =
						&video->active_fmt.fmt.pix_mp;
	//struct sg_table *sgt;
	unsigned int i;
	//int ret = 0;
	//struct dma_buf *dma_buf;
	void *vaddr;
	//dma_addr_t paddr_ptr;
	//size_t len_ptr;
	pr_err("%s: format->num_planes = %d\n", __func__, format->num_planes);
	for (i = 0; i < format->num_planes; i++) {
		vaddr = vb2_plane_vaddr(vb, i);
		pr_err("%s: virtual addr = 0x%p\n", __func__, vaddr);
#if 0
        sgt = vb2_dma_sg_plane_desc(vb, i);
		if (!sgt)
			return -EFAULT;
		buffer->table = sgt;
		//dummy DMA buf
		dma_buf = vb2_dma_sg_get_dmabuf((void *)buffer, (size_t ) sizeof(struct camss_buffer), O_RDWR);
		if (!dma_buf) {
			pr_err("%s: vb2_dma_sg_get_dmabuf failed\n", __func__);
		}

		buffer->buf = dma_buf;
		buffer->fd = dma_buf_fd(dma_buf, 0);
		pr_err("%s: dma_buf fd = %d\n", __func__, buffer->fd);
#if 0
		ret = msm_dma_map_sg_lazy(video->camss->dev, sgt->sgl,
				sgt->nents, DMA_BIDIRECTIONAL, dma_buf);
		if (!ret) {
			pr_err("%s: msm_dma_map_sg_lazy failed\n", __func__);
		}
#else
		ret = cam_smmu_get_phy_addr(video->camss->iommu_hdl,
					buffer->fd, buffer->buf, buffer->table,
					CAM_SMMU_MAP_RW,
					&paddr_ptr,
					&len_ptr);

#endif
	//	buffer->addr[i] = sg_dma_address(sgt->sgl);
#endif
    }

	if (format->pixelformat == V4L2_PIX_FMT_NV12 ||
			format->pixelformat == V4L2_PIX_FMT_NV21)
		buffer->addr[1] = buffer->addr[0] +
				format->plane_fmt[0].bytesperline *
				format->height;

	return 0;
}

static void video_buf_cleanup(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct camss_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct camss_buffer *buffer = container_of(vbuf, struct camss_buffer,
						   vb);
	const struct v4l2_pix_format_mplane *format =
						&video->active_fmt.fmt.pix_mp;
//	struct sg_table *sgt;
	unsigned int i;
//	struct dma_buf *dma_buf;
	pr_err("%s: format->num_planes = %d\n", __func__, format->num_planes);
	for (i = 0; i < format->num_planes; i++) {
#if 0
		sgt = buffer->table;
		dma_buf = buffer->buf;
		msm_dma_unmap_sg(video->camss->dev, sgt->sgl,
				sgt->nents, DMA_BIDIRECTIONAL, dma_buf);
#else
	pr_err("%s: dma_buf fd = %d\n", __func__, buffer->fd);
	cam_smmu_put_phy_addr(video->camss->iommu_hdl, buffer->fd);

#endif
	}
}


static int video_buf_prepare(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct camss_video *video = vb2_get_drv_priv(vb->vb2_queue);
	const struct v4l2_pix_format_mplane *format =
						&video->active_fmt.fmt.pix_mp;
	unsigned int i;
    
    struct camss_buffer *buffer = container_of(vbuf, struct camss_buffer,vb);

	pr_err("%s: format->num_planes = %d\n", __func__, format->num_planes);
	for (i = 0; i < format->num_planes; i++) {
        buffer->addr[i] = vb2_dma_contig_plane_dma_addr(vb, 0);
		if (format->plane_fmt[i].sizeimage > vb2_plane_size(vb, i))
			return -EINVAL;
		pr_err("%s: sizeimage = %d, plane size = %ld\n", __func__, format->plane_fmt[i].sizeimage, vb2_plane_size(vb, i));
		vb2_set_plane_payload(vb, i, format->plane_fmt[i].sizeimage);
	}

	vbuf->field = V4L2_FIELD_NONE;

	return 0;
}

static void video_buf_queue(struct vb2_buffer *vb)
{
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct camss_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct camss_buffer *buffer = container_of(vbuf, struct camss_buffer,
						   vb);

	video->ops->queue_buffer(video, buffer);
}

static int video_check_format(struct camss_video *video)
{
	struct v4l2_pix_format_mplane *pix = &video->active_fmt.fmt.pix_mp;
	struct v4l2_pix_format_mplane *sd_pix;
	struct v4l2_format format;
	int ret;

	sd_pix = &format.fmt.pix_mp;
	sd_pix->pixelformat = pix->pixelformat;
	ret = video_get_subdev_format(video, &format);
	if (ret < 0)
		return ret;

	if (pix->pixelformat != sd_pix->pixelformat ||
	    pix->height != sd_pix->height ||
	    pix->width != sd_pix->width ||
	    pix->num_planes != sd_pix->num_planes ||
	    pix->field != format.fmt.pix_mp.field)
		return -EINVAL;

	return 0;
}

static int video_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct camss_video *video = vb2_get_drv_priv(q);
	struct video_device *vdev = &video->vdev;
	struct media_entity *entity;
	struct media_pad *pad;
	struct v4l2_subdev *subdev;
	int ret;
	pr_err("[camera] : %s++++, %d\n", __func__, __LINE__);
	ret = media_entity_pipeline_start(&vdev->entity, &video->pipe);
	if (ret < 0)
		return ret;

	ret = video_check_format(video);
	if (ret < 0)
		goto error;

	entity = &vdev->entity;
	while (1) {
		pad = &entity->pads[0];
		if (!(pad->flags & MEDIA_PAD_FL_SINK))
			break;

		pad = media_entity_remote_pad(pad);
		if (!pad /*|| !is_media_entity_v4l2_subdev(pad->entity)*/)
			break;

		entity = pad->entity;
		subdev = media_entity_to_v4l2_subdev(entity);

		ret = v4l2_subdev_call(subdev, video, s_stream, 1);
		if (ret < 0 && ret != -ENOIOCTLCMD)
			goto error;
	}
	pr_err("[camera] : %s----, %d\n", __func__, __LINE__);
	return 0;

error:
	media_entity_pipeline_stop(&vdev->entity);

	video->ops->flush_buffers(video, VB2_BUF_STATE_QUEUED);

	return ret;
}

static void video_stop_streaming(struct vb2_queue *q)
{
	struct camss_video *video = vb2_get_drv_priv(q);
	struct video_device *vdev = &video->vdev;
	struct media_entity *entity;
	struct media_pad *pad;
	struct v4l2_subdev *subdev;
	pr_err("[camera] : %s++++, %d\n", __func__, __LINE__);
	entity = &vdev->entity;
	while (1) {
		pad = &entity->pads[0];
		if (!(pad->flags & MEDIA_PAD_FL_SINK))
			break;

		pad = media_entity_remote_pad(pad);
		if (!pad /*|| !is_media_entity_v4l2_subdev(pad->entity)*/)
			break;

		entity = pad->entity;
		subdev = media_entity_to_v4l2_subdev(entity);

		v4l2_subdev_call(subdev, video, s_stream, 0);
	}

	media_entity_pipeline_stop(&vdev->entity);

	video->ops->flush_buffers(video, VB2_BUF_STATE_ERROR);
	pr_err("[camera] : %s----, %d\n", __func__, __LINE__);
}

static const struct vb2_ops msm_video_vb2_q_ops = {
	.queue_setup     = video_queue_setup,
	.wait_prepare    = vb2_ops_wait_prepare,
	.wait_finish     = vb2_ops_wait_finish,
	.buf_init        = video_buf_init,
	.buf_cleanup = video_buf_cleanup,
	.buf_prepare     = video_buf_prepare,
	.buf_queue       = video_buf_queue,
	.start_streaming = video_start_streaming,
	.stop_streaming  = video_stop_streaming,
};

/* -----------------------------------------------------------------------------
 * V4L2 ioctls
 */

static int video_querycap(struct file *file, void *fh,
			  struct v4l2_capability *cap)
{
	struct camss_video *video = video_drvdata(file);
	pr_err("[camera] : %s, %d\n", __func__, __LINE__);
	strlcpy(cap->driver, "qcom-camss", sizeof(cap->driver));
	strlcpy(cap->card, "Qualcomm Camera Subsystem", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(video->camss->dev));
	  cap->capabilities = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING |V4L2_CAP_DEVICE_CAPS;
	 cap->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING | V4L2_CAP_READWRITE;;

	return 0;
}

static int video_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct camss_video *video = video_drvdata(file);

	if (f->type != video->type)
		return -EINVAL;
	pr_err("[camera] : %s, %d\n", __func__, __LINE__);
	return video_get_pixelformat(video, &f->pixelformat, f->index);
}

static int video_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camss_video *video = video_drvdata(file);
	pr_err("[camera] : %s, %d\n", __func__, __LINE__);
	if (f->type != video->type)
		return -EINVAL;

	*f = video->active_fmt;

	return 0;
}

static int video_try_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camss_video *video = video_drvdata(file);
	struct v4l2_plane_pix_format *p;
	u32 bytesperline[3] = { 0 };
	u32 sizeimage[3] = { 0 };
	u32 lines;
	int ret;
	int i;

	if (f->type != video->type)
		return -EINVAL;

	if (video->line_based)
		for (i = 0; i < f->fmt.pix_mp.num_planes && i < 3; i++) {
			p = &f->fmt.pix_mp.plane_fmt[i];
			bytesperline[i] = clamp_t(u32, p->bytesperline,
						  1, 65528);
			sizeimage[i] = clamp_t(u32, p->sizeimage,
					       bytesperline[i],
					       bytesperline[i] * 4096);
		}

	ret = video_get_subdev_format(video, f);
	if (ret < 0)
		return ret;

	if (video->line_based)
		for (i = 0; i < f->fmt.pix_mp.num_planes; i++) {
			p = &f->fmt.pix_mp.plane_fmt[i];
			p->bytesperline = clamp_t(u32, p->bytesperline,
						  1, 65528);
			p->sizeimage = clamp_t(u32, p->sizeimage,
					       p->bytesperline,
					       p->bytesperline * 4096);
			lines = p->sizeimage / p->bytesperline;

			if (p->bytesperline < bytesperline[i])
				p->bytesperline = ALIGN(bytesperline[i], 8);

			if (p->sizeimage < p->bytesperline * lines)
				p->sizeimage = p->bytesperline * lines;

			if (p->sizeimage < sizeimage[i])
				p->sizeimage = sizeimage[i];
		}

	return 0;
}

static int video_s_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct camss_video *video = video_drvdata(file);
	int ret;
	pr_err("[camera] : %s, %d\n", __func__, __LINE__);
	ret = video_try_fmt(file, fh, f);
	if (ret < 0)
		return ret;

	video->active_fmt = *f;

	return 0;
}

static int video_enum_input(struct file *file, void *fh,
			    struct v4l2_input *input)
{
	if (input->index > 0)
		return -EINVAL;
	pr_err("[camera] : %s, %d\n", __func__, __LINE__);
	strlcpy(input->name, "camera", sizeof(input->name));
	input->type = V4L2_INPUT_TYPE_CAMERA;

	return 0;
}

static int video_g_input(struct file *file, void *fh, unsigned int *input)
{
	*input = 0;
	pr_err("[camera] : %s, %d\n", __func__, __LINE__);
	return 0;
}

static int video_s_input(struct file *file, void *fh, unsigned int input)
{
	pr_err("[camera] : %s, %d\n", __func__, __LINE__);
	return input == 0 ? 0 : -EINVAL;
}

static const struct v4l2_ioctl_ops msm_vid_ioctl_ops = {
	.vidioc_querycap		= video_querycap,
	.vidioc_enum_fmt_vid_cap_mplane	= video_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane	= video_g_fmt,
	.vidioc_s_fmt_vid_cap_mplane	= video_s_fmt,
	.vidioc_try_fmt_vid_cap_mplane	= video_try_fmt,
	.vidioc_reqbufs			= vb2_ioctl_reqbufs,
	.vidioc_querybuf		= vb2_ioctl_querybuf,
	.vidioc_qbuf			= vb2_ioctl_qbuf,
	.vidioc_expbuf			= vb2_ioctl_expbuf,
	.vidioc_dqbuf			= vb2_ioctl_dqbuf,
	.vidioc_create_bufs		= vb2_ioctl_create_bufs,
	.vidioc_prepare_buf		= vb2_ioctl_prepare_buf,
	.vidioc_streamon		= vb2_ioctl_streamon,
	.vidioc_streamoff		= vb2_ioctl_streamoff,
	.vidioc_enum_input		= video_enum_input,
	.vidioc_g_input			= video_g_input,
	.vidioc_s_input			= video_s_input,
};

/* -----------------------------------------------------------------------------
 * V4L2 file operations
 */

/*
 * video_init_format - Helper function to initialize format
 *
 * Initialize all pad formats with default values.
 */
static int video_init_format(struct file *file, void *fh)
{
	struct v4l2_format format;

	memset(&format, 0, sizeof(format));
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;

	return video_s_fmt(file, fh, &format);
}

static int video_open(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	struct camss_video *video = video_drvdata(file);
	struct v4l2_fh *vfh;
	int ret;
	pr_err("[camera] : %s+++++, %d\n", __func__, __LINE__);

    video->alloc_ctx = vb2_dma_contig_init_ctx(video->camss->iommu_dev);
    if (IS_ERR(video->alloc_ctx)) {
    dev_err(&vdev->dev, "Failed to init vb2 dma ctx\n");
    }
    
    mutex_lock(&video->lock);

	vfh = kzalloc(sizeof(*vfh), GFP_KERNEL);
	if (vfh == NULL) {
		ret = -ENOMEM;
		goto error_alloc;
	}

	v4l2_fh_init(vfh, vdev);
	v4l2_fh_add(vfh);

	file->private_data = vfh;

//	ret = v4l2_pipeline_pm_use(&vdev->entity, 1);
	ret = msm_camss_pipeline_pm_use(&vdev->entity, 1);
	if (ret < 0) {
		dev_err(video->camss->dev, "Failed to power up pipeline\n");
		goto error_pm_use;
	}

	ret = video_init_format(file, vfh);
	if (ret < 0) {
		dev_err(video->camss->dev, "Failed to init format\n");
		goto error_init_format;
	}

	mutex_unlock(&video->lock);
	pr_err("[camera] : %s-----, %d\n", __func__, __LINE__);
	return 0;

error_init_format:
//	v4l2_pipeline_pm_use(&vdev->entity, 0);
	msm_camss_pipeline_pm_use(&vdev->entity, 0);
    vb2_dma_contig_cleanup_ctx(video->alloc_ctx);
error_pm_use:
	v4l2_fh_release(file);

error_alloc:
	mutex_unlock(&video->lock);

	return ret;
}

static int video_release(struct file *file)
{
	struct video_device *vdev = video_devdata(file);
	pr_err("[camera] : %s+++++, %d\n", __func__, __LINE__);
	vb2_fop_release(file);

//	v4l2_pipeline_pm_use(&vdev->entity, 0);
	msm_camss_pipeline_pm_use(&vdev->entity, 0);

	file->private_data = NULL;
	pr_err("[camera] : %s-----, %d\n", __func__, __LINE__);
	return 0;
}

static const struct v4l2_file_operations msm_vid_fops = {
	.owner          = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open           = video_open,
	.release        = video_release,
	.poll           = vb2_fop_poll,
	.mmap		= vb2_fop_mmap,
	.read		= vb2_fop_read,
};

/* -----------------------------------------------------------------------------
 * CAMSS video core
 */

static void msm_video_release(struct video_device *vdev)
{
	struct camss_video *video = video_get_drvdata(vdev);

	media_entity_cleanup(&vdev->entity);

	mutex_destroy(&video->q_lock);
	mutex_destroy(&video->lock);

	if (atomic_dec_and_test(&video->camss->ref_count))
		camss_delete(video->camss);
}

int msm_video_register(struct camss_video *video, struct v4l2_device *v4l2_dev,
		       const char *name)
{
	struct media_pad *pad = &video->pad;
	struct video_device *vdev;
	struct vb2_queue *q;
	int ret;

	vdev = &video->vdev;

	mutex_init(&video->q_lock);

	q = &video->vb2_q;
	q->drv_priv = video;
//	q->mem_ops = &vb2_dma_sg_memops;
    q->mem_ops = &vb2_dma_contig_memops;
    q->ops = &msm_video_vb2_q_ops;
	q->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	q->io_modes = VB2_DMABUF | VB2_MMAP | VB2_READ;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	q->buf_struct_size = sizeof(struct camss_buffer);
//	q->dev = video->camss->dev;
	q->lock = &video->q_lock;
	ret = vb2_queue_init(q);
	if (ret < 0) {
		dev_err(v4l2_dev->dev, "Failed to init vb2 queue\n");
		goto error_vb2_init;
	}

	pad->flags = MEDIA_PAD_FL_SINK;
	ret = media_entity_init(&vdev->entity, 1, pad, 0);
	if (ret < 0) {
		dev_err(v4l2_dev->dev, "Failed to init video entity\n");
		goto error_media_init;
	}

	mutex_init(&video->lock);

	vdev->fops = &msm_vid_fops;
//	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING |
//							V4L2_CAP_READWRITE;
	vdev->ioctl_ops = &msm_vid_ioctl_ops;
	vdev->release = msm_video_release;
	vdev->v4l2_dev = v4l2_dev;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->queue = &video->vb2_q;
	vdev->lock = &video->lock;
//	vdev->debug = V4L2_DEBUG_IOCTL;
	strlcpy(vdev->name, name, sizeof(vdev->name));

	ret = video_register_device(vdev, VFL_TYPE_GRABBER, -1);
	if (ret < 0) {
		dev_err(v4l2_dev->dev, "Failed to register video device\n");
		goto error_video_register;
	}

	video_set_drvdata(vdev, video);
	atomic_inc(&video->camss->ref_count);

	return 0;

error_video_register:
	media_entity_cleanup(&vdev->entity);
	mutex_destroy(&video->lock);
error_media_init:
	vb2_queue_release(&video->vb2_q);
error_vb2_init:
	mutex_destroy(&video->q_lock);

	return ret;
}

void msm_video_stop_streaming(struct camss_video *video)
{
	if (vb2_is_streaming(&video->vb2_q))
		vb2_queue_release(&video->vb2_q);
}

void msm_video_unregister(struct camss_video *video)
{
	atomic_inc(&video->camss->ref_count);
	video_unregister_device(&video->vdev);
	atomic_dec(&video->camss->ref_count);
}
