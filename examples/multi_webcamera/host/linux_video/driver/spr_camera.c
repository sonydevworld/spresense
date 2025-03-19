// SPDX-License-Identifier: GPL-2.0
/*
 *      spr_camera.c   --  Video Capture Driver with images
 *                         from user-space
 *
 *      Copyright (c) 2024
 *          Sony Semiconductor Solutions Corporation
 */

/*****************************************************************************
 * Include Files
 *****************************************************************************/

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-image-sizes.h>
#include <media/videobuf2-vmalloc.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/pm_qos.h>

#include "spr_camera.h"

// #define FAKE_CAMERA_IMAGE

#ifdef FAKE_CAMERA_IMAGE
#  include <linux/kthread.h>
#  include <linux/delay.h>

/* pictx.h file contains jpeg image binary for testing.
 * Jpeg image must be VGA size.
 * These header files are generated from 'xxd -i' command. */

#  include "test_code/include/pict1.h"
#  include "test_code/include/pict2.h"
#  include "test_code/include/pict3.h"
#  include "test_code/include/pict4.h"
#  include "test_code/include/pict5.h"
#endif  /* FAKE_CAMERA_IMAGE */

/*****************************************************************************
 * Preprocessor Definitions
 *****************************************************************************/

#define ISNT_EQ_FIELD(fld)  (fmt->fmt.pix.fld != sprcam_pix_format[0].fld)

/*****************************************************************************
 * Private Types
 *****************************************************************************/

struct spr_camera
{
  struct v4l2_device       v4dev;
  struct v4l2_ctrl_handler v4hdlr;
  struct video_device      vdev;
  struct platform_device   *pdev;
  struct mutex             lock;
  struct vb2_queue         vbq;
  struct list_head         bufq;
  u32                      seqno;
  struct v4l2_pix_format   current_pixfmt;
  struct pm_qos_request    qos;
  bool is_streaming;

#ifdef FAKE_CAMERA_IMAGE
  struct task_struct *dummy_thread;
#else
  /* For User I/F Device file */

  int cdev_major;
  struct class *cdev_clas;
  dev_t cdev_id;
  bool is_cdev_opened;
#endif
};

struct spr_vbuf_container
{
  struct vb2_v4l2_buffer    vbuf;
  struct list_head    queue;
};

/*****************************************************************************
 * Private Function Prototypes
 *****************************************************************************/

/* --------------------------------------------------------------------------*/
/* User I/F File Operations */

#ifndef FAKE_CAMERA_IMAGE
static int sprcam_cdev_open(struct inode *inode, struct file *file);
static int sprcam_cdev_release(struct inode *inode, struct file *file);
static ssize_t sprcam_cdev_write(struct file *file, const char __user *buff,
                                 size_t count, loff_t *pos);
#endif

/* --------------------------------------------------------------------------*/
/* vb2 callback Operations */

static void sprcam_vb2_queue(struct vb2_buffer *vb);
static int sprcam_vb2_prepare(struct vb2_buffer *vb);
static int sprcam_vb2_queue_setup(struct vb2_queue *vq,
                                  unsigned int *nbufs,
                                  unsigned int *num_planes,
                                  unsigned int sizes[],
                                  struct device *alloc_devs[]);
static int sprcam_vb2_start_streaming(struct vb2_queue *vq,
                                      unsigned int count);
static void sprcam_vb2_stop_streaming(struct vb2_queue *vq);

/* --------------------------------------------------------------------------*/
/* Video devide File operations */

static int sprcam_open(struct file *filp);
static int sprcam_release(struct file *filp);

/* --------------------------------------------------------------------------*/
/* Video Device IO Controls */

static int sprcam_enum_input(struct file *filp, void *priv,
                             struct v4l2_input *input);
static int sprcam_g_input(struct file *filp, void *priv, unsigned int *i);
static int sprcam_s_input(struct file *filp, void *priv, unsigned int i);
static int sprcam_enum_fmt_vid_cap(struct file *filp, void *priv,
                                   struct v4l2_fmtdesc *fmt);
static int sprcam_try_fmt_vid_cap(struct file *filp, void *priv,
                                  struct v4l2_format *fmt);
static int sprcam_g_fmt_vid_cap(struct file *filp, void *priv,
                                struct v4l2_format *fmt);
static int sprcam_s_fmt_vid_cap(struct file *filp, void *priv,
                                struct v4l2_format *fmt);
static int sprcam_querycap(struct file *filp, void *priv,
                           struct v4l2_capability *cap);
static int sprcam_g_parm(struct file *filp, void *priv,
                         struct v4l2_streamparm *parm);
static int sprcam_s_parm(struct file *filp, void *priv,
                         struct v4l2_streamparm *parm);
static int sprcam_enum_framesizes(struct file *filp, void *priv,
                                  struct v4l2_frmsizeenum *sizes);
static int sprcam_enum_frameintervals(struct file *filp, void *priv,
                                      struct v4l2_frmivalenum *interval);

/*----------------------------------------------------------------------------*/
/* Platform device driver probing / removing */

static int sprcam_probe(struct platform_device *pdev);
static void sprcam_remove(struct platform_device *pdev);

/*****************************************************************************
 * Private Data
 *****************************************************************************/

static struct spr_camera *g_sprcam = NULL;  /* Device Driver instance */

static struct platform_driver sprcam_driver =
{
  .driver =
    {
      .name = SPRCAM_DRVNAME,
    },
  .probe = sprcam_probe,
  .remove_new = sprcam_remove,
};

static struct platform_device *my_pdev = NULL;

/*----------------------------------------------------------------------------*/
/* User frontend character device file operations */

#ifndef FAKE_CAMERA_IMAGE
static struct file_operations sprcam_cdevops = {
	.owner          = THIS_MODULE,
	.open           = sprcam_cdev_open,
	.release        = sprcam_cdev_release,
	.write          = sprcam_cdev_write,
#if 0
	.read           = sprcam_cdev_read,
	.unlocked_ioctl = sprcam_cdev_ioctl,
#endif
};
#endif

/*----------------------------------------------------------------------------*/
/* Video Driver related structures */

static const struct v4l2_file_operations sprcam_vdev_fops =
{
  .owner          = THIS_MODULE,
  .open           = sprcam_open,
  .release        = sprcam_release,
  .read           = vb2_fop_read,
  .poll           = vb2_fop_poll,
  .mmap           = vb2_fop_mmap,
  .unlocked_ioctl = video_ioctl2,
};

static const struct v4l2_pix_format sprcam_pix_format[] =
{
  { /* MJPEG VGA pixel format */
    .width        = VGA_WIDTH,
    .height       = VGA_HEIGHT,
    .pixelformat  = V4L2_PIX_FMT_MJPEG,
    .field        = V4L2_FIELD_NONE,
    .bytesperline = VGA_WIDTH * 2,
    .sizeimage    = VGA_WIDTH * VGA_HEIGHT * 2,
    .colorspace   = V4L2_COLORSPACE_SRGB,
    .flags        = V4L2_FMT_FLAG_COMPRESSED
  },
};
#define SPRCAM_PIXFMTNUM  \
        (sizeof(sprcam_pix_format)/sizeof(sprcam_pix_format[0]))

static const struct vb2_ops sprcam_vb2_ops =
{
  .queue_setup      = sprcam_vb2_queue_setup,
  .buf_queue        = sprcam_vb2_queue,
  .buf_prepare      = sprcam_vb2_prepare,
  .start_streaming  = sprcam_vb2_start_streaming,
  .stop_streaming   = sprcam_vb2_stop_streaming,
  .wait_prepare     = vb2_ops_wait_prepare,
  .wait_finish      = vb2_ops_wait_finish,
};

static const struct v4l2_ioctl_ops sprcam_ioctl_ops =
{
  .vidioc_enum_input          = sprcam_enum_input,
  .vidioc_g_input             = sprcam_g_input,
  .vidioc_s_input             = sprcam_s_input,
  .vidioc_enum_fmt_vid_cap    = sprcam_enum_fmt_vid_cap,
  .vidioc_try_fmt_vid_cap     = sprcam_try_fmt_vid_cap,
  .vidioc_g_fmt_vid_cap       = sprcam_g_fmt_vid_cap,
  .vidioc_s_fmt_vid_cap       = sprcam_s_fmt_vid_cap,
  .vidioc_querycap            = sprcam_querycap,
  .vidioc_g_parm              = sprcam_g_parm,
  .vidioc_s_parm              = sprcam_s_parm,
  .vidioc_enum_framesizes     = sprcam_enum_framesizes,
  .vidioc_enum_frameintervals = sprcam_enum_frameintervals,

  .vidioc_reqbufs             = vb2_ioctl_reqbufs,
  .vidioc_create_bufs         = vb2_ioctl_create_bufs,
  .vidioc_querybuf            = vb2_ioctl_querybuf,
  .vidioc_prepare_buf         = vb2_ioctl_prepare_buf,
  .vidioc_qbuf                = vb2_ioctl_qbuf,
  .vidioc_dqbuf               = vb2_ioctl_dqbuf,
  .vidioc_expbuf              = vb2_ioctl_expbuf,
  .vidioc_streamon            = vb2_ioctl_streamon,
  .vidioc_streamoff           = vb2_ioctl_streamoff,
  .vidioc_subscribe_event     = v4l2_ctrl_subscribe_event,
  .vidioc_unsubscribe_event   = v4l2_event_unsubscribe,
};

static const struct video_device sprcam_video_dev =
{
  .name         = SPRCAM_DRVNAME,
  .minor        = -1,
  .fops         = &sprcam_vdev_fops,
  .ioctl_ops    = &sprcam_ioctl_ops,
  .release      = video_device_release_empty,
  .device_caps  = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE |
                  V4L2_CAP_STREAMING,
};

#ifdef FAKE_CAMERA_IMAGE
struct dummy_data
{
  unsigned char *data;
  unsigned int len;
};

static struct dummy_data fake_data[] = {
  { pict1, pict1_len },
  { pict2, pict2_len },
  { pict3, pict3_len },
  { pict4, pict4_len },
  { pict5, pict5_len },
};
#endif

/*****************************************************************************
 * Private Functions
 *****************************************************************************/

static struct spr_vbuf_container *sprcam_get_nextvbuff(struct spr_camera *sprcam)
{
  struct spr_vbuf_container *container = NULL;

  if (sprcam->is_streaming && !list_empty(&sprcam->bufq))
    {
      container = list_entry(sprcam->bufq.next,
                             struct spr_vbuf_container,
                             queue);
      if (container)
        list_del(&container->queue);
    }

  return container;
}

#ifdef FAKE_CAMERA_IMAGE
static int sprcam_copy_vbuff(int seqno, struct spr_vbuf_container *container,
                             void *data, int len)
{
  void *frame_mem;

  container->vbuf.vb2_buf.timestamp = ktime_get_ns();
  container->vbuf.field = V4L2_FIELD_NONE;
  container->vbuf.sequence = seqno;

  frame_mem = vb2_plane_vaddr(&container->vbuf.vb2_buf, 0);
  memcpy(frame_mem, data, len);
  vb2_set_plane_payload(&container->vbuf.vb2_buf, 0, len);

  vb2_buffer_done(&container->vbuf.vb2_buf, VB2_BUF_STATE_DONE);

  return 0;
}

static int dummy_frame_event(void *arg)
{
  struct spr_camera *sprcam = (struct spr_camera *)arg;
  struct spr_vbuf_container *container;
  int fnum;

  while (!kthread_should_stop())
    {
      msleep(33);
      container = sprcam_get_nextvbuff(sprcam);
      if (container)
        {
          fnum = sprcam->seqno % 5;
          sprcam_copy_vbuff(sprcam->seqno++, container,
                            fake_data[fnum].data, fake_data[fnum].len);
        }
    }

  return 0;
}

#else /* FAKE_CAMERA_IMAGE */

static int sprcam_publish_vbuf_fromuser(int seqno,
                                        struct spr_vbuf_container *container,
                                        const char __user *data, size_t len)
{
  void *frame_mem;
  enum vb2_buffer_state state;

  container->vbuf.vb2_buf.timestamp = ktime_get_ns();
  container->vbuf.field = V4L2_FIELD_NONE;
  container->vbuf.sequence = seqno;

  frame_mem = vb2_plane_vaddr(&container->vbuf.vb2_buf, 0);
  if (copy_from_user(frame_mem, data, len))
    {
      printk(KERN_ALERT "sprcam: Error on copy_from_user\n");
      state = VB2_BUF_STATE_ERROR;
    }
  else
    {
      vb2_set_plane_payload(&container->vbuf.vb2_buf, 0, len);
      state = VB2_BUF_STATE_DONE;
    }

  vb2_buffer_done(&container->vbuf.vb2_buf, state);

  return state == VB2_BUF_STATE_DONE ? 0 : -EIO;
}

/* --------------------------------------------------------------------------*/
/* User I/F file Operations */

static int sprcam_cdev_open(struct inode *inode, struct file *file)
{
  if (g_sprcam->is_cdev_opened)
    {
      return -EBUSY;
    }

  g_sprcam->is_cdev_opened = true;

  return 0;
}

static int sprcam_cdev_release(struct inode *inode, struct file *file)
{
  g_sprcam->is_cdev_opened = false;
  return 0;
}

static ssize_t sprcam_cdev_write(struct file *file, const char __user *buff,
                                 size_t count, loff_t *pos)
{
  int ret = -ENODEV;
  struct spr_vbuf_container *container;

  container = sprcam_get_nextvbuff(g_sprcam);
  if (container)
    {
      ret = sprcam_publish_vbuf_fromuser(g_sprcam->seqno++, container,
                                         buff, count);
    }

  return ret;
}

/* --------------------------------------------------------------------------*/
/* Initialize User I/F Character Device */

static int sprcam_cdev_init(struct spr_camera *sprcam)
{
  printk(KERN_ALERT "sprcam: chdev init sprcam=%p\n", sprcam);

  sprcam->cdev_major = register_chrdev(0, SPRCAM_DRVNAME, &sprcam_cdevops);
  if (sprcam->cdev_major < 0)
    {
      printk(KERN_ALERT "Registering sprcam_cdev failed with %d\n",
             sprcam->cdev_major);
      return sprcam->cdev_major;
    }

  sprcam->cdev_id = MKDEV(sprcam->cdev_major, 0);
  sprcam->cdev_clas = class_create(SPRCAM_DRVNAME);
  if (sprcam->cdev_clas == NULL)
    {
      printk(KERN_ALERT "Cannnot create class structure\n");
      goto err_class;
    }

  if (device_create(sprcam->cdev_clas, NULL, sprcam->cdev_id, sprcam,
                    SPRCAM_DRVNAME) == NULL)
    {
      printk(KERN_ALERT "Cannnot create device structure\n");
      goto err_devcreate;
    }

  return 0;

err_devcreate:
  class_destroy(sprcam->cdev_clas);

err_class:
  unregister_chrdev(sprcam->cdev_major, SPRCAM_DRVNAME);

  return -ENOMEM;
}

static void sprcam_cdev_uninit(struct spr_camera *sprcam)
{
  device_destroy(sprcam->cdev_clas, sprcam->cdev_id);
  class_destroy(sprcam->cdev_clas);
  unregister_chrdev(sprcam->cdev_major, SPRCAM_DRVNAME);
}
#endif /* FAKE_CAMERA_IMAGE */

/* --------------------------------------------------------------------------*/
/* vb2 callback Operations */

static void sprcam_vb2_queue(struct vb2_buffer *vb)
{
  struct spr_camera *sprcam = vb2_get_drv_priv(vb->vb2_queue);
  struct spr_vbuf_container *container =
                  (struct spr_vbuf_container *)to_vb2_v4l2_buffer(vb);

  list_add_tail(&container->queue, &sprcam->bufq);
}

static int sprcam_vb2_prepare(struct vb2_buffer *vb)
{
  struct spr_camera *sprcam = vb2_get_drv_priv(vb->vb2_queue);

  if (vb2_plane_size(vb, 0) < sprcam->current_pixfmt.sizeimage)
    {
      printk(KERN_ALERT "Plane size too small (%lu < %u)\n",
                        vb2_plane_size(vb, 0),
                        sprcam->current_pixfmt.sizeimage);
      return -EINVAL;
    }

  vb2_set_plane_payload(vb, 0, sprcam->current_pixfmt.sizeimage);

  return 0;
}

static int sprcam_vb2_queue_setup(struct vb2_queue *vq,
          unsigned int *nbufs,
          unsigned int *num_planes, unsigned int sizes[],
          struct device *alloc_devs[])
{
  struct spr_camera *sprcam = vb2_get_drv_priv(vq);
  int size = sprcam->current_pixfmt.sizeimage;

  if (*num_planes)
    {
      return sizes[0] < size ? -EINVAL : 0;
    }

  *num_planes = 1;
  sizes[0] = size;

  return 0;
}

static int sprcam_vb2_start_streaming(struct vb2_queue *vq,
                                      unsigned int count)
{
  struct spr_camera *sprcam = vb2_get_drv_priv(vq);

  if (sprcam->is_streaming)
    {
      return -EBUSY;
    }

  sprcam->seqno = 0;
  cpu_latency_qos_add_request(&sprcam->qos, 33 /* ms */);

#ifdef FAKE_CAMERA_IMAGE
  sprcam->dummy_thread = kthread_run(dummy_frame_event, sprcam,
                                     "spr_cam_dummy_frameirq");
  if (IS_ERR(sprcam->dummy_thread))
    {
      printk(KERN_ERR "sprcam: Failed to create kernel thread");
      cpu_latency_qos_remove_request(&sprcam->qos);
      return -ENOMEM;
    }
#endif /* FAKE_CAMERA_IMAGE */
  
  printk(KERN_INFO "sprcam: Start streaming");
  sprcam->is_streaming = true;

  return 0;
}

static void sprcam_vb2_stop_streaming(struct vb2_queue *vq)
{
  struct spr_camera *sprcam = vb2_get_drv_priv(vq);
  struct spr_vbuf_container *container, *tmp;

  printk(KERN_INFO "sprcam: Stop streaming");
  sprcam->is_streaming = false;

#ifdef FAKE_CAMERA_IMAGE
  if (sprcam->dummy_thread)
    {
      kthread_stop(sprcam->dummy_thread);
      sprcam->dummy_thread = NULL;
    }
#endif

  cpu_latency_qos_remove_request(&sprcam->qos);

  list_for_each_entry_safe(container, tmp, &sprcam->bufq, queue)
    {
      list_del(&container->queue);
      vb2_buffer_done(&container->vbuf.vb2_buf, VB2_BUF_STATE_ERROR);
    }
}

/* --------------------------------------------------------------------------*/
/* Video devide File operations */

static int sprcam_open(struct file *filp)
{
  int ret;

  ret = v4l2_fh_open(filp);

  return ret;
}

static int sprcam_release(struct file *filp)
{
  _vb2_fop_release(filp, NULL);
  return 0;
}

/*----------------------------------------------------------------------------*/
/* Video Device IO Controls */

static int sprcam_enum_input(struct file *filp, void *priv,
    struct v4l2_input *input)
{
  if (input->index >= SPRCAM_PIXFMTNUM)
    {
      return -EINVAL;
    }

  input->type = V4L2_INPUT_TYPE_CAMERA;
  strscpy(input->name, SPRCAM_DRVNAME, sizeof(input->name));
  return 0;
}

static int sprcam_g_input(struct file *filp, void *priv, unsigned int *i)
{
  *i = SPRCAM_PIXFMTNUM - 1;
  return 0;
}

static int sprcam_s_input(struct file *filp, void *priv, unsigned int i)
{
  if (i >= SPRCAM_PIXFMTNUM)
    {
      return -EINVAL;
    }

  return 0;
}

static int sprcam_enum_fmt_vid_cap(struct file *filp, void *priv,
    struct v4l2_fmtdesc *fmt)
{
  if (fmt->index >= SPRCAM_PIXFMTNUM)
    {
      return -EINVAL;
    }

  fmt->pixelformat = sprcam_pix_format[fmt->index].pixelformat;
  return 0;
}

static int sprcam_try_fmt_vid_cap(struct file *filp, void *priv,
    struct v4l2_format *fmt)
{
  memcpy(&fmt->fmt.pix, sprcam_pix_format, sizeof(struct v4l2_pix_format));
  return 0;
}


static int sprcam_g_fmt_vid_cap(struct file *filp, void *priv,
    struct v4l2_format *fmt)
{
  struct spr_camera *sprcam = video_drvdata(filp);

  fmt->fmt.pix = sprcam->current_pixfmt;
  return 0;
}

static int sprcam_s_fmt_vid_cap(struct file *filp, void *priv,
    struct v4l2_format *fmt)
{
  if (fmt->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
      return -ENOTSUPP;
    }

  if (ISNT_EQ_FIELD(width) || ISNT_EQ_FIELD(height) ||
      ISNT_EQ_FIELD(pixelformat))
    {
      return -ENOTSUPP;
    }

  return 0;
}

static int sprcam_querycap(struct file *filp, void *priv,
    struct v4l2_capability *cap)
{
  strscpy(cap->driver, SPRCAM_DRVNAME, sizeof(cap->driver));
  strscpy(cap->card, SPRCAM_DRVNAME, sizeof(cap->card));
  strscpy(cap->bus_info, "platform:" SPRCAM_DRVNAME, sizeof(cap->bus_info));
  return 0;
}

/* G/S_PARM */

static int sprcam_g_parm(struct file *filp, void *priv,
    struct v4l2_streamparm *parm)
{
  if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
      return -EINVAL;
    }

  parm->parm.capture.readbuffers = 2;
  parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
  parm->parm.capture.timeperframe.numerator = 30;
  parm->parm.capture.timeperframe.denominator = 1;

  return 0;
}

static int sprcam_s_parm(struct file *filp, void *priv,
    struct v4l2_streamparm *parm)
{
  if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
    {
      return -EINVAL;
    }

  memset(&parm->parm, 0, sizeof(parm->parm));
  parm->parm.capture.readbuffers = 2;
  parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
  parm->parm.capture.timeperframe.numerator = 30;
  parm->parm.capture.timeperframe.denominator = 1;

  return 0;
}

static int sprcam_enum_framesizes(struct file *filp, void *priv,
    struct v4l2_frmsizeenum *sizes)
{
  if (sizes->index >= SPRCAM_PIXFMTNUM)
    {
      return -EINVAL;
    }

  if (sizes->pixel_format != sprcam_pix_format[0].pixelformat)
    {
      return -EINVAL;
    }

  sizes->type = V4L2_FRMSIZE_TYPE_DISCRETE;
  sizes->discrete.width = VGA_WIDTH;
  sizes->discrete.height = VGA_HEIGHT;

  return 0;
}

static int sprcam_enum_frameintervals(struct file *filp, void *priv,
    struct v4l2_frmivalenum *interval)
{
  if (interval->index >= SPRCAM_PIXFMTNUM ||
      interval->pixel_format != sprcam_pix_format[0].pixelformat ||
      interval->width != VGA_WIDTH || interval->height != VGA_HEIGHT)
    {
      return -EINVAL;
    }

  interval->type                 = V4L2_FRMIVAL_TYPE_DISCRETE;
  interval->discrete.numerator   = 30;
  interval->discrete.denominator = 1;
  return 0;
}

/*----------------------------------------------------------------------------*/
/* Initialize as video driver */

static int sprcam_videodrv_init(struct platform_device *pdev,
                                struct spr_camera *sprcam)
{
  int ret;
  struct vb2_queue *vq;

  memcpy(&sprcam->current_pixfmt, &sprcam_pix_format[0],
         sizeof(struct v4l2_pix_format));

  sprcam->pdev = pdev;
  mutex_init(&sprcam->lock);
  INIT_LIST_HEAD(&sprcam->bufq);
  ret = v4l2_device_register(&pdev->dev, &sprcam->v4dev);
  if (ret != 0)
    {
      printk(KERN_ERR "sprcam: ERROR v4l2_device_register\n");
      return ret;
    }

  ret = v4l2_ctrl_handler_init(&sprcam->v4hdlr, 10);
  if (ret != 0)
    {
      printk(KERN_ERR "sprcam: ERROR v4l2_ctrl_handler_init\n");
      goto err_devreg;
    }
  sprcam->v4dev.ctrl_handler = &sprcam->v4hdlr;

  vq = &sprcam->vbq;
  vq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  vq->io_modes = VB2_MMAP | VB2_USERPTR | VB2_DMABUF | VB2_READ;
  vq->drv_priv = sprcam;
  vq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
  vq->buf_struct_size = sizeof(struct spr_vbuf_container);
  vq->dev = sprcam->v4dev.dev;

  vq->ops = &sprcam_vb2_ops;
  vq->mem_ops = &vb2_vmalloc_memops;
  vq->lock = &sprcam->lock;

  ret = vb2_queue_init(vq);
  if (ret != 0)
    {
      printk(KERN_ERR "sprcam: ERROR Video Buffer Queue Init\n");
      goto err_ctrl_handler;
    }

  memcpy(&sprcam->vdev, &sprcam_video_dev, sizeof(struct video_device));
  sprcam->vdev.v4l2_dev = &sprcam->v4dev;
  sprcam->vdev.lock = &sprcam->lock;
  sprcam->vdev.queue = vq;
  video_set_drvdata(&sprcam->vdev, sprcam);
  ret = video_register_device(&sprcam->vdev, VFL_TYPE_VIDEO, -1);
  if (ret != 0)
    {
      printk(KERN_ERR "sprcam: ERROR video_register_device\n");
      goto err_ctrl_handler;
    }

  /* Initialize success */

  return 0;

err_ctrl_handler:
  v4l2_ctrl_handler_free(&sprcam->v4hdlr);

err_devreg:
  v4l2_device_unregister(&sprcam->v4dev);

  return ret;
}

static void sprcam_videodrv_uninit(struct spr_camera *sprcam)
{
  printk(KERN_DEBUG "sprcam: Bye Bye\n");
  video_unregister_device(&sprcam->vdev);
  v4l2_ctrl_handler_free(&sprcam->v4hdlr);
  v4l2_device_unregister(&sprcam->v4dev);
}

/*----------------------------------------------------------------------------*/
/* Platform device driver probing / removing */

static int sprcam_probe(struct platform_device *pdev)
{
  int ret;

  printk(KERN_DEBUG "sprcam: Proved.\n");

  g_sprcam = kzalloc(sizeof(struct spr_camera), GFP_KERNEL);
  if (g_sprcam == NULL)
    {
      printk(KERN_ERR "sprcam: NOMEM for spr_camera\n");
      return -ENOMEM;
    }

  ret = sprcam_videodrv_init(pdev, g_sprcam);
  if (ret != 0)
    {
      kfree(g_sprcam);
      g_sprcam = NULL;
      return ret;
    }

#ifndef FAKE_CAMERA_IMAGE
  ret = sprcam_cdev_init(g_sprcam);
  if (ret != 0)
    {
      sprcam_videodrv_uninit(g_sprcam);
      kfree(g_sprcam);
      g_sprcam = NULL;
    }
#endif

  return ret;
}

static void sprcam_remove(struct platform_device *pdev)
{
  struct spr_camera *sprcam = dev_get_drvdata(&pdev->dev);

#ifndef FAKE_CAMERA_IMAGE
  sprcam_cdev_uninit(sprcam);
#endif
  sprcam_videodrv_uninit(sprcam);

  kfree(sprcam);
  g_sprcam = NULL;  /* Just in case */
}

/*----------------------------------------------------------------------------*/
/* Initialization / Remove Functions */

static int __init sprcam_init(void)
{
  int ret;
  my_pdev = NULL;
  printk(KERN_ALERT "sprcam: Initialize: %p\n", &sprcam_driver);
  ret = platform_driver_register(&sprcam_driver);
  if (ret >= 0)
    {
      my_pdev = platform_device_register_simple(SPRCAM_DRVNAME, 0, NULL, 0);
      if (!my_pdev)
        {
          printk(KERN_ALERT "sprcam: ERROR!!!\n");
          return -ENOMEM;
        }
      printk(KERN_ALERT "sprcam: dev=%p\n", my_pdev);
    }
  else
    {
      printk(KERN_ALERT "sprcam: 1:ERROR!!!\n");
      return -EIO;
    }

  return 0;
}

static void __exit sprcam_exit(void)
{
  printk(KERN_ALERT "sprcam: Exit\n");
  if (my_pdev) platform_device_unregister(my_pdev);
  platform_driver_unregister(&sprcam_driver);
  my_pdev = NULL;
}

module_init(sprcam_init)
module_exit(sprcam_exit)

MODULE_AUTHOR("Takayoshi Koizumi <takayoshi.koizumi@sony.com>");
MODULE_DESCRIPTION("Camera image injection driver for SPRESENSE");
MODULE_LICENSE("GPL");
