#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>
#include <linux/spinlock.h>
#include <linux/kref.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-v4l2.h>
#include <media/videobuf2-vmalloc.h>

#define VENDOR_ID  0x2ce3
#define PRODUCT_ID 0x3828

#define INTERFACE_NUM 1
#define EP_IN  0x81
#define EP_OUT 0x01
#define CONNECT_CMD_SIZE 5

#define BUFFER_SIZE (64*1024)  // Buffer for USB transfers
#define MAX_FRAMES  5          // Number of frames to keep in ring buffer
#define MIN_JPEG_SIZE 1024
#define CAMERA_WIDTH 640
#define CAMERA_HEIGHT 480

struct camera_buffer {
    struct vb2_v4l2_buffer vb;
    struct list_head list;
};

// Simple frame container
struct camera_frame {
    unsigned char *data;
    size_t size;
    bool ready;
};

struct camera_dev {
    struct usb_device *udev;
    struct usb_interface *interface;
    struct urb *urb;
    unsigned char *urb_buffer;
    
    // Connection command
    unsigned char connect_cmd[CONNECT_CMD_SIZE];
    
    // Frame ring buffer
    struct camera_frame frames[MAX_FRAMES];
    int current_frame;
    int read_frame;
    
    // Character device
    struct cdev cdev;
    dev_t dev_num;
    struct class *class;
    struct device *device;
    struct mutex io_mutex;
    bool device_open;
    
    // Read position
    size_t read_pos;
    
    // State
    bool streaming;
    
    // Wait queue for blocking reads
    wait_queue_head_t read_wait;

    /* V4L2 capture interface. */
    struct v4l2_device v4l2_dev;
    struct video_device *vdev;
    struct vb2_queue vb2_queue;
    struct mutex v4l2_lock;
    spinlock_t vb2_lock;
    struct list_head vb2_buffers;
    unsigned int sequence;
    unsigned char *jpeg_data;
    size_t jpeg_size;
    bool jpeg_active;
    bool jpeg_previous_ff;
    bool vb2_queue_initialized;
    bool v4l2_streaming;
    bool disconnected;
    struct kref refcount;
};

static int major_num;

// Forward declarations
static int camera_open(struct inode *inode, struct file *file);
static int camera_release(struct inode *inode, struct file *file);
static ssize_t camera_read(struct file *file, char __user *buf, size_t count, loff_t *pos);
static void camera_device_release(struct kref *refcount);
static int camera_v4l2_open(struct file *file);

static struct file_operations camera_fops = {
    .owner = THIS_MODULE,
    .open = camera_open,
    .release = camera_release,
    .read = camera_read,
};

static struct camera_dev *camera_from_queue(struct vb2_queue *queue)
{
    return container_of(queue, struct camera_dev, vb2_queue);
}

static int camera_vb2_queue_setup(struct vb2_queue *queue,
                                 unsigned int *num_buffers,
                                 unsigned int *num_planes,
                                 unsigned int sizes[],
                                 struct device *alloc_devs[])
{
    if (*num_planes) {
        if (sizes[0] < BUFFER_SIZE)
            return -EINVAL;
        return 0;
    }

    *num_planes = 1;
    sizes[0] = BUFFER_SIZE;
    if (*num_buffers < 2)
        *num_buffers = 2;
    return 0;
}

static int camera_vb2_buffer_prepare(struct vb2_buffer *buffer)
{
    if (vb2_plane_size(buffer, 0) < BUFFER_SIZE)
        return -EINVAL;

    vb2_set_plane_payload(buffer, 0, 0);
    return 0;
}

static void camera_vb2_buffer_queue(struct vb2_buffer *buffer)
{
    struct camera_dev *dev = camera_from_queue(buffer->vb2_queue);
    struct camera_buffer *camera_buffer =
        container_of(to_vb2_v4l2_buffer(buffer), struct camera_buffer, vb);
    unsigned long flags;

    spin_lock_irqsave(&dev->vb2_lock, flags);
    list_add_tail(&camera_buffer->list, &dev->vb2_buffers);
    spin_unlock_irqrestore(&dev->vb2_lock, flags);
}

static int camera_vb2_start_streaming(struct vb2_queue *queue,
                                      unsigned int count)
{
    struct camera_dev *dev = camera_from_queue(queue);
    unsigned long flags;
    bool empty;

    spin_lock_irqsave(&dev->vb2_lock, flags);
    empty = list_empty(&dev->vb2_buffers);
    if (!empty) {
        dev->sequence = 0;
        dev->v4l2_streaming = true;
    }
    spin_unlock_irqrestore(&dev->vb2_lock, flags);
    pr_info("Camera: V4L2 stream start, queued buffers=%u\n", count);
    return empty ? -ENOBUFS : 0;
}

static void camera_vb2_stop_streaming(struct vb2_queue *queue)
{
    struct camera_dev *dev = camera_from_queue(queue);
    struct camera_buffer *buffer, *next;
    LIST_HEAD(returned_buffers);
    unsigned long flags;

    spin_lock_irqsave(&dev->vb2_lock, flags);
    dev->v4l2_streaming = false;
    list_for_each_entry_safe(buffer, next, &dev->vb2_buffers, list) {
        list_move_tail(&buffer->list, &returned_buffers);
    }
    spin_unlock_irqrestore(&dev->vb2_lock, flags);

    list_for_each_entry_safe(buffer, next, &returned_buffers, list) {
        list_del(&buffer->list);
        vb2_buffer_done(&buffer->vb.vb2_buf, VB2_BUF_STATE_ERROR);
    }
}

static const struct vb2_ops camera_vb2_ops = {
    .queue_setup = camera_vb2_queue_setup,
    .buf_prepare = camera_vb2_buffer_prepare,
    .buf_queue = camera_vb2_buffer_queue,
    .start_streaming = camera_vb2_start_streaming,
    .stop_streaming = camera_vb2_stop_streaming,
    .wait_prepare = vb2_ops_wait_prepare,
    .wait_finish = vb2_ops_wait_finish,
};

static int camera_querycap(struct file *file, void *private,
                           struct v4l2_capability *capability)
{
    struct camera_dev *dev = video_drvdata(file);

    strscpy(capability->driver, "supercamera", sizeof(capability->driver));
    strscpy(capability->card, "Geek szitman supercamera",
            sizeof(capability->card));
    snprintf(capability->bus_info, sizeof(capability->bus_info),
             "usb-%s", dev_name(&dev->interface->dev));
    capability->device_caps = V4L2_CAP_VIDEO_CAPTURE |
                              V4L2_CAP_STREAMING |
                              V4L2_CAP_READWRITE;
    capability->capabilities = capability->device_caps |
                               V4L2_CAP_DEVICE_CAPS;
    return 0;
}

static int camera_enum_format(struct file *file, void *private,
                              struct v4l2_fmtdesc *format)
{
    if (format->index != 0)
        return -EINVAL;
    format->pixelformat = V4L2_PIX_FMT_MJPEG;
    strscpy(format->description, "Motion-JPEG",
            sizeof(format->description));
    return 0;
}

static int camera_enum_framesizes(struct file *file, void *private,
                                  struct v4l2_frmsizeenum *size)
{
    if (size->index != 0 || size->pixel_format != V4L2_PIX_FMT_MJPEG)
        return -EINVAL;

    size->type = V4L2_FRMSIZE_TYPE_DISCRETE;
    size->discrete.width = CAMERA_WIDTH;
    size->discrete.height = CAMERA_HEIGHT;
    return 0;
}

static int camera_enum_frameintervals(struct file *file, void *private,
                                      struct v4l2_frmivalenum *interval)
{
    if (interval->index != 0 ||
        interval->pixel_format != V4L2_PIX_FMT_MJPEG ||
        interval->width != CAMERA_WIDTH ||
        interval->height != CAMERA_HEIGHT)
        return -EINVAL;

    interval->type = V4L2_FRMIVAL_TYPE_DISCRETE;
    interval->discrete.numerator = 1;
    interval->discrete.denominator = 20;
    return 0;
}

static int camera_get_streamparm(struct file *file, void *private,
                                 struct v4l2_streamparm *parm)
{
    if (parm->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;

    memset(&parm->parm.capture, 0, sizeof(parm->parm.capture));
    parm->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
    parm->parm.capture.timeperframe.numerator = 1;
    parm->parm.capture.timeperframe.denominator = 20;
    return 0;
}

static int camera_set_streamparm(struct file *file, void *private,
                                 struct v4l2_streamparm *parm)
{
    return camera_get_streamparm(file, private, parm);
}

static void camera_fill_format(struct v4l2_pix_format *pixel_format)
{
    pixel_format->width = CAMERA_WIDTH;
    pixel_format->height = CAMERA_HEIGHT;
    pixel_format->pixelformat = V4L2_PIX_FMT_MJPEG;
    pixel_format->field = V4L2_FIELD_NONE;
    pixel_format->bytesperline = 0;
    pixel_format->sizeimage = BUFFER_SIZE;
    pixel_format->colorspace = V4L2_COLORSPACE_JPEG;
    pixel_format->priv = 0;
}

static int camera_get_format(struct file *file, void *private,
                             struct v4l2_format *format)
{
    if (format->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;
    camera_fill_format(&format->fmt.pix);
    return 0;
}

static int camera_try_format(struct file *file, void *private,
                             struct v4l2_format *format)
{
    if (format->type != V4L2_BUF_TYPE_VIDEO_CAPTURE)
        return -EINVAL;
    camera_fill_format(&format->fmt.pix);
    return 0;
}

static int camera_set_format(struct file *file, void *private,
                             struct v4l2_format *format)
{
    struct camera_dev *dev = video_drvdata(file);

    if (vb2_is_busy(&dev->vb2_queue))
        return -EBUSY;
    return camera_try_format(file, private, format);
}

static const struct v4l2_ioctl_ops camera_ioctl_ops = {
    .vidioc_querycap = camera_querycap,
    .vidioc_enum_fmt_vid_cap = camera_enum_format,
    .vidioc_enum_framesizes = camera_enum_framesizes,
    .vidioc_enum_frameintervals = camera_enum_frameintervals,
    .vidioc_g_parm = camera_get_streamparm,
    .vidioc_s_parm = camera_set_streamparm,
    .vidioc_g_fmt_vid_cap = camera_get_format,
    .vidioc_try_fmt_vid_cap = camera_try_format,
    .vidioc_s_fmt_vid_cap = camera_set_format,
    .vidioc_reqbufs = vb2_ioctl_reqbufs,
    .vidioc_create_bufs = vb2_ioctl_create_bufs,
    .vidioc_prepare_buf = vb2_ioctl_prepare_buf,
    .vidioc_querybuf = vb2_ioctl_querybuf,
    .vidioc_qbuf = vb2_ioctl_qbuf,
    .vidioc_dqbuf = vb2_ioctl_dqbuf,
    .vidioc_streamon = vb2_ioctl_streamon,
    .vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_file_operations camera_v4l2_fops = {
    .owner = THIS_MODULE,
    .open = camera_v4l2_open,
    .release = vb2_fop_release,
    .read = vb2_fop_read,
    .poll = vb2_fop_poll,
    .unlocked_ioctl = video_ioctl2,
    .mmap = vb2_fop_mmap,
};

static void camera_deliver_jpeg(struct camera_dev *dev)
{
    struct camera_buffer *buffer;
    void *destination;
    bool queued;
    size_t jpeg_size = dev->jpeg_size;
    unsigned long flags;

    spin_lock_irqsave(&dev->vb2_lock, flags);
    if (!dev->v4l2_streaming || list_empty(&dev->vb2_buffers)) {
        queued = !list_empty(&dev->vb2_buffers);
        spin_unlock_irqrestore(&dev->vb2_lock, flags);
        pr_info_ratelimited("Camera: dropping JPEG, streaming=%d queued=%d\n",
                            dev->v4l2_streaming, queued);
        return;
    }

    buffer = list_first_entry(&dev->vb2_buffers, struct camera_buffer, list);
    list_del(&buffer->list);
    buffer->vb.sequence = dev->sequence++;
    buffer->vb.vb2_buf.timestamp = ktime_get_ns();
    buffer->vb.field = V4L2_FIELD_NONE;

    destination = vb2_plane_vaddr(&buffer->vb.vb2_buf, 0);
    if (!destination || jpeg_size > vb2_plane_size(&buffer->vb.vb2_buf, 0)) {
        vb2_buffer_done(&buffer->vb.vb2_buf, VB2_BUF_STATE_ERROR);
        spin_unlock_irqrestore(&dev->vb2_lock, flags);
        return;
    }

    memcpy(destination, dev->jpeg_data, jpeg_size);
    vb2_set_plane_payload(&buffer->vb.vb2_buf, 0, jpeg_size);
    vb2_buffer_done(&buffer->vb.vb2_buf, VB2_BUF_STATE_DONE);
    spin_unlock_irqrestore(&dev->vb2_lock, flags);
    pr_info_ratelimited("Camera: delivered V4L2 JPEG, size=%zu\n", jpeg_size);
}

static void camera_process_jpeg(struct camera_dev *dev,
                                const unsigned char *data, size_t length)
{
    size_t i;

    for (i = 0; i < length; i++) {
        unsigned char byte = data[i];

        if (!dev->jpeg_active) {
            if (dev->jpeg_previous_ff && byte == 0xd8) {
                dev->jpeg_data[0] = 0xff;
                dev->jpeg_data[1] = 0xd8;
                dev->jpeg_size = 2;
                dev->jpeg_active = true;
            }
            dev->jpeg_previous_ff = byte == 0xff;
            continue;
        }

        if (dev->jpeg_size == BUFFER_SIZE) {
            dev->jpeg_active = false;
            dev->jpeg_size = 0;
            dev->jpeg_previous_ff = byte == 0xff;
            continue;
        }

        dev->jpeg_data[dev->jpeg_size++] = byte;
        if (dev->jpeg_previous_ff && byte == 0xd9) {
            if (dev->jpeg_size >= MIN_JPEG_SIZE) {
                pr_info_ratelimited("Camera: assembled JPEG, size=%zu\n",
                                    dev->jpeg_size);
                camera_deliver_jpeg(dev);
            } else {
                pr_info_ratelimited("Camera: ignoring short JPEG, size=%zu\n",
                                    dev->jpeg_size);
            }
            dev->jpeg_active = false;
            dev->jpeg_size = 0;
            dev->jpeg_previous_ff = false;
        } else {
            dev->jpeg_previous_ff = byte == 0xff;
        }
    }
}

static void camera_video_release(struct video_device *vdev)
{
    struct camera_dev *dev = video_get_drvdata(vdev);

    video_device_release(vdev);
    kref_put(&dev->refcount, camera_device_release);
}

static int camera_register_v4l2(struct camera_dev *dev)
{
    struct vb2_queue *queue = &dev->vb2_queue;
    int result;

    result = v4l2_device_register(&dev->interface->dev, &dev->v4l2_dev);
    if (result)
        return result;

    queue->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    queue->io_modes = VB2_MMAP | VB2_READ;
    queue->drv_priv = dev;
    queue->buf_struct_size = sizeof(struct camera_buffer);
    queue->ops = &camera_vb2_ops;
    queue->mem_ops = &vb2_vmalloc_memops;
    queue->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
    queue->lock = &dev->v4l2_lock;
    queue->dev = &dev->interface->dev;

    result = vb2_queue_init(queue);
    if (result)
        goto error_v4l2_device;
    dev->vb2_queue_initialized = true;

    dev->vdev = video_device_alloc();
    if (!dev->vdev) {
        result = -ENOMEM;
        goto error_v4l2_device;
    }

    strscpy(dev->vdev->name, "Geek szitman supercamera",
            sizeof(dev->vdev->name));
    dev->vdev->v4l2_dev = &dev->v4l2_dev;
    dev->vdev->fops = &camera_v4l2_fops;
    dev->vdev->ioctl_ops = &camera_ioctl_ops;
    dev->vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE |
                             V4L2_CAP_STREAMING |
                             V4L2_CAP_READWRITE;
    dev->vdev->release = camera_video_release;
    dev->vdev->lock = &dev->v4l2_lock;
    dev->vdev->queue = queue;
    video_set_drvdata(dev->vdev, dev);

    /* Keep the queue and v4l2_device alive through the core's final release,
     * including opens racing with unregister and failed opens.
     */
    kref_get(&dev->refcount);
    result = video_register_device(dev->vdev, VFL_TYPE_VIDEO, -1);
    if (result) {
        camera_video_release(dev->vdev);
        dev->vdev = NULL;
        goto error_v4l2_device;
    }

    printk(KERN_INFO "Camera: V4L2 device registered (/dev/video%d)",
           dev->vdev->num);
    return 0;

error_v4l2_device:
    if (dev->vb2_queue_initialized) {
        vb2_queue_release(queue);
        dev->vb2_queue_initialized = false;
    }
    v4l2_device_unregister(&dev->v4l2_dev);
    return result;
}

static void camera_unregister_v4l2(struct camera_dev *dev)
{
    if (dev->vdev) {
        vb2_video_unregister_device(dev->vdev);
        dev->vdev = NULL;
        dev->vb2_queue_initialized = false;
    }
    v4l2_device_unregister(&dev->v4l2_dev);
}

static void camera_device_release(struct kref *refcount)
{
    struct camera_dev *dev = container_of(refcount, struct camera_dev,
                                           refcount);
    int i;

    for (i = 0; i < MAX_FRAMES; i++)
        kfree(dev->frames[i].data);
    kfree(dev->jpeg_data);
    usb_put_dev(dev->udev);
    kfree(dev);
}

static int camera_v4l2_open(struct file *file)
{
    struct camera_dev *dev = video_drvdata(file);

    if (READ_ONCE(dev->disconnected))
        return -ENODEV;
    return v4l2_fh_open(file);
}

// Process received data by adding it to the current frame
static void process_data(struct camera_dev *dev, unsigned char *data, int length)
{
    struct camera_frame *frame = &dev->frames[dev->current_frame];
    static int packet_count = 0;

    // Custom header appears to be 12 bytes
    const int HEADER_SIZE = 12;

    // Debug first few packets
    if (packet_count < 10) {
        printk(KERN_INFO "Camera: Packet %d (%d bytes): ", packet_count, length);
        for (int i = 0; i < min(16, length); i++) {
            printk(KERN_CONT "%02x ", data[i]);
        }
        printk(KERN_CONT "\n");
        packet_count++;
    }

    // Relaxed header check - only check first 3 bytes
    if (length >= 3 && data[0] == 0xaa && data[1] == 0xbb && data[2] == 0x07) {
        // Valid packet header - using consistent 12-byte header

        // Allocate frame buffer if needed
        if (!frame->data) {
            frame->data = kmalloc(BUFFER_SIZE, GFP_ATOMIC);
            if (!frame->data) {
                printk(KERN_ERR "Camera: Failed to allocate frame buffer");
                return;
            }
            frame->size = 0;
            frame->ready = false;
        }

        // Skip header and append payload data to current frame
        if (length > HEADER_SIZE) {
            int payload_size = length - HEADER_SIZE;

            camera_process_jpeg(dev, data + HEADER_SIZE, payload_size);

            // Check if we have enough space
            if (frame->size + payload_size <= BUFFER_SIZE) {
                memcpy(frame->data + frame->size, data + HEADER_SIZE, payload_size);
                frame->size += payload_size;
            } else {
                // Current frame is full - finalize it and move to next
                frame->ready = true;
                printk(KERN_INFO "Camera: Frame %d ready (buffer full), size %zu", 
                      dev->current_frame, frame->size);
                wake_up_interruptible(&dev->read_wait);

                // Move to next frame
                dev->current_frame = (dev->current_frame + 1) % MAX_FRAMES;
                frame = &dev->frames[dev->current_frame];

                // Initialize new frame if needed
                if (!frame->data) {
                    frame->data = kmalloc(BUFFER_SIZE, GFP_ATOMIC);
                    if (!frame->data) {
                        printk(KERN_ERR "Camera: Failed to allocate frame buffer");
                        return;
                    }
                }
                frame->size = 0;
                frame->ready = false;

                // Add this payload to the new frame
                memcpy(frame->data, data + HEADER_SIZE, payload_size);
                frame->size = payload_size;
            }

            // Look for JPEG EOI marker to detect complete image
            for (int i = 0; i < payload_size - 1; i++) {
                if (data[HEADER_SIZE + i] == 0xFF && data[HEADER_SIZE + i + 1] == 0xD9) {
                    // Found JPEG EOI marker
                    printk(KERN_INFO "Camera: Found EOI at packet offset %d, frame size %zu", 
                           HEADER_SIZE + i, frame->size);
                }
            }

            // Make finished frame available after accumulating enough data
            // Reduced threshold to 64KB which seems enough based on your results
            if (frame->size >= 64000 && !frame->ready) {
                frame->ready = true;
                printk(KERN_INFO "Camera: Frame %d ready (size threshold), size %zu", 
                       dev->current_frame, frame->size);
                wake_up_interruptible(&dev->read_wait);

                // Move to next frame
                dev->current_frame = (dev->current_frame + 1) % MAX_FRAMES;
                frame = &dev->frames[dev->current_frame];

                // Initialize next frame
                if (!frame->data) {
                    frame->data = kmalloc(BUFFER_SIZE, GFP_ATOMIC);
                    if (!frame->data) {
                        printk(KERN_ERR "Camera: Failed to allocate frame buffer");
                        return;
                    }
                }
                frame->size = 0;
                frame->ready = false;
            }
        }
    } else {
        // Still log invalid headers but continue processing the data
        // This approach ensures we don't miss data even if headers change
        if (length >= 4) {
            printk(KERN_INFO "Camera: Invalid packet header: %02x %02x %02x %02x",
                   data[0], data[1], data[2], data[3]);
        }

        // Try to process data anyway - it might contain valid image data
        if (frame->data) {
            camera_process_jpeg(dev, data, length);
            // Check if we have enough space
            if (frame->size + length <= BUFFER_SIZE) {
                memcpy(frame->data + frame->size, data, length);
                frame->size += length;
            } else {
                // Current frame is full - finalize it and move to next
                frame->ready = true;
                printk(KERN_INFO "Camera: Frame %d ready (buffer full from invalid packet), size %zu", 
                      dev->current_frame, frame->size);
                wake_up_interruptible(&dev->read_wait);

                // Move to next frame
                dev->current_frame = (dev->current_frame + 1) % MAX_FRAMES;
                frame = &dev->frames[dev->current_frame];

                // Initialize new frame if needed
                if (!frame->data) {
                    frame->data = kmalloc(BUFFER_SIZE, GFP_ATOMIC);
                    if (!frame->data) {
                        printk(KERN_ERR "Camera: Failed to allocate frame buffer");
                        return;
                    }
                }
                frame->size = 0;
                frame->ready = false;

                // Add this data to the new frame
                memcpy(frame->data, data, length);
                frame->size = length;
            }
        }
    }
}
// Send command to camera
static int send_command(struct camera_dev *dev, unsigned char *data, int len)
{
    int actual_length;
    int result;
    
    result = usb_bulk_msg(dev->udev, usb_sndbulkpipe(dev->udev, EP_OUT),
                         data, len, &actual_length, 5000);
    
    if (result) {
        printk(KERN_ERR "Camera: Failed to send command: %d", result);
        return result;
    }
    
    if (actual_length != len) {
        printk(KERN_WARNING "Camera: Short write (%d / %d)", actual_length, len);
    }
    
    return 0;
}

static void camera_urb_complete(struct urb *urb)
{
    struct camera_dev *dev = urb->context;
    int status = urb->status;

    // Handle URB status
    if (status) {
        if (!(status == -ENOENT || status == -ECONNRESET || status == -ESHUTDOWN)) {
            printk(KERN_ERR "Camera: URB error %d", status);
        }
        return;
    }

    // Process the data if we received any
    if (urb->actual_length > 0) {
        // Change this line - call process_uvc_data instead of process_data
        process_data(dev, dev->urb_buffer, urb->actual_length);
    }

    // Resubmit the URB if still streaming
    if (dev->streaming) {
        status = usb_submit_urb(urb, GFP_ATOMIC);
        if (status) {
            printk(KERN_ERR "Camera: Failed to resubmit URB: %d", status);
        }
    }
}
// Start streaming
static int start_streaming(struct camera_dev *dev)
{
    int result;
    
    // Set alt setting to 1
    result = usb_set_interface(dev->udev, INTERFACE_NUM, 1);
    if (result) {
        printk(KERN_ERR "Camera: Failed to set alt setting: %d", result);
        return result;
    }
    
    // Send connect command
    result = send_command(dev, dev->connect_cmd, CONNECT_CMD_SIZE);
    if (result) {
        return result;
    }
    
    // Allocate URB
    dev->urb = usb_alloc_urb(0, GFP_KERNEL);
    if (!dev->urb) {
        return -ENOMEM;
    }
    
    // Allocate buffer
    dev->urb_buffer = kmalloc(BUFFER_SIZE, GFP_KERNEL);
    if (!dev->urb_buffer) {
        usb_free_urb(dev->urb);
        dev->urb = NULL;
        return -ENOMEM;
    }
    
    // Fill URB
    usb_fill_bulk_urb(dev->urb, dev->udev, usb_rcvbulkpipe(dev->udev, EP_IN),
                     dev->urb_buffer, BUFFER_SIZE, camera_urb_complete, dev);

    /* The completion callback may run as soon as the URB is submitted. */
    dev->streaming = true;
    
    // Submit URB
    result = usb_submit_urb(dev->urb, GFP_KERNEL);
    if (result) {
        dev->streaming = false;
        printk(KERN_ERR "Camera: Failed to submit URB: %d", result);
        kfree(dev->urb_buffer);
        usb_free_urb(dev->urb);
        dev->urb = NULL;
        dev->urb_buffer = NULL;
        return result;
    }
    
    return 0;
}

// Stop streaming
static void stop_streaming(struct camera_dev *dev)
{
    if (!dev->streaming) {
        return;
    }

    dev->streaming = false;
    
    // Kill URB
    if (dev->urb) {
        usb_kill_urb(dev->urb);
        usb_free_urb(dev->urb);
        dev->urb = NULL;
    }
    
    // Free buffer
    kfree(dev->urb_buffer);
    dev->urb_buffer = NULL;
    
    // Reset alt setting
    usb_set_interface(dev->udev, INTERFACE_NUM, 0);
    
}

// Device open
static int camera_open(struct inode *inode, struct file *file)
{
    struct camera_dev *dev = container_of(inode->i_cdev, struct camera_dev, cdev);
    
    file->private_data = dev;
    
    if (mutex_lock_interruptible(&dev->io_mutex)) {
        return -ERESTARTSYS;
    }

    if (dev->disconnected) {
        mutex_unlock(&dev->io_mutex);
        return -ENODEV;
    }
    
    if (dev->device_open) {
        mutex_unlock(&dev->io_mutex);
        return -EBUSY;
    }
    
    dev->device_open = true;
    dev->read_pos = 0;
    dev->read_frame = 0;
    kref_get(&dev->refcount);
    
    mutex_unlock(&dev->io_mutex);
    
    return 0;
}

// Device release
static int camera_release(struct inode *inode, struct file *file)
{
    struct camera_dev *dev = file->private_data;
    
    mutex_lock(&dev->io_mutex);
    dev->device_open = false;
    mutex_unlock(&dev->io_mutex);
    kref_put(&dev->refcount, camera_device_release);
    
    return 0;
}

// Device read
static ssize_t camera_read(struct file *file, char __user *buf, size_t count, loff_t *pos)
{
    struct camera_dev *dev = file->private_data;
    ssize_t bytes_read = 0;
    
    if (mutex_lock_interruptible(&dev->io_mutex)) {
        return -ERESTARTSYS;
    }

    if (dev->disconnected) {
        mutex_unlock(&dev->io_mutex);
        return -ENODEV;
    }
    
    // Find ready frame
    int frame_idx = dev->read_frame;
    int checked = 0;
    
    while (checked < MAX_FRAMES) {
        if (dev->frames[frame_idx].ready) {
            break;
        }
        frame_idx = (frame_idx + 1) % MAX_FRAMES;
        checked++;
    }
    
    if (checked == MAX_FRAMES) {
        // No ready frames - wait or return
        mutex_unlock(&dev->io_mutex);
        
        if (file->f_flags & O_NONBLOCK) {
            return -EAGAIN;
        }
        
        if (wait_event_interruptible(dev->read_wait,
                                    dev->disconnected ||
                                    dev->frames[dev->read_frame].ready)) {
            return -ERESTARTSYS;
        }
        
        if (mutex_lock_interruptible(&dev->io_mutex)) {
            return -ERESTARTSYS;
        }
        if (dev->disconnected) {
            mutex_unlock(&dev->io_mutex);
            return -ENODEV;
        }
    }
    
    dev->read_frame = frame_idx;
    struct camera_frame *frame = &dev->frames[dev->read_frame];
    
    if (frame->ready) {
        // Calculate bytes to read
        bytes_read = min_t(size_t, count, frame->size - dev->read_pos);
        
        if (bytes_read <= 0) {
            // End of frame - move to next
            frame->ready = false;  // Mark as consumed
            dev->read_frame = (dev->read_frame + 1) % MAX_FRAMES;
            dev->read_pos = 0;
            mutex_unlock(&dev->io_mutex);
            return 0;
        }
        
        // Copy data to user
        if (copy_to_user(buf, frame->data + dev->read_pos, bytes_read)) {
            mutex_unlock(&dev->io_mutex);
            return -EFAULT;
        }
        
        dev->read_pos += bytes_read;
        
        // If end of frame, mark as not ready and move to next
        if (dev->read_pos >= frame->size) {
            frame->ready = false;
            dev->read_frame = (dev->read_frame + 1) % MAX_FRAMES;
            dev->read_pos = 0;
        }
    }
    
    mutex_unlock(&dev->io_mutex);
    return bytes_read;
}

// Probe function
static int camera_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    struct usb_device *udev = interface_to_usbdev(interface);
    struct camera_dev *dev;
    int result;
    int i;
    
    // Check interface
    if (interface->cur_altsetting->desc.bInterfaceNumber != INTERFACE_NUM) {
        return -ENODEV;
    }
    
    // Allocate device
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if (!dev) {
        return -ENOMEM;
    }
    
    // Initialize device
    dev->udev = usb_get_dev(udev);
    dev->interface = interface;
    dev->streaming = false;
    dev->device_open = false;
    /* Published nodes must not accept opens until probe has succeeded. */
    dev->disconnected = true;
    kref_init(&dev->refcount);
    mutex_init(&dev->io_mutex);
    mutex_init(&dev->v4l2_lock);
    spin_lock_init(&dev->vb2_lock);
    INIT_LIST_HEAD(&dev->vb2_buffers);
    init_waitqueue_head(&dev->read_wait);

    dev->jpeg_data = kmalloc(BUFFER_SIZE, GFP_KERNEL);
    if (!dev->jpeg_data) {
        result = -ENOMEM;
        goto error_alloc_chrdev;
    }
    
    // Initialize frame buffers
    for (i = 0; i < MAX_FRAMES; i++) {
        dev->frames[i].data = NULL;
        dev->frames[i].size = 0;
        dev->frames[i].ready = false;
    }
    
    // Initialize connect command
    dev->connect_cmd[0] = 0xbb;
    dev->connect_cmd[1] = 0xaa;
    dev->connect_cmd[2] = 0x05;
    dev->connect_cmd[3] = 0x00;
    dev->connect_cmd[4] = 0x00;
    
    // Create character device
    result = alloc_chrdev_region(&dev->dev_num, 0, 1, "supercamera");
    if (result < 0) {
        printk(KERN_ERR "Camera: Failed to allocate device number");
        goto error_alloc_chrdev;
    }
    
    major_num = MAJOR(dev->dev_num);
    
    cdev_init(&dev->cdev, &camera_fops);
    dev->cdev.owner = THIS_MODULE;
    
    result = cdev_add(&dev->cdev, dev->dev_num, 1);
    if (result < 0) {
        printk(KERN_ERR "Camera: Failed to add character device");
        goto error_cdev_add;
    }
    
    // Create device class
    dev->class = class_create("supercamera");
    if (IS_ERR(dev->class)) {
        result = PTR_ERR(dev->class);
        printk(KERN_ERR "Camera: Failed to create device class");
        goto error_class_create;
    }
    
    // Create device file
    dev->device = device_create(dev->class, NULL, dev->dev_num, NULL, "supercamera");
    if (IS_ERR(dev->device)) {
        result = PTR_ERR(dev->device);
        printk(KERN_ERR "Camera: Failed to create device");
        goto error_device_create;
    }

    result = camera_register_v4l2(dev);
    if (result) {
        printk(KERN_ERR "Camera: Failed to register V4L2 device: %d", result);
        goto error_v4l2_register;
    }
    
    // Start streaming
    result = start_streaming(dev);
    if (result < 0) {
        printk(KERN_ERR "Camera: Failed to start streaming: %d", result);
        goto error_start_streaming;
    }
    
    // Store device in interface
    usb_set_intfdata(interface, dev);
    mutex_lock(&dev->io_mutex);
    WRITE_ONCE(dev->disconnected, false);
    mutex_unlock(&dev->io_mutex);
    
    printk(KERN_INFO "Camera: Device connected (/dev/supercamera)");
    return 0;
    
error_start_streaming:
    camera_unregister_v4l2(dev);
error_v4l2_register:
    device_destroy(dev->class, dev->dev_num);
error_device_create:
    class_destroy(dev->class);
error_class_create:
    cdev_del(&dev->cdev);
error_cdev_add:
    unregister_chrdev_region(dev->dev_num, 1);
error_alloc_chrdev:
    kref_put(&dev->refcount, camera_device_release);
    return result;
}

// Disconnect function
static void camera_disconnect(struct usb_interface *interface)
{
    struct camera_dev *dev = usb_get_intfdata(interface);
    
    if (!dev) {
        return;
    }
    
    WRITE_ONCE(dev->disconnected, true);
    wake_up_interruptible(&dev->read_wait);

    // Stop USB and V4L2 streaming before releasing the device nodes.
    stop_streaming(dev);
    if (dev->vb2_queue_initialized)
        vb2_queue_error(&dev->vb2_queue);
    camera_unregister_v4l2(dev);
    
    // Clean up character device
    device_destroy(dev->class, dev->dev_num);
    class_destroy(dev->class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->dev_num, 1);
    
    kref_put(&dev->refcount, camera_device_release);
    
    printk(KERN_INFO "Camera: Device disconnected");
}

// USB device ID table
static const struct usb_device_id camera_id_table[] = {
    { USB_DEVICE(VENDOR_ID, PRODUCT_ID) },
    {}
};
MODULE_DEVICE_TABLE(usb, camera_id_table);

// USB driver
static struct usb_driver camera_driver = {
    .name = "supercamera_simple",
    .probe = camera_probe,
    .disconnect = camera_disconnect,
    .id_table = camera_id_table,
};

// Module init and exit
module_usb_driver(camera_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Mustafa Akcanca");
MODULE_DESCRIPTION("Simple USB camera driver");
