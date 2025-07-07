#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/usb.h>
#include <linux/slab.h>
#include <linux/fs.h>
#include <linux/mutex.h>
#include <linux/uaccess.h>
#include <linux/cdev.h>

#define VENDOR_ID  0x2ce3
#define PRODUCT_ID 0x3828

#define INTERFACE_NUM 1
#define EP_IN  0x81
#define EP_OUT 0x01

// common header for all data packets
struct camera_header {
    unsigned short magic;
    unsigned char type;
    unsigned short length;
} __attribute__((packed));

#define HEADER_SIZE 5   // sizeof(struct camera_header)

// values for magic
#define MAGIC_COMMAND   0xaabb  // host to device
#define MAGIC_DATA      0xbbaa  // device to host
// values for type
#define TYPE_START      0x05    // Command: start streaming, no payload
#define TYPE_IMAGE      0x07    // Data: image, payload: see camera_image_header

// header for image packets
struct camera_image_header {
    struct camera_header header;
    unsigned char index;    // image index, might jump...
    unsigned char data1;    // unknown
    unsigned char data2;    // unknown
    unsigned int data3;     // unknown
} __attribute__((packed));

#define IMAGE_HEADER_SIZE 12    // sizeof(struct camera_image_header)

// Ring buffer entry header
struct camera_frame_header {
    unsigned int magic;
    unsigned int index;
    unsigned int length;
} __attribute__((packed));

#define FRAME_HEADER_SIZE 12    // sizeof(struct camera_frame_header)
#define FRAME_HEADER_MAGIC 0xaabbccdd

#define BUFFER_SIZE (64*1024)  // Buffer for USB transfers
#define FRAMEBUFFER_SIZE (4*BUFFER_SIZE)    // Ring buffer for received frames

#define LOG_PREFIX "supercamera: "

struct camera_dev {
    struct usb_device *udev;
    struct usb_interface *interface;
    struct urb *urb;
    unsigned char *urb_buffer;
    
    // Frame ring buffer
    struct camera_frame_header *framebuffer;
    struct camera_frame_header *framebuffer_limit;
    
    // Write position
    struct camera_frame_header *current_frame;
    size_t packet_count;
    size_t images_received;
    size_t max_lag;
    
    // Character device
    struct cdev cdev;
    dev_t dev_num;
    struct class *class;
    struct device *device;
    struct mutex io_mutex;
    bool device_open;
    
    // Read position
    struct camera_frame_header *read_frame;
    size_t images_read;
    size_t read_pos;
    
    // State
    bool streaming;
    
    // Wait queue for blocking reads
    wait_queue_head_t read_wait;
};

static void dump_packet(struct camera_dev *dev, unsigned char *data, int length)
{
    printk(KERN_INFO LOG_PREFIX "Packet %zu (%d bytes): ", dev->packet_count, length);
    for (int i = 0; i < min(16, length); i++) {
        printk(KERN_CONT "%02x ", data[i]);
    }
    printk(KERN_CONT "\n");
}

static struct camera_frame_header *next_frame(struct camera_dev *dev, struct camera_frame_header *frame) {
    struct camera_frame_header *next = (void *) (frame + 1) + frame->length;
    if (next > dev->framebuffer_limit) {
        next = dev->framebuffer;
    }
    return next;
}

static void init_frame(struct camera_frame_header *frame, unsigned int index) {
    frame->magic = FRAME_HEADER_MAGIC;
    frame->index = index;
    frame->length = 0;
}

static struct camera_frame_header *check_frame(struct camera_frame_header* frame) {
    if (frame->magic == FRAME_HEADER_MAGIC) {
        return frame;
    }
    return NULL;
}

static unsigned char *frame_data(struct camera_frame_header* frame) {
    return (unsigned char *) (frame + 1);
}

// Process received data by adding it to the current frame
static void process_data(struct camera_dev *dev, unsigned char *data, int length)
{
    struct camera_image_header *header = (struct camera_image_header *)data;

    // Debug first few packets
    dev->packet_count++;
    if (length < IMAGE_HEADER_SIZE || header->header.magic != MAGIC_DATA ||
            header->header.type != TYPE_IMAGE || header->header.length + HEADER_SIZE != length) {
        printk(KERN_WARNING LOG_PREFIX "Invalid packet %zu\n", dev->packet_count);
        dump_packet(dev, data, length);
        return;
    }
    if (dev->packet_count <= 10) {
        dump_packet(dev, data, length);
    }

    // Skip header and append payload data to current frame
    if (length > IMAGE_HEADER_SIZE) {
        struct camera_frame_header *frame = check_frame(dev->current_frame);
        if (!frame) {
            printk(KERN_ERR LOG_PREFIX "Frame buffer corrupted! %p/%p/%p/%p\n",
                    dev->framebuffer, dev->read_frame, dev->current_frame, dev->framebuffer_limit);
            return;
        }

        unsigned char *framedata = frame_data(frame);
        if (header->index != frame->index && frame->length) {
            printk(KERN_INFO LOG_PREFIX "Received image %u, size %u\n",
                    frame->index, frame->length);
            if (framedata[0] == 0xff && framedata[1] == 0xd8 &&
                    framedata[frame->length - 2] == 0xff && framedata[frame->length - 1] == 0xd9) {
                frame = next_frame(dev, frame);
                init_frame(frame, header->index);
                framedata = (unsigned char*) (frame + 1);
                dev->current_frame = frame;
                dev->images_received++;
                if (!dev->max_lag && dev->current_frame == dev->framebuffer) {
                    dev->max_lag = dev->images_received / 2;
                    printk(KERN_INFO LOG_PREFIX "Max. client lag is %zu frames\n", dev->max_lag);
                }
                wake_up_interruptible(&dev->read_wait);
            } else {
                printk(KERN_WARNING LOG_PREFIX "Image %u is corrupt, skipping\n", frame->index);
                init_frame(frame, header->index);
            }
        }

        int payload_length = length - IMAGE_HEADER_SIZE;

        // Check if we have enough space
        if (frame->length + payload_length <= BUFFER_SIZE - FRAME_HEADER_SIZE) {
            memcpy(framedata + frame->length, data + IMAGE_HEADER_SIZE, payload_length);
            frame->length += payload_length;
        } else {
            printk(KERN_WARNING LOG_PREFIX "Image %u exceeds buffer size\n", frame->index);
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
        printk(KERN_ERR LOG_PREFIX "Failed to send command: %d\n", result);
        return result;
    }
    
    if (actual_length != len) {
        printk(KERN_WARNING LOG_PREFIX "Short write (%d / %d)\n", actual_length, len);
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
            printk(KERN_ERR LOG_PREFIX "URB error %d\n", status);
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
            printk(KERN_ERR LOG_PREFIX "Failed to resubmit URB: %d\n", status);
        }
    }
}

// Start streaming
static int start_streaming(struct camera_dev *dev)
{
    int result;
    
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
    
    // Set alt setting to 1
    result = usb_set_interface(dev->udev, INTERFACE_NUM, 1);
    if (result) {
        printk(KERN_ERR LOG_PREFIX "Failed to set alt setting: %d\n", result);
        goto error_start_streaming;
    }
    
    // Initialize connect command
    struct camera_header *header = (struct camera_header *)dev->urb_buffer;
    header->magic = MAGIC_COMMAND;
    header->type = TYPE_START;
    header->length = 0;
    
    // Send connect command
    result = send_command(dev, dev->urb_buffer, HEADER_SIZE);
    if (result) {
        goto error_start_streaming;
    }
    
    // Fill URB
    usb_fill_bulk_urb(dev->urb, dev->udev, usb_rcvbulkpipe(dev->udev, EP_IN),
                     dev->urb_buffer, BUFFER_SIZE, camera_urb_complete, dev);
    
    // Submit URB
    result = usb_submit_urb(dev->urb, GFP_KERNEL);
    if (result) {
        printk(KERN_ERR LOG_PREFIX "Failed to submit URB: %d\n", result);
        goto error_start_streaming;
    }
    
    dev->packet_count = 0;
    dev->images_received = 0;
    dev->images_read = 0;
    dev->max_lag = 0;
    dev->streaming = true;
    return 0;

error_start_streaming:
    kfree(dev->urb_buffer);
    usb_free_urb(dev->urb);
    dev->urb = NULL;
    dev->urb_buffer = NULL;
    return result;
}

// Stop streaming
static void stop_streaming(struct camera_dev *dev)
{
    if (!dev->streaming) {
        return;
    }
    
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
    
    dev->streaming = false;
}

// Device open
static int camera_open(struct inode *inode, struct file *file)
{
    struct camera_dev *dev = container_of(inode->i_cdev, struct camera_dev, cdev);
    
    file->private_data = dev;
    
    if (mutex_lock_interruptible(&dev->io_mutex)) {
        return -ERESTARTSYS;
    }
    
    if (dev->device_open) {
        mutex_unlock(&dev->io_mutex);
        return -EBUSY;
    }
    
    dev->device_open = true;
    dev->read_pos = 0;
    dev->read_frame = dev->current_frame;
    dev->images_read = dev->images_received;
    
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
    
    if (!dev->max_lag) {
        printk(KERN_INFO LOG_PREFIX "Waiting for max_lag calculation\n");

        // No ready frames - wait or return
        mutex_unlock(&dev->io_mutex);

        if (file->f_flags & O_NONBLOCK) {
            return -EAGAIN;
        }

        if (wait_event_interruptible(dev->read_wait, dev->max_lag)) {
            return -ERESTARTSYS;
        }

        if (mutex_lock_interruptible(&dev->io_mutex)) {
            return -ERESTARTSYS;
        }
    }

    // check for client lagging behind
    if (dev->images_received - dev->images_read > dev->max_lag) {
        printk(KERN_INFO LOG_PREFIX "Client is too slow (%zu/%zu)), re-setting read_frame\n", dev->images_read, dev->images_received);
        dev->read_frame = dev->current_frame;
        dev->read_pos = 0;
        dev->images_read = dev->images_received;
    }

    // Find ready frame
    while (dev->read_frame == dev->current_frame) {
        printk(KERN_INFO LOG_PREFIX "No image available, waiting\n");

        // No ready frames - wait or return
        mutex_unlock(&dev->io_mutex);
        
        if (file->f_flags & O_NONBLOCK) {
            return -EAGAIN;
        }
        
        if (wait_event_interruptible(dev->read_wait, 
                                    dev->read_frame != dev->current_frame)) {
            return -ERESTARTSYS;
        }
        
        if (mutex_lock_interruptible(&dev->io_mutex)) {
            return -ERESTARTSYS;
        }
    }
    
    struct camera_frame_header *frame = check_frame(dev->read_frame);
    if (!frame) {
        printk(KERN_ERR LOG_PREFIX "Invalid frame buffer! %p/%p/%p/%p\n",
                dev->framebuffer, dev->read_frame, dev->current_frame, dev->framebuffer_limit);
        mutex_unlock(&dev->io_mutex);
        return -EIO;
    }
    
    // Calculate bytes to read
    bytes_read = min_t(size_t, count, frame->length - dev->read_pos);

    printk(KERN_INFO LOG_PREFIX "Reading image %d from position %zu, size %zd\n", frame->index, dev->read_pos, bytes_read);

    if (bytes_read <= 0) {
        // End of frame - move to next
        frame = next_frame(dev, frame);
        dev->read_frame = frame;
        dev->read_pos = 0;
        dev->images_read++;
        mutex_unlock(&dev->io_mutex);
        return 0;
    }

    // Copy data to user
    unsigned char *framedata = frame_data(frame);
    if (copy_to_user(buf, framedata + dev->read_pos, bytes_read)) {
        mutex_unlock(&dev->io_mutex);
        return -EFAULT;
    }

    dev->read_pos += bytes_read;

    // If end of frame, mark as not ready and move to next
    if (dev->read_pos >= frame->length) {
        frame = next_frame(dev, frame);
        dev->read_frame = frame;
        dev->read_pos = 0;
        dev->images_read++;
    }

    mutex_unlock(&dev->io_mutex);
    return bytes_read;
}

static char* camera_devnode(const struct device *dev, umode_t *mode) {
    if (mode) {
        *mode = 0444;
    }
    return NULL;
}

static struct file_operations camera_fops = {
    .owner = THIS_MODULE,
    .open = camera_open,
    .release = camera_release,
    .read = camera_read,
};

// Probe function
static int camera_probe(struct usb_interface *interface, const struct usb_device_id *id)
{
    struct usb_device *udev = interface_to_usbdev(interface);
    struct camera_dev *dev;
    int result;
    
    // Check interface
    if (interface->cur_altsetting->desc.bInterfaceNumber != INTERFACE_NUM) {
        return -ENODEV;
    }
    
    // Allocate device
    dev = kzalloc(sizeof(*dev), GFP_KERNEL);
    if (!dev) {
        return -ENOMEM;
    }
    
    // Initialize frame buffers
    dev->framebuffer = kmalloc(FRAMEBUFFER_SIZE, GFP_KERNEL);
    if (!dev->framebuffer) {
        printk(KERN_ERR LOG_PREFIX "Failed to allocate frame buffer\n");
        goto error_alloc_frames;
    }
    dev->framebuffer_limit = (void *) dev->framebuffer + FRAMEBUFFER_SIZE - BUFFER_SIZE;
    dev->current_frame = dev->read_frame = dev->framebuffer;
    init_frame(dev->current_frame, 0);

    // Initialize device
    dev->udev = usb_get_dev(udev);
    dev->interface = interface;
    dev->streaming = false;
    dev->device_open = false;
    mutex_init(&dev->io_mutex);
    init_waitqueue_head(&dev->read_wait);
    
    // Create character device
    result = alloc_chrdev_region(&dev->dev_num, 0, 1, "supercamera");
    if (result < 0) {
        printk(KERN_ERR LOG_PREFIX "Failed to allocate device number\n");
        goto error_alloc_chrdev;
    }
    
    cdev_init(&dev->cdev, &camera_fops);
    dev->cdev.owner = THIS_MODULE;
    
    result = cdev_add(&dev->cdev, dev->dev_num, 1);
    if (result < 0) {
        printk(KERN_ERR LOG_PREFIX "Failed to add character device\n");
        goto error_cdev_add;
    }
    
    // Create device class
    dev->class = class_create("supercamera");
    if (IS_ERR(dev->class)) {
        result = PTR_ERR(dev->class);
        printk(KERN_ERR LOG_PREFIX "Failed to create device class\n");
        goto error_class_create;
    }
    dev->class->devnode = camera_devnode;
    
    // Create device file
    dev->device = device_create(dev->class, NULL, dev->dev_num, NULL, "supercamera");
    if (IS_ERR(dev->device)) {
        result = PTR_ERR(dev->device);
        printk(KERN_ERR LOG_PREFIX "Failed to create device\n");
        goto error_device_create;
    }
    
    // Start streaming
    result = start_streaming(dev);
    if (result < 0) {
        printk(KERN_ERR LOG_PREFIX "Failed to start streaming: %d\n", result);
        goto error_start_streaming;
    }
    
    // Store device in interface
    usb_set_intfdata(interface, dev);
    
    printk(KERN_INFO LOG_PREFIX "Device connected (/dev/supercamera)\n");
    return 0;
    
error_start_streaming:
    device_destroy(dev->class, dev->dev_num);
error_device_create:
    class_destroy(dev->class);
error_class_create:
    cdev_del(&dev->cdev);
error_cdev_add:
    unregister_chrdev_region(dev->dev_num, 1);
error_alloc_chrdev:
    usb_put_dev(dev->udev);
error_alloc_frames:
    kfree(dev->framebuffer);
    kfree(dev);
    return result;
}

// Disconnect function
static void camera_disconnect(struct usb_interface *interface)
{
    struct camera_dev *dev = usb_get_intfdata(interface);
    
    if (!dev) {
        return;
    }
    
    // Stop streaming
    stop_streaming(dev);
    
    // Clean up character device
    device_destroy(dev->class, dev->dev_num);
    class_destroy(dev->class);
    cdev_del(&dev->cdev);
    unregister_chrdev_region(dev->dev_num, 1);
    
    // Free frame buffers
    kfree(dev->framebuffer);
    
    // Free device
    usb_put_dev(dev->udev);
    kfree(dev);
    
    printk(KERN_INFO LOG_PREFIX "Device disconnected\n");
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
