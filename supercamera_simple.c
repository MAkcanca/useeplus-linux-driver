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
#define CONNECT_CMD_SIZE 5

#define BUFFER_SIZE (64*1024)  // Buffer for USB transfers
#define MAX_FRAMES  5          // Number of frames to keep in ring buffer

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
};

static int major_num;

// Forward declarations
static int camera_open(struct inode *inode, struct file *file);
static int camera_release(struct inode *inode, struct file *file);
static ssize_t camera_read(struct file *file, char __user *buf, size_t count, loff_t *pos);

static struct file_operations camera_fops = {
    .owner = THIS_MODULE,
    .open = camera_open,
    .release = camera_release,
    .read = camera_read,
};

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
    
    // Submit URB
    result = usb_submit_urb(dev->urb, GFP_KERNEL);
    if (result) {
        printk(KERN_ERR "Camera: Failed to submit URB: %d", result);
        kfree(dev->urb_buffer);
        usb_free_urb(dev->urb);
        dev->urb = NULL;
        dev->urb_buffer = NULL;
        return result;
    }
    
    dev->streaming = true;
    return 0;
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
    dev->read_frame = 0;
    
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
                                    dev->frames[dev->read_frame].ready)) {
            return -ERESTARTSYS;
        }
        
        if (mutex_lock_interruptible(&dev->io_mutex)) {
            return -ERESTARTSYS;
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
    mutex_init(&dev->io_mutex);
    init_waitqueue_head(&dev->read_wait);
    
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
    
    // Start streaming
    result = start_streaming(dev);
    if (result < 0) {
        printk(KERN_ERR "Camera: Failed to start streaming: %d", result);
        goto error_start_streaming;
    }
    
    // Store device in interface
    usb_set_intfdata(interface, dev);
    
    printk(KERN_INFO "Camera: Device connected (/dev/supercamera)");
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
    for (int i = 0; i < MAX_FRAMES; i++) {
        kfree(dev->frames[i].data);
    }
    
    // Free device
    usb_put_dev(dev->udev);
    kfree(dev);
    
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
