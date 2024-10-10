#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/interrupt.h>
#include <linux/ktime.h>

#define SUCCESS 0
#define DEVICE_NAME "dht11"
#define BUFFER_LENGTH 80

// cat /sys/kernel/debug/gpio
#define GPIO21 533
#define GPIO_PIN 21
#define GPIO_BASE 0x3f200000

// http://www.tortosaforum.com/raspberrypi/dht11driver.htm
// set GPIO pin g as input
#define GPIO_DIR_INPUT(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
// set GPIO pin g as output
#define GPIO_DIR_OUTPUT(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
// get logical value from gpio pin g
#define GPIO_READ_PIN(g) (*(gpio+13) & (1<<(g))) && 1
// sets   bits which are 1 ignores bits which are 0
#define GPIO_SET_PIN(g)    *(gpio+7) = 1<<g;
// clears bits which are 1 ignores bits which are 0
#define GPIO_CLEAR_PIN(g) *(gpio+10) = 1<<g;
// Clear GPIO interrupt on the pin we use
#define GPIO_INT_CLEAR(g) *(gpio+16) = (*(gpio+16) | (1<<g));
// GPREN0 GPIO Pin Rising Edge Detect Enable/Disable
#define GPIO_INT_RISING(g,v) *(gpio+19) = v ? (*(gpio+19) | (1<<g)) : (*(gpio+19) ^ (1<<g))
// GPFEN0 GPIO Pin Falling Edge Detect Enable/Disable
#define GPIO_INT_FALLING(g,v) *(gpio+22) = v ? (*(gpio+22) | (1<<g)) : (*(gpio+22) ^ (1<<g))


static int major;
static struct class *cls;
volatile unsigned *gpio;
static spinlock_t lock;
static int irq_number;
static ktime_t last_irq_time;

static unsigned bitcount = 0;
static unsigned started = 0;

static unsigned bits[40];

static char out_buffer[BUFFER_LENGTH];

static irqreturn_t irq_handler(int i, void *blah, struct pt_regs *regs) {

    ktime_t now = ktime_get();
    u64 delta = ktime_to_ns(now) - ktime_to_ns(last_irq_time);
    last_irq_time = now;
    unsigned signal = GPIO_READ_PIN(GPIO_PIN);

    if (signal) {
        started = 1;
        return IRQ_HANDLED;
    }

    if (!signal && started) {
        if (delta > 80000 || delta < 15000)
            return IRQ_HANDLED;
        bits[bitcount++] = delta > 60000;
    }

    return IRQ_HANDLED;
}

static int setup_interrupts(void) {
    int result;
    unsigned long flags;

    irq_number = gpio_to_irq(GPIO21);

    result = request_irq(irq_number, (irq_handler_t)irq_handler, 0, DEVICE_NAME, (void*) gpio);
    if (result < 0) {
        pr_info("request_irq failed %d\n", irq_number);
        return irq_number;
    }

    spin_lock_irqsave(&lock, flags);

    GPIO_INT_RISING(GPIO_PIN, 1);
    GPIO_INT_FALLING(GPIO_PIN, 1);
    GPIO_INT_CLEAR(GPIO_PIN);

    spin_unlock_irqrestore(&lock, flags);
    return SUCCESS;
}

static void clear_interrupts(void) {
    unsigned long flags;

    spin_lock_irqsave(&lock, flags);

    GPIO_INT_RISING(GPIO_PIN, 0);
    GPIO_INT_FALLING(GPIO_PIN, 0);

    spin_unlock_irqrestore(&lock, flags);

    free_irq(irq_number, (void *) gpio);
}

static int dht11_open(struct inode *inode, struct file *file) {
    try_module_get(THIS_MODULE);

    started = 0;
    bitcount = 0;
    for (size_t i = 0; i < 40; i++) {
       bits[i] = 0;
    }

    GPIO_DIR_OUTPUT(GPIO_PIN);
    GPIO_CLEAR_PIN(GPIO_PIN);
    mdelay(20);
    GPIO_SET_PIN(GPIO_PIN);
    udelay(40);
    GPIO_DIR_INPUT(GPIO_PIN);

    last_irq_time = ktime_get();
    setup_interrupts();

    mdelay(10);

    clear_interrupts();

    int dht[5];
    for (int i = 0; i < 5; i++)
        dht[i] = 0;

    for (int i = 0; i < 5; i++) {
        for (int j = 0; j < 8; j++) {
            dht[i] |= bits[i * 8 + j] << (7 - j);
        }
    }

    for (int i = 0; i < BUFFER_LENGTH; i++)
      out_buffer[i] = '\0';

    if (dht[0] + dht[1] + dht[2] + dht[3] == dht[4]) {
        sprintf(out_buffer, "Temperature: %d.%d. Humidity: %d.%d\n", dht[2], dht[3], dht[0], dht[1]);
    } else {
        sprintf(out_buffer, "Checksum validation failed\n");
    }

    return SUCCESS;
}

static int dht11_release(struct inode *inode, struct file *file) {
    module_put(THIS_MODULE);
    pr_info("release %s", DEVICE_NAME);
    return SUCCESS;
}

static ssize_t dht11_read(struct file *filp, char __user *buffer, size_t length, loff_t *offset) {
    if (*offset >= BUFFER_LENGTH) {
        return 0;
    }
    unsigned remaining = copy_to_user(buffer, out_buffer, BUFFER_LENGTH);
    if (remaining > 0) {
        pr_alert("copy_to_user failed. remaining %d characters\n", remaining);
        return remaining;
    }
    *offset += BUFFER_LENGTH;
    return BUFFER_LENGTH;
}

static struct file_operations fops = {
    .read = dht11_read,
    .open = dht11_open,
    .release = dht11_release,
};

static int __init dht11_init(void) {
    int res = gpio_request(GPIO21, "gpio21");
    if (res < 0) {
        pr_alert("gpio_request failed: %d", res);
        return res;
    }
    gpio = ioremap(GPIO_BASE, SZ_4K);
    if (gpio == NULL) {
        gpio_free(GPIO21);
        pr_alert("ioremap failed");
        return -EBUSY;
    }
    major = register_chrdev(0, DEVICE_NAME, &fops);
    if (major < 0) {
        pr_alert("register_chrdev failed: %d", major);
        return major;
    }
    cls = class_create(DEVICE_NAME);
    device_create(cls, NULL, MKDEV(major, 0), NULL, DEVICE_NAME);

    pr_info("Initialized dht11");
    return 0;
}

static void __exit dht11_exit(void) {
    gpio_free(GPIO21);
    if (gpio != NULL) {
        iounmap(gpio);
    }
    device_destroy(cls, MKDEV(major, 0));
    class_destroy(cls);
    unregister_chrdev(major, DEVICE_NAME);
    pr_info("Dht11 exit\n");
}

module_init(dht11_init);
module_exit(dht11_exit);

MODULE_AUTHOR("Oleg Rybalko");
MODULE_LICENSE("GPL");

