#include <linux/init.h>
#include <linux/module.h>

#include <linux/interrupt.h>
#include <linux/tty.h>
#include <linux/tty_driver.h>
#include <linux/tty_flip.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/printk.h>

struct tty_driver *gpiotty_driver;

#define MAX_TOTAL_PORTS 1

#define TTY_DEV_NAME "gpiotty"
static const char tty_dev_name[] = TTY_DEV_NAME;

struct gpiotty_serial {
  struct tty_struct *tty;
  int open_count;
  struct semaphore sem;

  int rx_gpio;
};

static struct gpiotty_serial *gpiotty_table[MAX_TOTAL_PORTS];

static irqreturn_t cb_gpio_irq(int irq, void *dev)
{
  struct gpiotty_serial *serial = (struct gpiotty_serial*)dev;
  char byte = 0;
  int i;

  for (i=0;i<8;i++) {
    byte |= gpio_get_value(serial->rx_gpio);
    byte = byte << 1;
    mdelay(1);
  }

  tty_insert_flip_char(serial->tty->port, byte, TTY_NORMAL);
  tty_flip_buffer_push(serial->tty->port);

  return IRQ_HANDLED;
}

static int gpiotty_open(struct tty_struct *tty, struct file *fp)
{
  struct gpiotty_serial *gpiotty;
  int index;
  int err = 0;

  tty->driver_data = NULL;

  index = tty->index;
  gpiotty = gpiotty_table[index];
  if (gpiotty == NULL) {

    gpiotty = kmalloc(sizeof(*gpiotty), GFP_KERNEL);
    if (!gpiotty)
      return -ENOMEM;

    sema_init(&gpiotty->sem, 1);
    gpiotty->open_count = 0;
    gpiotty->rx_gpio = index;

    gpiotty_table[index] = gpiotty;

    //FIXME: Unique names
    err = gpio_request_one(gpiotty->rx_gpio, GPIOF_DIR_IN | GPIOF_EXPORT_DIR_FIXED, "ttyGPIO0");
    if (!err)
      return -EBUSY;

    err = request_irq(gpio_to_irq(gpiotty->rx_gpio), cb_gpio_irq, IRQF_TRIGGER_RISING, "ttyGPIO0-rx", tty);
    if (!err)
      return -EIO;
  }

  down(&gpiotty->sem);

  tty->driver_data = gpiotty;
  gpiotty->tty = tty;

  gpiotty->open_count++;

  if (gpiotty->open_count == 1) {
    //TODO: Init GPIO pins
  }

  return err;
}

static void gpiotty_close(struct tty_struct *tty, struct file *fp)
{
  struct gpiotty_serial *gpiotty = tty->driver_data;

  if (gpiotty) {
    down(&gpiotty->sem);

    if (!gpiotty->open_count) {
      goto exit;
    }

    free_irq(gpio_to_irq(gpiotty->rx_gpio), tty);
    gpio_free(gpiotty->rx_gpio);

    gpiotty->open_count--;
  }

exit:
  up(&gpiotty->sem);
}

static const struct tty_operations gpiotty_ops = {
	.open = gpiotty_open,
	.close = gpiotty_close,
};

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Torrie Fischer <tdfischer@hackerbots.net>");
MODULE_DESCRIPTION("Wire up GPIO pins to act as serial TTY devices");
static int __init gpiotty_init(void)
{
	int err;

	gpiotty_driver = tty_alloc_driver(MAX_TOTAL_PORTS, TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV);

	if (IS_ERR(gpiotty_driver)) {
		err = PTR_ERR(gpiotty_driver);
		return err;
	}

	gpiotty_driver->driver_name = KBUILD_MODNAME;
	gpiotty_driver->name = tty_dev_name;
	gpiotty_driver->major = 0;
	gpiotty_driver->minor_start = 0;
	gpiotty_driver->type = TTY_DRIVER_TYPE_SERIAL;
	gpiotty_driver->subtype = SERIAL_TYPE_NORMAL;
	gpiotty_driver->init_termios = tty_std_termios;
	gpiotty_driver->init_termios.c_cflag |= CLOCAL;

	tty_set_operations(gpiotty_driver, &gpiotty_ops);

	err = tty_register_driver(gpiotty_driver);
	if (err) {
		printk(KERN_ERR "register tty driver failed (%d)\n", err);
		goto put_tty;
	}

  tty_register_device(gpiotty_driver, 0, NULL);

put_tty:
	put_tty_driver(gpiotty_driver);

	return err;
}

static void __exit gpiotty_exit(void)
{
  tty_unregister_device(gpiotty_driver, 0);
  tty_unregister_driver(gpiotty_driver);
  printk(KERN_ALERT "Unloaded!\n");
}

module_init(gpiotty_init);
module_exit(gpiotty_exit);
