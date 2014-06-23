/*
 * GPIO Serial driver
 *
 * Copyright (C) 2014 Torrie Fischer <tdfischer@hackerbots.net>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, version 2 of the License.
 *
 * This driver emulates a UART in software over GPIO pins.
 *
 */

#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial.h>
#include <linux/serial_core.h>
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/delay.h>

#define DRIVER_AUTHOR "Torrie Fischer <tdfischer@hackerbots.net>"
#define DRIVER_DESC "UART serial emulation over GPIO pins"

MODULE_AUTHOR( DRIVER_AUTHOR );
MODULE_DESCRIPTION ( DRIVER_DESC );
MODULE_LICENSE ( "GPL" );

#define GPIOTTY_SERIAL_MAJOR 240
#define GPIOTTY_SERIAL_MINORS 1
#define GPIOTTY_RX_PIN 24

#define UART_NR 1

#define GPIOTTY_NAME "ttyGPIO"

#define MY_NAME GPIOTTY_NAME

static irqreturn_t cb_gpio_irq(int irq, void *dev)
{
  struct uart_port *port;
  struct tty_struct *tty;
  struct tty_port *tty_port;
  char byte = 0;
  int i;

  port = (struct uart_port*)dev;
  tty = port->state->port.tty;
  tty_port = tty->port;

  printk(KERN_DEBUG "gpiotty: got serial port activity on pin\n");

  for (i=0;i<8;i++) {
    byte |= gpio_get_value(GPIOTTY_RX_PIN);
    byte = byte << 1;
    mdelay(1);
  }

  tty_insert_flip_char(tty_port, byte, TTY_NORMAL);
  tty_flip_buffer_push(tty_port);

  return IRQ_HANDLED;
}

static void gpiotty_stop_tx(struct uart_port *port)
{
}

static void gpiotty_stop_rx(struct uart_port *port)
{
}

static void gpiotty_enable_ms(struct uart_port *port)
{
}

static void gpiotty_start_tx(struct uart_port *port)
{
}

static unsigned int gpiotty_tx_empty(struct uart_port *port)
{
  return 0;
}

static unsigned int gpiotty_get_mctrl(struct uart_port *port)
{
  return TIOCM_CAR | TIOCM_CTS | TIOCM_DSR;
}

static void gpiotty_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
}

static void gpiotty_break_ctl(struct uart_port *port, int break_state)
{
}

static void gpiotty_set_termios(struct uart_port *port, struct ktermios *new, struct ktermios *old)
{
}

static int gpiotty_startup(struct uart_port *port)
{
  int err;

  printk(KERN_ERR "gpiotty: starting gpiotty on pin\n");

  err = gpio_request_one(GPIOTTY_RX_PIN, GPIOF_DIR_IN | GPIOF_EXPORT_DIR_FIXED, "gpiotty rx pin");
  if (err) {
    printk(KERN_ERR "gpiotty: could not request pin: %i\n", err);
    return - EBUSY;
  }

  err = request_irq(gpio_to_irq(GPIOTTY_RX_PIN), cb_gpio_irq, IRQF_TRIGGER_RISING, "gpiotty rx pin", port);
  if (err) {
    printk(KERN_ERR "gpiotty: could not request IRQ for pin: %i\n", err);
    return -EIO;
  }

  printk(KERN_DEBUG "gpiotty: pin opened\n");

  return 0;
}

static void gpiotty_shutdown(struct uart_port *port)
{
  free_irq(gpio_to_irq(GPIOTTY_RX_PIN), port);
  gpio_free(GPIOTTY_RX_PIN);
  printk(KERN_DEBUG "gpiotty: pin closed and released\n");
}

static const char *gpiotty_type(struct uart_port *port)
{
  return "gpiotty";
}

static void gpiotty_release_port(struct uart_port *port)
{
}

static int gpiotty_request_port(struct uart_port *port)
{
  return 0;
}

static void gpiotty_config_port(struct uart_port *port, int flags)
{
}

static int gpiotty_verify_port(struct uart_port *port, struct serial_struct *ser)
{
  return 0;
}

static struct uart_ops gpiotty_ops = {
  .tx_empty = gpiotty_tx_empty,
  .set_mctrl  = gpiotty_set_mctrl,
  .get_mctrl  = gpiotty_get_mctrl,
  .stop_tx  = gpiotty_stop_tx,
  .start_tx = gpiotty_start_tx,
  .stop_rx  = gpiotty_stop_rx,
  .enable_ms  = gpiotty_enable_ms,
  .break_ctl  = gpiotty_break_ctl,
  .startup  = gpiotty_startup,
  .shutdown = gpiotty_shutdown,
  .set_termios  = gpiotty_set_termios,
  .type   = gpiotty_type,
  .release_port = gpiotty_release_port,
  .request_port = gpiotty_request_port,
  .config_port  = gpiotty_config_port,
  .verify_port  = gpiotty_verify_port,
};

static struct uart_port gpiotty_port = {
  .ops = &gpiotty_ops,
};

static struct uart_driver gpiotty_reg = {
  .owner = THIS_MODULE,
  .driver_name = GPIOTTY_NAME,
  .dev_name = GPIOTTY_NAME,
  .major = GPIOTTY_SERIAL_MAJOR,
  .minor = GPIOTTY_SERIAL_MINORS,
  .nr = UART_NR,
};

static int __init gpiotty_init(void)
{
  int err;

  err = uart_register_driver(&gpiotty_reg);
  if (err) {
    printk(KERN_ERR "gpiotty: could not register driver: %d\n", err);
    return err;
  }

  err = uart_add_one_port(&gpiotty_reg, &gpiotty_port);
  if (err) {
    printk(KERN_ERR "gpiotty: could not add port: %d\n", err);
    uart_unregister_driver(&gpiotty_reg);
    return err;
  }

  printk(KERN_DEBUG "gpiotty: loaded\n");

  return err;
}

static void __exit gpiotty_exit(void)
{
  uart_remove_one_port(&gpiotty_reg, &gpiotty_port);
  uart_unregister_driver(&gpiotty_reg);
  printk(KERN_DEBUG "gpiotty: unloaded\n");
}

module_init(gpiotty_init);
module_exit(gpiotty_exit);
