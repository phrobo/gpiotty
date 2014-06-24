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

#define GPIOTTY_RINGSIZE 1024

#define GPIOTTY_BAUD 9600
#define GPIOTTY_BAUD_US (1.0/GPIOTTY_BAUD)*1000000

#define UART_NR 1

#define GPIOTTY_NAME "ttyGPIO"

#define MY_NAME GPIOTTY_NAME

struct gpiotty_uart_port {
  struct uart_port uart;
  struct tasklet_struct tasklet;
  struct circ_buf rx_ring;
};

struct gpiotty_uart_char {
  u16 ch;
};

static inline struct gpiotty_uart_port *
to_gpiotty_uart_port(struct uart_port *uart)
{
  return container_of(uart, struct gpiotty_uart_port, uart);
}

static inline void gpiotty_buffer_rx_char(struct uart_port *port, unsigned int byte)
{
  struct gpiotty_uart_port *gpiotty_port = to_gpiotty_uart_port(port);
  struct circ_buf *ring = &gpiotty_port->rx_ring;
  struct gpiotty_uart_char *c;

  if (!CIRC_SPACE(ring->head, ring->tail, GPIOTTY_RINGSIZE))
    return;

  c = &((struct gpiotty_uart_char *)ring->buf)[ring->head];
  c->ch = byte;

  smp_wmb();

  ring->head = (ring->head + 1 ) & (GPIOTTY_RINGSIZE - 1);
}

static irqreturn_t cb_gpio_irq(int irq, void *dev)
{
  struct uart_port *port = (struct uart_port *)dev;
  struct gpiotty_uart_port *gpiotty_port = to_gpiotty_uart_port(port);
  u8 byte = 0;
  u8 i, noti;

  //Wait a third of a bit to not sample the edges, and to have some margin
  //for CPU cycles spent not-reading
  udelay(GPIOTTY_BAUD_US/3);

  for (i = 0x1; i; i <<= 1) {
    udelay(GPIOTTY_BAUD_US);
    noti = ~i;
    if (gpio_get_value(GPIOTTY_RX_PIN))
      byte |= i;
    else // Else added to ensure timing is ~balanced
      byte &= noti;
  }

  // Skip stop bit
  udelay(GPIOTTY_BAUD_US);

  gpiotty_buffer_rx_char(port, byte);

  tasklet_schedule(&gpiotty_port->tasklet);

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

static void gpiotty_tasklet_func(unsigned long data)
{
  struct uart_port *port = (struct uart_port *)data;
  struct gpiotty_uart_port *gpiotty_port = to_gpiotty_uart_port(port);
  struct circ_buf *ring = &gpiotty_port->rx_ring;

  while (ring->head != ring->tail) {
    struct gpiotty_uart_char c;

    smp_rmb();

    c = ((struct gpiotty_uart_char *)ring->buf)[ring->tail];
    ring->tail = (ring->tail + 1) & (GPIOTTY_RINGSIZE - 1);
    port->icount.rx++;

    uart_insert_char(port, 0, 0, c.ch, TTY_NORMAL);
  }

  tty_flip_buffer_push(&port->state->port);
}

static int gpiotty_startup(struct uart_port *port)
{
  int err;
  struct gpiotty_uart_port *gpiotty_port = to_gpiotty_uart_port(port);
  void *data;

  printk(KERN_ERR "gpiotty: starting gpiotty on pin\n");

  tasklet_init(&gpiotty_port->tasklet, gpiotty_tasklet_func, (unsigned long)port);

  memset(&gpiotty_port->rx_ring, 0, sizeof(gpiotty_port->rx_ring));

  err = -ENOMEM;
  data = kmalloc(sizeof(struct gpiotty_uart_char) * GPIOTTY_RINGSIZE, GFP_KERNEL);

  if (!data)
    goto err_alloc_ring;

  gpiotty_port->rx_ring.buf = data;

  err = gpio_request_one(GPIOTTY_RX_PIN, GPIOF_DIR_IN | GPIOF_EXPORT_DIR_FIXED, "gpiotty rx pin");
  if (err) {
    printk(KERN_ERR "gpiotty: could not request pin: %i\n", err);
    goto err_add_port;
  }

  err = request_irq(gpio_to_irq(GPIOTTY_RX_PIN), cb_gpio_irq, IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_TRIGGER_LOW | IRQF_NOBALANCING | IRQF_NO_THREAD, "gpiotty rx pin", port);
  if (err) {
    printk(KERN_ERR "gpiotty: could not request IRQ for pin: %i\n", err);
    goto err_add_port;
  }

  printk(KERN_DEBUG "gpiotty: pin opened\n");

  return 0;

err_add_port:
  kfree(gpiotty_port->rx_ring.buf);
  gpiotty_port->rx_ring.buf = NULL;
err_alloc_ring:
  return err;
}

static void gpiotty_shutdown(struct uart_port *port)
{
  struct gpiotty_uart_port *gpiotty_port = to_gpiotty_uart_port(port);

  tasklet_kill(&gpiotty_port->tasklet);
  gpiotty_port->rx_ring.head = 0;
  gpiotty_port->rx_ring.tail = 0;

  kfree(gpiotty_port->rx_ring.buf);

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

static struct gpiotty_uart_port static_gpiotty_port = {
  .uart = {
    .ops = &gpiotty_ops,
    .type = PORT_MAX
  }
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

  err = uart_add_one_port(&gpiotty_reg, &static_gpiotty_port.uart);
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
  uart_remove_one_port(&gpiotty_reg, &static_gpiotty_port.uart);
  uart_unregister_driver(&gpiotty_reg);
  printk(KERN_DEBUG "gpiotty: unloaded\n");
}

module_init(gpiotty_init);
module_exit(gpiotty_exit);
