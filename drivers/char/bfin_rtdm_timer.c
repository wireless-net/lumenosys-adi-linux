/* bfin_rtdm_timer.c --- 
 * 
 * Filename: bfin_rtdm_timer.c
 * Description: 
 * Author: Devin Butterfield
 * Maintainer: 
 * Created: Sat Feb  1 12:00:06 2014 (-0800)
 * Version: 
 * Last-Updated: Fri Feb  7 21:56:24 2014 (-0800)
 *           By: Devin Butterfield
 *     Update #: 121
 * URL: 
 * Keywords: 
 * Compatibility: 
 * 
 */

/* Commentary: 
 * 
 * 
 * 
 */

/* Change Log:
 * 
 * 
 */

/* *This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 3, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; see the file COPYING.  If not, write to
 * the Free Software Foundation, Inc., 51 Franklin Street, Fifth
 * Floor, Boston, MA 02110-1301, USA.
 */

/* Code: */
#include <linux/module.h>
#include <rtdm/rtdm_driver.h>

#include <asm/gptimers.h>
#include <asm/bfin_rtdm_timer.h>
#include <asm/bfin-global.h>
#include <asm/portmux.h>

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Devin Butterfield <db@lumenosys.com>");

#define DEVICE_NAME			"bfin_rtdm_timer"
#define SOME_SUB_CLASS			4711

struct timer {
  unsigned short id, bit;
  unsigned short per_pin;
  unsigned long irqbit, isr_count;
  int irq;
};

static struct timer timer_code[MAX_BLACKFIN_GPTIMERS] = {
	{TIMER0_id,  TIMER0bit,  P_TMR0, TIMER_STATUS_TIMIL0, 0, IRQ_TIMER0},
	{TIMER1_id,  TIMER1bit,  P_TMR1, TIMER_STATUS_TIMIL1, 0, IRQ_TIMER1},
	{TIMER2_id,  TIMER2bit,  P_TMR2, TIMER_STATUS_TIMIL2, 0, IRQ_TIMER2},
#if (MAX_BLACKFIN_GPTIMERS > 3)
	{TIMER3_id,  TIMER3bit,  P_TMR3, TIMER_STATUS_TIMIL3, 0, IRQ_TIMER3},
	{TIMER4_id,  TIMER4bit,  P_TMR4, TIMER_STATUS_TIMIL4, 0, IRQ_TIMER4},
	{TIMER5_id,  TIMER5bit,  P_TMR5, TIMER_STATUS_TIMIL5, 0, IRQ_TIMER5},
	{TIMER6_id,  TIMER6bit,  P_TMR6, TIMER_STATUS_TIMIL6, 0, IRQ_TIMER6},
	{TIMER7_id,  TIMER7bit,  P_TMR7, TIMER_STATUS_TIMIL7, 0, IRQ_TIMER7},
#endif
#if (MAX_BLACKFIN_GPTIMERS > 8)
	{TIMER8_id,  TIMER8bit,  P_TMR8, TIMER_STATUS_TIMIL8, 0, IRQ_TIMER8},
	{TIMER9_id,  TIMER9bit,  P_TMR9, TIMER_STATUS_TIMIL9, 0, IRQ_TIMER9},
	{TIMER10_id, TIMER10bit, P_TMR10, TIMER_STATUS_TIMIL10, 0, IRQ_TIMER10},
#if (MAX_BLACKFIN_GPTIMERS > 11)
	{TIMER11_id, TIMER11bit, P_TMR11, TIMER_STATUS_TIMIL11, 0, IRQ_TIMER11},
#endif
#endif
};

struct bfin_rtdm_timer_context {
  rtdm_irq_t irq_handle;		/* device IRQ handle */
  rtdm_lock_t lock;		/* lock to protect context struct */

  rtdm_event_t in_event;		/* raised to unblock reader */

  struct timer *timer;
};

static struct rtdm_device *device[MAX_BLACKFIN_GPTIMERS];


/*
 * timer driver ioctl
 * 
 */
int bfin_rtdm_timer_ioctl(struct rtdm_dev_context *context,
                          rtdm_user_info_t * user_info,
                          unsigned int request, void *arg)
{
  //  rtdm_lockctx_t lock_ctx;
  struct bfin_rtdm_timer_context *ctx;
  struct timer *t;
  unsigned long period = 0, width = 0, mode = 0;  
  int err;
  
  ctx = (struct bfin_rtdm_timer_context *)context->dev_private;
  t = ctx->timer;

  switch (request) {
  case BFIN_SIMPLE_TIMER_SET_PERIOD:
    period = (unsigned long)arg;
    if (period < 2)
      return -EFAULT;
    set_gptimer_period(t->id, period);
    pr_debug(DEVICE_NAME ": TIMER_SET_PERIOD: period=%lu\n", period);
    break;
  case BFIN_SIMPLE_TIMER_SET_WIDTH:
    width = (unsigned long)arg;
    set_gptimer_pwidth(t->id, width);
    pr_debug(DEVICE_NAME ": TIMER_SET_WIDTH: width=%lu\n", width);
    break;
  case BFIN_SIMPLE_TIMER_SET_MODE:
    mode = (unsigned long)arg;
    switch (mode) {
      pr_debug(DEVICE_NAME ": TIMER_SET_MODE: mode %lu\n", mode);
    case BFIN_SIMPLE_TIMER_MODE_PWM_ONESHOT:
#ifdef CONFIG_BF60x
      set_gptimer_config(t->id,  TIMER_OUT_DIS | TIMER_MODE_PWM
                         /* | TIMER_PULSE_HI */ | TIMER_IRQ_WID_DLY);
#else
      set_gptimer_config(t->id, /* OUT_DIS | */PWM_OUT /* | PERIOD_CNT | IRQ_ENA */);
#endif
      break;
    case BFIN_SIMPLE_TIMER_MODE_PWMOUT_CONT:
#ifdef CONFIG_BF60x
      set_gptimer_config(t->id,  TIMER_MODE_PWM_CONT
                         /* | TIMER_PULSE_HI */ | TIMER_IRQ_PER);
#else
      set_gptimer_config(t->id,  PWM_OUT | PERIOD_CNT | IRQ_ENA);
#endif
      break;
    case BFIN_SIMPLE_TIMER_MODE_WDTH_CAP:
#ifdef CONFIG_BF60x
      set_gptimer_config(t->id, TIMER_MODE_WDTH | TIMER_IRQ_PER);
#else
      set_gptimer_config(t->id, WDTH_CAP | PERIOD_CNT | IRQ_ENA);
#endif
      break;
    case BFIN_SIMPLE_TIMER_MODE_PWMOUT_CONT_NOIRQ:
#ifdef CONFIG_BF60x
      set_gptimer_config(t->id,  TIMER_MODE_PWM_CONT
                         /* | TIMER_PULSE_HI */);
#else
      set_gptimer_config(t->id, PWM_OUT | PERIOD_CNT);
#endif
      break;
      
    case BFIN_SIMPLE_TIMER_MODE_PWM_ONESHOT_HIGH:
#ifdef CONFIG_BF60x
      set_gptimer_config(t->id,  TIMER_OUT_DIS | TIMER_MODE_PWM
                         | TIMER_PULSE_HI | TIMER_IRQ_WID_DLY);
#else
      set_gptimer_config(t->id, /* OUT_DIS | */ TIMER_PULSE_HI |PWM_OUT /* | PERIOD_CNT | IRQ_ENA */);
#endif
      break;
    case BFIN_SIMPLE_TIMER_MODE_PWMOUT_CONT_HIGH:
#ifdef CONFIG_BF60x
      set_gptimer_config(t->id,  TIMER_MODE_PWM_CONT
                         | TIMER_PULSE_HI | TIMER_IRQ_PER);
#else
      set_gptimer_config(t->id, TIMER_PULSE_HI | PWM_OUT | PERIOD_CNT | IRQ_ENA);
#endif
      break;
    case BFIN_SIMPLE_TIMER_MODE_PWMOUT_CONT_NOIRQ_HIGH:
#ifdef CONFIG_BF60x
      set_gptimer_config(t->id,  TIMER_MODE_PWM_CONT
                         | TIMER_PULSE_HI);
#else
      set_gptimer_config(t->id, TIMER_PULSE_HI | PWM_OUT | PERIOD_CNT);
#endif
      break;
      
    default:
      printk(DEVICE_NAME ": error: invalid mode\n");
    }
    break;
  case BFIN_SIMPLE_TIMER_START:
    period = get_gptimer_period(t->id);
    width = get_gptimer_pwidth(t->id);
    pr_debug(DEVICE_NAME ": TIMER_START: period=%lu, width=%lu\n",
           period, width);
    enable_gptimers(t->bit);
    break;
  case BFIN_SIMPLE_TIMER_STOP:
    disable_gptimers(t->bit);
    break;
  case BFIN_SIMPLE_TIMER_READ:
    period = t->isr_count;
    err = rtdm_safe_copy_to_user(user_info, arg, &period,
                                 sizeof(unsigned long));
    break;
  case BFIN_SIMPLE_TIMER_READ_COUNTER:
    period = get_gptimer_count(t->id);
    err = rtdm_safe_copy_to_user(user_info, arg, &period,
                                 sizeof(unsigned long));
    break;
  default:
    return -EINVAL;
  }

  return 0;
}

/*
 * Timer interrupt handler
 */
static int bfin_rtdm_timer_interrupt(rtdm_irq_t *irq_context)
{
  struct bfin_rtdm_timer_context *ctx;

  ctx = rtdm_irq_get_arg(irq_context, struct bfin_rtdm_timer_context);
  rtdm_lock_get(&ctx->lock);

  if (get_gptimer_intr(ctx->timer->id)) {
    clear_gptimer_intr(ctx->timer->id);
    ctx->timer->isr_count++;
        
    /* unblock any readers */
	rtdm_event_signal(&ctx->in_event);
  }
  
  rtdm_lock_put(&ctx->lock);
  
  return RTDM_IRQ_HANDLED;
}


/**
 * Open the device
 *
 * This function is called when the device shall be opened.
 *
 */
static int bfin_rtdm_timer_open(struct rtdm_dev_context *context,
				rtdm_user_info_t * user_info, int oflags)
{
  struct bfin_rtdm_timer_context *ctx;
  int dev_id = context->device->device_id;
  int err;
  //rtdm_lockctx_t lock_ctx;

  pr_debug("bfin_rtdm_timer_open(): called\n");

  ctx = (struct bfin_rtdm_timer_context *)context->dev_private;

  rtdm_lock_init(&ctx->lock);

  /* get context */
  ctx->timer = &timer_code[dev_id];

  ctx->timer->isr_count = 0;
  
  /* init the event */
  rtdm_event_init(&ctx->in_event, 0);
  
  /* request the IRQ */
  err = rtdm_irq_request(&ctx->irq_handle, ctx->timer->irq,
                         bfin_rtdm_timer_interrupt, RTDM_IRQTYPE_SHARED | RTDM_IRQTYPE_EDGE,
                         context->device->proc_name, ctx);
  if (err) {
    return err;
  }
  
  /* request the timer pin */
  err = peripheral_request(ctx->timer->per_pin, "RTDM timer driver");
  if (err) {
    printk(KERN_ERR "request pin(%d) failed\n", ctx->timer->per_pin);
    rtdm_irq_free(&ctx->irq_handle);
    return err;
  }
  
  return 0;
}

/**
 * Close the device
 *
 * This function is called when the device shall be closed.
 *
 */
static int bfin_rtdm_timer_close(struct rtdm_dev_context *context,
				rtdm_user_info_t * user_info)
{
  	struct bfin_rtdm_timer_context *ctx;
	rtdm_lockctx_t lock_ctx;

    pr_debug("bfin_rtdm_timer_close(): called\n");
    
	ctx = (struct bfin_rtdm_timer_context *)context->dev_private;

	rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);

	disable_gptimers(ctx->timer->bit);
	peripheral_free(ctx->timer->per_pin);
    
	rtdm_lock_put_irqrestore(&ctx->lock, lock_ctx);

	rtdm_irq_free(&ctx->irq_handle);

    rtdm_event_destroy(&ctx->in_event);

    return 0;
}

/**
 * Read from the device
 *
 * This function is called when the device is read in realtime context.
 *
 */
static ssize_t bfin_rtdm_timer_read_rt(struct rtdm_dev_context *context,
                                       rtdm_user_info_t * user_info, void *buf,
                                       size_t nbyte)
{
  int ret, len;
  unsigned long data[2];
  struct bfin_rtdm_timer_context *ctx;
  //  rtdm_lockctx_t lock_ctx;

  pr_debug("bfin_rtdm_timer_read_rt(): called, sleeping... \n");

  if (nbyte == 0)
    return 0;
  
  if (user_info && !rtdm_rw_user_ok(user_info, buf, nbyte))
    return -EFAULT;
  
  ctx = (struct bfin_rtdm_timer_context *)context->dev_private;

  /* wait for data event */
  rtdm_event_wait(&ctx->in_event);

  //rtdm_lock_get_irqsave(&ctx->lock, lock_ctx);
  
  pr_debug("bfin_rtdm_timer_read_rt(): woke up!\n");

  /* read the current values (which were captured) */
  data[0] = get_gptimer_pwidth(ctx->timer->id);
  data[1] = get_gptimer_period(ctx->timer->id);

  /* don't give them more than they asked for */
  len = (nbyte < (2*sizeof(unsigned long))) ? nbyte : (2*sizeof(unsigned long));

  /* copy up to userspace */
  ret = rtdm_safe_copy_to_user(user_info, buf, data, len);
  
  /* if an error has occured, send it to user */
  if (ret) {
    printk("bfin_rtdm_timer_read_rt(): error in copy_to_user\n");
    return ret;
  }

  return len;
}

/**
 * Write in the device
 *
 * This function is called when the device is written in realtime context.
 *
 */
static ssize_t bfin_rtdm_timer_write_rt(struct rtdm_dev_context *context,
				     rtdm_user_info_t * user_info,
				     const void *buf, size_t nbyte)
{
  //int ret;
  /* struct bfin_rtdm_timer_context *ctx; */

  printk("bfin_rtdm_timer_write_rt(): called\n");
  
  /* if (nbyte == 0) */
  /*   return 0; */
  
  /* if (user_info && !rtdm_rw_user_ok(user_info, buf, nbyte)) */
  /*   return -EFAULT; */
  
  /* ctx = (struct bfin_rtdm_timer_context *)context->dev_private; */
  
  
  /* /\* write the user buffer in the kernel buffer *\/ */
  /* buffer.size = (nbyte > BFIN_RDTM_TIMER_SIZE_MAX) ? BFIN_RDTM_TIMER_SIZE_MAX : nbyte; */
  /* ret = rtdm_safe_copy_from_user(user_info, buffer.data, buf, buffer.size); */
  
  /* /\* if an error has occured, send it to user *\/ */
  /* if (ret) { */
  /*   printk("bfin_rtdm_timer_write_rt(): error in copy_from_user\n"); */
  /*   return ret; */
  /* } */
  
  
  /* release the semaphore */
  /* printk("bfin_rtdm_timer_read_rt(): releasing the reader\n"); */
  /* rtdm_sem_up(&ctx->sem); */
  
  return nbyte;
}

/**
 * The RTDM device template
 *
 */
static struct rtdm_device __initdata device_tmpl = {
  .struct_version = RTDM_DEVICE_STRUCT_VER,
  
  .device_flags = RTDM_NAMED_DEVICE,
  .context_size = sizeof(struct bfin_rtdm_timer_context),
  .device_name = "",          /* filled in by init */
  
  .open_nrt = bfin_rtdm_timer_open,
  //.open_rt  = bfin_rtdm_timer_open,
  
  .ops = {
    .close_nrt = bfin_rtdm_timer_close,
    //.close_rt  = bfin_rtdm_timer_close,
    .ioctl_rt  = bfin_rtdm_timer_ioctl,
    .ioctl_nrt = bfin_rtdm_timer_ioctl,
    .read_rt   = bfin_rtdm_timer_read_rt,
    .write_rt  = bfin_rtdm_timer_write_rt,
  },
  
  .device_class = RTDM_CLASS_EXPERIMENTAL,
  .device_sub_class = SOME_SUB_CLASS,
  .profile_version = 1,
  .driver_name = "bfin_rtdm_timer",
  .driver_version = RTDM_DRIVER_VER(0, 1, 0),
  .peripheral_name = "Blackfin Real-time Timer Driver",
  .provider_name = "Devin Butterfield",
};


void bfin_rtdm_timer_exit(void);

/**
 * This function is called when the module is loaded
 *
 * Initialize and register device instances
 *
 */
int __init bfin_rtdm_timer_init(void)
{
  struct rtdm_device *dev;
  int i, ret;

  printk(DEVICE_NAME ": Real-Time timer driver for the ADI Blackfin\n");
  for (i = 0; i < MAX_BLACKFIN_GPTIMERS; i++) {
    dev = kmalloc(sizeof(struct rtdm_device), GFP_KERNEL);
    if (!dev)
      return -ENOMEM;

    memcpy(dev, &device_tmpl, sizeof(struct rtdm_device));
    snprintf(dev->device_name, RTDM_MAX_DEVNAME_LEN, "bfin_rtdm_timer%d", i);
    dev->device_id = i;

    pr_debug(DEVICE_NAME ": setting up %s\n", dev->device_name);
    
    dev->proc_name = dev->device_name;

    /* do any other init stuff here */

    ret = rtdm_dev_register(dev);
    if (ret)
      goto cleanup_out;
    
    device[i] = dev;
  }

  return ret;
  

 cleanup_out:
  bfin_rtdm_timer_exit();
  
  return ret;
}

/**
 * This function is called when the module is unloaded
 *
 * It unregister the RTDM device, polling at 1000 ms for pending users.
 *
 */
void __exit bfin_rtdm_timer_exit(void)
{
  int i;
  
  for (i = 0; i < MAX_BLACKFIN_GPTIMERS; i++)
    if (device[i]) {
      rtdm_dev_unregister(device[i], 1000);
      kfree(device[i]);
    }
}

module_init(bfin_rtdm_timer_init);
module_exit(bfin_rtdm_timer_exit);

/* bfin_rtdm_timer.c ends here */
