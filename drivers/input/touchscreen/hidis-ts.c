/*
 * linux/drivers/input/touchscreen/aesop-v210-ts.c
 *
 * aESOP S5PV210 platform Touch Screen Driver
 *
 * Copyright (C) 2010 aESOP Embedded Forum (http://www.aesop.or.kr)
 * Written by Jhoonkim <jhoon_kim@nate.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 */
 
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/pm.h>
#include <linux/i2c.h>
//#include <linux/i2c-id.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/kthread.h>

#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#include <linux/earlysuspend.h>
#include <linux/suspend.h>
#endif
#include <asm/io.h>
#include <asm/irq.h>
#include <mach/hardware.h>
#include <mach/irqs.h>

#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/io.h>
#include <mach/gpio.h>
#include <mach/map.h>
#include <asm/delay.h>

#include <mach/regs-gpio.h>
#include <mach/regs-clock.h>
//#include <mach/gpio-bank.h>
#include <plat/ts.h>
#include <mach/irqs.h>
#include <plat/irqs.h>
#include <plat/gpio-cfg.h>

#define DEBUG 1

/* General Touch Screen Information */
#define HIDIS_VERSION		0x0101
#define HIDIS_VENDOR		0xDEAD
#define HIDIS_PRODUCT		0xBEEF

#define IRQ_EINT11 			11+32 // datasheet pg#609
#define HIDIS_IRQ_NUM		IRQ_EINT11
#define HIDIS_X_RESOLUTION	480
#define HIDIS_Y_RESOLUTION	800
#define TOUCH_READ_TIME		( (HZ/100) )  // 10ms

#define S5PV210_GPH1DAT S5P_VA_GPIO+(0xC24)

/* Defined for use k_thread */ 
wait_queue_head_t idle_wait;
static struct task_struct *kidle_task;

/* Touch Screen Private Data */
static struct hidis_priv2
{
	struct delayed_work	task_work;
	struct i2c_client 	*i2c_client;
	struct i2c_driver 	hidis_i2c_driver;
	struct s3c2410ts 	*ts;
	struct timer_list	pwn_down_check_timer;

	long ts_xp_old;
	long ts_yp_old;

	unsigned char touch_irq_flag;
	unsigned char touch_ready_flag;
	unsigned int interval;
	unsigned int pen_down;
//	unsigned int pen_downed;
//	unsigned long jiffies_old;

} *p_hidis_priv2;


static struct hidis_priv
{
	struct delayed_work	task_work;
	struct i2c_client 	*i2c_client;
	struct i2c_driver 	hidis_i2c_driver;

	struct device *dev;
	struct input_dev *input;
	struct clk *clock;
	void __iomem *io;
	unsigned long xp;
	unsigned long yp;
	int irq_tc;
	int count;
	int shift;
	int features;

	struct timer_list	pwn_down_check_timer;

	long ts_xp_old;
	long ts_yp_old;

	unsigned char touch_irq_flag;
	unsigned char touch_ready_flag;
	unsigned int interval;
	unsigned int pen_down;

} *p_hidis_priv;



static inline void *hidis_priv_get_i2c_client(const struct hidis_priv *priv)
{
	return priv->i2c_client;
}

#ifdef TS_ANDROID_PM_ON
#ifdef CONFIG_HAS_WAKELOCK
static void ts_early_suspend(struct early_suspend *h)
{
	ts = container_of(h, struct s3c_ts_info, early_suspend);
	hidis_i2c_suspend(NULL, PMSG_SUSPEND); // platform_device is now used
}
static void ts_late_resume(struct early_suspend *h)
{
	ts = container_of(h, struct s3c_ts_info, early_suspend);
	hidis_i2c_resume(NULL); // platform_device is now used
}
#endif
#endif //TS_ANDROID_PM_ON

#ifdef CONFIG_HAS_WAKELOCK
extern suspend_state_t get_suspend_state(void);
#endif

/* Touch Screen Read Data Position 
 *
 * Protocol Spec (7bit I2C Address).
 * X-Position MSB : LSB 4bit (0x22)
 * X-Position LSB : LSB 8bit (0x20)
 * Y-Position MSB : MSB 4bit (0x21)
 * Y-Position LSB : LSB 8bit (0x22)
 *
 */
static void hidis_i2c_read_data_position(void)
{
	long i,j;
	unsigned char x_pos_lsb, x_y_pos_msb, y_pos_lsb;
	unsigned char x_pos_msb, y_pos_msb;
	struct i2c_client *client = p_hidis_priv->i2c_client; // hidis_priv_get_i2c_client(p_hidis_priv);
		
	x_pos_lsb = i2c_smbus_read_byte_data(client, 0x20);
	dev_dbg(&client->dev, "%s : Check hidis ts 0x20 = %x \n", __func__, x_pos_lsb);

	x_y_pos_msb = i2c_smbus_read_byte_data(client, 0x21);
	dev_dbg(&client->dev, "%s : Check hidis ts 0x21 = %x \n", __func__, x_y_pos_msb);

	y_pos_lsb = i2c_smbus_read_byte_data(client, 0x22);
	dev_dbg(&client->dev, "%s : Check hidis ts 0x22 = %x \n", __func__, y_pos_lsb);

	x_pos_msb = (x_y_pos_msb & 0xf);
	y_pos_msb = (x_y_pos_msb >> 4) & 0xf;

	dev_dbg(&client->dev,"%s : x_pos_msb - %x, y_pos_msb - %x\n", __func__, x_pos_msb, y_pos_msb);

	p_hidis_priv->xp = (unsigned long)((x_pos_msb << 8) | (x_pos_lsb & 0x00ff));
	p_hidis_priv->yp = (unsigned long)((y_pos_msb << 8) | (y_pos_lsb & 0x00ff));

	i = (x_pos_msb << 8) | (x_pos_lsb & 0xff);
        j = (y_pos_msb << 8) | (y_pos_lsb & 0xff);

	dev_dbg(&client->dev, "%s : TS Pos X - %ld(%lx), TS Pos Y - %ld(%lx)\n",
			__func__, p_hidis_priv->xp, p_hidis_priv->xp, p_hidis_priv->yp, p_hidis_priv->yp);
} 

/* Touch Screen Report Data Position for Applications */ 
static void hidis_touch_report_position(void)
{
	//struct s3c2410ts *ts = hidis_priv_get_s3c_ts_dev(p_hidis_priv);

	if((p_hidis_priv->xp > 0) && (p_hidis_priv->yp > 0) &&
		(p_hidis_priv->xp <= HIDIS_X_RESOLUTION) && (p_hidis_priv->yp <= HIDIS_Y_RESOLUTION)) {
			input_report_abs(p_hidis_priv->input, ABS_X, p_hidis_priv->xp);
			input_report_abs(p_hidis_priv->input, ABS_Y, p_hidis_priv->yp);
			input_report_key(p_hidis_priv->input, BTN_TOUCH, 1);
			input_report_abs(p_hidis_priv->input, ABS_PRESSURE, 1);
			p_hidis_priv->ts_xp_old = p_hidis_priv->xp;
			p_hidis_priv->ts_yp_old = p_hidis_priv->yp;
	}
	else {
		p_hidis_priv->xp = 0;
		p_hidis_priv->yp = 0;
	}
	input_sync(p_hidis_priv->input);
}

/* Kernel Thread, Touch Screen Read Data Position on Interrupt */
int thread_touch_report_position(void *kthread)
{
	unsigned long flags;
	unsigned int ret;

	struct i2c_client *client = p_hidis_priv->i2c_client; //hidis_priv_get_i2c_client(p_hidis_priv);

	init_waitqueue_head(&idle_wait);
	
	dev_info(&client->dev, "%s : Touch Report thread Initialized.\n", __func__);	
	
	do {
		if(p_hidis_priv->pen_down) {
			dev_dbg(&client->dev, "%s : Touch Report thread : pen-down\n", __func__);	
			
			
			// ret = interruptible_sleep_on_timeout(&idle_wait, TOUCH_READ_TIME);
			ret = wait_event_interruptible_timeout(idle_wait,0,TOUCH_READ_TIME); 
			if( ret ) {
				dev_dbg(&client->dev, "%s : Homing routine Fatal kernel error or EXIT to run.\n", __func__);	
			}
			else {
				local_irq_save(flags);
				hidis_i2c_read_data_position();
				hidis_touch_report_position();
				local_irq_restore(flags);		
			}				
		}
		else {
			dev_dbg(&client->dev, "%s : Touch Report thread : Pen-up\n", __func__);
			interruptible_sleep_on(&idle_wait);	
		}
		
	} while (!kthread_should_stop());
	
	return 0;
}

/* Touch Screen Interrupt Handler */
static irqreturn_t hidis_touch_isr(int irq, void *dev_id)
{
	struct i2c_client *client = p_hidis_priv->i2c_client;
	
	unsigned int ret_val;

	ret_val = readl(S5PV210_GPH1DAT) & (0x1 << 3);
	p_hidis_priv->pen_down = (ret_val >> 3) & 0x1;

	if(p_hidis_priv->pen_down) {		/* Touch Pen Down */
		dev_dbg(&client->dev, "%s : Touch pen-down IRQ Occured - %x\n", __func__, p_hidis_priv->pen_down);
		wake_up_interruptible(&idle_wait);
	}
	else {							/* Touch Pen Up */
		dev_dbg(&client->dev, "%s : Touch pen-up IRQ Occured - %x\n", __func__, p_hidis_priv->pen_down);

		input_report_abs(p_hidis_priv->input, ABS_X, p_hidis_priv->ts_xp_old);
		input_report_abs(p_hidis_priv->input, ABS_Y, p_hidis_priv->ts_yp_old);
		input_report_key(p_hidis_priv->input, BTN_TOUCH, 0);
		input_report_abs(p_hidis_priv->input, ABS_PRESSURE, 0);
		input_sync(p_hidis_priv->input);

		p_hidis_priv->xp = 0;
		p_hidis_priv->yp = 0;
	} 

	return IRQ_HANDLED;	
}

static int __devexit hidis_i2c_remove(struct i2c_client *client)
{
        free_irq(HIDIS_IRQ_NUM, NULL);
	return 0;
}

static void hidis_i2c_early_suspend(struct early_suspend *h)
{
        disable_irq(HIDIS_IRQ_NUM);
        return ;
}

static void hidis_i2c_early_resume(struct early_suspend *h)
{
        enable_irq(HIDIS_IRQ_NUM);
	return ;
}

static int hidis_i2c_suspend(struct i2c_client *client, pm_message_t state)
{
        disable_irq(HIDIS_IRQ_NUM);
        return 0;
}

static int hidis_i2c_resume(struct i2c_client *client)
{
        enable_irq(HIDIS_IRQ_NUM);
        return 0;
}

/* Touch Screen Driver Initialize */
static int __devinit hidis_i2c_probe(struct i2c_client *client, const struct i2c_device_id *i2c_id)
{
	int ret, err;

	p_hidis_priv =  kzalloc(sizeof(struct hidis_priv), GFP_KERNEL);
	p_hidis_priv->interval = msecs_to_jiffies(30);
	p_hidis_priv->input = input_allocate_device();

	if (!p_hidis_priv->input)
	{
		dev_err(&client->dev, "Failed to allocate input device.\n");
		err = -ENOMEM;
		
		return err;
	}

	p_hidis_priv->input->evbit[0] = BIT_MASK(EV_SYN) | BIT_MASK(EV_KEY) | BIT_MASK(EV_ABS);
	p_hidis_priv->input->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);

	input_set_abs_params(p_hidis_priv->input, ABS_X, 0, HIDIS_X_RESOLUTION, 0, 0);
	input_set_abs_params(p_hidis_priv->input, ABS_Y, 0, HIDIS_Y_RESOLUTION, 0, 0);
	input_set_abs_params(p_hidis_priv->input, ABS_PRESSURE, 0, 1, 0, 0);


	p_hidis_priv->input->name = client->name;
	p_hidis_priv->input->id.bustype = BUS_HOST;
	p_hidis_priv->input->dev.parent = &client->dev;
	
	//input_set_drvdata(ts->input, ts);

	p_hidis_priv->input->id.vendor = HIDIS_VERSION;
	p_hidis_priv->input->id.product = HIDIS_PRODUCT;
	p_hidis_priv->input->id.version = HIDIS_VERSION;


	p_hidis_priv->i2c_client = client;

	ret = request_irq(HIDIS_IRQ_NUM, hidis_touch_isr, IRQ_TYPE_EDGE_BOTH, "hidis-touch", NULL);
	
	if (ret) {
		dev_err(&client->dev, "request_irq failed (IRQ_TOUCH)!\n");
		ret = -EIO;
	}

	/* All went ok, so register to the input system */
	ret = input_register_device(p_hidis_priv->input);

	kidle_task = kthread_run(thread_touch_report_position, NULL, "kidle_timeout");
	
	if( IS_ERR(kidle_task) ) {
		dev_err(&client->dev, "HIDIS Touch Screen Driver Initialize Faled.\n");
		return -EIO;
	}
	dev_info(&client->dev, "aESOP S5PV210 Touch Screen Driver Initialized.\n");

	return 0;
}

static const struct i2c_device_id hidis_ids[] = {
        { "hidis-ts", 0 },
        { },
};
MODULE_DEVICE_TABLE(i2c, hidis_ids);

static struct i2c_driver hidis_i2c_driver = {
	.probe		= hidis_i2c_probe,
	.remove		= __devexit_p(hidis_i2c_remove),
	.driver 	=
    		{
    		.name = "hidis-touchscreen",
    		},
	.id_table 	= hidis_ids,
	.suspend	= hidis_i2c_suspend,
	.resume		= hidis_i2c_resume,
};

static int __init hidis_i2c_init(void)
{
	return i2c_add_driver(&hidis_i2c_driver);
}

static void __exit hidis_i2c_exit(void)
{
	i2c_del_driver(&hidis_i2c_driver);
	return;
}

module_init(hidis_i2c_init)
module_exit(hidis_i2c_exit)

MODULE_DESCRIPTION("aESOP S5PV210 Touch Driver");
MODULE_AUTHOR("JhoonKim");
MODULE_VERSION("1.2");
MODULE_LICENSE("GPL");
