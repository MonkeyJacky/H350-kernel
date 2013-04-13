/*
 * linux/drivers/char/jzchar/sensor.c
 *
 * Common G-Sensor Driver
 *
 * Copyright (C) 2006  Ingenic Semiconductor Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 */

#include <linux/module.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/interrupt.h>
#include <linux/major.h>
#include <linux/string.h>
#include <linux/fcntl.h>
#include <linux/mm.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/spinlock.h>
#include <linux/poll.h>
#include <linux/spinlock.h>
#include <asm/irq.h>
#include <asm/uaccess.h>
#include <asm/jzsoc.h>
#include <linux/proc_fs.h>

#include "jzchars.h"
#include "gsensor_i2c.h"

MODULE_AUTHOR("caijicheng@umidotech.com");
MODULE_DESCRIPTION("mxc6225xu Sensor Driver");
MODULE_LICENSE("GPL");

#define SENSOR_NAME	"mxc6225xu"
#define SENSOR_I2C_ADDR		0x15

char xout,yout;
#define READ_INTER 8


char read_xdata = 0x00;
char read_ydata = 0x01;

static void l009_gsensor_reset()
{
#if 0
	i2c_close();
	__cpm_stop_i2c();	
	__cpm_start_i2c();	
	i2c_open();
#else
	__i2c_disable(1);
	cpm_stop_clock(CGM_I2C1);
	cpm_start_clock(CGM_I2C1);
	__i2c_enable(1);
#endif
} 


unsigned int l009_gsensor_read()
{
	static unsigned int val = 0;
	static unsigned long old_jiffies = 0;
	
	if((jiffies - old_jiffies) < READ_INTER) 
	return val;
	else
	old_jiffies = jiffies;
	
	val = 0;
        int ret = 0;
        read_xdata = 0x00;
        ret = gsensor_i2c_rxdata((char *)&read_xdata, 1); // read x
        if(ret < 0)
        {
          l009_gsensor_reset();
          return val;
        }
	read_ydata = 0x01;
        ret = gsensor_i2c_rxdata((char *)&read_ydata, 1); // read y
	if(ret < 0)
        {
          l009_gsensor_reset();
          return val;
        }

	//printk("gsensor read xdata is  %d  ydata is %d\n",read_xdata,read_ydata);

	if(read_ydata > 12 )	    val |= 0x02;    //UP
	if(read_ydata < -12 )     val |= 0x01;    //DOWN
	if(read_xdata < -12 )     val |=  0x04;   //LEFT
	if(read_xdata > 12 )      val |=  0x08;   //RIGHT
	return val;
}
EXPORT_SYMBOL(l009_gsensor_read);

unsigned int l009_gsensor_flag = 0;
EXPORT_SYMBOL(l009_gsensor_flag);

static int proc_gsensor_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	return sprintf(page, "%lu\n", l009_gsensor_flag);
}

static int proc_gsensor_write_proc(
			struct file *file, const char *buffer,
			unsigned long count, void *data)
{
	l009_gsensor_flag =  simple_strtoul(buffer, 0, 10);

	if(l009_gsensor_flag)
	{
		//__gpio_as_output0(GPIO_GSENSOR_VCC);
		//mdelay(2000);
		cpm_start_clock(CGM_I2C1);
		__i2c_disable(1);
		__i2c_enable(1);
	}
	else
	{
		//mdelay(100);
		__i2c_disable(1);
		cpm_stop_clock(CGM_I2C1);
		//__gpio_as_output1(GPIO_GSENSOR_VCC);

		mdelay(10);
	}
	return count;
}

/*  AMP and Headphone */
#define HP_DETE_IRQ  (IRQ_GPIO_0 + GPIO_HP_PIN)
static int hp_in;
static unsigned int sound_flag = 0;  //default the amp is off
static struct timer_list hp_irq_timer;
static unsigned int crypt_flag = 0;
extern void crypt_reset();

static int proc_amp_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	return sprintf(page, "%lu\n", sound_flag);
}
static int proc_hp_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	return sprintf(page, "%lu\n", hp_in);
}


static int proc_hp_write_proc(
			struct file *file, const char *buffer,
			unsigned long count, void *data)
{

  return count;
}

#ifdef CRYPT_CHECK
static int proc_crypt_read_proc(
		char *page, char **start, off_t off,
		int count, int *eof, void *data)
{


	__gpio_as_i2c(0);
	mdelay(10);

	return sprintf(page, "%lu\n", crypt_flag);
}


static int proc_crypt_write_proc(
			struct file *file, const char *buffer,
			unsigned long count, void *data)
{

  return count;
}
#endif


static int wifi_pw_on_flag = 0;
static int proc_wifi_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	return sprintf(page, "%lu\n", wifi_pw_on_flag);
}


static int proc_wifi_write_proc(
		struct file *file, const char *buffer,
		unsigned long count, void *data)
{

	wifi_pw_on_flag =  simple_strtoul(buffer, 0, 10);
	printk("\n wifi pw flag = %d\n",wifi_pw_on_flag);
	if (wifi_pw_on_flag){
		__gpio_clear_pin(GPIO_WIFI_PW);
	}else{
		__gpio_set_pin(GPIO_WIFI_PW);
	}

	return count;
}
static int moto_pw_on_flag = 0;
static int proc_moto_read_proc(
			char *page, char **start, off_t off,
			int count, int *eof, void *data)
{
	return sprintf(page, "%lu\n", moto_pw_on_flag);
}


static int proc_moto_write_proc(
		struct file *file, const char *buffer,
		unsigned long count, void *data)
{

	moto_pw_on_flag =  simple_strtoul(buffer, 0, 10);
	//printk("\n medive moto pw flag = %d\n",moto_pw_on_flag);
	if (moto_pw_on_flag){
		__gpio_set_pin(GPIO_MOTO_PW);
	}else{
		__gpio_clear_pin(GPIO_MOTO_PW);
	}

	return count;
}


	extern unsigned int l009_globle_volume;
int is_close_amp_hp = 1;
static int proc_amp_write_proc(
			struct file *file, const char *buffer,
			unsigned long count, void *data)
{
		sound_flag =  simple_strtoul(buffer, 0, 10);
		//printk("\n medive printk amp write proc! sound_flag = %d\n",sound_flag);

		if(sound_flag == 1)   //sound on
		{
			if(hp_in == 0 && l009_globle_volume)
				__gpio_set_pin(GPIO_AMPEN);

			//__gpio_clear_pin(GPIO_HP_OFF);
			is_close_amp_hp = 0;
		}
		else if(sound_flag == 0)  //mute
		{
			__gpio_clear_pin(GPIO_AMPEN);
			
			//__gpio_set_pin(GPIO_HP_OFF);
			is_close_amp_hp = 1;
		}
		else
			;
}

extern void dlv_disable_hp_out();
extern void dlv_enable_hp_out();
extern void dlv_disable_line_out();
extern void dlv_enable_line_out();
extern void dlv_disable_hp_mute();
static void
hp_ack_timer(unsigned long data)
{


	//printk("\n l009  globle volume = %d\n",l009_globle_volume);
	if(__gpio_get_pin(GPIO_HP_PIN))  //HP OFF
	{
		//printk("hp_pnp_irq----, hp off\n");

		dlv_disable_hp_out();
		hp_in = 0;
		if(l009_globle_volume != 0)
		{
			dlv_enable_line_out();
			__gpio_set_pin(GPIO_AMPEN);	
		}else
			__gpio_clear_pin(GPIO_AMPEN);	

	}
	else  //HP ON
	{
		//printk("hp_pnp_irq----, hp on\n");

		dlv_disable_line_out();
		dlv_enable_hp_out();
		hp_in = 1;
		if(l009_globle_volume != 0)
		{
			dlv_disable_hp_mute();
			__gpio_clear_pin(GPIO_AMPEN);	
		}
	}

	hp_irq_timer.expires = jiffies + HZ*3;

	add_timer(&hp_irq_timer);

}

static irqreturn_t hp_pnp_irq(int irq, void *dev_id)
{
	/* mask interrupt */
	__gpio_mask_irq(GPIO_HP_PIN); 

		hp_irq_timer.expires = jiffies + HZ;
		del_timer(&hp_irq_timer);
		add_timer(&hp_irq_timer);
	
		return IRQ_HANDLED;
}

/*
 * Module init and exit
 */

static int __init sensor_init(void)
{


	int ret;
	struct proc_dir_entry *res, *res2,*res_hp,*res3,*res4,*res5,*res6;

#if 0
	__gpio_as_func0(GPIO_GSENSOR_VCC);
	__gpio_disable_pull(GPIO_GSENSOR_VCC);
	__gpio_as_output1(GPIO_GSENSOR_VCC);
#endif




#if 0
		__gpio_as_output0(32*5+13);  //init
		__gpio_as_output0(32*4+31);
		__gpio_as_output0(32*4+30);
		udelay(200);
#endif

		__gpio_as_i2c(1);
		udelay(200);
#if 0
		cpm_start_clock(CGM_I2C1);

		__i2c_disable(1);
		__i2c_enable(1);
#endif


		res = create_proc_entry("jz/gsensor", 0, NULL);
		if(res)
		{
			res->read_proc = proc_gsensor_read_proc;
			res->write_proc = proc_gsensor_write_proc;	
			res->data   = NULL;
		}	

		//maddrone add HeadPhone init here
		__gpio_clear_pin(GPIO_AMPEN);	

		__gpio_as_input(GPIO_HP_PIN);
		//__gpio_enable_pull(GPIO_HP_PIN);
		__gpio_disable_pull(GPIO_HP_PIN);
		udelay(1);

		if(__gpio_get_pin(GPIO_HP_PIN))  //HP OFF
		{
			hp_in = 0;

			printk("++++++++++++ HP OUT +++++++++++++\n");
		}
		else  //HP ON
		{
			hp_in = 1;

			printk("++++++++++++ HP IN +++++++++++++\n");
		}



		init_timer(&hp_irq_timer);
		hp_irq_timer.function = hp_ack_timer;
		hp_irq_timer.data = 0;
		hp_irq_timer.expires = jiffies + HZ;
		add_timer(&hp_irq_timer);



		res2 = create_proc_entry("jz/amp", 0, NULL);
		if(res2)
		{
			res2->read_proc = proc_amp_read_proc;
			res2->write_proc = proc_amp_write_proc;	
			res2->data = NULL;
		}

		res3 = create_proc_entry("jz/wifi_pw", 0, NULL);
		if(res3)
		{
			res3->read_proc = proc_wifi_read_proc;
			res3->write_proc = proc_wifi_write_proc;	
			res3->data = NULL;
		}
		res4 = create_proc_entry("jz/moto_pw", 0, NULL);
		if(res4)
		{
			res4->read_proc = proc_moto_read_proc;
			res4->write_proc = proc_moto_write_proc;	
			res4->data = NULL;
		}
		res5 = create_proc_entry("jz/hp_l009", 0, NULL);
		if(res5)
		{
			res5->read_proc = proc_hp_read_proc;
			res5->write_proc = proc_hp_write_proc;	
			res5->data = NULL;
		}
		return 0;
}

static void __exit sensor_exit(void)
{
}

module_init(sensor_init);
module_exit(sensor_exit);
