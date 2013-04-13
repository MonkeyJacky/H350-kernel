/*
 * PMU driver for act8600 PMU
 *
 * Copyright 2010 Ingenic Semiconductor LTD.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/module.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/mod_devicetable.h>
#include <linux/i2c.h>
#include <asm/jzsoc.h>
#include <linux/act8600_power.h>

#define DEBUG
#ifdef DEBUG
#define dprintk(x...) printk(x)
#else
#define dprintk(x...)
#endif

extern void i2c_jz_setclk(struct i2c_client *client,unsigned long i2cclk);


struct act8600_device {
	struct i2c_client *client;
};

struct act8600_device *act8600;

void act8600_start_recharging(void)
{
	char intr0;
	act8600_read_reg(ACT8600_APCH_INTR0,&intr0);
	act8600_write_reg(ACT8600_APCH_INTR0,intr0 | ACT8600_APCH_INTR0_SUS);
	msleep(5);
	act8600_write_reg(ACT8600_APCH_INTR0,intr0 & ~(ACT8600_APCH_INTR0_SUS));
}
EXPORT_SYMBOL_GPL(act8600_start_recharging);

int act8600_set_double_q3(int enable)//when set to 1 , usb charg = 800MA ,or 400MA
{
	char otg_con;
	act8600_read_reg(ACT8600_OTG_CON,&otg_con);
	if(enable)
		return act8600_write_reg(ACT8600_OTG_CON,otg_con | ACT8600_OTG_CON_DBLIMITQ3);
	return act8600_write_reg(ACT8600_OTG_CON,otg_con & ~(ACT8600_OTG_CON_DBLIMITQ3));
}
EXPORT_SYMBOL_GPL(act8600_set_double_q3);

int act8600_set_q1(int enable)
{
	char otg_con;
	//when Q1 enabled,Q2 and Q3 must disabled
	if(enable)
		return act8600_write_reg(ACT8600_OTG_CON,ACT8600_OTG_CON_Q1);
	act8600_read_reg(ACT8600_OTG_CON,&otg_con);
	return act8600_write_reg(ACT8600_OTG_CON,otg_con & ~(ACT8600_OTG_CON_Q1));
}
EXPORT_SYMBOL_GPL(act8600_set_q1);

int act8600_set_q3(int enable)
{
	char otg_con;
	if(enable)
		return act8600_write_reg(ACT8600_OTG_CON,ACT8600_OTG_CON_Q3);
	act8600_read_reg(ACT8600_OTG_CON,&otg_con);
	return act8600_write_reg(ACT8600_OTG_CON,otg_con & ~(ACT8600_OTG_CON_Q3));
}
EXPORT_SYMBOL_GPL(act8600_set_q3);

int act8600_write_reg(char reg,char val)
{
	char msg[2];  

	memcpy(&msg[0], &reg, 1);      
	memcpy(&msg[1], &val, 1);      

	return i2c_master_send(act8600->client, msg, 2);
}
EXPORT_SYMBOL_GPL(act8600_write_reg);

int act8600_read_reg(char reg,char *val)
{
	i2c_master_send(act8600->client,&reg,1);
	return i2c_master_recv(act8600->client, val, 1);
}
EXPORT_SYMBOL_GPL(act8600_read_reg);

int act8600_output_set(int outnum,int regvalue)
{
	char reg;
	switch(outnum) {
		case ACT8600_OUT1:reg = ACT8600_REG1_VSET;break;
		case ACT8600_OUT2:reg = ACT8600_REG2_VSET;break;
		case ACT8600_OUT3:reg = ACT8600_REG3_VSET;break;
		case ACT8600_OUT4:reg = ACT8600_REG4_VSET;break;
		case ACT8600_OUT5:reg = ACT8600_REG5_VSET;break;
		case ACT8600_OUT6:reg = ACT8600_REG6_VSET;break;
		case ACT8600_OUT7:reg = ACT8600_REG7_VSET;break;
		case ACT8600_OUT8:reg = ACT8600_REG8_VSET;break;
		default:return -1;
	}
	
	return act8600_write_reg(reg,regvalue);
}

int act8600_output_enable(int outnum,int enable)
{
	char reg,value,timeout;
	switch(outnum) {
		case ACT8600_OUT1:reg = ACT8600_REG1_VCON;break;
		case ACT8600_OUT2:reg = ACT8600_REG2_VCON;break;
		case ACT8600_OUT3:reg = ACT8600_REG3_VCON;break;
		case ACT8600_OUT4:reg = ACT8600_REG4_VCON;break;
		case ACT8600_OUT5:reg = ACT8600_REG5_VCON;break;
		case ACT8600_OUT6:reg = ACT8600_REG6_VCON;break;
		case ACT8600_OUT7:reg = ACT8600_REG7_VCON;break;
		case ACT8600_OUT8:reg = ACT8600_REG8_VCON;break;
		default:return -1;
	}

	act8600_read_reg(reg,&value);
	if(enable) {
		value |= ACT8600_REG_VCON_ON;
		if(outnum >= ACT8600_OUT5 && outnum <= ACT8600_OUT8)
			value &= ~ACT8600_REG_VCON_DIS;
	} else {
		value &= ~ACT8600_REG_VCON_ON;
		if(outnum >= ACT8600_OUT5 && outnum <= ACT8600_OUT8)
			value |= ACT8600_REG_VCON_DIS;
	}

	act8600_write_reg(reg,value);

	for(timeout=100;timeout<=0;--timeout) {
		act8600_read_reg(reg,&value);
		msleep(10);
		if(!(value & ACT8600_REG_VCON_OK))
			printk("out%d is not stable,wait 10 msec.\n",outnum);
		else
			break;
	}
	if(timeout == 0)
		printk("out%d set failed , this may cause system halt.\n",outnum);
	return timeout;
}
EXPORT_SYMBOL_GPL(act8600_output_enable);

static int act8600_probe(struct i2c_client *i2c,const struct i2c_device_id *id)
{
	int i;
	struct act8600_platform_pdata_t *pdata = i2c->dev.platform_data;

	act8600 = kzalloc(sizeof(struct act8600_device), GFP_KERNEL);
	if (act8600 == NULL) {
		return -ENOMEM;
	}

	act8600->client = i2c;
	i2c_set_clientdata(i2c, act8600);
	i2c_jz_setclk(i2c,100*1000);

	//call this function to enable q3 and disable q1&q2;
	//usb driver will set q1&q3 later
	act8600_set_q3(1);
	act8600_set_double_q3(0);

#if defined(CONFIG_SOC_JZ4770)
	/*new cpu need rtc power 3.3v , use pmu default value */
//	if(strcmp(jz4770_cpu_version,JZ4770_V1)) { 
		/** change rtc power to 2.35V **/
		act8600_write_reg(0x91,0xd0);
		act8600_write_reg(0x90,0x17);
		act8600_write_reg(0x91,0xc0);
//	}
#endif

	dprintk("act8600_power:\n");
	for (i = 0; i < pdata->nr_outputs; i++) {
		struct act8600_outputs_t *p = &pdata->outputs[i];
		act8600_output_set(p->no,p->value);
		act8600_output_enable(p->no,p->active_on);

		dprintk("%d\t\t%d\t\t%d\n",p->no,p->value,p->active_on);
	}
	return 0;
}

static int act8600_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id act8600_id[] = {
	{ ACT8600_NAME, 0 },
	{ }
};

static struct i2c_driver act8600_pmu_driver = {
	.probe		= act8600_probe,
	.remove		= act8600_remove,
	.id_table	= act8600_id,
	.driver = {
		.name	= ACT8600_NAME,
		.owner	= THIS_MODULE,
	},
};

static int __devinit act8600_pmu_init(void)
{	
	return i2c_add_driver(&act8600_pmu_driver);
}

static void __exit act8600_pmu_exit(void)
{
	i2c_del_driver(&act8600_pmu_driver);
}

//arch_initcall(act8600_pmu_init);
//medive change
subsys_initcall(act8600_pmu_init);
module_exit(act8600_pmu_exit);

MODULE_DESCRIPTION("ACT8600 PMU Driver");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("ztyan@ingenic.cn");

