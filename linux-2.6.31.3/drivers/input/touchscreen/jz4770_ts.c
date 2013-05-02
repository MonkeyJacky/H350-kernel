/*
 * JZ Touch Screen Driver
 *
 * Copyright (c) 2005 - 2009  Ingenic Semiconductor Inc.
 *
 * Author: Jason <xwang@ingenic.cn> 20090219
 *         Regen <lhhuang@ingenic.cn> 20090324 add adkey
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 */

#include <linux/init.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>
#include <linux/stat.h>
#include <linux/seq_file.h>

#include <asm/mach-jz4770/jz4770sadc.h>
#include <asm/irq.h>
#include <asm/gpio.h>
#include <asm/jzsoc.h>
#include <asm/uaccess.h>

//#undef DEBUG
#define DEBUG

#ifdef DEBUG
#define dprintk(msg...)	printk("jz-sadc: " msg)
#else
#define dprintk(msg...)
#endif

#define TS_NAME "jz-ts"

#define KEY_AUX_INDEX       1
#define KEY_SCAN_INTERVAL   5
#define TS_SCAN_INTERVAL	0

/* from qwerty.kl of android */
#define  DPAD_CENTER            28
#define  DPAD_DOWN              108
#define  DPAD_UP                103
#define  DPAD_LEFT              105
#define  DPAD_RIGHT             106

/* TS event status */
#define PENUP			0x00
#define PENDOWN			0x01

/* Sample times in one sample process	*/
//#define SAMPLE_TIMES		3

#define SAMPLE_TIMES		5  //5
#define DROP_SAMPLE_TIMES   0  /* min drop 1 sample */
#define CAL_SAMPLE_TIMES (SAMPLE_TIMES - DROP_SAMPLE_TIMES)
#define VIRTUAL_SAMPLE      3 /* min >= 2 */
/* Min pressure value. If less than it, filt the point.
 * Mask it if it is not useful for you
 */
//#define MIN_PRESSURE		0x100

/* Max delta x distance between current point and last point.	*/
#define MAX_DELTA_X_OF_2_POINTS	200
/* Max delta x distance between current point and last point.	*/
#define MAX_DELTA_Y_OF_2_POINTS	120

/* Max delta between points in one sample process
 * Verify method :
 * 	(diff value / min value) * 100 <= MAX_DELTA_OF_SAMPLING
 */
#define MAX_DELTA_OF_SAMPLING	20


#define TS_ABS(x)       ((x) > 0 ? (x): -(x))
#define DIFF(a,b)		(((a)>(b))?((a)-(b)):((b)-(a)))
#define MIN(a,b)		(((a)<(b))?(a):(b))



/************************************************************************/
/*	SAR ADC OPS							*/
/************************************************************************/

unsigned long times = 0;
unsigned long delay = 0;
unsigned long interval = 0;

unsigned int vbat_val[100] = {0};

unsigned int jz_read_battery(void);

static unsigned int battery_mv = 4200;

unsigned int ts_value = 0;
EXPORT_SYMBOL(ts_value);

static void print_reg(void);
static inline void sadc_start_pbat(void)
{
	SETREG8(SADC_ADENA, ADENA_VBATEN);      /* Enable pbat adc */
}

static int sadc_open(struct inode *inode, struct file *filp){
	return 0;
}
static int sadc_release(struct inode *inode, struct file *filp){
	return 0;
}

static ssize_t sadc_read(struct file *filp, char __user *to, size_t count, loff_t *loff)
{
	printk("%s:count:%d\n",__func__,count);
	copy_to_user(to, &vbat_val, count);
	return 0;	
} 

static ssize_t sadc_write(struct file *filp, const char __user *from, size_t count, loff_t *loff)
{
	size_t i;
	printk("%s:count:%d\n",__func__,count);
	copy_from_user(&vbat_val, from, count);
	for(i=0; i < count; i++)
		printk("value.%d: %d\n", i, vbat_val[i]);
	return count;	
} 

#define SET_WAIT_TIME 5

static int sadc_ioctl(struct inode *inode, struct file *filp, unsigned int cmd, unsigned long arg){
	switch(cmd){
		case 0:
			SETREG8(SADC_ADENA, ADENA_POWER);
			mdelay(50);
			printk("Disable VBAT.\n");
			break;
		case 1:
			times = arg;
			printk("Set times to: %lu\n",times);
			break;
		case 2:
			delay = arg;
			printk("Set delay to: %lu\n",delay);
			break;
		case 3:
			interval = arg;	
			printk("Set interval to: %lu\n",interval);
			break;
		case 4:
			printk("Starting Read VBAT: ");
			jz_read_battery();
			printk("\nRead over!\n");
			break;
		case SET_WAIT_TIME:
			if (arg == 1)
			OUTREG16(SADC_ADWAIT,200);     /* about 3.33 ms,you can change it */
			else
			OUTREG16(SADC_ADWAIT,1000);     /* about 3.33 ms,you can change it */
			break;
		default:
			printk("Not support CMD!\n");
			return -EINVAL;
			break;
	};
	return 0;
}

#define SADC_ID 128

static struct file_operations sadc_fops =
{
	.open = sadc_open,
	.release = sadc_release,
	.read = sadc_read,
	.write = sadc_write,
	.ioctl = sadc_ioctl,
};

static struct miscdevice sadc_dev = 
{
	.minor = SADC_ID,
	.name = "SADC_VBAT",
	.fops = &sadc_fops,
};

typedef struct datasource {
	u16 xbuf;
	u16 ybuf;
	u16 zbuf;
	u16 reserve;
}datasource_t;
struct ts_event {
	u16 status;
	u16 x;
	u16 y;
	u16 pressure;
	u16 pad;
};
#define TOUCH_TYPE 1

//sadc touch fifo size 3 * 32bit
#define FIFO_MAX_SIZE 3
#define ADJUST_TIMES  3

#define DATA_MASK 0x0fff

int adjust_count;
unsigned int bak_data = 0;
unsigned int fifo[FIFO_MAX_SIZE] = {0,0,0};

/*
 * TS deriver
 */
struct jz_ts_t {
	int touch_cal_count;

	unsigned int ts_fifo[FIFO_MAX_SIZE][CAL_SAMPLE_TIMES];
	datasource_t data_s;
	struct ts_event event;
	int event_valid;

	int cal_type; /* current calibrate type */
	//struct timer_list acq_timer;	// Timer for triggering acquisitions
#ifdef CONFIG_JZ_ADKEY
	struct timer_list key_timer;	// for adkey
	int active_low;		// for adkey's interrupt pin
#endif
	wait_queue_head_t wait;		// read wait queue
	spinlock_t lock;

	/* Following 4 members use to pass arguments from u-boot to tell us the ts data.
	 * But in Android we do not use them.
	 */
	/*
	   int minx, miny, maxx, maxy;
	   */
	int first_read;

	int adjusted;
	int adjust_time;
	int refer_x;
	int refer_y;

	char	phys[32];
	struct input_dev *input_dev; /* for touch screen */
	struct input_dev *input_dev1; /* for adkey */
};

static struct jz_ts_t *jz_ts;

/*
 * TS Event type
 */

#ifdef CONFIG_JZ_ADKEY
struct ad_keys_button {
	int code;		/* input event code */
	int val;                /* the ad value of the key */
	int fuzz;               /* the error(+-fuzz) allowed of the ad value of the key */
};
static struct ad_keys_button ad_buttons[] = {
	{
		.code = DPAD_LEFT,
		.val = DPAD_LEFT_LEVEL,
		.fuzz = 40,
	},
	{
		.code = DPAD_DOWN,
		.val = DPAD_DOWN_LEVEL,
		.fuzz = 40,
	},
	{
		.code = DPAD_UP,
		.val = DPAD_UP_LEVEL,
		.fuzz = 40,
	},
	{
		.code = DPAD_CENTER,
		.val = DPAD_CENTER_LEVEL,
		.fuzz = 40,
	},
	{
		.code = DPAD_RIGHT,
		.val = DPAD_RIGHT_LEVEL,
		.fuzz = 40,
	},
};
#define KEY_NUM (sizeof(ad_buttons) / sizeof(struct ad_keys_button))
#endif

static DECLARE_WAIT_QUEUE_HEAD (sadc_wait_queue);

extern unsigned int (*codec_read_battery)(void);
#if 0
static void reg_debug(void)
{
	printk("\t####CTRL####################################################\n");
	printk("\tPEND %s,   ", REG_SADC_CTRL & SADC_CTRL_PENDM ? "masked" : "enabled");
	printk("PENU %s,   ", REG_SADC_CTRL & SADC_CTRL_PENUM ? "masked" : "enabled");
	printk("TSRDY %s\n", REG_SADC_CTRL & SADC_CTRL_TSRDYM ? "masked" : "enabled");
	printk("\t----STATE---------------------------------------------------\n");
	printk("\tIRQ actived: %s,   %s,   %s\n",
			REG_SADC_STATE & SADC_STATE_PEND ? "pen down" : "        ",
			REG_SADC_STATE & SADC_STATE_PENU ? "pen up  " : "        ",
			REG_SADC_STATE & SADC_STATE_TSRDY ? "sample  " : "        ");
	printk("\t############################################################\n");
}
#endif
/*
 * set adc clock to 24MHz/div. A/D works at freq between 500KHz to 8MHz.
 */
static void sadc_init_clock(int div)
{
	cpm_start_clock(CGM_SADC);

	div = 120 - 1;	/* working at 100 KHz */
	CMSREG32(SADC_ADCLK, div, ADCLK_CLKDIV_MASK);
}

static inline void sadc_start_aux(void)
{
	SETREG8(SADC_ADENA, ADENA_AUXEN);
}

static inline void ts_enable_pendown_irq(void)
{
	CLRREG8(SADC_ADCTRL, ADCTRL_PENDM);
}

static inline void ts_enable_penup_irq(void)
{
	CLRREG8(SADC_ADCTRL, ADCTRL_PENUM);
}

static inline void ts_disable_pendown_irq(void)
{
	SETREG8(SADC_ADCTRL, ADCTRL_PENDM);
}

static inline void ts_disable_penup_irq(void)
{
	SETREG8(SADC_ADCTRL, ADCTRL_PENUM);
}

static inline void sadc_enable_ts(void)
{
//	SETREG8(SADC_ADENA, (ADENA_TCHEN | ADENA_PENDEN));
	SETREG8(SADC_ADENA, (ADENA_TCHEN));
}

static inline void sadc_disable_ts(void)
{
	CLRREG8(SADC_ADENA, ADENA_TCHEN);
}

#if 0
static inline void sadc_start_ts(void)
{
        unsigned int tmp;

        OUTREG16(SADC_ADSAME, 1);       /* about 0.02 ms,you can change it */
        OUTREG16(SADC_ADWAIT, 500);     /* about 3.33 ms,you can change it */

        /* set ts mode and sample times */
	tmp = REG32(SADC_ADCFG);
        tmp |= ADCFG_CMD_SEL | ADCFG_SNUM(SAMPLE_TIMES);
//        tmp |= ADCFG_XYZ_XYZ1Z2X2Y2 | ADCFG_SNUM(SAMPLE_TIMES);
//      tmp |= ADCFG_SPZZ | ADCFG_XYZ_XYZ1Z2 | ADCFG_SNUM(SAMPLE_TIMES);
        OUTREG32(SADC_ADCFG, tmp);

	/* Activate the command logic. */
	tmp = REG32(SADC_ADCMD);

	tmp = 0x00000000;
	tmp |= ADCMD_XPSUP | ADCMD_XNGRU | ADCMD_VREFNXN | ADCMD_VREFPXP | ADCMD_YPADC;
        OUTREG32(SADC_ADCMD, tmp);

	tmp = 0x00000000;
	tmp |= ADCMD_YPSUP | ADCMD_YNGRU | ADCMD_VREFNYN | ADCMD_VREFPYP | ADCMD_XPADC;
        OUTREG32(SADC_ADCMD, tmp);

	tmp = 0x00000000;
        OUTREG32(SADC_ADCMD, tmp);

        /* mask all the intr except PEN-DOWN */
        tmp = ADCTRL_SLPENDM | ADCTRL_PENUM | ADCTRL_DTCHM | ADCTRL_VRDYM | ADCTRL_ARDYM;
        OUTREG8(SADC_ADCTRL, tmp);

        /* clear all the intr status if needed*/
        OUTREG8(SADC_ADSTATE, INREG8(SADC_ADSTATE));

        sadc_enable_ts();
}
#else

static inline void sadc_start_ts(void)
{
        unsigned int tmp;

        OUTREG16(SADC_ADSAME, 1);       /* about 0.02 ms,you can change it */
        OUTREG16(SADC_ADWAIT, 1000);     /* about 3.33 ms,you can change it */

        /* set ts mode and sample times */
	tmp = REG32(SADC_ADCFG);
        tmp |= ADCFG_SNUM(SAMPLE_TIMES);
//        tmp |= ADCFG_XYZ_XYZ1Z2X2Y2 | ADCFG_SNUM(SAMPLE_TIMES);
      	//tmp |= ADCFG_SPZZ | ADCFG_XYZ_XYZ1Z2 | ADCFG_SNUM(SAMPLE_TIMES);
        OUTREG32(SADC_ADCFG, tmp);

        /* mask all the intr except PEN-DOWN */
        //tmp = ADCTRL_SLPENDM | ADCTRL_PENUM | ADCTRL_DTCHM | ADCTRL_VRDYM | ADCTRL_ARDYM;
        tmp = ADCTRL_SLPENDM | ADCTRL_PENUM | ADCTRL_VRDYM | ADCTRL_ARDYM | ADCTRL_PENDM;
        //tmp = ADCTRL_SLPENDM | ADCTRL_PENUM | ADCTRL_VRDYM | ADCTRL_ARDYM;
        OUTREG8(SADC_ADCTRL, tmp);

	//tmp = REG32(SADC_ADTCH);
	//tmp &= 0x7fff7fff;

        /* clear all the intr status if needed*/
        OUTREG8(SADC_ADSTATE, INREG8(SADC_ADSTATE));

        sadc_enable_ts();
}
#endif

/**
 * Read the battery voltage
 */
unsigned int jz_read_battery(void)
{
	unsigned int pbat;
	unsigned int loop;
	
	u16 timeout;
	
//	printk("times:%lu delay:%lu interval:%lu\n\n",times,delay,interval);
	
	memset(vbat_val, 0x00,sizeof(vbat_val));
	

	for(loop = 0;loop < 1;loop++){
		timeout = 0x3fff;
		sadc_start_pbat();
		udelay(100);
		while((!((INREG8(SADC_ADSTATE) & ADSTATE_VRDY))) && (--timeout));
		if (!timeout){
			printk(KERN_ERR "Reading battery timeout!\n");
			return 0;
		}

		pbat = INREG16(SADC_ADVDAT) & ADVDAT_VDATA_MASK;

		OUTREG8(SADC_ADSTATE, ADSTATE_VRDY);
	}

	
	return pbat;
}

/**
 * Read the aux voltage
 */
unsigned short jz_read_aux(int index)
{
	unsigned int timeout = 0x3ff;
	u16 val;

	//medive change
	CLRREG8(SADC_ADENA, ADENA_POWER);
	mdelay(delay);

	CMSREG32(SADC_ADCFG, index, ADCFG_CMD_MASK);
	
//	printk("aux.%d ADCCONFIG:%x\n",index, REG32(SADC_ADCFG));

	sadc_start_aux();
	udelay(300);

	while(!(INREG8(SADC_ADSTATE) & ADSTATE_ARDY) && --timeout);

    if (!timeout)
        printk(KERN_ERR "Reading aux timeout!");

	val = INREG16(SADC_ADADAT) & ADADAT_ADATA_MASK;

	OUTREG8(SADC_ADSTATE, ADSTATE_ARDY);
	CLRREG8(SADC_ADENA, ADENA_AUXEN); // hardware may not shut down really

	//medive change
	SETREG8(SADC_ADENA, ADENA_POWER);
	mdelay(50);

	printk("read aux.%d val=%d\n",index, val);    
 	return val;
}

#define UP_KEY_0 0x01
#define DOWN_KEY_0 0x02
#define LEFT_KEY_0 0x04
#define RIGHT_KEY_0 0x08

#define ADJUST_VALUE 500
#define ORIGIN_COORDINATES 1600
#define ADJUST_COORDINATES_MAX (ORIGIN_COORDINATES + ADJUST_VALUE)
#define ADJUST_COORDINATES_MIN (ORIGIN_COORDINATES - ADJUST_VALUE)


//maddrone add
static unsigned int ts_key_data = 0;
#if 0
unsigned int   jz_read_ts_data(void)
{
	int tsdata_x;
	int tsdata_y;
	int type1,type0; 
	unsigned int val = 0;

	u32 timeout;

	//CLRREG8(SADC_ADENA, ADENA_POWER);
	//mdelay(10);
  	//schedule_timeout(50*HZ);

	//sadc_enable_ts();
	//udelay(50);

	//print_reg();
	timeout = 0x3fff * 100;
	//while(!((INREG8(SADC_ADSTATE) & (~INREG8(SADC_ADCTRL))) & ADSTATE_DTCH) && (--timeout));
	while((!((INREG8(SADC_ADSTATE) & ADSTATE_DTCH))) && (--timeout));
	if (!timeout){
		printk(KERN_ERR "Reading tsdata timeout!\n");
	//	return 0;
	}else{
	//	printk("\n no time out !\n");
	}

	int value = INREG32(SADC_ADTCH);
	tsdata_x = value & 0xfff;
	tsdata_y = (value & 0xfff0000) >> 16;
	type0  =  (value & 0x8000) >> 15;
	type1  =  (value & 0x80000000) >> 31;

	

	OUTREG8(SADC_ADSTATE, ADSTATE_DTCH);

	//SETREG8(SADC_ADENA, ADENA_POWER);
	//mdelay(10);
  	//schedule_timeout(50*HZ);
	//printk("\n medive printk : Type0:%d  data_x:%d  Type1:%d  data_y:%d\n",type0,tsdata_x,type1,tsdata_y);
	if (((tsdata_x > (MID_VALUE-ADJUST_VALUE)) && (tsdata_x < (MID_VALUE+ADJUST_VALUE)))  &&  (tsdata_y >= 0 && tsdata_y < ADJUST_VALUE ))
	{
		val |= 	UP_KEY_0;
	}else if (((tsdata_x > (MID_VALUE-ADJUST_VALUE)) && (tsdata_x < (MID_VALUE+ADJUST_VALUE)))  &&  (tsdata_y > (MAX_VALUE - ADJUST_VALUE) && tsdata_y < MAX_VALUE ))
	{
		val |= 	DOWN_KEY_0;
	}else if (((tsdata_x > (MAX_VALUE-ADJUST_VALUE)) && (tsdata_x < MAX_VALUE))  &&  (tsdata_y > (MID_VALUE - ADJUST_VALUE) && tsdata_y < (MID_VALUE + ADJUST_VALUE) ))
	{
		val |= 	LEFT_KEY_0;
	}else if (((tsdata_x >= 0) && (tsdata_x < ADJUST_VALUE))  &&  (tsdata_y > (MID_VALUE - ADJUST_VALUE) && tsdata_y < (MID_VALUE + ADJUST_VALUE )))
	{
		val |= 	RIGHT_KEY_0;
	}

	return val; 
}
#endif
unsigned int   jz_read_ts_data(void)
{
	return ts_key_data;
}

static void jz_cal_ts_data(unsigned int value)
{
	int tsdata_x;
	int tsdata_y;
	unsigned int val = 0;

	tsdata_x = value & 0xfff;
	tsdata_y = (value & 0xfff0000) >> 16;
	/*printk("\n medive printk : tsdata_x = %d tsdata_y = %d \n",tsdata_x,tsdata_y);*/
		
	if (tsdata_x > ADJUST_COORDINATES_MAX && tsdata_y < ADJUST_COORDINATES_MIN)
	{
	    //printk("\n up   left   \n");
	    val |= 	UP_KEY_0 | LEFT_KEY_0;
	} else if (tsdata_x > ADJUST_COORDINATES_MAX && tsdata_y > ADJUST_COORDINATES_MAX)
	{
	    //printk("\n down  left   \n");
	    val |= 	DOWN_KEY_0 | LEFT_KEY_0;
	} else if (tsdata_x < ADJUST_COORDINATES_MIN && tsdata_y < ADJUST_COORDINATES_MIN)
	{
	    //printk("\n up   right   \n");
	    val |= 	UP_KEY_0 | RIGHT_KEY_0;
	} else if (tsdata_x < ADJUST_COORDINATES_MIN && tsdata_y > ADJUST_COORDINATES_MAX)
	{
	    //printk("\n down   right   \n");
	    val |= 	DOWN_KEY_0 | RIGHT_KEY_0;
	} else 	if (tsdata_x >= ADJUST_COORDINATES_MIN && tsdata_x <= ADJUST_COORDINATES_MAX && tsdata_y < ADJUST_COORDINATES_MIN)
	{
	    val |= 	UP_KEY_0;
	} else if (tsdata_x >= ADJUST_COORDINATES_MIN && tsdata_x <= ADJUST_COORDINATES_MAX && tsdata_y > ADJUST_COORDINATES_MAX)
	{
	    val |= 	DOWN_KEY_0;
	} else if (tsdata_y >= ADJUST_COORDINATES_MIN && tsdata_y <= ADJUST_COORDINATES_MAX && tsdata_x > ADJUST_COORDINATES_MAX)
	{
	    val |= 	LEFT_KEY_0;
	} else if (tsdata_y >= ADJUST_COORDINATES_MIN && tsdata_y <= ADJUST_COORDINATES_MAX && tsdata_x < ADJUST_COORDINATES_MIN)
	{
	    val |= 	RIGHT_KEY_0;
	} else
	    val = 0;

	ts_key_data = val;
}

EXPORT_SYMBOL(jz_read_ts_data);


#define POWEROFF_VOL 3600
extern int jz_pm_hibernate(void);

static void
battery_track_timer(unsigned long data)
{
  //printk("%s %d \n",__FILE__,__LINE__);
  //printk("kernel battery_track_time thread start!\n");
  int over_time = 0;
  unsigned int value = 0;
  while(1)
  {
    value = jz_read_battery();
    if (value != 0){
	    battery_mv = value;
    }
    unsigned int mv = battery_mv + 580;//battery_mv*4+250;
    battery_mv = mv;
    //printk("%s %d mv is %d\n",__FILE__,__LINE__,mv);
    if(mv < POWEROFF_VOL)
    {
      over_time++;
      if(over_time > 5)
      {
        //printk("low power !\n");

        if(!__gpio_get_pin(GPIO_DC_PIN))
        {
          printk("the power is too low do hibernate!!!!!\n");
          extern void run_sbin_poweroff();
          run_sbin_poweroff();
          jz_pm_hibernate();
        }
      }
    }
    else
    {
      over_time = 0;
    }
    set_current_state(TASK_INTERRUPTIBLE);
    schedule_timeout(HZ*10);
  }
}



static inline void ts_data_ready(void)
{
	SETREG8(SADC_ADCTRL, ADCTRL_DTCHM);
}

#ifdef CONFIG_JZ_ADKEY

static unsigned int key_scan(int ad_val)
{
	int i;

	for(i = 0; i<KEY_NUM; i++) {
		if((ad_buttons[i].val + ad_buttons[i].fuzz >= ad_val) &&
				(ad_val >=ad_buttons[i].val - ad_buttons[i].fuzz)) {
			return ad_buttons[i].code;
		}
	}
	return -1;
}

static void key_timer_callback(unsigned long data)
{
	struct jz_ts_t *ts = (struct jz_ts_t *)data;
	int state;
	int active_low = ts->active_low;
	int ad_val, code;
	static int old_code;

	state = __gpio_get_pin(GPIO_ADKEY_INT);
	ad_val = jz_read_aux(KEY_AUX_INDEX);

	if (active_low) {
		if (state == 0) {
			/* press down */
			code = key_scan(ad_val);
			old_code = code;
			input_report_key(ts->input_dev1, code, 1);
			dprintk("code=%d\n",code);
			//input_sync(ts->input_dev1);
			mod_timer(&ts->key_timer, jiffies + KEY_SCAN_INTERVAL);
		} else {
			/* up */
			input_report_key(ts->input_dev1, old_code, 0);
			//input_sync(ts->input_dev1);
			//udelay(1000);
			__gpio_as_irq_fall_edge(GPIO_ADKEY_INT);
		}
	} else {
		if (state == 1) {
			/* press down */
			code = key_scan(ad_val);
			old_code = code;
			input_report_key(ts->input_dev1, code, 1);
			//input_sync(ts->input_dev1);
			mod_timer(&ts->key_timer, jiffies + KEY_SCAN_INTERVAL);
		} else {
			/* up */
			input_report_key(ts->input_dev1, old_code, 0);
			//input_sync(ts->input_dev1);
			//udelay(1000);
			__gpio_as_irq_rise_edge(GPIO_ADKEY_INT);
		}
	}
}

static irqreturn_t key_interrupt(int irq, void * dev_id)
{
	struct jz_ts_t *ts = dev_id;


	__gpio_as_input(GPIO_ADKEY_INT);

	if (!timer_pending(&ts->key_timer))
        mod_timer(&ts->key_timer, jiffies + KEY_SCAN_INTERVAL);
	return IRQ_HANDLED;
}
#endif

/************************************************************************/
/*	Touch Screen module						*/
/************************************************************************/

#define TSMAXX		3920
#define TSMAXY		3700
#define TSMAXZ		(1024) /* measure data */

#define TSMINX		150
#define TSMINY		270
#define TSMINZ		0


#define SCREEN_MAXX	1023
#define SCREEN_MAXY	1023
#define PRESS_MAXZ      256

static unsigned long transform_to_screen_x(struct jz_ts_t *ts, unsigned long x )
{
	/* Now we don't need u-boot to tell us the ts data.	*/
	/*
	   if (ts->minx)
	   {
	   if (x < ts->minx) x = ts->minx;
	   if (x > ts->maxx) x = ts->maxx;

	   return (x - ts->minx) * SCREEN_MAXX / (ts->maxx - ts->minx);
	   }
	   else
	   {
	   */
	if (x < TSMINX) x = TSMINX;
	if (x > TSMAXX) x = TSMAXX;

	return (x - TSMINX) * SCREEN_MAXX / (TSMAXX - TSMINX);
	/*
	   }
	   */
}

static unsigned long transform_to_screen_y(struct jz_ts_t *ts, unsigned long y)
{
	/* Now we don't need u-boot to tell us the ts data.	*/
	/*
	   if (ts->miny)
	   {
	   if (y < ts->miny) y = ts->miny;
	   if (y > ts->maxy) y = ts->maxy;

	   return (ts->maxy - y) * SCREEN_MAXY / (ts->maxy - ts->miny);
	   }
	   else
	   {
	   */
	if (y < TSMINY) y = TSMINY;
	if (y > TSMAXY) y = TSMAXY;

	return (TSMAXY - y) * SCREEN_MAXY / (TSMAXY - TSMINY);
	/*
	   }
	   */
}
static unsigned long transform_to_screen_z(struct jz_ts_t *ts, unsigned long z){
	if(z < TSMINZ) z = TSMINZ;
	if (z > TSMAXY) z = TSMAXY;
	return (TSMAXZ - z) * PRESS_MAXZ / (TSMAXZ - TSMINZ);
}
/* R plane calibrate,please look up spec 11th page*/

#define Yr_PLANE  480
#define Xr_PLANE  800

#define Touch_Formula_One(z1,z2,ref,r) ({	\
		int z;					\
		if((z1) > 0){				\
		z = ((ref) * (z2)) / (z1);		\
		if((z2) > (z1)) z = (z * r - (ref) * r) / (4096);	\
		else z = 0;				\
		}else					\
		z = 4095;				\
		z;					\
		})


static int ts_data_filter(struct jz_ts_t *ts){
	int i,xt = 0,yt = 0,zt1 = 0,zt2 = 0,zt3 = 0,zt4 = 0,t1_count = 0,t2_count = 0,z;

	datasource_t *ds = &ts->data_s;
	int t,xmin = 0x0fff,ymin = 0x0fff,xmax = 0,ymax = 0;//,z1min = 0xfff,z1max = 0,z2min = 0xfff,z2max = 0;

	/* fifo high 16 bit = y,fifo low 16 bit = x */

	for(i = 0;i < CAL_SAMPLE_TIMES;i++){

		t = (ts->ts_fifo[0][i] & 0x0fff);
#if (CAL_SAMPLE_TIMES >= 3)
		if(t > xmax) xmax = t;
		if(t < xmin) xmin = t;
#endif
		xt += t;
		t = (ts->ts_fifo[0][i] >> 16) & 0x0fff;
#if (CAL_SAMPLE_TIMES >= 3)
		if(t > ymax) ymax = t;
		if(t < ymin) ymin = t;
#endif

		yt += t;
		if(ts->ts_fifo[1][i] & 0x8000)
		{
			t = (ts->ts_fifo[1][i] & 0x0fff);
			zt1 += t;

			t = (ts->ts_fifo[1][i] >> 16) & 0x0fff;
			zt2 += t;

			t1_count++;
		}else
		{
			t = (ts->ts_fifo[1][i] & 0x0fff);
			zt3 += t;

			t = (ts->ts_fifo[1][i] >> 16) & 0x0fff;
			zt4 += t;

			t2_count++;
		}
	}
#if (CAL_SAMPLE_TIMES >= 3)
	xt = xt - xmin - xmax;
	yt = yt - ymin - ymax;
#endif

	xt /= (CAL_SAMPLE_TIMES - 2);
	yt /= (CAL_SAMPLE_TIMES - 2);
	if(t1_count > 0)
	{
		zt1 /= t1_count;
		zt2 /= t1_count;
		zt1 = Touch_Formula_One(zt1,zt2,xt,Xr_PLANE);
	}
	if(t2_count)
	{
		zt3 /= t2_count;
		zt4 /= t2_count;
		zt3 = Touch_Formula_One(zt3,zt4,yt,Yr_PLANE);
	}
	if((t1_count) && (t2_count))
		z = (zt1 + zt3) / 2;
	else if(t1_count)
		z = zt1;
	else if(t2_count)
		z = zt3;
	else
		z = 0;

	ds->xbuf = xt;
	ds->ybuf = yt;
	ds->zbuf = z;
	return 1;

}
static void ts_transform_data(struct jz_ts_t *ts){
	struct ts_event *event = &ts->event;
	event->x = transform_to_screen_x(ts,ts->data_s.xbuf);
	event->y = transform_to_screen_y(ts,ts->data_s.ybuf);
	event->pressure = transform_to_screen_z(ts,ts->data_s.zbuf);
	if(event->pressure == 0) event->pressure = 1;
}
static void handle_ts_event(struct jz_ts_t *ts){
	struct ts_event *event = &ts->event;
	input_report_abs(ts->input_dev, ABS_X, event->x);
	input_report_abs(ts->input_dev, ABS_Y, event->y);
	input_report_abs(ts->input_dev, ABS_PRESSURE, event->pressure);

	/* Android need it ... */
	input_report_key(ts->input_dev, BTN_TOUCH, 1);
	input_sync(ts->input_dev);

	//printk("event->x = %d,event->y = %d event->pressure = %d\n",event->x,event->y,event->pressure);
}

static void handle_touch(struct jz_ts_t *ts, unsigned int *data, int size){
	/* drop no touch calibrate points */
	if(ts->cal_type & (~TOUCH_TYPE))
		ts->cal_type |= ~TOUCH_TYPE;
	if(ts->event_valid){
		handle_ts_event(ts);
		ts->event_valid = 0;
	}

	if(ts->touch_cal_count >= DROP_SAMPLE_TIMES)
	{
		if(ts->touch_cal_count < SAMPLE_TIMES){
			ts->ts_fifo[0][ts->touch_cal_count - DROP_SAMPLE_TIMES] = data[0];
			ts->ts_fifo[1][ts->touch_cal_count - DROP_SAMPLE_TIMES] = data[1];
			ts->ts_fifo[2][ts->touch_cal_count - DROP_SAMPLE_TIMES] = data[2];
		}else
		{
			/* drop sample*/
			if(ts->cal_type & TOUCH_TYPE){
				if(ts_data_filter(ts)){
					ts->event_valid = 1;
					ts_transform_data(ts);
				}

			}
			ts->touch_cal_count = 0;
		}
	}
	ts->touch_cal_count++;
}

static void handle_adjust(struct jz_ts_t *ts, unsigned int *data)
{
	if(adjust_count < ADJUST_TIMES){
		ts->refer_x += data[2] & DATA_MASK;
		ts->refer_y += (data[2] >> 16) & DATA_MASK;
		adjust_count ++;
	}

	if(adjust_count == ADJUST_TIMES){
		ts->refer_x /= ADJUST_TIMES;
		ts->refer_y /= ADJUST_TIMES;
		adjust_count = 0;
		ts->adjusted = 1;
		printk("refer_x:%d refer_y:%d\n",ts->refer_x,ts->refer_y);
	}

	return;
}

static void handle_multi_touch(struct jz_ts_t *ts, unsigned int *data)
{
	printk("bak:%d  %d\n",bak_data & DATA_MASK, (bak_data >> 16) & DATA_MASK);

	if(((bak_data & DATA_MASK) < (data[2] & DATA_MASK)) &&	\
	   (((bak_data >> 16) & DATA_MASK) < ((data[2] >> 16) & DATA_MASK)))
		printk("Shrink.\n");
	if(((bak_data & DATA_MASK) > (data[2] & DATA_MASK)) &&	\
	   (((bak_data >> 16) & DATA_MASK) > ((data[2] >> 16) & DATA_MASK)))
		printk("Expand.\n");
}

static irqreturn_t sadc_interrupt(int irq, void * dev_id)
{
//	printk("\n medive printk : go to sadc interrupt !\n");
        unsigned char tmp;
	struct jz_ts_t *ts = dev_id;
	unsigned int state;
	static int pen_is_down = 0;

	spin_lock_irq(&ts->lock);

	state = INREG8(SADC_ADSTATE) & (~INREG8(SADC_ADCTRL));
	//printk("irqno.%d irq_map:%x  state:%x\n",irq,REG_INTC_ISR(0),state);
	/* first handle pen up interrupt */
	//if(state & ADSTATE_PEND){    //0x10
	//	SETREG8(SADC_ADCTRL,  ADSTATE_PEND);
	//	OUTREG8(SADC_ADSTATE, ADSTATE_PEND);
       if(state & ADSTATE_DTCH){  //0x4
		jz_cal_ts_data(INREG32(SADC_ADTCH));
		OUTREG8(SADC_ADSTATE, ADSTATE_DTCH);
	}

	spin_unlock_irq(&ts->lock);

	return IRQ_HANDLED;
}

static void print_reg(void)
{
	return;
	printk("ADENA:%x\n",REG32(SADC_ADENA));
	printk("ADCFG:%x\n",REG32(SADC_ADCFG));
	printk("ADCTRL:%x\n",REG32(SADC_ADCTRL));
	printk("ADSTATE:%x\n",REG32(SADC_ADSTATE));
	printk("ADSAME:%x\n",REG32(SADC_ADSAME));
	printk("ADWAIT:%x\n",REG32(SADC_ADWAIT));
	printk("ADTCH:%x\n",REG32(SADC_ADTCH));
	printk("ADVDAT:%x\n",REG32(SADC_ADVDAT));
	printk("ADADAT:%x\n",REG32(SADC_ADADAT));
	printk("ADCLK:%x\n",REG32(SADC_ADCLK));
//	printk("ADCMD:%x\n",REG32(SADC_ADCMD));  // NOTE:This register couldn't Read.
}

static ssize_t vbat_read(struct file *file, char __user *buf,
		size_t count, loff_t *ppos)
{
	printk("Read vbat proc.count:%u\n",count);
//	copy_to_user(buf, vbat_val, count);
	return 0;
}

static ssize_t vbat_write(struct file *file, const char __user *buf,
		size_t count, loff_t *ppos)
{
	printk("Write vbat proc.count:%u\n",count);
	return count;
}


static int proc_sadc_read_proc(char *page,char **start,off_t off,int count,int *eof,void *data)
{
	return sprintf(page,"%1u\n",vbat_val[0]);
}

static int proc_sadc_write_proc(struct file *file,const char *buffer,unsigned long count,void *data)
{
	return count;
}

/* medive add for read vbat*/

static int proc_sadc_battery_show(struct seq_file *m,void *v)
{
	
	unsigned int mv = battery_mv;
	//__gpio_disable_pull(OTG_HOTPLUG_PIN);
	//__gpio_as_input(OTG_HOTPLUG_PIN);

	if (__gpio_get_pin(OTG_HOTPLUG_PIN)){
		mv |= 0x80000000;
	//}else{
	//	mv &= 0x7fffffff;
	}

	if (__gpio_get_pin(GPIO_DC_PIN)){
		mv |= 0x40000000;
	//}else{
	//	mv |= 0xbfffffff;
	}

	seq_printf(m,"%u\n",mv);
	return 0;
}

static int proc_sadc_battery_open (struct inode *inode,struct file *file)
{
	return single_open(file,proc_sadc_battery_show,NULL);
}


static const struct file_operations proc_sadc_battery_fops = {
	.open         = proc_sadc_battery_open,
	.read         = seq_read,
	.llseek       = seq_lseek,
	.release      = single_release,
};

/* end */

static const struct file_operations proc_vbat_operations = {
	.read = vbat_read,
	.write = vbat_write,
};

void init_dc_pin()
{
	__gpio_disable_pull(GPIO_DC_PIN);
	__gpio_as_input(GPIO_DC_PIN);
}

static struct task_struct * battery_monitor;
static int __init jz_ts_init(void)
{
	struct input_dev	*input_dev;
	struct jz_ts_t		*ts;
	int	error;
	struct proc_dir_entry *res_sadc;

	ts = jz_ts = kzalloc(sizeof(struct jz_ts_t), GFP_KERNEL);
	input_dev = input_allocate_device();
	if (!ts || !input_dev)
		return -ENOMEM;

	input_dev->name = "touchscreen"; /* Android will load /system/usr/keychars/qwerty.kcm.bin by default */
	input_dev->phys = ts->phys;

	/*
old:
input_dev->evbit[0] = BIT(EV_KEY) | BIT(EV_ABS);
input_dev->keybit[BIT_WORD(BTN_TOUCH)] = BIT_MASK(BTN_TOUCH);
*/

	/* For Android */
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(ABS_X, input_dev->absbit);
	set_bit(ABS_Y, input_dev->absbit);
	set_bit(ABS_PRESSURE, input_dev->absbit);
	set_bit(EV_KEY, input_dev->evbit);
	set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, SCREEN_MAXX + 1, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, SCREEN_MAXY + 1, 0, 0);
	input_set_abs_params(input_dev, ABS_PRESSURE, 0, PRESS_MAXZ + 1, 0, 0);
	input_set_drvdata(input_dev, ts);
	error = input_register_device(input_dev);

	strcpy(ts->phys, "input/ts0");
	spin_lock_init(&ts->lock);

	ts->input_dev = input_dev;
	
	ts->adjusted = 0;
	ts->adjust_time = 0;
	ts->refer_x = 0;
	ts->refer_y = 0;

	adjust_count = 0;

#ifdef CONFIG_JZ_ADKEY
	ts->input_dev1 = input_allocate_device();
	if (!ts->input_dev1)
		return -ENOMEM;

        ts->input_dev1->name = "adkey";
	set_bit(EV_KEY, ts->input_dev1->evbit);
	set_bit(DPAD_CENTER, ts->input_dev1->keybit);
	set_bit(DPAD_DOWN, ts->input_dev1->keybit);
	set_bit(DPAD_UP, ts->input_dev1->keybit);
	set_bit(DPAD_LEFT, ts->input_dev1->keybit);
	set_bit(DPAD_RIGHT, ts->input_dev1->keybit);
	error = input_register_device(ts->input_dev1);
#endif

	if (error) {
		printk("Input device register failed !\n");
		goto err_free_dev;
	}

	sadc_init_clock(6);

	CLRREG8(SADC_ADENA, ADENA_POWER);
	CLRREG32(CPM_LCR, LCR_VBATIR);
	mdelay(50);

	OUTREG8(SADC_ADCTRL, ADCTRL_MASK_ALL);

#if 1
	error = request_irq(IRQ_SADC, sadc_interrupt, IRQF_DISABLED, TS_NAME, ts);
	if (error) {
		pr_err("unable to get PenDown IRQ %d", IRQ_SADC);
		goto err_free_irq;
	}
#endif
	ts->cal_type = 0;

#ifdef CONFIG_JZ_ADKEY
	// Init key acquisition timer function
	init_timer(&ts->key_timer);
	ts->key_timer.function = key_timer_callback;
	ts->key_timer.data = (unsigned long)ts;
	ts->active_low = ACTIVE_LOW_ADKEY;

	error = request_irq(IRQ_GPIO_0 + GPIO_ADKEY_INT, key_interrupt, IRQF_DISABLED, "jz-adkey", ts);
	if (error) {
		pr_err("unable to get AD KEY IRQ %d", IRQ_GPIO_0 + GPIO_ADKEY_INT);
		goto err_free_irq;
	}

	__gpio_disable_pull(GPIO_ADKEY_INT);

	if(ts->active_low)
		__gpio_as_irq_fall_edge(GPIO_ADKEY_INT);
	else
		__gpio_as_irq_rise_edge(GPIO_ADKEY_INT);

#endif
	//medive change
	sadc_start_ts();
	print_reg();

	printk("input: JZ Touch Screen registered.\n");
	printk("Create vbat proc entry.\n");
	//proc_create("jz/vbat", S_IRUSR | S_IWUSR, NULL, &proc_vbat_operations);
	

	init_dc_pin();
	

#if 1
	res_sadc = create_proc_entry("jz/battery",0,NULL);
	if (res_sadc){
		res_sadc->proc_fops = &proc_sadc_battery_fops;
	}
#endif




#if 0
	while(1){
	//	jz_read_battery();
		jz_read_aux(1);
		jz_read_aux(2);
		printk("\n");
	}
#endif

	misc_register(&sadc_dev);
#if 1
	battery_monitor = kthread_run(battery_track_timer, NULL, "battery _monitor");
	if(IS_ERR(battery_monitor))
	{
		printk("Kernel battery _monitor thread start error!\n");
		return;
	}
#endif


	//	jz_read_battery(10, 100, 1000);

	//SETREG8(SADC_ADENA, ADENA_POWER);
	//mdelay(50);
	
	print_reg();

	return 0;

err_free_irq:
	free_irq(IRQ_SADC, ts);
#ifdef CONFIG_JZ_ADKEY
	free_irq(IRQ_GPIO_0 + GPIO_ADKEY_INT, ts);
#endif
err_free_dev:
	input_free_device(ts->input_dev);
	kfree(ts);
	return 0;
}

static void __exit jz_ts_exit(void)
{
	ts_disable_pendown_irq();
	ts_disable_penup_irq();
	sadc_disable_ts();
	free_irq(IRQ_SADC, jz_ts);
	input_unregister_device(jz_ts->input_dev);
	misc_deregister(&sadc_dev);
}

module_init(jz_ts_init);
module_exit(jz_ts_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("JZ TouchScreen and VBAT Driver");
MODULE_AUTHOR("Benson.Yang@ingenic");
