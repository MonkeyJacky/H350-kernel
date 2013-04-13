#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/delay.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>

#include <asm/system.h>
#include <asm/page.h>
#include <asm/pgtable.h>
#include <asm/uaccess.h>

#include <asm/jzsoc.h>


#include "tve.h"

#define CPUFREQ_MINOR 0x43
#define CPUFREQ_DEV_NAME "cpufreq"
static const struct file_operations tve_fops;

static char tvestr[]="tve";



static void jz4770_scale_pll(unsigned int cppcr,unsigned int flag)
{
	unsigned int cppcr1;
	unsigned int cppsr;



	cppcr1 = REG_CPM_CPPCR0;
	//	cppcr1 &= ~(regs->cppcr_mask | CPM_CPPCR_PLLS | CPM_CPPCR_PLLEN | CPM_CPPCR_PLLST_MASK);
	cppcr1 &= ~(0xffff0000 | CPPCR0_PLLS | CPPCR0_PLLEN | CPPCR0_PLLST_MASK);
	//      cppcr1 &= ~CPM_CPPCR_PLLEN;
        cppcr1 |= (cppcr | 0xff);




	/* 
	 * Update PLL, align code to cache line.
	 */
	//printk("00CPM_CPPCR===0x%x\n",REG_CPM_CPPCR0);
	//printk("-2 cppsr=0x%x\n",REG_CPM_CPPSR);
	REG_CPM_CPPSR &= ~CPPSR_PS;
	REG_CPM_CPPSR &= ~CPPSR_CS;
	REG_CPM_CPPSR &= ~CPPSR_FS;

	REG_CPM_CPPSR |= CPPSR_PM;
	//printk("-1 cppsr=0x%x\n",REG_CPM_CPPSR);
	
	//medive add
#if 0
	volatile unsigned int temp;
	temp = REG_CPM_CPCCR;
	printk("\n medive printk: reg cpm cpccr is %x\n",REG_CPM_CPCCR);
	if (flag == 0){                   //  1:2
		temp &= ~0xf0;
		temp |= 0x10;
		printk("\n medive printk: temp reg cpm cpccr is %x\n",temp);
		REG_CPM_CPCCR = temp;
	}else{                            //   1:4
		temp &= ~0xf0;
		temp |= 0x30;
		REG_CPM_CPCCR = temp;
		printk("\n medive printk: 1temp reg cpm cpccr is %x\n",temp);
	}
	printk("\n medive printk: reg cpm cpccr is %x\n",REG_CPM_CPCCR);
#endif




	cppcr1 |= CPPCR0_PLLEN;
	cppcr1 |= CPPCR0_LOCK;




	__asm__ __volatile__(
		".set noreorder\n\t"
		".align 5\n"
		"sw %1,0(%0)\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		"nop\n\t"
		".set reorder\n\t"
		:
		: "r" (CPM_CPPCR0), "r" (cppcr1));


	while (!(REG_CPM_CPPCR0 & CPPCR0_PLLS));
	//printk("11CPM_CPPCR===0x%x\n",REG_CPM_CPPCR0);

	cppsr = REG_CPM_CPPSR;
	//printk("0 cppsr=0x%x\n",cppsr);
	//printk("1 cppsr=0x%x\n",REG_CPM_CPPSR);
	

}

static int cpufreq_ioctl(struct inode *inode, struct file *file, unsigned int cmd,
	      unsigned long arg) 
{
	int ret = 0;
	unsigned int pllm = ((REG_CPM_CPPCR0 & (~(1 << 31)))>> 24) & 0xff;
	//printk("pllm = %d", pllm);
	unsigned int cppcr;
	unsigned int bs;
	if (pllm >= 49)
		bs = 1;
	else 
		bs = 0;

	switch(cmd) {
	case TEST0:

		pllm += 1;
		//printk ("freq + 12Mhz, CPM_CPPCR0=0x%x pllm=%d\n", REG_CPM_CPPCR0, pllm);
		cppcr = (pllm << 24) | (0 << 18) | (0 << 16) | (bs << 31);

		jz4770_scale_pll(cppcr,0);

		break;
	case TEST1:

		pllm -= 1;
		//printk ("freq - 12Mhz, CPM_CPPCR0=0x%x pllm=%d\n", REG_CPM_CPPCR0, pllm);
		cppcr = (pllm << 24) | (0 << 18) | ( 0 << 16 ) | (bs << 31);

		jz4770_scale_pll(cppcr,0);

		break;
	case TEST2:
		pllm = arg;
		if  (pllm >= 600)
			bs = 1;
		else 
			bs = 0;
		pllm = (pllm / 12) - 1 ;
		cppcr = (pllm << 24) | (0 << 18) | (0 << 16) | (bs << 31); //for 12MHz

		jz4770_scale_pll(cppcr,0);
		break;
	default:
		//printk ("error:ioctl number is worng,please check code\n");
		break;
	};

	return ret;
}

static int cpufreq_open(struct inode *inode, struct file *filp)
{
	return 0;
}
static int cpufreq_release(struct inode *inode, struct file *filp)
{
	return 0;
}
static struct file_operations cpufreq_fops = {
	.owner   = THIS_MODULE,
	.open    = cpufreq_open,
	.ioctl   = cpufreq_ioctl,
	.release = cpufreq_release,
};

static struct miscdevice cpufreq_device = {
	.minor = CPUFREQ_MINOR,
	.name  = CPUFREQ_DEV_NAME,
	.fops  = &cpufreq_fops,
};
	unsigned int cur_flag;
static int proc_freq_read_proc(char *page,char **start,off_t off,int count,int *eof,void *data)
{
	return sprintf(page,"%1u\n",cur_flag);
}

/* medive add dynamic adjusting cpu core freq*/


#if 0
static int proc_freq_write_proc(struct file *file,const char *buffer,unsigned long count,void *data)
{
	unsigned int pllm = ((REG_CPM_CPPCR0 & (~(1 << 31)))>> 24) & 0xff;
	//printk("pllm = %d", pllm);
	unsigned int cppcr;
	unsigned int bs;
	unsigned int pll_flag;

	cur_flag = simple_strtoul(buffer,0,10);
	//medive add
#if 1
		if (cur_flag >= 432){
			pll_flag = 1;
		}else{
			pll_flag = 0;
		}
#endif
	if (pllm >= 49)
		bs = 1;
	else 
		bs = 0;

	pllm = cur_flag;
	if  (pllm >= 600)
		bs = 1;
	else 
		bs = 0;
	pllm = (pllm / 12) - 1 ;
	cppcr = (pllm << 24) | (0 << 18) | (0 << 16) | (bs << 31); //for 12MHz

	jz4770_scale_pll(cppcr,pll_flag);

	return count;
}
#else

#define MID_FREQ 432
#define FREQ_2 0
#define FREQ_4 1
static void set_pll(unsigned int freq,unsigned int flag)
{
	unsigned int bs;
	unsigned int cppcr;
	unsigned int pllm = ((REG_CPM_CPPCR0 & (~(1 << 31)))>> 24) & 0xff;
	if (pllm >= 49)
		bs = 1;
	else 
		bs = 0;

	pllm = freq;
	if  (pllm >= 600)
		bs = 1;
	else 
		bs = 0;
	pllm = (pllm / 12) - 1 ;
	cppcr = (pllm << 24) | (0 << 18) | (0 << 16) | (bs << 31); //for 12MHz

	jz4770_scale_pll(cppcr,flag);


}

static void reinit_lcd_pixclk(unsigned int freq)
{
	unsigned int cur_pixclk, val;
	__cpm_stop_lcd();
	cur_pixclk = __cpm_get_pixclk();
	val = freq*1000*1000 / cur_pixclk;
	val --;
	__cpm_set_pixdiv(val);
	__cpm_select_pixclk_lcd();
	REG_CPM_CPCCR |= CPCCR_CE;
	__cpm_start_lcd();
	printk("\n medive printk :  -----------lcd pixclk----------  = %d\n",__cpm_get_pixclk());

}

static int proc_freq_write_proc(struct file *file,const char *buffer,unsigned long count,void *data)
{
	unsigned int cur_freq;

	cur_flag = simple_strtoul(buffer,0,10);
	if (cur_flag > 1000 || cur_flag < 336){
		printk("\n This cpu core freq is no support! \n");
		return count;
	}
	cur_freq = __cpm_get_pllout2()/1000000;
	if (cur_freq == cur_flag){
		return count;
	}
	//reinit_lcd_pixclk(cur_flag);
	set_pll(cur_flag,FREQ_4);
#if 0
	printk("\n medive printk : cpm get pllout2 is %d   cur_flag  is  %d\n",cur_freq,cur_flag);
	if (cur_freq <= MID_FREQ){
		if (cur_flag > MID_FREQ){
			set_pll(MID_FREQ,FREQ_2);
			mdelay(100);
			set_pll(cur_flag,FREQ_4);
		}else{
			set_pll(cur_flag,FREQ_2);
		}
	}else
	{
		if (cur_flag > MID_FREQ){
			set_pll(cur_flag,FREQ_4);
		}else{
			set_pll(MID_FREQ,FREQ_4);
			mdelay(100);
			set_pll(cur_flag,FREQ_2);
		}

	}
#endif

	return count;
}
#endif



/* add  end */


static int __init cpufreq_init(void)
{
	struct proc_dir_entry * freq;
	int ret = 0;

	printk("init cpufreq \n");
	ret = misc_register(&cpufreq_device);
	if (ret < 0)
	{
		printk("misc register cpufreq err!\n");
		return ret;
	}

	freq = create_proc_entry("jz/cpufreq01",0,NULL);
	if (freq)
	{
		//res_tvout->owner = THIS_MODULE;
		freq->read_proc = proc_freq_read_proc;
		freq->write_proc = proc_freq_write_proc;
		freq->data = NULL;
	}


	return 0;	
}

static void __exit cpufreq_exit(void)
{
	printk("exit cpufreq!\n");
	misc_deregister(&cpufreq_device);
}

module_init(cpufreq_init);
module_exit(cpufreq_exit);
