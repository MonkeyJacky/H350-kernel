/*
 *  linux arch/mips/include/asm/mach-jz4770/board-f4770.h
 *
 *  JZ4770-based F4770 board ver 1.x definition.
 *
 *  Copyright (C) 2008 Ingenic Semiconductor Inc.
 *
 *  Author: <cwjia@ingenic.cn>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __ASM_JZ4770_F4770_H__
#define __ASM_JZ4770_F4770_H__

//#define CONFIG_FPGA

/*======================================================================
 * Frequencies of on-board oscillators
 */
#define JZ_EXTAL		12000000  /* Main extal freq:	12 MHz */
#define JZ_EXTAL2		32768     /* RTC extal freq:	32.768 KHz */
//#define CFG_DIV		        2	/* cpu/extclk; only for FPGA */

#define TVE_MODE_PCLK 27000000
#define H350_V0 0


//define the key used in key driver and wakeup
#if H350_V0

#define H350_KEY_UP        32*0+7
#define H350_KEY_DOWN      32*0+5
#define H350_KEY_LEFT      32*0+6
#define H350_KEY_RIGHT     32*0+4
#define H350_KEY_A         32*0+2
#define H350_KEY_B         32*0+1
#define H350_KEY_X         32*0+3
#define H350_KEY_Y         32*0+0
#define H350_KEY_Left      32*3+17
#define H350_KEY_Right     32*1+0
#define H350_KEY_START     32*3+18
#define H350_KEY_SELECT    32*3+19

#define H350_KEY_UP_ACTIVELOW     1
#define H350_KEY_DOWN_ACTIVELOW   1
#define H350_KEY_LEFT_ACTIVELOW   1
#define H350_KEY_RIGHT_ACTIVELOW  1
#define H350_KEY_A_ACTIVELOW      1
#define H350_KEY_B_ACTIVELOW      1
#define H350_KEY_X_ACTIVELOW      1
#define H350_KEY_Y_ACTIVELOW      1
#define H350_KEY_Left_ACTIVELOW   0
#define H350_KEY_Right_ACTIVELOW  1
#define H350_KEY_START_ACTIVELOW  0
#define H350_KEY_SELECT_ACTIVELOW 1


#define GPIO_AMPEN (32*5+20)
#define GPIO_HP_PIN     (32*5+21)
#define GPIO_HP_OFF  (32*5+3)
#define GPIO_GSENSOR_VCC  (32*4+18)
#define GPIO_WIFI_PW (32*5+10)
#define GPIO_DC_PIN (32*5+5)


#define H350_HOLD_PIN (32*5+11)

#else

#define H350_KEY_UP        32*4+21
#define H350_KEY_DOWN      32*4+25
#define H350_KEY_LEFT      32*4+23
#define H350_KEY_RIGHT     32*4+24
#define H350_KEY_A         32*4+29
#define H350_KEY_B         32*4+20
#define H350_KEY_X         32*4+27
#define H350_KEY_Y         32*4+28
#define H350_KEY_Left      32*1+20
#define H350_KEY_Right     32*4+26
#define H350_KEY_START     32*1+21
#define H350_KEY_SELECT    32*3+18

#define H350_KEY_UP_ACTIVELOW     1
#define H350_KEY_DOWN_ACTIVELOW   1
#define H350_KEY_LEFT_ACTIVELOW   1
#define H350_KEY_RIGHT_ACTIVELOW  1
#define H350_KEY_A_ACTIVELOW      1
#define H350_KEY_B_ACTIVELOW      1
#define H350_KEY_X_ACTIVELOW      1
#define H350_KEY_Y_ACTIVELOW      1
#define H350_KEY_Left_ACTIVELOW   1
#define H350_KEY_Right_ACTIVELOW  1
#define H350_KEY_START_ACTIVELOW  1
#define H350_KEY_SELECT_ACTIVELOW 0


#define GPIO_AMPEN (32*5+20)
#define GPIO_HP_PIN     (32*5+21)
#define GPIO_HP_OFF  (32*5+3)
#define GPIO_GSENSOR_VCC  (32*4+18)
#define GPIO_WIFI_PW (32*5+10)
#define GPIO_MOTO_PW (32*4+4)
#define GPIO_DC_PIN (32*5+5)


#define H350_HOLD_PIN (32*5+11)

#endif

#ifdef CONFIG_HDMI_IT6610_OLD
#define HDMI_RST_N_PIN  (32*4 + 6)
#define SIO_C          (32*3+7)
#define SIO_D          (32*3+6)
#endif

#ifdef CONFIG_HDMI_IT6610
#define GPIO_HDMI_RST_N (32*4 + 6)
#define GPIO_HDMI_PS2_KCLK (32*3+7)
#define GPIO_HDMI_PS2_KDATA (32*3+6)
#define GPIO_HDMI_HPD       (32*5+12)
#endif



/*======================================================================
 * GPIO
 */
#define JZMAC_PHY_RESET_PIN	GPE(8)

#define OTG_HOTPLUG_PIN         (32 + 5)
#define GPIO_OTG_ID_PIN         (32*5+18)
#define OTG_HOTPLUG_IRQ         (IRQ_GPIO_0 + OTG_HOTPLUG_PIN)
#define GPIO_OTG_ID_IRQ         (IRQ_GPIO_0 + GPIO_OTG_ID_PIN)
#define GPIO_OTG_STABLE_JIFFIES 10


#define GPIO_I2C1_SDA           (32*3+5)       /* GPE0 */
#define GPIO_I2C1_SCK           (32*3+4)       /* GPE17 */

#define      EP932M_RESET_PIN (32*4+6)


//medive del
//#define GPIO_SD0_VCC_EN_N	GPF(20)
#define GPIO_SD0_CD_N		GPA(25)
#define GPIO_SD0_WP_N		GPA(26)
#define GPIO_SD1_VCC_EN_N	GPE(9)
#define GPIO_SD1_CD_N		GPB(2)

#define ACTIVE_LOW_MSC0_CD	1
#define ACTIVE_LOW_MSC1_CD	1

#define MSC0_WP_PIN		GPIO_SD0_WP_N
#define MSC0_HOTPLUG_PIN	GPIO_SD0_CD_N
#define MSC0_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD0_CD_N)

#define MSC1_HOTPLUG_PIN	GPIO_SD1_CD_N
#define MSC1_HOTPLUG_IRQ	(IRQ_GPIO_0 + GPIO_SD1_CD_N)

#define GPIO_USB_DETE		102 /* GPD6 */
#define GPIO_DC_DETE_N		103 /* GPD7 */
#define GPIO_CHARG_STAT_N	111 /* GPD15 */
#define GPIO_DISP_OFF_N		121 /* GPD25, LCD_REV */
//#define GPIO_LED_EN       	124 /* GPD28 */

#define GPIO_UDC_HOTPLUG	GPIO_USB_DETE

#define GPIO_POWER_ON           (32 * 0 + 30)  /* GPA30 */

#define GPIO_TS_I2C_INT         GPE(0)
#define GPIO_TS_I2C_IRQ         (IRQ_GPIO_0 + GPIO_TS_I2C_INT)
/*======================================================================
 * LCD backlight
 */

#define GPIO_LCD_PWM   		(32*4+1) /* GPE4 PWM4 */
#define LCD_PWM_CHN 1    /* pwm channel */
#define LCD_PWM_FULL 101
#define LCD_DEFAULT_BACKLIGHT		80
#define LCD_MAX_BACKLIGHT		100
#define LCD_MIN_BACKLIGHT		1
#define PWM_BACKLIGHT_CHIP	1	/*0: digital pusle; 1: PWM*/

/* 100 level: 0,1,...,100 */

#if PWM_BACKLIGHT_CHIP

#define __lcd_init_backlight(n)					\
do {    							\
	__lcd_set_backlight_level(n);				\
} while (0)

/* 100 level: 0,1,...,100 */
#define __lcd_set_backlight_level(n)				\
do {								\
	__gpio_as_pwm(1);                               \
	__tcu_disable_pwm_output(LCD_PWM_CHN);			\
	__tcu_stop_counter(LCD_PWM_CHN);			\
	__tcu_init_pwm_output_high(LCD_PWM_CHN);		\
	__tcu_set_pwm_output_shutdown_abrupt(LCD_PWM_CHN);	\
	__tcu_select_clk_div1(LCD_PWM_CHN);			\
	__tcu_mask_full_match_irq(LCD_PWM_CHN);			\
	__tcu_mask_half_match_irq(LCD_PWM_CHN);			\
	__tcu_clear_counter_to_zero(LCD_PWM_CHN);		\
	__tcu_set_full_data(LCD_PWM_CHN, JZ_EXTAL / 30000);	\
	__tcu_set_half_data(LCD_PWM_CHN, JZ_EXTAL / 30000 * n / LCD_PWM_FULL); \
	__tcu_enable_pwm_output(LCD_PWM_CHN);			\
	__tcu_select_extalclk(LCD_PWM_CHN);			\
	__tcu_start_counter(LCD_PWM_CHN);			\
} while (0)

#define __lcd_close_backlight()					\
do {								\
      __tcu_stop_counter(LCD_PWM_CHN)   ;			\
      __gpio_as_output0(GPIO_LCD_PWM);				\
} while (0)

#else	/* PWM_BACKLIGHT_CHIP */

#define __send_low_pulse(n)					\
do {								\
	unsigned int i;						\
	for (i = n; i > 0; i--)	{				\
		__gpio_as_output0(GPIO_LCD_PWM);		\
		udelay(1);					\
		__gpio_as_output1(GPIO_LCD_PWM);		\
		udelay(3);					\
	}							\
} while (0)

#define MAX_BRIGHTNESS_STEP	16				/* RT9365 supports 16 brightness step */
#define CONVERT_FACTOR		(256/MAX_BRIGHTNESS_STEP)	/* System support 256 brightness step */

#define __lcd_init_backlight(n)					\
do {								\
	unsigned int tmp = (n)/CONVERT_FACTOR + 1;		\
	__gpio_as_output1(GPIO_LCD_PWM);			\
	udelay(30);						\
	__send_low_pulse(MAX_BRIGHTNESS_STEP-tmp);		\
} while (0)

#define __lcd_set_backlight_level(n)					\
do {									\
	unsigned int last = lcd_backlight_level / CONVERT_FACTOR + 1;	\
	unsigned int tmp = (n) / CONVERT_FACTOR + 1;			\
	if (tmp <= last) {						\
		__send_low_pulse(last-tmp);				\
	} else {							\
		__send_low_pulse(last + MAX_BRIGHTNESS_STEP - tmp);	\
	}								\
	udelay(30);							\
} while (0)

#define __lcd_close_backlight()					\
do {								\
	__gpio_as_output0(GPIO_LCD_PWM); 			\
} while (0)

#endif	/*PWM_BACKLIGHT_CHIP*/

#define ACTIVE_LOW_WAKE_UP 	1

#if 1
/* use uart2 as default */
#define JZ_BOOTUP_UART_TXD	(32 * 2 + 30)
#define JZ_BOOTUP_UART_RXD	(32 * 2 + 28)
#define JZ_EARLY_UART_BASE	UART2_BASE
#else
#define JZ_BOOTUP_UART_TXD	(32 * 4 + 5)
#define JZ_BOOTUP_UART_RXD	(32 * 3 + 12)
#define JZ_EARLY_UART_BASE	UART3_BASE
#endif

#endif /* __ASM_JZ4770_F4770_H__ */
