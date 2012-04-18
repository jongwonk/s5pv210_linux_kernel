/* linux/arch/arm/mach-s5pv210/mach-smdkv210.c
 *
 * Copyright (c) 2010 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
*/

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/serial_core.h>
#include <linux/device.h>
#include <linux/smsc911x.h>
#include <linux/fb.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/pwm_backlight.h>

#include <asm/hardware/vic.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/setup.h>
#include <asm/mach-types.h>

#include <video/platform_lcd.h>

#include <mach/map.h>
#include <mach/regs-clock.h>

#include <plat/regs-serial.h>
#include <plat/regs-srom.h>
#include <plat/gpio-cfg.h>
#include <plat/devs.h>
#include <plat/cpu.h>
#include <plat/adc.h>
#include <plat/ts.h>
#include <plat/ata.h>
#include <plat/iic.h>
#include <plat/pm.h>
#include <plat/fb.h>
#include <plat/s5p-time.h>
#include <plat/backlight.h>
#include <plat/regs-fb-v4.h>
#include <plat/mfc.h>

#include <mach/regs-gpio.h>
#include "common.h"

#include <linux/spi/spi_gpio.h>
#include <linux/spi/spi.h>

#include <mach/regs-gpio.h>
#include <linux/gpio_keys.h>
#include <linux/input.h>
#include <linux/kxtf9.h>


#define S5P_PA_SMC9115 0xA0000000 
#define IRQ_EINT6 6+32

#define SMC9115_Tacs	(0)	// 0clk		address set-up
#define SMC9115_Tcos	(4)	// 4clk		chip selection set-up
#define SMC9115_Tacc	(13)	// 14clk	access cycle
#define SMC9115_Tcoh	(1)	// 1clk		chip selection hold
#define SMC9115_Tah	    (4)	// 4clk		address holding time
#define SMC9115_Tacp	(6)	// 6clk		page mode access cycle
#define SMC9115_PMC	    (0)	// normal(1data)page mode configuration

/* Following are default values for UCON, ULCON and UFCON UART registers */
#define SMDKV210_UCON_DEFAULT	(S3C2410_UCON_TXILEVEL |	\
				 S3C2410_UCON_RXILEVEL |	\
				 S3C2410_UCON_TXIRQMODE |	\
				 S3C2410_UCON_RXIRQMODE |	\
				 S3C2410_UCON_RXFIFO_TOI |	\
				 S3C2443_UCON_RXERR_IRQEN)

#define SMDKV210_ULCON_DEFAULT	S3C2410_LCON_CS8

#define SMDKV210_UFCON_DEFAULT	(S3C2410_UFCON_FIFOMODE |	\
				 S5PV210_UFCON_TXTRIG4 |	\
				 S5PV210_UFCON_RXTRIG4)

static struct s3c2410_uartcfg smdkv210_uartcfgs[] __initdata = {
	[0] = {
		.hwport		= 0,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
	[1] = {
		.hwport		= 1,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
	[2] = {
		.hwport		= 2,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
	[3] = {
		.hwport		= 3,
		.flags		= 0,
		.ucon		= SMDKV210_UCON_DEFAULT,
		.ulcon		= SMDKV210_ULCON_DEFAULT,
		.ufcon		= SMDKV210_UFCON_DEFAULT,
	},
};


static struct smsc911x_platform_config smsc911x_config = {
        .irq_polarity   = SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
        .irq_type       = SMSC911X_IRQ_TYPE_PUSH_PULL,
        .flags          = SMSC911X_USE_32BIT,
        .phy_interface  = PHY_INTERFACE_MODE_MII,
};

static struct resource s5p_smsc911x_resources[] = {
        [0] = {
                .start = S5P_PA_SMC9115,
                //.end   = S5P_PA_SMC9115 + 0xff,
                .end   = S5P_PA_SMC9115 + SZ_1K,
                .flags = IORESOURCE_MEM,
        },
        [1] = {
                .start = IRQ_EINT6,
                .end   = IRQ_EINT6,
                .flags = IORESOURCE_IRQ | IORESOURCE_IRQ_LOWLEVEL,
        }
};

struct platform_device s5p_device_smsc911x = {
        .name           = "smsc911x",
        .id             =  -1,
        .num_resources  = ARRAY_SIZE(s5p_smsc911x_resources),
        .resource       = s5p_smsc911x_resources,
        .dev = {
                .platform_data = &smsc911x_config,
        },
};

struct kxtf9_platform_data mapphone_kxtf9_data = {

      .min_interval	= 2,
      .poll_interval	= 200,

      .g_range	= KXTF9_G_8G,

      .axis_map_x	= 1,
      .axis_map_y	= 0,
      .axis_map_z	= 2,

      .negate_x	= 0,
      .negate_y	= 0,
      .negate_z	= 1,

      .data_odr_init		= ODR25,
      .ctrl_reg1_init		= RES_12BIT | KXTF9_G_8G | TPE | TDTE,
      .int_ctrl_init		= IEA | IEN,
      .tilt_timer_init	= 0x03,
      .engine_odr_init	= OTP12_5 | OWUF25 | OTDT400,
      .wuf_timer_init		= 0x0A,
      .wuf_thresh_init	= 0x20,
      .tdt_timer_init		= 0x78,
      .tdt_h_thresh_init	= 0xB6,
      .tdt_l_thresh_init	= 0x1A,
      .tdt_tap_timer_init	= 0xA2,
      .tdt_total_timer_init	= 0x24,
      .tdt_latency_timer_init	= 0x28,
      .tdt_window_timer_init	= 0xA0,

      .gpio = IRQ_EINT5,
      .gesture = 0,
      .sensitivity_low = {
		0x50, 0xFF, 0xFF, 0x32, 0x09, 0x0A, 0xA0,
      },
      .sensitivity_medium = {
		0x50, 0xFF, 0x68, 0xA3, 0x09, 0x0A, 0xA0,
      },
      .sensitivity_high = {
		0x78, 0xB6, 0x1A, 0xA2, 0x24, 0x28, 0xA0,
      },
};

static struct gpio_keys_button aesop_gpio_keys_table[] = {
	{
		.code 	= KEY_DOWN,
		.gpio		= S5PV210_GPC1(0),
		.desc		= "KEY_DOWN",
		.type		= EV_KEY,
		.active_low	= 1,
		.wakeup		= 1,
		.debounce_interval = 5,
		
	},
	
	{
		.code 	= KEY_ENTER,
		.gpio		= S5PV210_GPC1(1),
		.desc		= "KEY_ENTER",
		.type		= EV_KEY,
		.active_low	= 1,
		.wakeup		= 0,
		.debounce_interval = 5,
	},
	{
		.code 	= KEY_RIGHT,
		.gpio		= S5PV210_GPC1(2),
		.desc		= "KEY_RIGHT",
		.type		= EV_KEY,
		.active_low	= 1,
		.wakeup		= 0,
		.debounce_interval = 5,
	},
	{
		.code 	= KEY_LEFT,
		.gpio		= S5PV210_GPA1(2),
		.desc		= "KEY_LEFT",
		.type		= EV_KEY,
		.active_low	= 1,
		.wakeup		= 0,
		.debounce_interval = 5,
	},
	{
		.code 	= KEY_UP,
		.gpio		= S5PV210_GPA1(3),
		.desc		= "KEY_UP",
		.type		= EV_KEY,
		.active_low	= 1,
		.wakeup		= 0,
		.debounce_interval = 5,
	},	

};

static struct gpio_keys_platform_data aesop_gpio_keys_data = {
	.buttons	= aesop_gpio_keys_table,
	.nbuttons	= ARRAY_SIZE(aesop_gpio_keys_table),
};

static struct platform_device aesop_device_gpiokeys = {
	.name = "gpio-keys",
	.id = -1,
	.dev = {
		.platform_data = &aesop_gpio_keys_data,
	},
};

static struct spi_gpio_platform_data s3c64xx_spi_gpio_data = {
        .sck    = S5PV210_GPB(4),
        .mosi   = S5PV210_GPB(7),
        .miso   = S5PV210_GPB(6),
        .num_chipselect = 1,
};

static struct spi_board_info spidev_spi_board_info[] __initdata = {
  [0] = {
	  .modalias       = "ams369fg06", 
	  .platform_data  = &s3c64xx_spi_gpio_data,
	  .max_speed_hz   = 100000,
	  .bus_num        = 1,
	  .chip_select    = 0,
	  .mode           = SPI_MODE_3,
	  .controller_data = (void *)S5PV210_GPB(5), // SPI1CS
	},
};

static struct spi_gpio_platform_data tl2796_spi_gpio_data = {
        .sck    = S5PV210_GPB(4),
        .mosi   = S5PV210_GPB(7),
        .miso   = S5PV210_GPB(6),
        .num_chipselect = 1,
};

static struct platform_device s3c_device_spi_gpio = {
        .name   = "spi_gpio",
        .id     = 1,
        .dev    = {
                .parent         = &s3c_device_fb.dev,
                .platform_data  = &tl2796_spi_gpio_data,
        },
};


static struct s3c_fb_pd_win smdkv210_fb_win0 = {

	.win_mode = {
		.left_margin	= 13,
		.right_margin	= 8,
		.upper_margin	= 7,
		.lower_margin	= 5,
		.hsync_len	= 3,
		.vsync_len	= 1,
		.xres		= 480,
		.yres		= 800,
	},

/*
	.win_mode = {
		.left_margin	= 16,
		.right_margin	= 16,
		.upper_margin	= 2,
		.lower_margin	= 28,
		.hsync_len	= 2,
		.vsync_len	= 1,
		.xres		= 480,
		.yres		= 800,
		.refresh	= 60,
	},
*/
	.max_bpp	= 32,
	.default_bpp	= 24,
	
};

static struct s3c_fb_platdata smdkv210_lcd0_pdata __initdata = {
	.win[0]		= &smdkv210_fb_win0,
	.vidcon0	= VIDCON0_VIDOUT_RGB | VIDCON0_PNRMODE_RGB|VIDCON0_CLKSEL_LCD,
	.vidcon1	= VIDCON1_INV_VCLK | VIDCON1_INV_VDEN,
	.setup_gpio	= s5pv210_fb_gpio_setup_24bpp,
};

static struct platform_device *smdkv210_devices[] __initdata = {
	&s3c_device_cfcon,
	&s5p_device_smsc911x,
	&s3c_device_hsmmc0,
	&s3c_device_hsmmc1,
	&s3c_device_hsmmc2,
	&s3c_device_hsmmc3,
	&s3c_device_rtc,
//	&s3c_device_ts,

	&s3c_device_i2c0,
	&s3c_device_i2c1,
	&s3c_device_i2c2,

	&s3c_device_wdt,
	&s5p_device_fimc0,
	&s5p_device_fimc1,
	&s5p_device_fimc2,
	&s5p_device_fimc_md,
	&s5p_device_jpeg,
	&s5p_device_mfc,
	&s5p_device_mfc_l,
	&s5p_device_mfc_r,
	&s5pv210_device_ac97,
	&s5pv210_device_iis0,
	&s5pv210_device_spdif,
	&samsung_asoc_dma,
	&samsung_asoc_idma,
	&s3c_device_fb,
	&s3c_device_spi_gpio,
	&aesop_device_gpiokeys,
};

static void __init s5pv210_smc911x_set(void)
{
        unsigned int tmp;

        tmp = ((SMC9115_Tacs<<28)|(SMC9115_Tcos<<24)|(SMC9115_Tacc<<16)|(SMC9115_Tcoh<<12)|(SMC9115_Tah<<8)|(SMC9115_Tacp<<4)|(SMC9115_PMC));
        __raw_writel(tmp, (S5P_SROM_BW+0x14));

        tmp = __raw_readl(S5P_SROM_BW);
        tmp &= ~(0xf<<16);

        tmp |= (0x1<<16);
        tmp |= (0x1<<17);

        __raw_writel(tmp, S5P_SROM_BW);

        tmp = __raw_readl(S5PV210_MP01CON);
        tmp &= ~(0xf<<16);
        tmp |=(2<<16);

        __raw_writel(tmp,(S5PV210_MP01CON));
}

static struct i2c_board_info smdkv210_i2c_devs0[] __initdata = {
	{I2C_BOARD_INFO("hidis-ts", 0x46),},
	{ I2C_BOARD_INFO("max98088", 0x10), },
	{ 	I2C_BOARD_INFO("accel", 0x0F),
 		.platform_data = &mapphone_kxtf9_data,
 	},	
};

static struct i2c_board_info smdkv210_i2c_devs1[] __initdata = {
	/* To Be Updated */
};

static struct i2c_board_info smdkv210_i2c_devs2[] __initdata = {
	/* To Be Updated */
};

/* LCD Backlight data */
static struct samsung_bl_gpio_info smdkv210_bl_gpio_info = {
	.no = S5PV210_GPD0(3),
	.func = S3C_GPIO_SFN(2),
};

static struct platform_pwm_backlight_data smdkv210_bl_data = {
	.pwm_id = 3,
	.pwm_period_ns = 1000,
};

static void __init aesop_navi_init(void)
{
	int gpio;

	gpio = S5PV210_GPC1(0);		
	 s5p_register_gpio_interrupt(gpio);

	gpio = S5PV210_GPC1(1);		
	s5p_register_gpio_interrupt(gpio);

	gpio = S5PV210_GPC1(2);		
	s5p_register_gpio_interrupt(gpio);

	gpio = S5PV210_GPA1(2);		
	s5p_register_gpio_interrupt(gpio);

	gpio = S5PV210_GPA1(3);		
	s5p_register_gpio_interrupt(gpio);
		
}	

static void __init smdkv210_map_io(void)
{
	s5pv210_init_io(NULL, 0);
	s3c24xx_init_clocks(24000000);
	s3c24xx_init_uarts(smdkv210_uartcfgs, ARRAY_SIZE(smdkv210_uartcfgs));
	s5p_set_timer_source(S5P_PWM2, S5P_PWM4);
}

static void __init smdkv210_reserve(void)
{
	s5p_mfc_reserve_mem(0x43000000, 8 << 20, 0x51000000, 8 << 20);
}

static void __init smdkv210_machine_init(void)
{
	  
	s3c_pm_init();

	s5pv210_smc911x_set();

//	s3c24xx_ts_set_platdata(NULL);
	
	s3c_i2c0_set_platdata(NULL);
	i2c_register_board_info(0, smdkv210_i2c_devs0, 
			ARRAY_SIZE(smdkv210_i2c_devs0));

	s3c_i2c1_set_platdata(NULL);			
	i2c_register_board_info(1, smdkv210_i2c_devs1,
			ARRAY_SIZE(smdkv210_i2c_devs1));

	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(2, smdkv210_i2c_devs2,
			ARRAY_SIZE(smdkv210_i2c_devs2));
			
/*	
	s3c_i2c1_set_platdata(NULL);
	s3c_i2c2_set_platdata(NULL);
	i2c_register_board_info(0, smdkv210_i2c_devs0,
			ARRAY_SIZE(smdkv210_i2c_devs0));
	i2c_register_board_info(1, smdkv210_i2c_devs1,
			ARRAY_SIZE(smdkv210_i2c_devs1));
	i2c_register_board_info(2, smdkv210_i2c_devs2,
			ARRAY_SIZE(smdkv210_i2c_devs2));
	
	s3c_ide_set_platdata(&smdkv210_ide_pdata);
*/

      
	spi_register_board_info(spidev_spi_board_info, ARRAY_SIZE(spidev_spi_board_info));
	
	s3c_fb_set_platdata(&smdkv210_lcd0_pdata);

	samsung_bl_set(&smdkv210_bl_gpio_info, &smdkv210_bl_data);

	platform_add_devices(smdkv210_devices, ARRAY_SIZE(smdkv210_devices));
	
	aesop_navi_init();

}

MACHINE_START(SMDKV210, "SMDKV210")
	/* Maintainer: Kukjin Kim <kgene.kim@samsung.com> */
	.atag_offset	= 0x100,
	.init_irq	= s5pv210_init_irq,
	.handle_irq	= vic_handle_irq,
	.map_io		= smdkv210_map_io,
	.init_machine	= smdkv210_machine_init,
	.timer		= &s5p_timer,
	.restart	= s5pv210_restart,
	.reserve	= &smdkv210_reserve,
MACHINE_END

