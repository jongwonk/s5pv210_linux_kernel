/*
 * ams369fg06 AMOLED LCD panel driver.
 *
 * Copyright (c) 2011 Samsung Electronics Co., Ltd.
 * Author: Jingoo Han  <jg1.han@samsung.com>
 *
 * Derived from drivers/video/s6e63m0.c
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#include <linux/wait.h>
#include <linux/module.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/spi/spi.h>
#include <linux/lcd.h>
#include <linux/backlight.h>

#define SLEEPMSEC		0x1000
#define ENDDEF			0x2000
#define	DEFMASK			0xFF00
#define COMMAND_ONLY		0xFE
#define DATA_ONLY		0xFF

#define MAX_GAMMA_LEVEL		5
#define GAMMA_TABLE_COUNT	21

#define MIN_BRIGHTNESS		0
#define MAX_BRIGHTNESS		255
#define DEFAULT_BRIGHTNESS	150

struct ams369fg06 {
	struct device			*dev;
	struct spi_device		*spi;
	unsigned int			power;
	struct lcd_device		*ld;
	struct backlight_device		*bd;
	struct lcd_platform_data	*lcd_pd;
};


static struct ams369fg06 *lcd1;
    
static const unsigned short seq_display_on[] = {

	0x14, 0x03,
	ENDDEF, 0x0000

};

static const unsigned short seq_display_off[] = {
	0x14, 0x00,
	ENDDEF, 0x0000
};

static const unsigned short seq_stand_by_on[] = {
	0x1D, 0xA1,
	SLEEPMSEC, 200,
	ENDDEF, 0x0000
};

static const unsigned short seq_stand_by_off[] = {
	0x1D, 0xA0,
	SLEEPMSEC, 250,
	ENDDEF, 0x0000
};

static const unsigned short seq_setting[] = {
	0x31, 0x08,
	0x32, 0x14,
	0x30, 0x02,
	0x27, 0x01,
	0x12, 0x08,
	0x13, 0x08,
	0x15, 0x00,
	0x16, 0x12, // 0x16, 0x00,

	0xef, 0xd0,
	DATA_ONLY, 0xe8,

	0x39, 0x44,
	0x40, 0x00,
	0x41, 0x3f,
	0x42, 0x2a,
	0x43, 0x27,
	0x44, 0x27,
	0x45, 0x1f,
	0x46, 0x44,
	0x50, 0x00,
	0x51, 0x00,
	0x52, 0x17,
	0x53, 0x24,
	0x54, 0x26,
	0x55, 0x1f,
	0x56, 0x43,
	0x60, 0x00,
	0x61, 0x3f,
	0x62, 0x2a,
	0x63, 0x25,
	0x64, 0x24,
	0x65, 0x1b,
	0x66, 0x5c,

	0x17, 0x22,
	0x18, 0x33,
	0x19, 0x03,
	0x1a, 0x01,
	0x22, 0xa4,
	0x23, 0x00,
	0x26, 0xa0,

	0x1d, 0xa0,
	SLEEPMSEC, 300,

	0x14, 0x03,

	ENDDEF, 0x0000
	
};

static int ams369fg06_spi_write_byte(struct ams369fg06 *lcd, int addr, int data)
{
	u16 buf[1];
	struct spi_message msg;

	struct spi_transfer xfer = {
		.len		= 2,
		.tx_buf		= buf,
	};

	buf[0] = (addr << 8) | data;

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	return spi_sync(lcd->spi, &msg);
}

static int ams369fg06_spi_write(struct ams369fg06 *lcd, unsigned char address,
	unsigned char command)
{
	int ret = 0;

	if (address != DATA_ONLY)
		ret = ams369fg06_spi_write_byte(lcd, 0x70, address);

	ret = ams369fg06_spi_write_byte(lcd, 0x72, command);

	return ret;
}

static int ams369fg06_panel_send_sequence(struct ams369fg06 *lcd,
	const unsigned short *wbuf)
{
	int ret = 0, i = 0;

	while ((wbuf[i] & DEFMASK) != ENDDEF) {
		if ((wbuf[i] & DEFMASK) != SLEEPMSEC) {
			ret = ams369fg06_spi_write(lcd, wbuf[i], wbuf[i+1]);
		} else
			mdelay(wbuf[i+1]);
		i += 2;
	}

	return ret;
}

static int ams369fg06_ldi_init(struct ams369fg06 *lcd)
{
	int ret, i;
	static const unsigned short *init_seq[] = {
		seq_setting,
		seq_stand_by_off,
	};

	for (i = 0; i < ARRAY_SIZE(init_seq); i++) {
		ret = ams369fg06_panel_send_sequence(lcd, init_seq[i]);
		if (ret)
			break;
	}

	return ret;
}

static int ams369fg06_ldi_enable(struct ams369fg06 *lcd)
{
	int ret, i;
	static const unsigned short *init_seq[] = {
		seq_display_on,
		seq_stand_by_off,		
	};

	for (i = 0; i < ARRAY_SIZE(init_seq); i++) {
		ret = ams369fg06_panel_send_sequence(lcd, init_seq[i]);
		if (ret)
			break;
	}

	return ret;
}

static int __devinit ams369fg06_probe(struct spi_device *spi)
{
	int ret = 0;
	struct ams369fg06 *lcd = NULL;
	struct lcd_device *ld = NULL;
	struct backlight_device *bd = NULL;
	struct backlight_properties props;
	
	lcd = kzalloc(sizeof(struct ams369fg06), GFP_KERNEL);
	if (!lcd)
		return -ENOMEM;

	lcd1 = lcd;
	
	/* ams369fg06 lcd panel uses 3-wire 16bits SPI Mode. */
	spi->bits_per_word = 16;

	ret = spi_setup(spi);
	if (ret < 0) {
		dev_err(&spi->dev, "spi setup failed.\n");
		goto out_free_lcd;
	}

	lcd->spi = spi;
	lcd->dev = &spi->dev;

	ams369fg06_ldi_init(lcd);
	ams369fg06_ldi_enable(lcd);

	dev_set_drvdata(&spi->dev, lcd);

	dev_info(&spi->dev, "ams369fg06 panel driver has been probed.\n");
	
	return 0;

out_lcd_unregister:
	lcd_device_unregister(ld);
out_free_lcd:
	kfree(lcd);
	return ret;
}

static int __devexit ams369fg06_remove(struct spi_device *spi)
{
	struct ams369fg06 *lcd = dev_get_drvdata(&spi->dev);

	backlight_device_unregister(lcd->bd);
	lcd_device_unregister(lcd->ld);
	kfree(lcd);

	return 0;
}

#if defined(CONFIG_PM)
static unsigned int before_power;

static int ams369fg06_suspend(struct spi_device *spi, pm_message_t mesg)
{
	int ret = 0;
	struct ams369fg06 *lcd = dev_get_drvdata(&spi->dev);

	dev_dbg(&spi->dev, "lcd->power = %d\n", lcd->power);

	before_power = lcd->power;


	return ret;
}

static int ams369fg06_resume(struct spi_device *spi)
{
	int ret = 0;
	struct ams369fg06 *lcd = dev_get_drvdata(&spi->dev);

	/*
	 * after suspended, if lcd panel status is FB_BLANK_UNBLANK
	 * (at that time, before_power is FB_BLANK_UNBLANK) then
	 * it changes that status to FB_BLANK_POWERDOWN to get lcd on.
	 */
	if (before_power == FB_BLANK_UNBLANK)
		lcd->power = FB_BLANK_POWERDOWN;

	dev_dbg(&spi->dev, "before_power = %d\n", before_power);

	return ret;
}
#else
#define ams369fg06_suspend	NULL
#define ams369fg06_resume	NULL
#endif

static void ams369fg06_shutdown(struct spi_device *spi)
{
	struct ams369fg06 *lcd = dev_get_drvdata(&spi->dev);

}

static struct spi_driver ams369fg06_driver = {
	.driver = {
		.name	= "ams369fg06",
		.bus	= &spi_bus_type,
		.owner	= THIS_MODULE,
	},
	.probe		= ams369fg06_probe,
	.remove		= __devexit_p(ams369fg06_remove),
	.shutdown	= ams369fg06_shutdown,
	.suspend	= ams369fg06_suspend,
	.resume		= ams369fg06_resume,
};

module_spi_driver(ams369fg06_driver);

MODULE_AUTHOR("Jingoo Han <jg1.han@samsung.com>");
MODULE_DESCRIPTION("ams369fg06 LCD Driver");
MODULE_LICENSE("GPL");




// aesop

#include <linux/platform_device.h>
#include <linux/sysfs.h>

#define	DEVICE_NAME	"ams369fg06"

int str2int(const char *buf, size_t count)
{
	int i;
	if (buf[0] == '0' && (buf[1] == 'x' || buf[1] == 'X')) {
	  i = simple_strtoul(&buf[2], NULL, 16);
	} else {
	  i = simple_strtoul(buf, NULL, 10);
	}
	if (i <   0) i = 0;
	if (i > 255) i = 255;
	printk("input=%d(0x%x)\n", i, i);
	return i;
}


static void tl2796_spi_write(  unsigned char address, unsigned char command )
{
    ams369fg06_spi_write(lcd1, address, command );
}

static ssize_t tl2796_11_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x11, color);  return count; } static DEVICE_ATTR(11, (S_IWUGO), NULL, tl2796_11_write);
static ssize_t tl2796_12_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x12, color);  return count; } static DEVICE_ATTR(12, (S_IWUGO), NULL, tl2796_12_write);
static ssize_t tl2796_13_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x13, color);  return count; } static DEVICE_ATTR(13, (S_IWUGO), NULL, tl2796_13_write);
static ssize_t tl2796_14_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x14, color);  return count; } static DEVICE_ATTR(14, (S_IWUGO), NULL, tl2796_14_write);
static ssize_t tl2796_15_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x15, color);  return count; } static DEVICE_ATTR(15, (S_IWUGO), NULL, tl2796_15_write);
static ssize_t tl2796_16_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x16, color);  return count; } static DEVICE_ATTR(16, (S_IWUGO), NULL, tl2796_16_write);
static ssize_t tl2796_17_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x17, color);  return count; } static DEVICE_ATTR(17, (S_IWUGO), NULL, tl2796_17_write);
static ssize_t tl2796_18_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x18, color);  return count; } static DEVICE_ATTR(18, (S_IWUGO), NULL, tl2796_18_write);
static ssize_t tl2796_19_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x19, color);  return count; } static DEVICE_ATTR(19, (S_IWUGO), NULL, tl2796_19_write);
static ssize_t tl2796_1A_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x1A, color);  return count; } static DEVICE_ATTR(1A, (S_IWUGO), NULL, tl2796_1A_write);
static ssize_t tl2796_1B_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x1B, color);  return count; } static DEVICE_ATTR(1B, (S_IWUGO), NULL, tl2796_1B_write);
static ssize_t tl2796_1C_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x1C, color);  return count; } static DEVICE_ATTR(1C, (S_IWUGO), NULL, tl2796_1C_write);
static ssize_t tl2796_1D_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x1D, color);  return count; } static DEVICE_ATTR(1D, (S_IWUGO), NULL, tl2796_1D_write);
static ssize_t tl2796_1E_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x1E, color);  return count; } static DEVICE_ATTR(1E, (S_IWUGO), NULL, tl2796_1E_write);
static ssize_t tl2796_1F_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x1F, color);  return count; } static DEVICE_ATTR(1F, (S_IWUGO), NULL, tl2796_1F_write);
static ssize_t tl2796_20_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x20, color);  return count; } static DEVICE_ATTR(20, (S_IWUGO), NULL, tl2796_20_write);
static ssize_t tl2796_21_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x21, color);  return count; } static DEVICE_ATTR(21, (S_IWUGO), NULL, tl2796_21_write);
static ssize_t tl2796_22_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x22, color);  return count; } static DEVICE_ATTR(22, (S_IWUGO), NULL, tl2796_22_write);
static ssize_t tl2796_23_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x23, color);  return count; } static DEVICE_ATTR(23, (S_IWUGO), NULL, tl2796_23_write);
static ssize_t tl2796_24_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x24, color);  return count; } static DEVICE_ATTR(24, (S_IWUGO), NULL, tl2796_24_write);
static ssize_t tl2796_25_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x25, color);  return count; } static DEVICE_ATTR(25, (S_IWUGO), NULL, tl2796_25_write);
static ssize_t tl2796_26_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x26, color);  return count; } static DEVICE_ATTR(26, (S_IWUGO), NULL, tl2796_26_write);
static ssize_t tl2796_27_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x27, color);  return count; } static DEVICE_ATTR(27, (S_IWUGO), NULL, tl2796_27_write);
static ssize_t tl2796_28_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x28, color);  return count; } static DEVICE_ATTR(28, (S_IWUGO), NULL, tl2796_28_write);
static ssize_t tl2796_29_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x29, color);  return count; } static DEVICE_ATTR(29, (S_IWUGO), NULL, tl2796_29_write);
static ssize_t tl2796_2A_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x2A, color);  return count; } static DEVICE_ATTR(2A, (S_IWUGO), NULL, tl2796_2A_write);
static ssize_t tl2796_2B_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x2B, color);  return count; } static DEVICE_ATTR(2B, (S_IWUGO), NULL, tl2796_2B_write);
static ssize_t tl2796_2C_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x2C, color);  return count; } static DEVICE_ATTR(2C, (S_IWUGO), NULL, tl2796_2C_write);
static ssize_t tl2796_2D_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x2D, color);  return count; } static DEVICE_ATTR(2D, (S_IWUGO), NULL, tl2796_2D_write);
static ssize_t tl2796_2E_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x2E, color);  return count; } static DEVICE_ATTR(2E, (S_IWUGO), NULL, tl2796_2E_write);
static ssize_t tl2796_2F_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x2F, color);  return count; } static DEVICE_ATTR(2F, (S_IWUGO), NULL, tl2796_2F_write);
static ssize_t tl2796_30_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x30, color);  return count; } static DEVICE_ATTR(30, (S_IWUGO), NULL, tl2796_30_write);
static ssize_t tl2796_31_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x31, color);  return count; } static DEVICE_ATTR(31, (S_IWUGO), NULL, tl2796_31_write);
static ssize_t tl2796_32_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x32, color);  return count; } static DEVICE_ATTR(32, (S_IWUGO), NULL, tl2796_32_write);
static ssize_t tl2796_33_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x33, color);  return count; } static DEVICE_ATTR(33, (S_IWUGO), NULL, tl2796_33_write);
static ssize_t tl2796_34_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x34, color);  return count; } static DEVICE_ATTR(34, (S_IWUGO), NULL, tl2796_34_write);
static ssize_t tl2796_35_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x35, color);  return count; } static DEVICE_ATTR(35, (S_IWUGO), NULL, tl2796_35_write);
static ssize_t tl2796_36_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x36, color);  return count; } static DEVICE_ATTR(36, (S_IWUGO), NULL, tl2796_36_write);
static ssize_t tl2796_37_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x37, color);  return count; } static DEVICE_ATTR(37, (S_IWUGO), NULL, tl2796_37_write);
static ssize_t tl2796_38_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x38, color);  return count; } static DEVICE_ATTR(38, (S_IWUGO), NULL, tl2796_38_write);
static ssize_t tl2796_39_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x39, color);  return count; } static DEVICE_ATTR(39, (S_IWUGO), NULL, tl2796_39_write);
static ssize_t tl2796_3A_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x3A, color);  return count; } static DEVICE_ATTR(3A, (S_IWUGO), NULL, tl2796_3A_write);
static ssize_t tl2796_3B_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x3B, color);  return count; } static DEVICE_ATTR(3B, (S_IWUGO), NULL, tl2796_3B_write);
static ssize_t tl2796_3C_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x3C, color);  return count; } static DEVICE_ATTR(3C, (S_IWUGO), NULL, tl2796_3C_write);
static ssize_t tl2796_3D_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x3D, color);  return count; } static DEVICE_ATTR(3D, (S_IWUGO), NULL, tl2796_3D_write);
static ssize_t tl2796_3E_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x3E, color);  return count; } static DEVICE_ATTR(3E, (S_IWUGO), NULL, tl2796_3E_write);
static ssize_t tl2796_3F_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x3F, color);  return count; } static DEVICE_ATTR(3F, (S_IWUGO), NULL, tl2796_3F_write);
static ssize_t tl2796_40_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x40, color);  return count; } static DEVICE_ATTR(40, (S_IWUGO), NULL, tl2796_40_write);
static ssize_t tl2796_41_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x41, color);  return count; } static DEVICE_ATTR(41, (S_IWUGO), NULL, tl2796_41_write);
static ssize_t tl2796_42_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x42, color);  return count; } static DEVICE_ATTR(42, (S_IWUGO), NULL, tl2796_42_write);
static ssize_t tl2796_43_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x43, color);  return count; } static DEVICE_ATTR(43, (S_IWUGO), NULL, tl2796_43_write);
static ssize_t tl2796_44_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x44, color);  return count; } static DEVICE_ATTR(44, (S_IWUGO), NULL, tl2796_44_write);
static ssize_t tl2796_45_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x45, color);  return count; } static DEVICE_ATTR(45, (S_IWUGO), NULL, tl2796_45_write);
static ssize_t tl2796_46_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x46, color);  return count; } static DEVICE_ATTR(46, (S_IWUGO), NULL, tl2796_46_write);
static ssize_t tl2796_47_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x47, color);  return count; } static DEVICE_ATTR(47, (S_IWUGO), NULL, tl2796_47_write);
static ssize_t tl2796_48_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x48, color);  return count; } static DEVICE_ATTR(48, (S_IWUGO), NULL, tl2796_48_write);
static ssize_t tl2796_49_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x49, color);  return count; } static DEVICE_ATTR(49, (S_IWUGO), NULL, tl2796_49_write);
static ssize_t tl2796_4A_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x4A, color);  return count; } static DEVICE_ATTR(4A, (S_IWUGO), NULL, tl2796_4A_write);
static ssize_t tl2796_4B_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x4B, color);  return count; } static DEVICE_ATTR(4B, (S_IWUGO), NULL, tl2796_4B_write);
static ssize_t tl2796_4C_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x4C, color);  return count; } static DEVICE_ATTR(4C, (S_IWUGO), NULL, tl2796_4C_write);
static ssize_t tl2796_4D_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x4D, color);  return count; } static DEVICE_ATTR(4D, (S_IWUGO), NULL, tl2796_4D_write);
static ssize_t tl2796_4E_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x4E, color);  return count; } static DEVICE_ATTR(4E, (S_IWUGO), NULL, tl2796_4E_write);
static ssize_t tl2796_4F_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x4F, color);  return count; } static DEVICE_ATTR(4F, (S_IWUGO), NULL, tl2796_4F_write);
static ssize_t tl2796_50_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x50, color);  return count; } static DEVICE_ATTR(50, (S_IWUGO), NULL, tl2796_50_write);
static ssize_t tl2796_51_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x51, color);  return count; } static DEVICE_ATTR(51, (S_IWUGO), NULL, tl2796_51_write);
static ssize_t tl2796_52_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x52, color);  return count; } static DEVICE_ATTR(52, (S_IWUGO), NULL, tl2796_52_write);
static ssize_t tl2796_53_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x53, color);  return count; } static DEVICE_ATTR(53, (S_IWUGO), NULL, tl2796_53_write);
static ssize_t tl2796_54_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x54, color);  return count; } static DEVICE_ATTR(54, (S_IWUGO), NULL, tl2796_54_write);
static ssize_t tl2796_55_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x55, color);  return count; } static DEVICE_ATTR(55, (S_IWUGO), NULL, tl2796_55_write);
static ssize_t tl2796_56_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x56, color);  return count; } static DEVICE_ATTR(56, (S_IWUGO), NULL, tl2796_56_write);
static ssize_t tl2796_57_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x57, color);  return count; } static DEVICE_ATTR(57, (S_IWUGO), NULL, tl2796_57_write);
static ssize_t tl2796_58_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x58, color);  return count; } static DEVICE_ATTR(58, (S_IWUGO), NULL, tl2796_58_write);
static ssize_t tl2796_59_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x59, color);  return count; } static DEVICE_ATTR(59, (S_IWUGO), NULL, tl2796_59_write);
static ssize_t tl2796_5A_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x5A, color);  return count; } static DEVICE_ATTR(5A, (S_IWUGO), NULL, tl2796_5A_write);
static ssize_t tl2796_5B_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x5B, color);  return count; } static DEVICE_ATTR(5B, (S_IWUGO), NULL, tl2796_5B_write);
static ssize_t tl2796_5C_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x5C, color);  return count; } static DEVICE_ATTR(5C, (S_IWUGO), NULL, tl2796_5C_write);
static ssize_t tl2796_5D_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x5D, color);  return count; } static DEVICE_ATTR(5D, (S_IWUGO), NULL, tl2796_5D_write);
static ssize_t tl2796_5E_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x5E, color);  return count; } static DEVICE_ATTR(5E, (S_IWUGO), NULL, tl2796_5E_write);
static ssize_t tl2796_5F_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x5F, color);  return count; } static DEVICE_ATTR(5F, (S_IWUGO), NULL, tl2796_5F_write);
static ssize_t tl2796_60_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x60, color);  return count; } static DEVICE_ATTR(60, (S_IWUGO), NULL, tl2796_60_write);
static ssize_t tl2796_61_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x61, color);  return count; } static DEVICE_ATTR(61, (S_IWUGO), NULL, tl2796_61_write);
static ssize_t tl2796_62_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x62, color);  return count; } static DEVICE_ATTR(62, (S_IWUGO), NULL, tl2796_62_write);
static ssize_t tl2796_63_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x63, color);  return count; } static DEVICE_ATTR(63, (S_IWUGO), NULL, tl2796_63_write);
static ssize_t tl2796_64_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x64, color);  return count; } static DEVICE_ATTR(64, (S_IWUGO), NULL, tl2796_64_write);
static ssize_t tl2796_65_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x65, color);  return count; } static DEVICE_ATTR(65, (S_IWUGO), NULL, tl2796_65_write);
static ssize_t tl2796_66_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t count) {  int color = str2int(buf, count);  tl2796_spi_write(0x66, color);  return count; } static DEVICE_ATTR(66, (S_IWUGO), NULL, tl2796_66_write);

static struct attribute *tl2796_sysfs_attributes[] = {
	&dev_attr_11.attr,
	&dev_attr_12.attr,
	&dev_attr_13.attr,
	&dev_attr_14.attr,
	&dev_attr_15.attr,
	&dev_attr_16.attr,
	&dev_attr_17.attr,
	&dev_attr_18.attr,
	&dev_attr_19.attr,
	&dev_attr_1A.attr,
	&dev_attr_1B.attr,
	&dev_attr_1C.attr,
	&dev_attr_1D.attr,
	&dev_attr_1E.attr,
	&dev_attr_1F.attr,
	&dev_attr_20.attr,
	&dev_attr_21.attr,
	&dev_attr_22.attr,
	&dev_attr_23.attr,
	&dev_attr_24.attr,
	&dev_attr_25.attr,
	&dev_attr_26.attr,
	&dev_attr_27.attr,
	&dev_attr_28.attr,
	&dev_attr_29.attr,
	&dev_attr_2A.attr,
	&dev_attr_2B.attr,
	&dev_attr_2C.attr,
	&dev_attr_2D.attr,
	&dev_attr_2E.attr,
	&dev_attr_2F.attr,
	&dev_attr_30.attr,
	&dev_attr_31.attr,
	&dev_attr_32.attr,
	&dev_attr_33.attr,
	&dev_attr_34.attr,
	&dev_attr_35.attr,
	&dev_attr_36.attr,
	&dev_attr_37.attr,
	&dev_attr_38.attr,
	&dev_attr_39.attr,
	&dev_attr_3A.attr,
	&dev_attr_3B.attr,
	&dev_attr_3C.attr,
	&dev_attr_3D.attr,
	&dev_attr_3E.attr,
	&dev_attr_3F.attr,
	&dev_attr_40.attr,
	&dev_attr_41.attr,
	&dev_attr_42.attr,
	&dev_attr_43.attr,
	&dev_attr_44.attr,
	&dev_attr_45.attr,
	&dev_attr_46.attr,
	&dev_attr_47.attr,
	&dev_attr_48.attr,
	&dev_attr_49.attr,
	&dev_attr_4A.attr,
	&dev_attr_4B.attr,
	&dev_attr_4C.attr,
	&dev_attr_4D.attr,
	&dev_attr_4E.attr,
	&dev_attr_4F.attr,
	&dev_attr_50.attr,
	&dev_attr_51.attr,
	&dev_attr_52.attr,
	&dev_attr_53.attr,
	&dev_attr_54.attr,
	&dev_attr_55.attr,
	&dev_attr_56.attr,
	&dev_attr_57.attr,
	&dev_attr_58.attr,
	&dev_attr_59.attr,
	&dev_attr_5A.attr,
	&dev_attr_5B.attr,
	&dev_attr_5C.attr,
	&dev_attr_5D.attr,
	&dev_attr_5E.attr,
	&dev_attr_5F.attr,
	&dev_attr_60.attr,
	&dev_attr_61.attr,
	&dev_attr_62.attr,
	&dev_attr_63.attr,
	&dev_attr_64.attr,
	&dev_attr_65.attr,
	&dev_attr_66.attr,
	0
};

static struct attribute_group tl2796_sysfs_attribute_group = {
	.name   = NULL,
	.attrs  = tl2796_sysfs_attributes,
};

static int __devinit tl2796_sysfs_probe(struct platform_device *pdev)
{
	int ret;
	ret = sysfs_create_group(&pdev->dev.kobj, &tl2796_sysfs_attribute_group);
	if (ret)
	{
		printk("%s: sysfs_create_group failed. error=%d\n", __FUNCTION__, ret);
		return ret;
	}
//	printk("%s: sysfs_create_group succeed.\n", __FUNCTION__);
	return 0;
}

//-----------------------------------------------------------------------------

static int __devexit tl2796_sysfs_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &tl2796_sysfs_attribute_group);
	return 0;
}

//=============================================================================

//-----------------------------------------------------------------------------

static struct platform_device tl2796_sysfs_device = {
	.name = DEVICE_NAME,
	.id = -1,
};

//-----------------------------------------------------------------------------

static struct platform_driver tl2796_sysfs_driver = {
	.probe = tl2796_sysfs_probe,
	.remove = tl2796_sysfs_remove,
	.driver = {
		.owner = THIS_MODULE,
		.name = DEVICE_NAME,
	},
};

//=============================================================================

//-----------------------------------------------------------------------------

static int __init tl2796_sysfs_init(void)
{
	int ret;

	// device
	ret = platform_device_register(&tl2796_sysfs_device);
	if (ret)
	{
		printk("%s: failed to add platform device %s (%d) \n", __FUNCTION__, tl2796_sysfs_device.name, ret);
		return ret;
	}
//	printk("%s: %s device registered\n", __FUNCTION__, tl2796_sysfs_device.name);

	// driver
	ret = platform_driver_register(&tl2796_sysfs_driver);
	if (ret)
	{
		printk("%s: failed to add platform driver %s (%d) \n", __FUNCTION__, tl2796_sysfs_driver.driver.name, ret);
		return ret;
	}
//	printk("%s: %s driver registered\n", __FUNCTION__, tl2796_sysfs_driver.driver.name);

	printk("[%s] driver initialized\n", DEVICE_NAME);
	return 0;
}

module_init(tl2796_sysfs_init);

//-----------------------------------------------------------------------------

static void __exit tl2796_sysfs_exit(void)
{
	// driver
	platform_driver_unregister(&tl2796_sysfs_driver);
//	printk("%s: %s driver unregistered\n", __FUNCTION__, tl2796_sysfs_driver.driver.name);

	// device
	platform_device_unregister(&tl2796_sysfs_device);
//	printk("%s: %s device unregistered\n", __FUNCTION__, tl2796_sysfs_device.name);

	printk("[%s] driver released\n", DEVICE_NAME);
}

module_exit(tl2796_sysfs_exit);

