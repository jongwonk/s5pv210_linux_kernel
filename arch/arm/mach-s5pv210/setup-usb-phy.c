/*
 * Copyright (C) 2012 Samsung Electronics Co.Ltd
 * Author: Joonyoung Shim <jy0922.shim@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundationr
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <mach/map.h>
#include <mach/regs-sys.h>
#include <plat/cpu.h>
#include <plat/regs-usb-hsotg-phy.h>
#include <plat/usb-phy.h>

static int s5pv210_usb_otgphy_init(struct platform_device *pdev)
{
	struct clk *xusbxti;

	xusbxti = clk_get(&pdev->dev, "otg");
	clk_enable(xusbxti);

	writel(readl(S5PV210_USB_PHY_CON) | S5PV210_USB_PHY1_EN, S5PV210_USB_PHY_CON);
	
	writel((readl(S3C_PHYPWR)&~(0x1<<7)&~(0x1<<6))|(0x1<<8)|(0x1<<5) , S3C_PHYPWR);
	mdelay(1);
	__raw_writel((__raw_readl(S3C_PHYCLK)&~(0x1<<7))|(0x3<<0), S3C_PHYCLK);
	__raw_writel((__raw_readl(S3C_RSTCON))|(0x1<<4)|(0x1<<3), S3C_RSTCON);
	udelay(10);
	
	__raw_writel(0x0000, S3C_RSTCON);
	//__raw_writel(__raw_readl(S3C_RSTCON) &~(0x1<<4)&~(0x1<<3), S3C_RSTCON);
		
	return 0;
}

static int s5pv210_usb_otgphy_exit(struct platform_device *pdev)
{
	writel((readl(S3C_PHYPWR) | S3C_PHYPWR_ANALOG_POWERDOWN |
				S3C_PHYPWR_OTG_DISABLE), S3C_PHYPWR);

	writel(readl(S5PV210_USB_PHY_CON) & ~S5PV210_USB_PHY1_EN,
			S5PV210_USB_PHY_CON);

	return 0;
}

int s5p_usb_phy_init(struct platform_device *pdev, int type)
{
	if (type == S5P_USB_PHY_DEVICE)
	{
	  return s5pv210_usb_otgphy_init(pdev);
	}
	
	return -EINVAL;
}

int s5p_usb_phy_exit(struct platform_device *pdev, int type)
{
	if (type == S5P_USB_PHY_DEVICE)
		return s5pv210_usb_otgphy_exit(pdev);

	return -EINVAL;
}
