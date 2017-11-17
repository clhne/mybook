/*
 *  linux/arch/arm/mach-pxa/saar.c
 *
 *  Support for the Marvell SAAR Development Platform.
 *
 *  Author:	Jason Chagas (largely modified code)
 *  Created:	Nov 20, 2006
 *  Copyright:	(C) Copyright 2006 Marvell International Ltd.
 *
 *  2007-11-22  modified to align with latest kernel
 *              eric miao <eric.miao@marvell.com>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License version 2 as
 *  publishhed by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/clk.h>
#include <linux/mtd/partitions.h>
#include <mtd/mtd-abi.h>
#include <linux/irq.h>
#include <linux/i2c.h>
#include <linux/i2c/si4703.h>

#include <asm/types.h>
#include <asm/setup.h>
#include <asm/memory.h>
#include <asm/mach-types.h>
#include <asm/hardware.h>
#include <asm/irq.h>

#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/mach/irq.h>
#include <asm/mach/flash.h>
#include <asm/gpio.h>
#include <asm/arch/pxa-regs.h>
#include <asm/arch/mfp-pxa3xx.h>
#include <asm/arch/mfp-pxa9xx.h>
#include <asm/arch/pxa3xx_nand.h>
#include <asm/arch/pxa3xx_keypad.h>
#include <asm/arch/pxa3xx_pmic.h>
#include <asm/arch/gpio.h>
#include <asm/arch/micco.h>
#include <asm/arch/micco-vibrator.h>
#include <asm/arch/irda.h>
#include <asm/arch/pxa3xx_u2d.h>
#include <asm/arch/pxa9xx_u2o.h>
#include <asm/arch/pxa9xx_u2h.h>
#include <asm/arch/ehci.h>
#include <asm/arch/udc.h>
#include <asm/arch/uart.h>
#include <asm/arch/mmc.h>
#include <asm/arch/pxa3xx_dvfm.h>
#include <asm/arch/imm.h>
#include <asm/arch/pxafb.h>
#include <asm/arch/pxa3xx_pm.h>
#include <asm/arch/pxa930_trkball.h>
#include <asm/arch/camera.h>
#include <asm/arch/pmu.h>
#include <asm/arch/part_table.h>
#include <asm/arch/pxa3xx-regs.h>
#include <asm/arch/i2c.h>
#include <asm/arch/leadcore_spi.h>
#include <asm/arch/sirf.h>
#include <asm/arch/innodev.h>
#ifdef CONFIG_BOOT_INFO
#include <asm/bootinfo.h>
#endif
#ifdef CONFIG_ANDROID_PMEM
#include <linux/android_pmem.h>
#include <asm/arch/pmem.h>
#endif
#include "devices.h"
#include "generic.h"

#include <asm/arch/lc6830.h>
#include <asm/arch/smb380.h>
#include <linux/mmc/host.h>

#define MAX_SLOTS      3
struct platform_mmc_slot saar_mmc_slot[MAX_SLOTS];
extern int is_android(void);
extern int is_comm_v75(void);
extern int leadcore_platform_add_devices(void);
extern int leadcore_i2c_register_board_info(void);

#define ARRAY_AND_SIZE(x)	(x), ARRAY_SIZE(x)

#ifdef CONFIG_MACH_LC6830_PIN_MUX
/* SAAR MFP configurations */
static mfp_cfg_t saar_mfp_cfg[] __initdata = {
/* Added by Leadcore Songlixin on: Mon Dec 21 21:41:36 PST 2009 BEGIN*/
/*misc, to save power*/
#if 1
	RF_IF5_GPIO,
#endif
/* Added by Leadcore Songlixin on: Mon Dec 21 21:41:36 PST 2009 END*/
	/* DFC */
	DF_INT_RnB_ND_INT_RnB,
	DF_nRE_nOE_ND_nRE,
	DF_nWE_ND_nWE,
	DF_CLE_nOE_ND_CLE,
	DF_nADV1_ALE_ND_ALE,
	DF_nADV2_ALE_nCS3,
	DF_nCS0_ND_nCS0,
	DF_IO0_ND_IO0,
	DF_IO1_ND_IO1,
	DF_IO2_ND_IO2,
	DF_IO3_ND_IO3,
	DF_IO4_ND_IO4,
	DF_IO5_ND_IO5,
	DF_IO6_ND_IO6,
	DF_IO7_ND_IO7,
	DF_IO8_ND_IO8,
	DF_IO9_ND_IO9,
	DF_IO10_ND_IO10,
	DF_IO11_ND_IO11,
	DF_IO12_ND_IO12,
	DF_IO13_ND_IO13,
	DF_IO14_ND_IO14,
	DF_IO15_ND_IO15,

	/* FFUART */
	GPIO63_FFTXD,
	GPIO64_FFRXD,

	/* BT UART */
	GPIO91_BTRXD,
	GPIO92_BTTXD,
	GPIO95_BTCTS_N,
	GPIO96_BTRTS_N,

	/* STUART */
	GPIO43_STRTS_N,
	GPIO44_STCTS_N,
	GPIO77_STRXD,
	GPIO78_STTXD,

	/* AP_BAT_UARTOE */
	GPIO16_GPIO,

	/* Keypad */
/* Modified by Leadcore Songlixin on: Thu Mar  4 09:19:06 CST 2010 BEGIN*/
#if	CONFIG_MACH_LC6830_BD_VERSION >= 21
	GPIO12_KP_MKIN_6,
	GPIO13_KP_MKOUT_6,
	GPIO14_KP_MKIN_7,
	GPIO10_KP_MKIN_5,
	GPIO15_KP_MKOUT_7,
#else
#if CONFIG_MACH_LC6830_BD_VERSION == 13
	GPIO42_GPIO,
	GPIO21_GPIO,
#endif
	GPIO12_KP_MKIN_6,
	GPIO13_KP_MKOUT_6,
	GPIO14_KP_MKIN_7,
#endif
/* Modified by Leadcore Songlixin on: Thu Mar  4 09:19:06 CST 2010 END */

	/* Keypad backlight */
	GPIO41_GPIO,

	/* i2c bus */
	GPIO89_CI2C_SCL,
	GPIO90_CI2C_SDA,

	/* Micco interrupt */
	PMIC_INT_GPIO_83,
	/* avoid GPIO83 confliction */
	GPIO83_SSP3_RXD,

/* Modified by Leadcore Songlixin on: Tue Mar 23 13:52:09 CST 2010 BEGIN*/
/* according to schematic*/
#if 1
	/* SSP3*/
	GPIO79_SSP3_CLK,
	GPIO80_SSP3_FRM,
	GPIO81_SSP3_TXD,
	GPIO82_SSP3_RXD,
#else
	/* GSSP1 */
	GPIO79_GSSP1_CLK,
	GPIO80_GSSP1_FRM,
	GPIO81_GSSP1_TXD,
	GPIO82_GSSP1_RXD,
#endif
/* Modified by Leadcore Songlixin on: Tue Mar 23 13:52:09 CST 2010 END */

	/* SSP2 */
	GPIO85_SSP2_BITCLK,
	GPIO87_SSP2_SYNC,
	GPIO88_SSP2_DATA_OUT,

	/* QCI */
	GPIO65_CI_DD_7,
	GPIO66_CI_DD_6,
	GPIO67_CI_DD_5,
	GPIO68_CI_DD_4,
	GPIO69_CI_DD_3,
	GPIO70_CI_DD_2,
	GPIO71_CI_DD_1,
	GPIO72_CI_DD_0,
	GPIO75_CI_MCLK,
	GPIO76_CI_PCLK,

#if CONFIG_MACH_LC6830_BD_VERSION >= 21
	GPIO164_STUART_PW_EN,
#endif

/* Added by Leadcore Songlixin on: Thu Jan 14 23:37:59 PST 2010 BEGIN*/
/* 0.3M & 3M Sensor control */
	GPIO7_CAM_1P8V_PIN,
#if CONFIG_MACH_LC6830_BD_VERSION >= 21
	GPIO42_CAM_1P2V_PIN,
	GPIO109_QCI_HI_PWDN_PIN,
	GPIO110_QCI_HI_RESET_PIN,
	GPIO61_QCI_LO_PWDN_PIN,
	GPIO107_QCI_HI_SCL_PIN,
	GPIO108_QCI_HI_SDA_PIN,
	GPIO111_QCI_LO_SCL_PIN,
	GPIO115_QCI_LO_SDA_PIN,
#else
#if CONFIG_MACH_LC6830_BD_VERSION < 21
	GPIO164_QCI_HI_PWDN_PIN,
#endif

	GPIO167_QCI_HI_RESET_PIN,
	GPIO166_QCI_LO_PWDN_PIN,
	GPIO165_QCI_LO_RESET_PIN,
#endif
/* Added by Leadcore Songlixin on: Thu Jan 14 23:38:00 PST 2010 END*/

	/* MMC1 */
	GPIO55_MMC1_CMD,
	GPIO56_MMC1_CLK,
	GPIO57_MMC1_DAT0,
	GPIO58_MMC1_DAT1,
	GPIO59_MMC1_DAT2,
	GPIO60_MMC1_DAT3,

/* Modified by Leadcore Songlixin on: Tue Feb  9 16:25:01 CST 2010 BEGIN*/
/* For Board_2, we has no gpio to detect tflash, it's internal*/
#if	CONFIG_MACH_LC6830_BD_VERSION < 20
	GPIO61_GPIO,		/* card dectect */
#endif
/* Modified by Leadcore Songlixin on: Tue Feb  9 16:25:01 CST 2010 END */

	/*AUDIO PA ENABLE */
	GPIO122_GPIO,

	/* MMC2 */
	GPIO101_MMC2_DAT3,
	GPIO102_MMC2_DAT2,
	GPIO103_MMC2_DAT1,
	GPIO104_MMC2_DAT0,
	GPIO105_MMC2_CMD,
	GPIO106_MMC2_CLK,

	/* LCD */
	GPIO23_LCD_DD0,
	GPIO24_LCD_DD1,
	GPIO25_LCD_DD2,
	GPIO26_LCD_DD3,
	GPIO27_LCD_DD4,
	GPIO28_LCD_DD5,
	GPIO29_LCD_DD6,
	GPIO30_LCD_DD7,
	GPIO31_LCD_DD8,
	GPIO32_LCD_DD9,
	GPIO33_LCD_DD10,
	GPIO34_LCD_DD11,
	GPIO35_LCD_DD12,
	GPIO36_LCD_DD13,
	GPIO37_LCD_DD14,
	GPIO38_LCD_DD15,
	GPIO39_LCD_DD16,
	GPIO40_LCD_DD17,
	GPIO17_LCD_FCLK_RD,
	GPIO18_LCD_LCLK_A0,
	GPIO19_LCD_PCLK_WR,
	GPIO20_LCD_BIAS,
	GPIO22_GPIO,		/* LCD reset */
	GPIO45_SSP4_TXD,
	GPIO46_SSP4_RXD,
	GPIO93_SSP4_CLK,
	GPIO94_SSP4_FRM,

/*USB Host controller & Modem Interface add by gyb begin*/
	GPIO50_USB_P4_SPEED_GPIO_OFF,	/*USB speed choose */
	GPIO84_USB_P4_SUSPEND_GPIO_OFF,	/* USB Host Transceiver SUSPEND */
	GPIO175_USB_P4_POWER_GPIO_OFF,	/* USB HOST-ONLY Transceiver Power */
	GPIO86_USB_P4_SOFTCON_GPIO_OFF,	/* USB Host Transceiver SOFTCON */

	GPIO112_BAT_DL_MODE_GPIO,		/* BAT DL MODE */
	GPIO113_BAT2AP_PWR_IND_GPIO,	/* BAT2AP PWR IND */
	GPIO114_AP2BAT_WAEKUP_GPIO,		/* AP2BAT WAKEUP */

#if CONFIG_MACH_LC6830_BD_VERSION >= 20
	GPIO62_BAT2AP_WAEKUP_GPIO,		/* BAT2AP_WAKEUP */
#else
	GPIO115_BAT2AP_WAEKUP_GPIO,		
#endif
	GPIO116_BAT2AP_WAEKUP_GPIO,		/* BAT2AP_SLEEP_IN */
	GPIO117_AP2BAT_SLEEP_GPIO,		/* AP2BAT_SLEEP_IND */
	GPIO118_AP2BAT_RESET_GPIO,		/* AP2BAT RESET */
	GPIO119_AP2BAT_PWR_ON_GPIO,		/* AP2BAT PWR ON */
	GPIO120_AP2BAT_COM_SEL1_GPIO,	/* AP2BAT COM SEL */
	GPIO121_AP2BAT_COM_SEL2_GPIO,	/* AP2BAT COM SEL */
/*USB Host controller & Modem Interface add by gyb end*/

/* Added by Leadcore Songlixin on: Tue Mar 23 13:19:19 CST 2010 BEGIN*/
/*For BT/WiFI, when they are not workign*/
#if 1
	GPIO174_BT_WIFI_POWER_GPIO,
	GPIO170_WLAN_A2W_WAKE_GPIO,
	GPIO172_WLAN_W2A_WAKE_GPIO,
	GPIO171_WLAN_RESET_GPIO,
	GPIO97_BT_A2B_WAKE_GPIO,
	GPIO99_BT_B2A_WAKE_GPIO,
	GPIO98_BT_RESET_GPIO,
#endif
/* Added by Leadcore Songlixin on: Tue Mar 23 13:19:19 CST 2010 END*/

};

static mfp_cfg_t wlan_bt_powerdown_high_mfp_cfg[] = {
	GPIO174_BT_WIFI_POWER_GPIO_ON,
};
static mfp_cfg_t wlan_bt_powerdown_low_mfp_cfg[] = {
	GPIO174_BT_WIFI_POWER_GPIO,
};

static mfp_cfg_t wlan_reset_high_mfp_cfg[] = {
	GPIO171_WLAN_RESET_GPIO_ON,
};
static mfp_cfg_t wlan_reset_low_mfp_cfg[] = {
	GPIO171_WLAN_RESET_GPIO,
};

static mfp_cfg_t bt_reset_high_mfp_cfg[] = {
	GPIO98_BT_RESET_GPIO_ON,
};
static mfp_cfg_t bt_reset_low_mfp_cfg[] = {
	GPIO98_BT_RESET_GPIO,
};
static mfp_cfg_t audio_pa_high_mfp_cfg[] = {
	GPIO122_AUDIO_DRIVE_HIGH,
};
static mfp_cfg_t audio_pa_low_mfp_cfg[] = {
	GPIO122_AUDIO_DRIVE_LOW,
};

#if CONFIG_MACH_LC6830_BD_VERSION >= 20
static mfp_cfg_t camif_on_mfp_cfg[] = {
		GPIO65_CI_DD_7,
		GPIO66_CI_DD_6,
		GPIO67_CI_DD_5,
		GPIO68_CI_DD_4,
		GPIO69_CI_DD_3,
		GPIO70_CI_DD_2,
		GPIO71_CI_DD_1,
		GPIO72_CI_DD_0,
};

static mfp_cfg_t camif_off_mfp_cfg[] = {
		GPIO65_GPIO,
		GPIO66_GPIO,
		GPIO67_GPIO,
		GPIO68_GPIO,
		GPIO69_GPIO,
		GPIO70_GPIO,
		GPIO71_GPIO,
		GPIO72_GPIO,
};
#endif
#endif

#if defined(CONFIG_MOUSE_PXA930_TRKBALL) || \
	defined(CONFIG_MOUSE_PXA930_TRKBALL_MODULE)
static struct pxa930_trkball_platform_data saar_mouse_info = {
	.x_filter = 2,
	.y_filter = 2,
};

static void __init saar_init_trackball(void)
{
	pxa_register_device(&pxa930_device_trkball, &saar_mouse_info);
}
#else
static inline void saar_init_trackball(void)
{
}
#endif

#ifdef CONFIG_BACKLIGHT_BD6091GU
static struct platform_device bd6091gu_bl_device = {
	.name = "bd6091gu-bl",
	.id = -1,
};
#endif

static struct platform_device micco_bl_device = {
	.name = "micco-bl",
	.id = -1,
};

static struct platform_device micco_kp_bl_device = {
	.name = "micco-kp-bl",
	.id = -1,
};

static struct platform_device micco_ts_device = {
	.name = "micco-ts",
	.id = -1,
};

static struct platform_device micco_headset_detect_device = {
	.name = "micco-headset",
	.id = -1,
};

static struct micco_vibrator_platform_data lc6830_vibrator_data = {
	.vib_strength = (0x55 << 1),
};

static struct platform_device micco_vibrator_device = {
	.name = "micco-vibrator",
	.id = -1,
	.dev = {
		.platform_data = &lc6830_vibrator_data,
		},
};

static struct resource pxa3xx_resource_imm[] = {
	[0] = {
	       .name = "phy_sram",
	       .start = PHYS_SRAM_START + 0x10000,	/* Moved up to offset 64KB due to collision with DVFM
							 * D2 entry code +
							 * The first 4KB are locked for pxa930 trusted */
	       .end = PHYS_SRAM_START + PHYS_SRAM_BANK_SIZE - 1,
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .name = "imm_sram",
	       .start = PHYS_SRAM_START + 0x10000,
	       .end = PHYS_SRAM_START + PHYS_SRAM_BANK_SIZE - 1,
	       .flags = IORESOURCE_MEM,
	       },
};

static struct resource pxa935_resource_imm[] = {
	[0] = {
	       .name = "phy_sram",
	       .start = PHYS_SRAM_START + 0x10000,	/* Moved up to offset 64KB due to collision with DVFM
							 * D2 entry code */
	       .end = PHYS_SRAM_START + 2 * PHYS_SRAM_BANK_SIZE - 1,	/* 2 banks on pxa935 */
	       .flags = IORESOURCE_MEM,
	       },
	[1] = {
	       .name = "imm_sram",
	       .start = PHYS_SRAM_START + 0x10000,
	       .end = PHYS_SRAM_START + 2 * PHYS_SRAM_BANK_SIZE - 1,
	       .flags = IORESOURCE_MEM,
	       },
};

static struct platform_device pxa3xx_device_imm = {
	.name = "pxa3xx-imm",
	.id = -1,
	.num_resources = ARRAY_SIZE(pxa3xx_resource_imm),
	.resource = pxa3xx_resource_imm,
};

static struct platform_device micco_charger_device = {
	.name = "micco-charger",
	.id = -1,
};

static struct platform_device pxa_device_battery = {
	.name = "battery",
	.id = -1,
};

static struct platform_device ispt_device = {
	.name = "pxa-ispt",
	.id = -1,
};

#ifdef CONFIG_ANDROID_PMEM
static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.start = 0xc7000000,
	.size = 0x1000000,
	.no_allocator = 0,
	.cached = 0,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 1,
	.dev = {.platform_data = &android_pmem_pdata},
};
#endif

/* liuchangmin 2009-11-24 led-gpio control driver enable, Begin */
#ifdef CONFIG_LEDS_GPIO
static struct gpio_led lc6830_led = {
	.name = "button-backlight",
	.gpio = 41,
	.default_trigger = "timer",
};

static struct gpio_led_platform_data lc6830_led_data = {
	.num_leds = 1,
	.leds = &lc6830_led,
};

static struct platform_device lc6830_led_dev = {
	.name = "leds-gpio",
	.id = 0,
	.dev = {.platform_data = &lc6830_led_data},
};
#endif
/* liuchangmin 2009-11-24 led-gpio control driver enable, End */

static struct platform_device *devices[] __initdata = {
#ifdef CONFIG_BACKLIGHT_BD6091GU
	&bd6091gu_bl_device,
#endif
	&micco_bl_device,
	&micco_kp_bl_device,
	&micco_ts_device,
	&pxa3xx_device_imm,
	&micco_vibrator_device,
	&micco_charger_device,
	&pxa_device_battery,
	&micco_headset_detect_device,
	&ispt_device,
#ifdef CONFIG_ANDROID_PMEM
	&android_pmem_device,
#endif
#ifdef CONFIG_LEDS_GPIO
	&lc6830_led_dev,
#endif
};

#if defined(CONFIG_KEYBOARD_PXA3xx) || defined(CONFIG_KEYBOARD_PXA3xx_MODULE)
static unsigned int saar_matrix_key_map[] = {
	KEY(6, 6, KEY_VOLUMEDOWN),  /* VOLUMEDOWN */
	KEY(7, 6, KEY_VOLUMEUP),    /* VOLUMEUP */
#if CONFIG_MACH_LC6830_BD_VERSION >= 20
	KEY(5, 7, KEY_HOME),    /* HOME */
	KEY(6, 7, KEY_BACK),    /* BACK */
	KEY(7, 7, KEY_MENU),    /* MENU */
#endif
};

static struct pxa3xx_keypad_platform_data saar_keypad_info = {
	.matrix_key_rows = 8,
#if CONFIG_MACH_LC6830_BD_VERSION >= 20
	.matrix_key_cols = 8,
#else
	.matrix_key_cols = 7,
#endif
	.matrix_key_map = saar_matrix_key_map,
	.matrix_key_map_size = ARRAY_SIZE(saar_matrix_key_map),
	.debounce_interval = 30,
};
#endif /* CONFIG_KEYBOARD_PXA3xx || CONFIG_KEYBOARD_PXA3xx_MODULE */

#if defined(CONFIG_MMC) || defined(CONFIG_MMC_MODULE)
static int saar_mci_init(struct device *dev,
			 irq_handler_t saar_detect_int, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	int err, cd_irq, gpio_cd;

	cd_irq = gpio_to_irq(saar_mmc_slot[pdev->id].gpio_cd);
	gpio_cd = saar_mmc_slot[pdev->id].gpio_cd;

	/*
	 * setup GPIO for saar MMC controller
	 */
	err = gpio_request(gpio_cd, "mmc card detect");
	if (err)
		goto err_request_cd;
	gpio_direction_input(gpio_cd);

	err = request_irq(cd_irq, saar_detect_int,
			  IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
			  "MMC card detect", data);
	if (err) {
		printk(KERN_ERR "%s: MMC/SD/SDIO: "
		       "can't request card detect IRQ\n", __func__);
		goto err_request_irq;
	}

	return 0;

      err_request_irq:
	gpio_free(gpio_cd);
      err_request_cd:
	return err;
}

static void saar_mci_exit(struct device *dev, void *data)
{
	struct platform_device *pdev = to_platform_device(dev);
	int cd_irq, gpio_cd;

	cd_irq = gpio_to_irq(saar_mmc_slot[pdev->id].gpio_cd);
	gpio_cd = saar_mmc_slot[pdev->id].gpio_cd;

	free_irq(cd_irq, data);
	gpio_free(gpio_cd);
}

static struct pxamci_platform_data saar_mci_platform_data = {
	.detect_delay = 20,
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
/* Modified by Leadcore Songlixin on: Tue Feb  9 16:30:58 CST 2010 BEGIN*/
/* For Board_2, we has no gpio to detect tflash, it's internal*/
#if CONFIG_MACH_LC6830_BD_VERSION < 20
	.init = saar_mci_init,
	.exit = saar_mci_exit,
#endif
/* Modified by Leadcore Songlixin on: Tue Feb  9 16:30:58 CST 2010 END */
};

static struct pxamci_platform_data saar_mci2_platform_data = {
	.detect_delay = 20,
	.ocr_mask = MMC_VDD_32_33 | MMC_VDD_33_34,
};

static void __init saar_init_mmc(void)
{
	pxa_set_mci_info(&saar_mci_platform_data);
	pxa3xx_set_mci2_info(&saar_mci2_platform_data);
/* Modified by Leadcore Songlixin on: Tue Feb  9 16:30:58 CST 2010 BEGIN*/
/* For Board_2, we has no gpio to detect tflash, it's internal*/
#if CONFIG_MACH_LC6830_BD_VERSION < 20
	saar_mmc_slot[0].gpio_cd = mfp_to_gpio(MFP_PIN_GPIO61);
#endif
/* Modified by Leadcore Songlixin on: Tue Feb  9 16:30:58 CST 2010 END */
}
#else
static inline void saar_init_mmc(void)
{
}
#endif

#if defined(CONFIG_MTD_ONENAND) || defined(CONFIG_MTD_ONENAND_MODULE)

static struct flash_platform_data saar_onenand_info;
static void __init saar_init_onenand(void)
{
	if (is_android()) {
		if (is_comm_v75()) {
			saar_onenand_info.parts = android_256m_v75_partitions;
			saar_onenand_info.nr_parts =
			    ARRAY_SIZE(android_256m_v75_partitions);
		} else {
			saar_onenand_info.parts = android_256m_partitions;
			saar_onenand_info.nr_parts =
			    ARRAY_SIZE(android_256m_partitions);
		}
	} else {
		if (is_comm_v75()) {
			saar_onenand_info.parts = pxa930_256m_v75_partitions;
			saar_onenand_info.nr_parts =
			    ARRAY_SIZE(pxa930_256m_v75_partitions);
		} else {
			saar_onenand_info.parts = pxa930_256m_partitions;
			saar_onenand_info.nr_parts =
			    ARRAY_SIZE(pxa930_256m_partitions);
		}
	}

	pxa3xx_device_onenand.dev.platform_data = &saar_onenand_info;
	platform_device_register(&pxa3xx_device_onenand);
}
#else
static inline void saar_init_onenand(void)
{
}
#endif

#if defined(CONFIG_MTD_NAND_PXA3xx) || defined(CONFIG_MTD_NAND_PXA3xx_MODULE)
static struct pxa3xx_nand_platform_data saar_nand_info;
static void __init saar_init_nand(void)
{
	saar_nand_info.parts = android_512m_partitions;
	saar_nand_info.nr_parts = ARRAY_SIZE(android_512m_partitions);

	pxa3xx_device_nand.dev.platform_data = &saar_nand_info;
	platform_device_register(&pxa3xx_device_nand);
}
#else
static inline void saar_init_nand(void)
{
}
#endif /* CONFIG_MTD_NAND_PXA3xx || CONFIG_MTD_NAND_PXA3xx_MODULE */

#if defined(CONFIG_USB_PXA27X_UDC) || defined(CONFIG_USB_PXA27X_UDC_MODULE)
static mfp_cfg_t pxa930_otg_init_pins[] = {
	GPIO64_GPIO,
};

static mfp_cfg_t pxa930_otg_pins[] = {
	GPIO64_USB_P2_7,
};

int saar_udc_is_miniA(void)
{
	int otg_id = mfp_to_gpio(MFP_PIN_GPIO64);
	int id_value;
	int err;

	pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa930_otg_init_pins));
	err = gpio_request(otg_id, "OTG ID");
	if (err) {
		gpio_free(otg_id);
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d return :%d\n", otg_id, err);
		return 0;
	}
	gpio_direction_input(otg_id);
	id_value = gpio_get_value(otg_id);
	gpio_free(otg_id);
	pxa3xx_mfp_config(ARRAY_AND_SIZE(pxa930_otg_pins));

	return (id_value == 0);
}

static struct pxa2xx_udc_mach_info saar_udc_info = {
	.udc_is_miniA = saar_udc_is_miniA,
};
#endif

#if defined(CONFIG_USB_PXA3XX_U2D) || defined(CONFIG_USB_USB_PXA3XX_U2D_MODULE)
#ifdef CONFIG_MACH_LC6830_PIN_MUX
static mfp_cfg_t saar_u2d_cfg[] __initdata = {
	/* ULPI */
	GPIO43_GPIO,
	GPIO31_USB_ULPI_D0,
	GPIO30_USB_ULPI_D1,
	GPIO33_USB_ULPI_D2,
	GPIO34_USB_ULPI_D3,
	GPIO35_USB_ULPI_D4,
	GPIO36_USB_ULPI_D5,
	GPIO41_USB_ULPI_D6,
#if CONFIG_MACH_LC6830_BD_VERSION >= 13
	GPIO42_USB_ULPI_D7, 	//gpio42 is used for 5M camera 1v2
#endif
	GPIO37_USB_ULPI_DIR,
	GPIO38_USB_ULPI_CLK,
	GPIO39_USB_ULPI_STP,
	GPIO40_USB_ULPI_NXT,
	DF_ADDR3_CLK26MOUTDMD,
};
#endif

void saar_reset_xcvr(void)
{
	int reset_pin;
	int err;

	reset_pin = mfp_to_gpio(MFP_PIN_GPIO43);
	err = gpio_request(reset_pin, "U2D Reset");
	if (err) {
		gpio_free(reset_pin);
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d return :%d\n", reset_pin, err);
		return;
	}
	gpio_direction_output(reset_pin, 0);
	mdelay(100);
	gpio_set_value(reset_pin, 1);

	gpio_free(reset_pin);
}

static struct pxa3xx_u2d_mach_info saar_u2d_info = {
	.reset_xcvr = saar_reset_xcvr,
};
#endif /* CONFIG_USB_PXA3XX_U2D || CONFIG_USB_PXA3XX_U2D_MODULE */

#if defined(CONFIG_USB_PXA9XX_U2O) || defined(CONFIG_USB_PXA9XX_U2O_MODULE)
extern void pxa9xx_u2o_xcvr_init(unsigned *base);
extern struct resource pxa9xx_u2o_resources[];
static struct pxa9xx_u2o_mach_info saar_u2o_info = {
	.xcvr_init = pxa9xx_u2o_xcvr_init,
};
#endif

#ifdef CONFIG_USB_EHCI_PXA935_HOSTONLY
//extern void pxa9xx_u2h_xcvr_init(unsigned *base);
extern struct resource pxa9xx_u2h_resources[];
static struct pxa9xx_u2h_mach_info saar_u2h_info = {
	.xcvr_init = NULL,
};
#endif

#ifdef CONFIG_USB_EHCI_PXA9XX
static void __init saar_init_ehci(void)
{
	if (!device_is_enabled(device_u2o))
		return;

	pxa_set_ehci_info(&saar_u2o_info);
}
#elif defined(CONFIG_USB_EHCI_PXA935_HOSTONLY)
static void __init saar_init_ehci(void)
{
	pxa_set_ehci_info(&saar_u2h_info);
}
#else
static inline void saar_init_ehci(void)
{
}
#endif

#if defined (CONFIG_USB_OTG) && defined(CONFIG_USB_PXA9XX_U2O)
static void __init saar_init_otg(void)
{
	if (!device_is_enabled(device_u2o))
		return;

	pxa_set_u2otg_info(&saar_u2o_info);
}
#else
static inline void saar_init_otg(void)
{
}
#endif

#if defined(CONFIG_PXA3xx_MICCO) || defined(CONFIG_PXA3xx_MICCO_MODULE)
static int micco_init_irq(void)
{
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO83));

	return 0;
}

static int micco_ack_irq(void)
{
	return 0;
}

static void saar_micco_init(void)
{
	u8 value;

	/* Mask interrupts that are not needed */
	micco_write(MICCO_IRQ_MASK_A, 0xFE);
	micco_write(MICCO_IRQ_MASK_B, 0xFF);
	micco_write(MICCO_IRQ_MASK_C, 0xFF);
	micco_write(MICCO_IRQ_MASK_D, 0xFF);

/* Modified by Leadcore Songlixin on: Tue Dec 15 23:18:47 PST 2009 BEGIN*/
/* Only enable needed ldos to save power*/
#if 1
	micco_write(MICCO_OVER1, 0x5);
	micco_write(MICCO_APP_OVER2, 0x20);
	micco_write(MICCO_APP_OVER3, 0xb9);
	micco_write(0x46, 0x0);	// disable OUT_32k
#else
	/* avoid SRAM power off during sleep */
	micco_write(0x10, 0x07);
	micco_write(0x11, 0xff);
	micco_write(0x12, 0xbf);	/*never enable LDO4: CP controls it via COMM_OVER3 reg (0x22) */
#endif
/* Modified by Leadcore Songlixin on: Tue Dec 15 23:18:47 PST 2009 END */

	/* Enable the ONKEY power down functionality */
//	micco_write(MICCO_SYSCTRL_B, 0x20);
	micco_write(MICCO_SYSCTRL_B, 0x00); // 20100511 by jhj
	micco_write(MICCO_SYSCTRL_A, 0x60);

	/* IRQ is masked during the power-up sequence and will not be released
	 * until they have been read for the first time */
	micco_read(MICCO_EVENT_A, &value);
	micco_read(MICCO_EVENT_B, &value);
	micco_read(MICCO_EVENT_C, &value);
	micco_read(MICCO_EVENT_D, &value);
}

/* micco_power_module[] should be consistent with enum
 * in include/asm-arm/arch-pxa/pxa3xx_pmic.h */
static struct power_supply_module miccoB0_power_modules[] = {
	/* {command,            power_module}, */
	{VCC_CORE, BUCK1},
	{VCC_SRAM, BUCK2},
	{VCC_MVT, LDO1},
	{VCC_3V_APPS, LDO3},
	{VCC_SDIO, LDO13},
	{VCC_CAMERA_ANA, LDO12},
	{VCC_USB, LDO5},
	{VCC_LCD, LDO3},
	{VCC_TSI, 0},
	{VCC_CAMERA_IO, LDO7},
	{VCC_1P8V, LDO4},
	{VCC_MEM, BUCK3},
	{HDMI_TX, 0},
	{TECH_3V, 0},
	{TECH_1P8V, 0},
};

/* micco_power_module[] should be consistent with enum
 * in include/asm-arm/arch-pxa/pxa3xx_pmic.h */
static struct power_supply_module miccoEA_power_modules[] = {
	/* {command,            power_module}, */
	{VCC_CORE, BUCK1},
	{VCC_SRAM, BUCK2},
	{VCC_MVT, LDO1},
	{VCC_3V_APPS, LDO3},
	{VCC_SDIO, LDO13},
	{VCC_CAMERA_ANA, LDO12},
	{VCC_USB, LDO5},
	{VCC_LCD, LDO3},
	{VCC_TSI, 0},
	{VCC_CAMERA_IO, LDO7},
	{VCC_1P8V, LDO4},
	{VCC_MEM, LDO2},
	{HDMI_TX, 0},
	{TECH_3V, 0},
	{TECH_1P8V, 0},
};

static struct power_chip micco_chips[] = {
	{MICCO_B0_ID, "miccoB0", miccoB0_power_modules},
	{MICCO_EA_ID, "miccoEA", miccoEA_power_modules},
	{MICCO_EB_ID, "miccoEB", miccoEA_power_modules},
	{0, NULL, NULL},
};

static struct micco_platform_data micco_data = {
	.init_irq = micco_init_irq,
	.ack_irq = micco_ack_irq,
	.platform_init = saar_micco_init,
	.power_chips = micco_chips,
};
#endif /* CONFIG_PXA3xx_MICCO || CONFIG_PXA3xx_MICCO_MODULE */

#if defined(CONFIG_PXA_IRDA) || defined(CONFIG_PXA_IRDA_MODULE)
static void saar_irda_transceiver_mode(struct device *dev, int mode)
{
	unsigned long flags;
	int err;
	static int irda_mfp_init;
	int gpio_ir_shdn = ((struct pxaficp_platform_data *)
			    dev->platform_data)->gpio_ir_shdn;

	if (!irda_mfp_init) {
		err = gpio_request(gpio_ir_shdn, "IRDA SHDN");
		if (err) {
			gpio_free(gpio_ir_shdn);
			printk(KERN_ERR "Request GPIO failed,"
			       "gpio: %d return :%d\n", gpio_ir_shdn, err);
			return;
		}
		gpio_direction_output(gpio_ir_shdn, 1);

		irda_mfp_init = 1;
	}

	local_irq_save(flags);
	if (mode & IR_SIRMODE) {
		gpio_set_value(gpio_ir_shdn, 0);
	} else if (mode & IR_FIRMODE) {
		/* do not support FIR */
	}
	if (mode & IR_OFF) {
		gpio_set_value(gpio_ir_shdn, 1);
	}
	local_irq_restore(flags);
}

static struct pxaficp_platform_data saar_ficp_platform_data = {
	.transceiver_cap = IR_SIRMODE | IR_OFF,
	.transceiver_mode = saar_irda_transceiver_mode,
};
#endif /* (CONFIG_PXA_IRDA) || (CONFIG_PXA_IRDA_MODULE) */

#if defined(CONFIG_PXA_CAMERA)
/*liuchangmin 2009-9-30 modified for lc6830. BEGIN*/
#define SPS_DEBUG
#undef  SPS_DEBUG

#ifdef SPS_DEBUG
#define SPS_TRACE(fmt, args...)	printk("sps-dbg:%s " fmt, __FUNCTION__, ## args)
#else
#define SPS_TRACE(fmt, args...)
#endif /* SPS_DEBUG */

/* sensor init */
static int sensor_power_on(int flag)
{
	int status = 0;
	unsigned char val;
	unsigned char val2;

	/*
	 * flag, 0, low resolution
	 * flag, 1, high resolution
	 */
	SPS_TRACE("ENTER flag is %d\n", flag);
	//fang
#if CONFIG_MACH_LC6830_BD_VERSION >= 20
	pxa3xx_mfp_config(ARRAY_AND_SIZE(camif_on_mfp_cfg));
#endif

	/*liuchangmin@leadcoretech.com, add for LDO EN check when set sensor power, Begin
	 *
	 *	enable sensor power
	 *	2.8V,1.8V,1.2V
	 */
	if (gpio_request(CAM_1P8V_PIN, "CAM_1P8V_PIN")) {
		printk(KERN_ERR "Request GPIO failed,"
						"gpio: %d \n", CAM_1P8V_PIN);
		return -EIO;
	}
#if CONFIG_MACH_LC6830_BD_VERSION >= 21
	if (gpio_request(CAM_1P2V_PIN, "CAM_1P2V_PIN")) {
		printk(KERN_ERR "Request GPIO failed,"
						"gpio: %d \n", CAM_1P2V_PIN);
		return -EIO;
	}
#endif
	status = micco_read(MICCO_APP_OVER2, &val);
	val2 = val;
	if (flag == SENSOR_HIGH) {
#if CONFIG_MACH_LC6830_BD_VERSION >= 20
		gpio_direction_output(CAM_1P8V_PIN, 1);

		if (!(val & MICCO_LDO9_ENABLE)) 
			val2 |= MICCO_LDO9_ENABLE;
#endif
#if CONFIG_MACH_LC6830_BD_VERSION >= 21
	gpio_direction_output(CAM_1P2V_PIN, 1);
#endif
		if (!(val & MICCO_LDO7_ENABLE)) 
			val2 |= MICCO_LDO7_ENABLE;
		
		if (!(val & MICCO_LDO12_ENABLE)) 
			val2 |= MICCO_LDO12_ENABLE;
	} else {
#if CONFIG_MACH_LC6830_BD_VERSION == 20
		gpio_direction_output(CAM_1P8V_PIN, 1);
#endif
		if (!(val & MICCO_LDO10_ENABLE)) 
			val2 |= MICCO_LDO10_ENABLE;
	}

	if (val2 != val) {
		status = micco_write(MICCO_APP_OVER2, val2);
	}

#if CONFIG_MACH_LC6830_BD_VERSION < 20
	gpio_direction_output(CAM_1P8V_PIN, 1);
#endif
	
	/*
	 * enable power down to choose camera
	 * then reset camera
	 */

	if (gpio_request(QCI_HI_PWDN_PIN, "CAM_EANBLE_HI_SENSOR")) {
		gpio_free(QCI_HI_PWDN_PIN);
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", QCI_HI_PWDN_PIN);
		return -EIO;
	}

	if (gpio_request(QCI_LO_PWDN_PIN, "CAM_EANBLE_LO_SENSOR")) {
		gpio_free(QCI_LO_PWDN_PIN);
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", QCI_LO_PWDN_PIN);
		return -EIO;
	}

	if (gpio_request(QCI_HI_RESET_PIN, "CAM_RESET_HI_SENSOR")) {
		gpio_free(QCI_HI_RESET_PIN);
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d\n", QCI_HI_RESET_PIN);
		return -EIO;
	}

#if CONFIG_MACH_LC6830_BD_VERSION < 20
	if (gpio_request(QCI_LO_RESET_PIN, "CAM_RESET_LO_SENSOR")) {
		gpio_free(QCI_LO_RESET_PIN);
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d\n", QCI_LO_RESET_PIN);
		return -EIO;
	}
#endif

	if (SENSOR_HIGH == flag) {
		gpio_direction_output(QCI_LO_PWDN_PIN, 1);

		gpio_direction_output(QCI_HI_PWDN_PIN, 0);
		mdelay(1);

		gpio_direction_output(QCI_HI_RESET_PIN, 0);
		mdelay(10);

		gpio_direction_output(QCI_HI_RESET_PIN, 1);
		mdelay(1);
	} else {
		gpio_direction_output(QCI_HI_PWDN_PIN, 1);
		gpio_direction_output(QCI_LO_PWDN_PIN, 0);
		mdelay(1);

#if CONFIG_MACH_LC6830_BD_VERSION < 20
		gpio_direction_output(QCI_LO_RESET_PIN, 0);
		mdelay(1);

		gpio_direction_output(QCI_LO_RESET_PIN, 1);
		mdelay(1);
#endif
	}

	gpio_free(CAM_1P8V_PIN);
	gpio_free(QCI_HI_PWDN_PIN);
	gpio_free(QCI_LO_PWDN_PIN);
	gpio_free(QCI_HI_RESET_PIN);
#if CONFIG_MACH_LC6830_BD_VERSION < 20
	gpio_free(QCI_LO_RESET_PIN);
#endif
#if CONFIG_MACH_LC6830_BD_VERSION >= 21
	gpio_free(CAM_1P2V_PIN);
#endif
	return 0;
}

static int sensor_power_off(int flag)
{
	int status = 0;
	unsigned char val;
	unsigned char val2;

	/*
	 * flag, 0, low resolution
	 * flag, 1, high resolution
	 */
	flag = flag;		/* power off all */
	SPS_TRACE("ENTER flag is %d\n", flag);
	//fang
	/*liuchangmin@leadcoretech.com, add for LDO EN check when set sensor power, Begin */
	/*
	 * disable sensor power
	 * 2.8V,1.8V,1.2V
	 */
	if (gpio_request(CAM_1P8V_PIN, "CAM_1P8V_PIN")) {
		printk(KERN_ERR "Request GPIO failed,"
     		   "gpio: %d \n", CAM_1P8V_PIN);
		return -EIO;
	}
#if CONFIG_MACH_LC6830_BD_VERSION >= 21
    if (gpio_request(CAM_1P2V_PIN, "CAM_1P2V_PIN")) {
    	printk(KERN_ERR "Request GPIO failed,"
    	       "gpio: %d \n", CAM_1P2V_PIN);
    	return -EIO;
    }

	//fang
	//camera data IO
    if (gpio_request(CAM_HI_DATA0, "CAM_HI_DATA0")) {
    	printk(KERN_ERR "Request GPIO failed,"
    	       "gpio: %d \n", CAM_HI_DATA0);
    	return -EIO;
    }
    if (gpio_request(CAM_HI_DATA1, "CAM_HI_DATA1")) {
    	printk(KERN_ERR "Request GPIO failed,"
    	       "gpio: %d \n", CAM_HI_DATA1);
    	return -EIO;
    }
    if (gpio_request(CAM_HI_DATA2, "CAM_HI_DATA2")) {
    	printk(KERN_ERR "Request GPIO failed,"
    	       "gpio: %d \n", CAM_HI_DATA2);
    	return -EIO;
    }
    if (gpio_request(CAM_HI_DATA3, "CAM_HI_DATA3")) {
    	printk(KERN_ERR "Request GPIO failed,"
    	       "gpio: %d \n", CAM_HI_DATA3);
    	return -EIO;
    }
    if (gpio_request(CAM_HI_DATA4, "CAM_HI_DATA4")) {
    	printk(KERN_ERR "Request GPIO failed,"
    	       "gpio: %d \n", CAM_HI_DATA4);
    	return -EIO;
    }
    if (gpio_request(CAM_HI_DATA5, "CAM_HI_DATA5")) {
    	printk(KERN_ERR "Request GPIO failed,"
    	       "gpio: %d \n", CAM_HI_DATA5);
    	return -EIO;
    }
    if (gpio_request(CAM_HI_DATA6, "CAM_HI_DATA6")) {
    	printk(KERN_ERR "Request GPIO failed,"
    	       "gpio: %d \n", CAM_HI_DATA6);
    	return -EIO;
    }
    if (gpio_request(CAM_HI_DATA7, "CAM_HI_DATA7")) {
    	printk(KERN_ERR "Request GPIO failed,"
    	       "gpio: %d \n", CAM_HI_DATA7);
    	return -EIO;
    }
#endif
	status = micco_read(MICCO_APP_OVER2, &val);
	val2 = val;
	if (flag == SENSOR_HIGH) {
#if CONFIG_MACH_LC6830_BD_VERSION >= 20
    	gpio_direction_output(CAM_1P8V_PIN, 0);

		if (val & MICCO_LDO9_ENABLE)
			val2 &= (~MICCO_LDO9_ENABLE);
#endif
#if CONFIG_MACH_LC6830_BD_VERSION >= 21
    	gpio_direction_output(CAM_1P2V_PIN, 0);
#endif
#if CONFIG_MACH_LC6830_BD_VERSION >= 20
		if (val & MICCO_LDO7_ENABLE)
			val2 &= (~MICCO_LDO7_ENABLE);
		
		if (val & MICCO_LDO12_ENABLE)
			val2 &= (~MICCO_LDO12_ENABLE);
#endif
	} else {
#if CONFIG_MACH_LC6830_BD_VERSION == 20
		gpio_direction_output(CAM_1P8V_PIN, 0);
#endif
		if (val & MICCO_LDO10_ENABLE) 
			val2 &= (~MICCO_LDO10_ENABLE);
	}

	if (val2 != val) {
		status = micco_write(MICCO_APP_OVER2, val2);
	}

	/*liuchangmin@leadcoretech.com, add for LDO EN check when set sensor power, End */
#if CONFIG_MACH_LC6830_BD_VERSION < 20
	gpio_direction_output(CAM_1P8V_PIN, 0);
#endif

	//fang
	/*
	 * disable power down
	 */
	if (gpio_request(QCI_HI_PWDN_PIN, "CAM_EANBLE_HI_SENSOR")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", QCI_HI_PWDN_PIN);
		return -EIO;
	}

	if (gpio_request(QCI_LO_PWDN_PIN, "CAM_EANBLE_LO_SENSOR")) {
		gpio_free(QCI_HI_PWDN_PIN);
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d\n", QCI_LO_PWDN_PIN);
		return -EIO;
	}

#if CONFIG_MACH_LC6830_BD_VERSION >= 20
	gpio_direction_output(QCI_HI_PWDN_PIN, 0);
#else
	gpio_direction_output(QCI_HI_PWDN_PIN, 1);
#endif
	gpio_direction_output(QCI_LO_PWDN_PIN, 1);

#if CONFIG_MACH_LC6830_BD_VERSION >= 21
	gpio_direction_output(CAM_HI_DATA0, 0);
	gpio_direction_output(CAM_HI_DATA1, 0);
	gpio_direction_output(CAM_HI_DATA2, 0);
	gpio_direction_output(CAM_HI_DATA3, 0);
	gpio_direction_output(CAM_HI_DATA4, 0);
	gpio_direction_output(CAM_HI_DATA5, 0);
	gpio_direction_output(CAM_HI_DATA6, 0);
	gpio_direction_output(CAM_HI_DATA7, 0);
#endif
	gpio_free(CAM_1P8V_PIN);
	gpio_free(QCI_HI_PWDN_PIN);
	gpio_free(QCI_LO_PWDN_PIN);
#if CONFIG_MACH_LC6830_BD_VERSION >= 21
	gpio_free(CAM_1P2V_PIN);
	gpio_free(CAM_HI_DATA0);
	gpio_free(CAM_HI_DATA1);
	gpio_free(CAM_HI_DATA2);
	gpio_free(CAM_HI_DATA3);
	gpio_free(CAM_HI_DATA4);
	gpio_free(CAM_HI_DATA5);
	gpio_free(CAM_HI_DATA6);
	gpio_free(CAM_HI_DATA7);
#endif
#if CONFIG_MACH_LC6830_BD_VERSION >= 20
	pxa3xx_mfp_config(ARRAY_AND_SIZE(camif_off_mfp_cfg));
#endif
	return 0;
}

static struct sensor_platform_data ov5642_sensor_data = {
	.id = SENSOR_HIGH,
	.power_on = sensor_power_on,
	.power_off = sensor_power_off,
};

static struct sensor_platform_data ov7690_sensor_data = {
	.id = SENSOR_LOW,
	.power_on = sensor_power_on,
	.power_off = sensor_power_off,
};

/*liuchangmin 2009-9-30 modified for lc6830. END*/

/* sensor init over */

/* camera platform data */
static mfp_cfg_t sync[] = {
	GPIO73_CI_HSYNC,
	GPIO74_CI_VSYNC,
};

static mfp_cfg_t sync_gpio[] = {
	GPIO73_CI_HSYNC_GPIO,
	GPIO74_CI_VSYNC_GPIO,
};

static void cam_sync_to_gpio(void)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(sync_gpio));
}

static void cam_sync_from_gpio(void)
{
	pxa3xx_mfp_config(ARRAY_AND_SIZE(sync));
}

static int cam_init(void)
{
	return 0;
}

static void cam_deinit(void)
{
}

static void cam_suspend(void)
{
	sensor_power_off(SENSOR_LOW);
}

static void cam_resume(void)
{
	sensor_power_off(SENSOR_LOW);
}

static struct cam_platform_data cam_ops = {
	.vsync_gpio = MFP_PIN_GPIO74,
	.init = cam_init,
	.deinit = cam_deinit,
	.suspend = cam_suspend,
	.resume = cam_resume,
	.sync_to_gpio = cam_sync_to_gpio,
	.sync_from_gpio = cam_sync_from_gpio,
};

static void __init saar_init_cam(void)
{
	pxa3xx_device_cam.dev.platform_data = &cam_ops;
	platform_device_register(&pxa3xx_device_cam);
}

/* QCI init over */

#endif

#if defined(CONFIG_RADIO_SI4703) || defined(CONFIG_RADIO_SI4703_MODULE)
int si4703_setup(struct i2c_client *client, void *context)
{
	int reset_pin = mfp_to_gpio(MFP_PIN_GPIO86);
	int irq_pin = mfp_to_gpio(MFP_PIN_GPIO96);

	gpio_request(reset_pin, "si4703 FM radio reset");
	gpio_request(irq_pin, "si4703 FM radio interrupt");

	/* clear GPIO96 edge detect */
	pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO96_GPIO), MFP_EDGE_NONE);

	/* configure interrupt pin as input */
	gpio_direction_input(reset_pin);

	/* assert reset for 100 ms */
	gpio_direction_output(reset_pin, 0);
	mdelay(100);

	/* deassert reset */
	gpio_set_value(reset_pin, 0);

	gpio_free(reset_pin);
	gpio_free(irq_pin);

	return 0;
}

static struct si4703_platform_data si4703_data = {
	.setup = si4703_setup,
};
#endif

#ifdef CONFIG_INNODEV_V2	//if202 driver
static void inno_if202_ssp_mfp_gpio(void)
{
	pxa3xx_mfp_set_afds(mfp_to_gpio(MFP_PIN_GPIO0), MFP_AF0, MFP_DS03X);
	pxa3xx_mfp_set_afds(mfp_to_gpio(MFP_PIN_GPIO1), MFP_AF0, MFP_DS03X);
	pxa3xx_mfp_set_afds(mfp_to_gpio(MFP_PIN_GPIO2), MFP_AF0, MFP_DS03X);
	pxa3xx_mfp_set_afds(mfp_to_gpio(MFP_PIN_GPIO3), MFP_AF0, MFP_DS03X);
}

static void inno_if202_ssp_mfp_ssp(void)
{
	pxa3xx_mfp_set_afds(mfp_to_gpio(MFP_PIN_GPIO0), MFP_AF2, MFP_DS03X);
	pxa3xx_mfp_set_afds(mfp_to_gpio(MFP_PIN_GPIO1), MFP_AF2, MFP_DS03X);
	pxa3xx_mfp_set_afds(mfp_to_gpio(MFP_PIN_GPIO2), MFP_AF2, MFP_DS03X);
	pxa3xx_mfp_set_afds(mfp_to_gpio(MFP_PIN_GPIO3), MFP_AF2, MFP_DS03X);
}

static int inno_if202_init(void)
{
	if (gpio_request(INNO_IF202_INTR_PIN, "IF101_IRQ")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", INNO_IF202_INTR_PIN);
		return -EIO;
	}
	gpio_direction_input(INNO_IF202_INTR_PIN);
	gpio_free(INNO_IF202_INTR_PIN);
	if (gpio_request(INNO_IF202_EN_PIN, "IF101_ENABLE")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", INNO_IF202_EN_PIN);
		return -EIO;
	}
	if (gpio_request(mfp_to_gpio(MFP_PIN_GPIO0), "IF202_GPIO12")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", mfp_to_gpio(MFP_PIN_GPIO12));
		return -EIO;
	}
	if (gpio_request(mfp_to_gpio(MFP_PIN_GPIO1), "IF202_GPIO13")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", mfp_to_gpio(MFP_PIN_GPIO13));
		return -EIO;
	}
	if (gpio_request(mfp_to_gpio(MFP_PIN_GPIO2), "IF202_GPIO14")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", mfp_to_gpio(MFP_PIN_GPIO14));
		return -EIO;
	}
	if (gpio_request(mfp_to_gpio(MFP_PIN_GPIO3), "IF202_GPIO15")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", mfp_to_gpio(MFP_PIN_GPIO15));
		return -EIO;
	}

	gpio_direction_output(INNO_IF202_EN_PIN, 0);	/* power voltage off */

	/* config ssp pins to gpio input to save power */
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO0));
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO1));
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO2));
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO3));
	inno_if202_ssp_mfp_gpio();

	gpio_free(INNO_IF202_EN_PIN);
	gpio_free(mfp_to_gpio(MFP_PIN_GPIO0));
	gpio_free(mfp_to_gpio(MFP_PIN_GPIO1));
	gpio_free(mfp_to_gpio(MFP_PIN_GPIO2));
	gpio_free(mfp_to_gpio(MFP_PIN_GPIO3));

	return 0;
}

int inno_if202_voltage_on(void)
{
	if (gpio_request(mfp_to_gpio(MFP_PIN_GPIO0), "IF202_GPIO12")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", mfp_to_gpio(MFP_PIN_GPIO12));
		return -EIO;
	}
	if (gpio_request(mfp_to_gpio(MFP_PIN_GPIO1), "IF202_GPIO13")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", mfp_to_gpio(MFP_PIN_GPIO13));
		return -EIO;
	}
	if (gpio_request(mfp_to_gpio(MFP_PIN_GPIO2), "IF202_GPIO14")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", mfp_to_gpio(MFP_PIN_GPIO14));
		return -EIO;
	}
	if (gpio_request(mfp_to_gpio(MFP_PIN_GPIO3), "IF202_GPIO15")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", mfp_to_gpio(MFP_PIN_GPIO15));
		return -EIO;
	}
	/* The if202 chip may not power up correctly if the connected pins are high,
	 *         *          * so config ssp pins to gpio input before power up */
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO0));
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO1));
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO2));
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO3));

	inno_if202_ssp_mfp_gpio();

	if (gpio_request(mfp_to_gpio(MFP_PIN_GPIO6), "IF202_ENABLE")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", INNO_IF202_EN_PIN);
		return -EIO;
	}

	pxa3xx_mfp_set_afds(INNO_IF202_EN_PIN, MFP_AF0, MFP_DS03X);
	if (gpio_get_value(INNO_IF202_EN_PIN) == 0) {
		printk("enable power*******\n");
		/* enable VDD (1.8V) & VIO (2.8V) */
		gpio_direction_output(INNO_IF202_EN_PIN, 1);

		/* IF202 requires at least 500ms delay after supply voltage */
		msleep(600);
	}
	printk("%x******\n", gpio_get_value(INNO_IF202_EN_PIN));
	gpio_free(INNO_IF202_EN_PIN);
	gpio_free(mfp_to_gpio(MFP_PIN_GPIO0));
	gpio_free(mfp_to_gpio(MFP_PIN_GPIO1));
	gpio_free(mfp_to_gpio(MFP_PIN_GPIO2));
	gpio_free(mfp_to_gpio(MFP_PIN_GPIO3));

	/* reconfig ssp pins to SSP after power up */
	inno_if202_ssp_mfp_ssp();
	return 0;
}

int inno_if202_voltage_off(void)
{
	if (gpio_request(mfp_to_gpio(MFP_PIN_GPIO0), "IF202_GPIO12")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", mfp_to_gpio(MFP_PIN_GPIO12));
		return -EIO;
	}
	if (gpio_request(mfp_to_gpio(MFP_PIN_GPIO1), "IF202_GPIO13")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", mfp_to_gpio(MFP_PIN_GPIO13));
		return -EIO;
	}
	if (gpio_request(mfp_to_gpio(MFP_PIN_GPIO2), "IF202_GPIO14")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", mfp_to_gpio(MFP_PIN_GPIO14));
		return -EIO;
	}
	if (gpio_request(mfp_to_gpio(MFP_PIN_GPIO3), "IF202_GPIO15")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", mfp_to_gpio(MFP_PIN_GPIO15));
		return -EIO;
	}
	if (gpio_request(INNO_IF202_EN_PIN, "IF101_CAM_ENABLE")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", INNO_IF202_EN_PIN);
		return -EIO;
	}
	gpio_direction_output(INNO_IF202_EN_PIN, 0);

	/* config ssp pins to gpio input to save power */
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO0));
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO1));
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO2));
	gpio_direction_input(mfp_to_gpio(MFP_PIN_GPIO3));

	gpio_free(INNO_IF202_EN_PIN);
	gpio_free(mfp_to_gpio(MFP_PIN_GPIO0));
	gpio_free(mfp_to_gpio(MFP_PIN_GPIO1));
	gpio_free(mfp_to_gpio(MFP_PIN_GPIO2));
	gpio_free(mfp_to_gpio(MFP_PIN_GPIO3));
	/* config ssp pins to gpio input to save power */
	inno_if202_ssp_mfp_gpio();

	return 0;
}

static int inno_if202_reset(void)
{
	if (gpio_request(mfp_to_gpio(MFP_PIN_GPIO4), "IF202_RESET")) {
		printk(KERN_ERR "Request GPIO failed,"
		       "gpio: %d \n", INNO_IF202_RESET);
		return -EIO;
	}
	printk("enter reset*****\n");
	pxa3xx_mfp_set_afds(INNO_IF202_RESET, MFP_AF0, MFP_DS03X);
	gpio_direction_output(INNO_IF202_RESET, 0);
	mdelay(10);
	gpio_set_value(INNO_IF202_RESET, 1);
	mdelay(5);
	gpio_free(INNO_IF202_RESET);

	return 0;
}

static struct inno_if202_platform_data inno_if202_data = {
	.voltage_on = inno_if202_voltage_on,
	.voltage_off = inno_if202_voltage_off,
	.reset = inno_if202_reset,
};
#endif /* #ifdef CONFIG_INNODEV_V2 */

#if defined(CONFIG_SENSORS_SMB380)
#if CONFIG_MACH_LC6830_BD_VERSION >= 20
#define SMB380_IRQ_GPIO         mfp_to_gpio(MFP_PIN_GPIO11)
#else
#define SMB380_IRQ_GPIO         mfp_to_gpio(MFP_PIN_GPIO10)
#endif

#define SMB380_IRQ              IRQ_GPIO(SMB380_IRQ_GPIO)

static int smb380_init_irq(void)
{
	int ret = 0;

	printk("request SMB380_IRQ_GPIO\n");
	ret = gpio_request(SMB380_IRQ_GPIO, "smb380 irq");
	if (ret) {
		printk("gpio_requset failed, return :%d\n", ret);
		goto err_request_cd;
	}
	ret = gpio_direction_input(SMB380_IRQ_GPIO);
	if (ret) {
		printk("set gpio pin direction for smb380 failed, return :%d\n", ret);
		goto err_request_irq;
	}

err_request_irq:
	gpio_free(SMB380_IRQ_GPIO);
err_request_cd:
	return ret;
}

static int smb380_ack_irq(void)
{
	return 0;
}

static struct smb380_platform_data smb380_data = {
	.init_irq = smb380_init_irq,
	.ack_irq = smb380_ack_irq,
#if CONFIG_MACH_LC6830_BD_VERSION < 20
	.xyz_dir = (  X_POS | Y_POS |  Z_POS),
#else	
	.xyz_dir = (X_POS),
#endif
};
#endif

static struct i2c_board_info saar_i2c_board_info[] = {
#if defined(CONFIG_PXA3xx_MICCO) || defined(CONFIG_PXA3xx_MICCO_MODULE)
	{
	 .driver_name = "micco",
	 .addr = 0x34,
	 .platform_data = &micco_data,
	 .irq = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO83)),
	 },
#endif
#if defined(CONFIG_PXA3xx_BD6091GU)
	{
	 .driver_name = "bd6091gu-i2c",
	 .addr = 0x76,
	 },
#endif
#if defined(CONFIG_PXA_CAMERA)
	{
	 .driver_name = "sensor_ov7690",
	 .addr = 0x21,
	 .platform_data = &ov7690_sensor_data,
	 },

	{
#if CONFIG_MACH_LC6830_BD_VERSION >= 20
	 .driver_name = "CLI6000",
#else
	 .driver_name = "sensor_ov5642",
	 .addr = 0x3C,
#endif
	 .platform_data = &ov5642_sensor_data,
	 },
#endif

#if defined(CONFIG_RADIO_SI4703) || defined(CONFIG_RADIO_SI4703_MODULE)
	{
	 .driver_name = "si4703",
	 .addr = 0x10,
	 .platform_data = &si4703_data,
	 .irq = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO96)),
	 },
#endif
#if defined(CONFIG_INNODEV_V2)	//if202 or if202
	{
	 .driver_name = "inno_if202",
	 .addr = 0x10,
	 .platform_data = &inno_if202_data,
	 .irq = INNO_IF202_IRQ,
	 },
#endif
#if defined(CONFIG_SENSORS_SMB380)
	{
	 .driver_name = "smb380",
	 .addr = 0x38,
	 .platform_data = &smb380_data,
#if CONFIG_MACH_LC6830_BD_VERSION >= 20
	 .irq = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO11)),
#else	 
	 .irq = IRQ_GPIO(mfp_to_gpio(MFP_PIN_GPIO10)),
#endif
	 },
#endif
#if defined(CONFIG_SENSORS_SHARP)
    {
     .driver_name = "gp2ap002-i2c-gpio",
     .addr = 0x44,
     .irq = mfp_to_gpio(MFP_PIN_GPIO41),
     },
#endif
};

static struct i2c_pxa_platform_data pxa3xx_i2c_platform_data = {
	.use_pio = 1,				 /*=0 =irq  mode instead POLLING (pio=1) */
	.flags = PXA_I2C_FAST_MODE,		 /*=1 =SLOW mode instead PXA_I2C_FAST_MODE=10b*/
};

struct pxa3xx_freq_mach_info saar_freq_mach_info = {
	.flags = PXA3xx_USE_POWER_I2C,
};

#if defined(CONFIG_FB_PXA) || (CONFIG_FB_PXA_MODULE)
#define LCDRST_GPIO		MFP_PIN_GPIO22

void lc6830_lcd_power(int on, struct fb_var_screeninfo *var)
{
	if (on) {
		printk(KERN_INFO "\nLCD POWER ON\n");

#if defined(OBM_LOGO_DISPLAY)
		if (system_state != SYSTEM_BOOTING) {
			gpio_direction_output(LCDRST_GPIO, 0);	/* Set output direction and low to Reset pin */
			mdelay(2);	/* wait min 1ms */
			gpio_direction_output(LCDRST_GPIO, 1);	/* Set output direction and high to Reset pin */
			mdelay(6);	/* wait min 5ms */
			printk(KERN_INFO "LCD Reset\n");
		} else {
			/* OBM had been reset LCD panel module. So, I don't need reset here! */
			printk(KERN_INFO "LCD None-Reset\n");
		}
#else /* Not OBM_LOGO_DISPLAY */
		/* OBM had not been reset LCD panel module. So I do LCD Panel Reset */
		gpio_direction_output(LCDRST_GPIO, 0);	/* Set output direction and low to Reset pin */
		mdelay(2);	/* wait min 1ms */
		gpio_direction_output(LCDRST_GPIO, 1);	/* Set output direction and high to Reset pin */
		mdelay(6);	/* wait min 5ms */
		printk(KERN_INFO "LCD Reset\n");
#endif /* OBM_LOGO_DISPLAY */
		init_leadcore_panel(1);	/* Send the power-on codes and waits to sharp panel through SSP Port */
	} else {
		printk(KERN_INFO "\nLCD POWER OFF\n");
		init_leadcore_panel(0);	/* Send the power-off codes and waits to sharp panel through SSP Port */
	}
}

static struct pxafb_mode_info sharp_ls035y8dx02a_modes[] = {
	[0] = {
	       .pixclock = 38461,	/* PixelClock = 1 / ( pixclock * 10^-12 ) */
	       .xres = 480,	/* PPL(Pixel Per Line) */
	       .yres = 800,	/* LPP(Lines Per Panel) */
	       .bpp = 32,	/* BPP(Bit Per Pixel) */
	       .hsync_len = 16,	/* Horizontal Sync Pulse Width (DCLK) */
	       .left_margin = 24,	/* BLW(Beginning-of-Line pixel clock Wait) (LINE) */
	       .right_margin = 16,	/* ELW(End-of-Line clock Wait) (LINE) */
	       .vsync_len = 2,	/* Vertical Sync Pulse Width (LINE) */
	       .upper_margin = 3,	/* BFW(Beginning-of-Frame pixel clock Wait) (DCLK) */
	       .lower_margin = 3,	/* EFW(End-of-Frame pixel clock Wait) (DCLK) */
	       .sync = 0,	/* Horizontal and Vertical Sync Active High */
	       },
};

static struct pxafb_mach_info lc6830_lcd_info = {
	.modes = sharp_ls035y8dx02a_modes,
	.num_modes = 1,
	.fixed_modes = 1,
	.lccr0 = LCCR0_Act,
	.pxafb_lcd_power = lc6830_lcd_power,
	.pxafb_config_panel = init_leadcore_panel,
};

static void __init lc6830_init_lcd(void)
{
	set_pxa_fb_info(&lc6830_lcd_info);
}
#else
static void __init lc6830_init_lcd(void)
{
}
#endif /* CONFIG_FB_PXA || CONFIG_FB_PXA_MODULE */
/* Modified by Leadcore Liuchangmin on 2009-09-19 END*/

//#ifdef CONFIG_PM
#if 1
extern irqreturn_t uartrx_detect_irq(int irq, void *_dev);

static mfp_cfg_t saar_ffuart_enable_mfp_cfg[] = {
	/* FFUART */
	GPIO63_FFTXD,
	GPIO64_FFRXD,
//      DF_ADDR0_GPIO_63,       /* need to verify */
};

static mfp_cfg_t saar_ffuart_disable_mfp_cfg[] = {
	/* FFUART */
	GPIO63_GPIO,
	GPIO64_GPIO,
	DF_ADDR0_NONE,
};

extern int is_uart_gpio(void);
static int saar_ffuart_switch(int enable)
{
	int gpio_rx, irq_rx;
	int ret;
	gpio_rx = mfp_to_gpio(MFP_PIN_GPIO64);
	irq_rx = gpio_to_irq(gpio_rx);

	if (enable) {
		gpio_free(gpio_rx);
		/* if edge detection is not clear, the altfun switch will
		 * gpio interrupt is generated on this gpio pin when
		 * switching pins from UART to gpio. so clear it.
		 */
		GRER(gpio_rx) &= ~GPIO_bit(gpio_rx);
		GFER(gpio_rx) &= ~GPIO_bit(gpio_rx);
		pxa3xx_mfp_config(ARRAY_AND_SIZE(saar_ffuart_enable_mfp_cfg));
	} else {
		pxa3xx_mfp_config(ARRAY_AND_SIZE(saar_ffuart_disable_mfp_cfg));
		pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO64_GPIO), MFP_EDGE_FALL);
		GRER(gpio_rx) |= GPIO_bit(gpio_rx);
		GFER(gpio_rx) |= GPIO_bit(gpio_rx);

		ret = gpio_request(gpio_rx, "FFUART RX");
		/* clear the MFPR */
		gpio_direction_input(gpio_rx);
	}
	return irq_rx;
}

static struct pxa_uart_mach_info saar_ffuart_info = {
//	.uart_pin_switch = saar_ffuart_switch, //2009-01-29 change by zj for gps sleep
};

static mfp_cfg_t saar_stuart_enable_mfp_cfg[] = {
	/* STUART */
	GPIO77_STRXD,
	GPIO78_STTXD,
	GPIO43_STRTS_N,
	GPIO44_STCTS_N,
};

static mfp_cfg_t saar_stuart_disable_mfp_cfg[] = {
	/* STUART */
	GPIO77_GPIO,
	GPIO78_GPIO,
	GPIO43_GPIO,
	GPIO44_GPIO,
};

#ifdef CONFIG_MODEM_INTERFACE
extern void a2b_sleep_ind_set(int state);
#endif
static int saar_stuart_switch(int enable)
{
	int gpio_rx, irq_rx, gpio_rts, gpio_cts;
	int ret;
	gpio_rx = mfp_to_gpio(MFP_PIN_GPIO77);
	gpio_rts = mfp_to_gpio(MFP_PIN_GPIO43);
	gpio_cts = mfp_to_gpio(MFP_PIN_GPIO44);

	irq_rx = gpio_to_irq(gpio_rx);

	if (enable) {
		gpio_free(gpio_rx);

		pxa3xx_mfp_config(ARRAY_AND_SIZE(saar_stuart_enable_mfp_cfg));
/* Added by Leadcore Songlixin on: Sat Mar 20 01:20:59 CST 2010 BEGIN*/
#ifdef CONFIG_MODEM_INTERFACE
#if CONFIG_MACH_LC6830_BD_VERSION >= 20
		a2b_sleep_ind_set(0);
#endif
#endif
/* Added by Leadcore Songlixin on: Sat Mar 20 01:20:59 CST 2010 END*/

	} else {
/* Added by Leadcore Songlixin on: Sat Mar 20 01:21:02 CST 2010 BEGIN*/
#ifdef CONFIG_MODEM_INTERFACE
#if CONFIG_MACH_LC6830_BD_VERSION >= 20
		a2b_sleep_ind_set(1);
#endif
#endif
/* Added by Leadcore Songlixin on: Sat Mar 20 01:21:02 CST 2010 END*/

		pxa3xx_mfp_config(ARRAY_AND_SIZE(saar_stuart_disable_mfp_cfg));
		pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO77_GPIO), MFP_EDGE_FALL);

		ret = gpio_request(gpio_rx, "STUART RX");
		/* clear the MFPR */
		gpio_direction_input(gpio_rx);

		ret = gpio_request(gpio_rts, "STUART RTS");
		gpio_direction_input(gpio_rts);
		
		ret = gpio_request(gpio_cts, "STUART CTS");
		gpio_direction_input(gpio_cts);

		gpio_free(gpio_rts);
		gpio_free(gpio_cts);
	}
	return irq_rx;
}

static struct pxa_uart_mach_info saar_stuart_info = {
	.uart_pin_switch = saar_stuart_switch,
};

static atomic_t broadcom_module_cnt = ATOMIC_INIT(0);

void broadcom_module_enter(void)
{
	int powerdown = mfp_to_gpio(MFP_PIN_GPIO174);

    if (0 == atomic_read(&broadcom_module_cnt)) {
		pxa3xx_mfp_config(ARRAY_AND_SIZE(wlan_bt_powerdown_high_mfp_cfg));

		gpio_request(powerdown, "Wlan/BT powerdown");
	    gpio_direction_output(powerdown, 1);    /* Disable powerdown */
		gpio_free(powerdown);

    } 

    atomic_inc(&broadcom_module_cnt);
}

void broadcom_module_exit(void)
{
	int powerdown = mfp_to_gpio(MFP_PIN_GPIO174);

	if (atomic_read(&broadcom_module_cnt)){
	    atomic_dec(&broadcom_module_cnt);

		if (0 == atomic_read(&broadcom_module_cnt)) {
			pxa3xx_mfp_config(ARRAY_AND_SIZE(wlan_bt_powerdown_low_mfp_cfg));

	        gpio_request(powerdown, "Wlan/BT powerdown");
	        gpio_direction_output(powerdown, 0);    /* Enable powerdown */
	        gpio_free(powerdown);
		}
	} 
}
EXPORT_SYMBOL(broadcom_module_enter);
EXPORT_SYMBOL(broadcom_module_exit);

static mfp_cfg_t saar_btuart_enable_mfp_cfg[] = {
    /* BTUART */
	GPIO91_BTRXD,
	GPIO92_BTTXD,
	GPIO96_BTRTS_N,
	GPIO99_GPIO,
};

static mfp_cfg_t saar_btuart_disable_mfp_cfg[] = {
    /* BTUART */
	GPIO91_GPIO,
	GPIO92_GPIO,
	GPIO96_GPIO,
};

static int saar_btuart_switch(int enable)
{
	int ap2bt_rts;
	int gpio_rx;
	int irq_rx;

	ap2bt_rts = mfp_to_gpio(MFP_PIN_GPIO96);
	gpio_rx = mfp_to_gpio(MFP_PIN_GPIO99);
	irq_rx = gpio_to_irq(gpio_rx);
   
    if (enable) {
		gpio_free(ap2bt_rts);
        gpio_free(gpio_rx);
        pxa3xx_mfp_config(ARRAY_AND_SIZE(saar_btuart_enable_mfp_cfg));
	} else {
        pxa3xx_mfp_config(ARRAY_AND_SIZE(saar_btuart_disable_mfp_cfg));

        pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO99_GPIO), MFP_EDGE_FALL);
        gpio_request(gpio_rx, "BT2HOST Wakeup");
        /* clear the MFPR */
        gpio_direction_input(gpio_rx);

		gpio_request(ap2bt_rts, "AP2BT RTS");
		gpio_direction_output(ap2bt_rts, 1);	/* Pull RTS Signal High */
	}

    return irq_rx;
}

static int saar_btuart_power(int onff)
{
	int a2b_reset = mfp_to_gpio(MFP_PIN_GPIO98);
	unsigned long val;

	val = pxa3xx_mfp_read(MFP_PIN_GPIO174);

	if (onff) {
		pxa3xx_mfp_config(ARRAY_AND_SIZE(bt_reset_high_mfp_cfg));
		if (gpio_request(a2b_reset, "BT reset"))
			return 0;

		broadcom_module_enter();

		val = pxa3xx_mfp_read(MFP_PIN_GPIO174);

		gpio_direction_output(a2b_reset, 0);	/* Reset Blutetooth die */
		msleep(100);
		gpio_direction_output(a2b_reset, 1);
		msleep(400);							/* Enable Bluetooth firmware run */

		gpio_free(a2b_reset);
	} else {
		pxa3xx_mfp_config(ARRAY_AND_SIZE(bt_reset_low_mfp_cfg));
		//FIXME enter into low power mode
		broadcom_module_exit();
		val = pxa3xx_mfp_read(MFP_PIN_GPIO174);
	}

	return 0;
}
void saa_audio_pa_sleep_enable(int onff)
{
	if(onff)
		pxa3xx_mfp_config(ARRAY_AND_SIZE(audio_pa_high_mfp_cfg));
	else
		pxa3xx_mfp_config(ARRAY_AND_SIZE(audio_pa_low_mfp_cfg));
	return;
}

static struct pxa_uart_mach_info saar_btuart_info = {
    .uart_pin_switch = saar_btuart_switch,
	.uart_power = saar_btuart_power,
};

static int saar_init_wakeup(pm_wakeup_src_t * src)
{
	memset(src, 0, sizeof(pm_wakeup_src_t));
	src->bits.rtc = 1;	/* for alarm, low power detect */
//	src->bits.mkey = 1;	
	src->bits.ext0 = 1;	/* for power on key wakeup from deepsleep */
	src->bits.tsi = 1;

/* Modified by Leadcore Songlixin on: Sat Jan  9 02:00:19 PST 2010 BEGIN*/
/* * only turn on needed wakeup src */
/* board2 has no mmc detect*/
#if CONFIG_MACH_LC6830_BD_VERSION >= 20
#ifdef CONFIG_MODEM_INTERFACE
	src->bits.modem = 1;	//for modem wakeup
#endif

#ifdef CONFIG_SENSORS_SHARP
	src->bits.psensor = 1;	//for psensor wakeup
#endif
#else
#if CONFIG_MACH_LC6830_BD_VERSION > 11
	src->bits.modem = 1;	//for modem wakeup
#else
	src->bits.uart3 = 1;	//ST UART, for AT channel
#endif
	src->bits.mmc1_cd = 1;	//for mmc wakeup

#endif
/* Modified by Leadcore Songlixin on: Sat Jan  9 02:00:19 PST 2010 END */

	return 0;
}

extern int pxa930_query_gwsr(int);
static int saar_query_wakeup(unsigned int reg, pm_wakeup_src_t * src)
{
	//printk(KERN_DEBUG "%s:reg:0x%x\n", __func__, reg);
	memset(src, 0, sizeof(pm_wakeup_src_t));
	if (reg & PXA3xx_PM_WE_RTC)
		src->bits.rtc = 1;
	if (reg & PXA3xx_PM_WE_OST)
		src->bits.ost = 1;
	if (reg & PXA3xx_PM_WE_MSL0)
		src->bits.msl = 1;
	if (reg & PXA3xx_PM_WE_EXTERNAL0)
		src->bits.ext0 = 1;
	if (reg & PXA3xx_PM_WE_KP)
		src->bits.mkey = 1;
	if (reg & PXA3xx_PM_WE_GENERIC(3))
		src->bits.tsi = 1;
	if (reg & PXA3xx_PM_WE_GENERIC(9)) {
		if (!is_uart_gpio()) {
			src->bits.uart1 = 1;
		}
	}
	if (reg & PXA3xx_PM_WE_GENERIC(2)) {
		if (!is_uart_gpio()) {
			src->bits.uart2 = 1;
		}
	}
	if (reg & PXA3xx_PM_WE_GENERIC(12))
		src->bits.cmwdt = 1;
	if (reg & PXA3xx_PM_WE_GENERIC(13)) {

/* Modified by Leadcore Songlixin on: Thu Jan 14 00:03:32 PST 2010 BEGIN*/
/*
 * update according to real world
 */
#if 1
		if (pxa930_query_gwsr(61)) {
			src->bits.mmc1_cd = 1;
		} else if (pxa930_query_gwsr(64)) {
			src->bits.uart1 = 1;
		} else if (pxa930_query_gwsr(77)) {
			src->bits.uart3 = 1;
		} else if (pxa930_query_gwsr(62)) {
			src->bits.modem = 1;
		}
                #ifdef CONFIG_SENSORS_SHARP
                else if (pxa930_query_gwsr(41)) {
			src->bits.psensor = 1;
		} 
                #endif
                else {
			int i;
			for (i = 0; i <= 106; i++) {
				if (pxa930_query_gwsr(i)) {
					break;
				}
			}

			if (i == 107) {
				for (i = 159; i <= 176; i++) {
					if (pxa930_query_gwsr(i)) {
						break;
					}
				}
			}


			if (i < 177)
				printk(KERN_DEBUG "gpio wakeup src:%d\n", i);
			else
				printk
				    ("very stange, no gpio wakeup src found!\n");
		}
#else
		if (pxa930_query_gwsr(97))
			src->bits.eth = 1;
		if (pxa930_query_gwsr(53))
			src->bits.uart1 = 1;
#endif
/* Modified by Leadcore Songlixin on: Thu Jan 14 00:03:32 PST 2010 END */

	}
	return 0;
}

static int saar_ext_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.ext0)
			ret |= PXA3xx_PM_WE_EXTERNAL0;
		if (src.bits.ext1)
			ret |= PXA3xx_PM_WE_EXTERNAL1;
	}
	return ret;
}

static int saar_key_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.mkey) {


/* Modified by Leadcore Songlixin on: Thu Jan 21 14:12:24 CST 2010 BEGIN*/
/* * Update according to hardware **/
#if 1
            pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO12_KP_MKIN_6),
                        MFP_EDGE_BOTH);
            pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO14_KP_MKIN_7),
                        MFP_EDGE_BOTH);
#else
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO0_KP_MKIN_0),
					    MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO2_KP_MKIN_1),
					    MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO4_KP_MKIN_2),
					    MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO6_KP_MKIN_3),
					    MFP_EDGE_BOTH);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO8_KP_MKIN_4),
					    MFP_EDGE_BOTH);
#endif
/* Modified by Leadcore Songlixin on: Thu Jan 21 14:12:24 CST 2010 END **/

			ret |= PXA3xx_PM_WE_KP;
		}
	} else {
		if (src.bits.mkey) {
/* Modified by Leadcore Songlixin on: Thu Jan 21 14:12:24 CST 2010 BEGIN*/
/* * Update according to hardware **/
#if 1
            pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO12_KP_MKIN_6),
                        MFP_EDGE_NONE);
            pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO14_KP_MKIN_7),
                        MFP_EDGE_NONE);
#else
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO0_KP_MKIN_0),
					    MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO2_KP_MKIN_1),
					    MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO4_KP_MKIN_2),
					    MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO6_KP_MKIN_3),
					    MFP_EDGE_NONE);
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO8_KP_MKIN_4),
					    MFP_EDGE_NONE);
#endif
/* Modified by Leadcore Songlixin on: Thu Jan 21 14:12:24 CST 2010 END **/
		}
	}
	return ret;
}

static int saar_mmc_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.mmc1_cd) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO61_GPIO),
					    MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(13);
		}
	} else
		pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO61_GPIO), MFP_EDGE_NONE);
	return ret;
}

static int saar_modem_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.modem) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO62_GPIO),
					    MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(13);
		}
	} else
		pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO62_GPIO), MFP_EDGE_NONE);
	return ret;
}
#ifdef CONFIG_SENSORS_SHARP
static int saar_psensor_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.psensor) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO41_GPIO),
					    MFP_EDGE_RISE);
			ret |= PXA3xx_PM_WE_GENERIC(13);
		}
	} else
		pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO41_GPIO), MFP_EDGE_NONE);
	return ret;
}
#endif
static int saar_uart_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;

	if (enable) {
		if (src.bits.uart1) {
			if (is_uart_gpio()) {
				pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO64_GPIO),
						    MFP_EDGE_FALL);
				ret |= PXA3xx_PM_WE_GENERIC(13);
			} else {
				pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO64_FFRXD),
						    MFP_EDGE_FALL);
				ret |= PXA3xx_PM_WE_GENERIC(9);
			}
		}
		if (src.bits.uart3) {
			if (is_uart_gpio()) {
				pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO77_GPIO),
						    MFP_EDGE_FALL);
				ret |= PXA3xx_PM_WE_GENERIC(13);
			} else {
				pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO77_STRXD),
						    MFP_EDGE_FALL);
				/* note: on pxa930, uart2 use this bit */
				ret |= PXA3xx_PM_WE_GENERIC(2);
			}
		}
	} else {
		if (src.bits.uart1) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO64_FFRXD),
					    MFP_EDGE_NONE);
		}
		if (src.bits.uart3) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(GPIO77_STRXD),
					    MFP_EDGE_NONE);
		}
	}
	return ret;
}

static int saar_tsi_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.tsi) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(PMIC_INT_GPIO_83),
					    MFP_EDGE_BOTH);
			ret |= PXA3xx_PM_WE_GENERIC(3);
		}
	} else {
		if (src.bits.tsi) {
			pxa3xx_mfp_set_edge(MFP_CFG_PIN(PMIC_INT_GPIO_83),
					    MFP_EDGE_NONE);
		}
	}
	return ret;
}

#if 0
static int saar_comm_wdt_wakeup(pm_wakeup_src_t src, int enable)
{
	unsigned int ret = 0;
	if (enable) {
		if (src.bits.cmwdt)
			ret |= PXA3xx_PM_WE_GENERIC(12);
	}
	return ret;
}
#endif

static struct pxa3xx_peripheral_wakeup_ops wakeup_ops = {
	.init = saar_init_wakeup,
	.query = saar_query_wakeup,
	.ext = saar_ext_wakeup,
	.key = saar_key_wakeup,
	.mmc = saar_mmc_wakeup,
	.uart = saar_uart_wakeup,
//      .eth    = saar_eth_wakeup,
//      .cmwdt  = saar_comm_wdt_wakeup,
    .tsi    = saar_tsi_wakeup,
	.modem = saar_modem_wakeup,
#ifdef CONFIG_SENSORS_SHARP
	.psensor = saar_psensor_wakeup,
#endif
};
#else
static struct pxa_uart_mach_info saar_ffuart_info = NULL;
static struct pxa_uart_mach_info saar_stuart_info = NULL;
#endif

/* Added by Leadcore Songlixin on: Tue Dec 15 23:34:14 PST 2009 BEGIN*/
/*
 * some gpio as controlling power, they should output right values!
 */
#if 1
static void lc6830_gpio_config(void)
{
	int the_gpio;

#if CONFIG_MACH_LC6830_BD_VERSION >= 21
	the_gpio = mfp_to_gpio(MFP_PIN_GPIO164);
	gpio_direction_output(the_gpio, 1);
	gpio_free(the_gpio);
#endif

	//AP2CMMB_PWREN, CMMB power
	the_gpio = mfp_to_gpio(MFP_PIN_GPIO6);
	gpio_direction_output(the_gpio, 0);
	gpio_free(the_gpio);

	//AP_CAM_1P8VPWREN, Camera Power 1.8V
	the_gpio = mfp_to_gpio(MFP_PIN_GPIO7);
	gpio_direction_output(the_gpio, 0);
	gpio_free(the_gpio);

	//AP_GPSPWR_EN, GPS power
	the_gpio = mfp_to_gpio(MFP_PIN_GPIO9);
	gpio_direction_output(the_gpio, 0);
	gpio_free(the_gpio);

	//AP_BAT_UARTOE, Uart vol level translator
	the_gpio = mfp_to_gpio(MFP_PIN_GPIO16);
	gpio_direction_output(the_gpio, 1);
	gpio_free(the_gpio);
#if CONFIG_MACH_LC6830_BD_VERSION < 20
	//AP_PWM_KBLT, LED backlight Enable
	the_gpio = mfp_to_gpio(MFP_PIN_GPIO41);
	gpio_direction_output(the_gpio, 0);
	gpio_free(the_gpio);
#endif
#if CONFIG_MACH_LC6830_BD_VERSION >= 21
	//AP_PWM_KBLT, LED backlight Enable
	the_gpio = mfp_to_gpio(MFP_PIN_GPIO41);
	gpio_direction_input(the_gpio);
	gpio_free(the_gpio);
#endif
	//SPK_SHDN, Speaker amplifier
	the_gpio = mfp_to_gpio(MFP_PIN_GPIO122);
	gpio_direction_output(the_gpio, 0);
	gpio_free(the_gpio);

	//USBHOST_5V_PWREN, USB Host power supply
	the_gpio = mfp_to_gpio(MFP_PIN_GPIO175);
	gpio_direction_output(the_gpio, 0);
	gpio_free(the_gpio);

	//LCD backlight
	the_gpio = mfp_to_gpio(MFP_PIN_GPIO176);
	gpio_direction_output(the_gpio, 0);
	gpio_free(the_gpio);

	// AP2TUSB_OE
	the_gpio = mfp_to_gpio(MFP_PIN_GPIO49);
	gpio_direction_output(the_gpio, 1);
	gpio_free(the_gpio);

	// AP2TUSB_SUSPEND
	the_gpio = mfp_to_gpio(MFP_PIN_GPIO84);
	gpio_direction_output(the_gpio, 1);
	gpio_free(the_gpio);
}
#endif
/* Added by Leadcore Songlixin on: Tue Dec 15 23:34:14 PST 2009 END*/

static void saar_set_lowpower_mfp(void)
{
	/* It will change mfp value set by OBM to save more power,
	 * it should be merged into OBM mfp setting eventually. */

	/* these values have been merged int OBM
	 * pxa3xx_mfp_write(48, 0xa440);
	 * pxa3xx_mfp_write(49, 0xa440);
	 * pxa3xx_mfp_write(51, 0xa440);
	 */
}

/********add by zhoujing********/
#if defined(CONFIG_SIRF_GSD3TW)
static struct sirf_platform_data lc6830_sirf_platform_data = {
	.reset_pin = mfp_to_gpio(MFP_CFG_PIN(MFP_AGPS_RST)),
	.power_pin = mfp_to_gpio(MFP_CFG_PIN(MFP_AGPS_PWR)),
	.on_off_pin = mfp_to_gpio(MFP_CFG_PIN(MFP_AGPS_PM)),
};

static struct platform_device lc6830_sirf_device = {
	.name = "sirf",
	.id = -1,
	.dev = {
		.platform_data = &lc6830_sirf_platform_data,
		},
};

static void __init lc6830_gps_sirf_init(void)
{
	if (gpio_request(mfp_to_gpio(MFP_CFG_PIN(MFP_AGPS_RST)), "gps reset")) {
		printk(KERN_ERR "<SiRF> GPIO RST request error!!\n");
	}
	if (gpio_request(mfp_to_gpio(MFP_CFG_PIN(MFP_AGPS_PWR)), "agps power")) {
		printk(KERN_ERR "<SiRF> GPIO POWER request error!!\n");
	}
	if (gpio_request
	    (mfp_to_gpio(MFP_CFG_PIN(MFP_AGPS_PM)), "gps pm status")) {
		printk(KERN_ERR "<SiRF> GPIO PM STATUS request error!!\n");
	}
	if (gpio_request(mfp_to_gpio(MFP_CFG_PIN(MFP_AGPS_FREQ)), "gps freq")) {
		printk(KERN_ERR "<SiRF> GPIO FREQ request error!!\n");
	}
	if (gpio_request(mfp_to_gpio(MFP_CFG_PIN(MFP_AGPS_TSYNC)), "gps tsync")) {
		printk(KERN_ERR "<SiRF> GPIO TSYNC request error!!\n");
	}
	gpio_direction_output(mfp_to_gpio(MFP_CFG_PIN(MFP_AGPS_PWR)), 0);	/* GPS_POWER */
	gpio_direction_output(mfp_to_gpio(MFP_CFG_PIN(MFP_AGPS_RST)), 0);	/* GPS_POWER */
	gpio_direction_output(mfp_to_gpio(MFP_CFG_PIN(MFP_AGPS_PM)), 0);	/* GPS_POWER */
	gpio_direction_output(mfp_to_gpio(MFP_CFG_PIN(MFP_AGPS_FREQ)), 0);	/* GPS_POWER */
	gpio_direction_output(mfp_to_gpio(MFP_CFG_PIN(MFP_AGPS_TSYNC)), 0);	/* GPS_POWER */

	platform_device_register(&lc6830_sirf_device);
}
#endif

#ifdef CONFIG_BOOT_INFO
void setup_boot_info(void)
{
	unsigned long bootup_reason = 0;
	unsigned long r;
	unsigned char value;
#if 0
	r = RTSR;
	/* bit 0: RTC Alarm detected
	 * bit 4: Wristwatch Alarm 1 detectd
	 * bit 6: Writewatch Alarm 2 detected
	 */
	if (r & 0x51) {
		bootup_reason = PU_REASON_RTC_ALARM;
		RTSR = 0x51;
	} else {
#else
	r = PWSR;
	if(r & 0x80000000){
		bootup_reason = PU_REASON_RTC_ALARM;
	}else{
#endif
		r = micco_read(MICCO_STATUS_B, &value);
		if (value & MICCO_STATUS_B_USBDEV)
			bootup_reason = PU_REASON_USB_CHARGER;
		else {
			r = ARSR;
			if (r & 0x1) {
				bootup_reason = PU_REASON_HARDWARE_RESET;
			} else if (r & 0x2)
				bootup_reason = PU_REASON_WATCHDOG;
			else if (r & 0x4)
				bootup_reason = PU_REASON_PWR_KEY_PRESS;
			else if (r & 0x8) {
				/*GPIO Reset, reboot, get reboot reason from PSPR */
				bootup_reason = PSPR + PU_REASON_REBOOT_NORMAL;
			} else {
				panic("no bootup reason got!");
			}
		}
	}
	set_boot_info(bootup_reason);
}
#endif

void request_usb_vcc(int hostordevice, int enable)
{
	static int usb_vcc_status = 1;
	usb_vcc_status = (usb_vcc_status & (~(hostordevice))) | (enable << (hostordevice - 1));
/*	if(usb_vcc_status) 
		pxa3xx_pmic_enable_voltage(VCC_USB, 1);
	else
		pxa3xx_pmic_enable_voltage(VCC_USB, 0);
*/		
	printk("usb_vcc_status %d\n", usb_vcc_status);	
	return;	
}

extern struct pxamci_host mmc_host_global;
//extern void mmc_rescan(struct work_struct *work);
extern void wlan_mmc_rescan(void *wlan_host);

void bcm_init(void)
{
	int a2w_wake = mfp_to_gpio(MFP_PIN_GPIO170);
	int a2w_reset = mfp_to_gpio(MFP_PIN_GPIO171);
	int w2a_wake = mfp_to_gpio(MFP_PIN_GPIO172);

	broadcom_module_enter();
	pxa3xx_mfp_config(ARRAY_AND_SIZE(wlan_reset_high_mfp_cfg));
	if (gpio_request(a2w_wake, "Wlan a2w wake"))
		goto err;

	if (gpio_request(a2w_reset, "Wlan reset"))
		goto err2;

	if (gpio_request(w2a_wake, "Wlan w2a wake"))
		goto err3;

	gpio_direction_output(a2w_wake, 1);		/* AP wakeup BCM Wlan chip */
	gpio_direction_input(w2a_wake);			/* BCM wakeup AP */

	gpio_direction_output(a2w_reset, 0);	/* Reset BCM */
	mdelay(100);
	gpio_direction_output(a2w_reset, 1);
	mdelay(400);                            /* Enable WIFI firmwre run */
	
	wlan_mmc_rescan(mmc_host_global.mmc);
	
	gpio_free(w2a_wake);
err3:
	gpio_free(a2w_reset);
err2:
	gpio_free(a2w_wake);
err:
	return;
}

void bcm_exit(void)
{
	int a2w_reset;

	a2w_reset = mfp_to_gpio(MFP_PIN_GPIO171);
	if (gpio_request(a2w_reset, "Wlan reset"))
		return;

	gpio_direction_output(a2w_reset, 0);	/* Reset BCM */
	gpio_free(a2w_reset);

	pxa3xx_mfp_config(ARRAY_AND_SIZE(wlan_reset_low_mfp_cfg));

	broadcom_module_exit();
	wlan_mmc_rescan(mmc_host_global.mmc);

	return;
}
EXPORT_SYMBOL(bcm_init);
EXPORT_SYMBOL(bcm_exit);

static void wlan_bt_init(void)
{
    int powerdown = mfp_to_gpio(MFP_PIN_GPIO174);
	int a2w_reset = mfp_to_gpio(MFP_PIN_GPIO171);
	int a2w_wake  = mfp_to_gpio(MFP_PIN_GPIO170);
	int w2a_wake  = mfp_to_gpio(MFP_PIN_GPIO172);

	int a2b_reset = mfp_to_gpio(MFP_PIN_GPIO98);
	int a2b_wake  = mfp_to_gpio(MFP_PIN_GPIO97);
	int b2a_wake  = mfp_to_gpio(MFP_PIN_GPIO99);

	gpio_request(powerdown, "Wlan/bt powerdown");
	gpio_direction_output(powerdown, 0);	/* Enable powerdown */
	gpio_free(powerdown);

	gpio_request(a2w_reset, "Wlan Reset");
	gpio_direction_output(a2w_reset, 0);	/* Reset WLAN chip */
	gpio_free(a2w_reset);

	gpio_request(a2b_reset, "Bluetooth Reset");
	gpio_direction_output(a2b_reset, 0);	/* Reset Bluetooth chip */
	gpio_free(a2b_reset);

	gpio_request(a2b_wake, "Bluetooth A2B wake");
	gpio_direction_output(a2b_wake, 1);	
	gpio_free(a2b_wake);

	gpio_request(b2a_wake, "Bluetooth B2A wake");
	gpio_direction_input(b2a_wake);
	gpio_free(b2a_wake);

	gpio_request(a2w_wake, "Wlan A2W wake");
	gpio_direction_output(a2w_wake, 0);	
	gpio_free(a2w_wake);

	gpio_request(w2a_wake, "Wlan W2A wake");
	gpio_direction_input(w2a_wake);
	gpio_free(w2a_wake);

	return;
}

static void __init saar_init(void)
{
	int gpio_stuartoe = mfp_to_gpio(MFP_PIN_GPIO16);

#ifdef CONFIG_MACH_LC6830_PIN_MUX
	/* initialize MFP configurations */
	pxa3xx_mfp_config(ARRAY_AND_SIZE(saar_mfp_cfg));
#endif
/* Added by Leadcore Songlixin on: Tue Dec 15 23:36:08 PST 2009 BEGIN*/
#if 1
	lc6830_gpio_config();
#endif
/* Added by Leadcore Songlixin on: Tue Dec 15 23:36:08 PST 2009 END*/

	/* register i2c device */
	pxa_set_i2c_info(&pxa3xx_i2c_platform_data);

	/* update ISRAM configration for pxa935 */
	pxa3xx_device_imm.resource = pxa935_resource_imm;

#if defined(CONFIG_PXA_IRDA) || defined(CONFIG_PXA_IRDA_MODULE)
	saar_ficp_platform_data.gpio_ir_shdn = mfp_to_gpio(MFP_PIN_GPIO62);
	saar_ficp_platform_data.uart_irq = IRQ_FFUART;
	saar_ficp_platform_data.uart_reg_base = __PREG(FFUART);
	saar_ficp_platform_data.p_dev = &pxa_device_ffuart;
#endif

	/* dvfm device */
	set_pxa3xx_freq_info(&saar_freq_mach_info);

	/* performance monitor unit */
	pxa3xx_set_pmu_info(NULL);

	/* lcd */
	lc6830_init_lcd();

	platform_add_devices(devices, ARRAY_SIZE(devices));

	/*Enable STUART OE for modem */
	gpio_direction_output(gpio_stuartoe, 1);

	if (is_uart_gpio()) {
		pxa_set_ffuart_info(&saar_ffuart_info);
		pxa_set_stuart_info(&saar_stuart_info);
	} else {
		pxa_set_ffuart_info(NULL);
		pxa_set_stuart_info(NULL);
	}
	pxa_set_btuart_info(&saar_btuart_info);

#if defined(CONFIG_KEYBOARD_PXA3xx) || defined(CONFIG_KEYBOARD_PXA3xx_MODULE)
	pxa_set_keypad_info(&saar_keypad_info);
#endif
	/* trackball */
	saar_init_trackball();

	/* nand */
	saar_init_nand();

#ifdef CONFIG_ISPT
	saar_init_ispt();	//this will be done by a user script
#endif

#if defined(CONFIG_USB_PXA3XX_U2D) || defined(CONFIG_USB_USB_PXA3XX_U2D_MODULE)
	/* u2d */
	if (device_is_enabled(device_u2d)) {
		gpio_free(ap2bt_rts);
#ifdef CONFIG_MACH_LC6830_PIN_MUX
		pxa3xx_mfp_config(ARRAY_AND_SIZE(saar_u2d_cfg));
#endif
		pxa_set_u2d_info(&saar_u2d_info);
	}
#endif

	/* usb otg */
	saar_init_otg();

#if defined(CONFIG_USB_PXA9XX_U2O) || defined(CONFIG_USB_USB_PXA9XX_U2O_MODULE)
	/* u2o */
	if (device_is_enabled(device_u2o)) {
		pxa_set_u2o_info(pxa9xx_u2o_resources, &saar_u2o_info);
	}
#endif

#ifdef CONFIG_USB_EHCI_PXA935_HOSTONLY
	pxa_set_u2h_info(pxa9xx_u2h_resources, &saar_u2h_info);
#endif

#if defined(CONFIG_PXA_CAMERA)
	/* initialize camera */
	saar_init_cam();
#endif
	/* i2c devices */
	i2c_register_board_info(0, ARRAY_AND_SIZE(saar_i2c_board_info));

#if defined(CONFIG_PXA_IRDA) || defined(CONFIG_PXA_IRDA_MODULE)
	pxa_set_ficp_info(&saar_ficp_platform_data);
#endif
	saar_init_mmc();

	saar_init_ehci();

#if defined(CONFIG_SIRF_GSD3TW)	//add by zhoujing
	lc6830_gps_sirf_init();
#endif

	wlan_bt_init();

#ifdef CONFIG_INNODEV_V2
	inno_if202_init();
#endif

#ifdef CONFIG_PM
	pxa3xx_wakeup_register(&wakeup_ops);
#endif
	saar_set_lowpower_mfp();
}

MACHINE_START(LC6830, "lc6830")
	.phys_io = 0x40000000,
	.boot_params = 0xa0000100,
	.io_pg_offst = (io_p2v(0x40000000) >> 18) & 0xfffc,
	.map_io = pxa_map_io,
	.init_irq = pxa3xx_init_irq,
	.timer = &pxa_timer,
	.init_machine = saar_init,
MACHINE_END
