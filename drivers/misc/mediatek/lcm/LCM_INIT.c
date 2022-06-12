#define LOG_TAG "LCM"

#ifndef BUILD_LK
#include <linux/string.h>
#include <linux/kernel.h>
#endif
#include "lcm_drv.h"
#include "lcm_define.h"

#ifndef BUILD_LK
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
/* #include <linux/jiffies.h> */
/* #include <linux/delay.h> */
#include <linux/uaccess.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#endif

#ifdef BUILD_LK
#include <platform/upmu_common.h>
#include <platform/mt_gpio.h>
#include <platform/mt_i2c.h>
#include <platform/mt_pmic.h>
#include <string.h>
#elif defined(BUILD_UBOOT)
#include <asm/arch/mt_gpio.h>
#else
#include "disp_dts_gpio.h"
#include <linux/gpio.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/regulator/consumer.h>
#include <linux/clk.h>
#include <linux/pinctrl/consumer.h>
#include <linux/of_gpio.h>
#endif
#endif

#ifndef MACH_FPGA
#include <lcm_pmic.h>
#endif

unsigned int GPIO_LCD_BL_EN_PIN;
unsigned int GPIO_LCD_BIAS_ENP;
unsigned int GPIO_LCD_BIAS_ENN;
unsigned int GPIO_TP_RST;

void lcm_request_gpio_control (struct device *dev)
{
	pr_notice("[Kernel/LCM] %s enter\n", __func__);

	GPIO_LCD_BL_EN_PIN = of_get_named_gpio(dev->of_node, "gpio_lcd_bl_en", 0);
	gpio_request(GPIO_LCD_BL_EN_PIN, "GPIO_LCD_BL_EN_PIN");

	GPIO_LCD_BIAS_ENP = of_get_named_gpio(dev->of_node,"gpio_lcd_bias_enp", 0);
	gpio_request(GPIO_LCD_BIAS_ENP, "GPIO_LCD_BIAS_ENP");
	GPIO_LCD_BIAS_ENN = of_get_named_gpio(dev->of_node,"gpio_lcd_bias_enn", 0);
	gpio_request(GPIO_LCD_BIAS_ENN, "GPIO_LCD_BIAS_ENN");
	GPIO_TP_RST = of_get_named_gpio(dev->of_node,"gpio_tp_rst", 0);
	gpio_request(GPIO_TP_RST, "GPIO_TP_RST");
}

static int lcm_driver_probe (struct device *dev, void const *data)
{
	lcm_request_gpio_control (dev);

	return 0;
}

static const struct of_device_id lcm_platform_of_match [] = {
	{
		.compatible = "ft,ft8201",
		.data = 0,
	}, {
		/* sentinel */
	}
};

MODULE_DEVICE_TABLE(of, lcm_platform_of_match);

static int lcm_platform_probe (struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_node(lcm_platform_of_match, pdev->dev.of_node);
	if (!id)
		return -ENODEV;

	return lcm_driver_probe(&pdev->dev, id->data);
}

static struct platform_driver lcm_driver = {
	.probe = lcm_platform_probe,
	.driver = {
		.name = "LCM_INIT",
		.owner = THIS_MODULE,
		.of_match_table = lcm_platform_of_match,
	},
};

static int __init lcm_drv_init(void)
{
	pr_notice("[Kernel/LCM] %s enter__lsy\n", __func__);		
	if (platform_driver_register(&lcm_driver)) {
		pr_notice("LCM: failed to register disp driver\n");
		return -ENODEV;
	}

	return 0;
}

static void __exit lcm_drv_exit(void)
{
	platform_driver_unregister(&lcm_driver);
	pr_notice("LCM: Unregister lcm driver done\n");
}

late_initcall(lcm_drv_init);
module_exit(lcm_drv_exit);
MODULE_AUTHOR("mediatek");
MODULE_DESCRIPTION("Display subsystem Driver");
MODULE_LICENSE("GPL");

