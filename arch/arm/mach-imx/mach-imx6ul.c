/*
 * Copyright (C) 2015-2016 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#include <linux/irqchip.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/micrel_phy.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <asm/system_misc.h>
#include <linux/of_gpio.h>


#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

	#define BT_WAKE IMX_GPIO_NR(4, 17)   /* BT_WAKE */
	#define BT_RESET_N IMX_GPIO_NR(1, 9) /* BT_RST# */
	#define WLAN_REG_EN IMX_GPIO_NR(1, 5) /* WL_REG_ON */
    #define UART_EN1 IMX_GPIO_NR(2, 12) /* UART_EN1*/
    #define UART_EN2 IMX_GPIO_NR(2, 13) /* UART_EN2*/ 
	#define ENET_RST IMX_GPIO_NR(1, 3) /*ENET_RST#*/

static inline void DMS_SE25_init(void){
	/*AP6255 Bluetooth power sequece*/

  
	pr_warn(">>>>> %s, %s: +%d()++\n", __FILE__, __FUNCTION__, __LINE__);
	/*
	struct device_node *np = NULL;
	np = of_find_node_by_name(NULL, "pin_enable_ctrl");
	if (!np)
		return;
	dmssr09_pin_enable(np, "en-io-gpio");
	dmssr09_pin_enable(np, "en-3v3-gpio");
	*/

	//WLAN_REG_EN
	//printk(">>>>> set gpio : WLAN_REG_EN => 0 \n");
	pr_info(">>>>> set gpio : WLAN_REG_EN[%d]=> 1 \n",WLAN_REG_EN);
	gpio_request(WLAN_REG_EN, "WLAN REG EN");
	gpio_direction_output(WLAN_REG_EN, 0);
	msleep(1);
	//printk(">>>>> set gpio : WLAN_REG_EN => 1 \n");
	gpio_direction_output(WLAN_REG_EN, 1);
	gpio_free(WLAN_REG_EN);

	//BT_WAKE to Low
    pr_info(">>>>> set gpio : BT_WAKE[%d]=> 1 \n",BT_WAKE);
	gpio_request(BT_WAKE, "BT WAKE");
	gpio_direction_output(BT_WAKE, 1);

	//BT_RESET_N
    pr_info(">>>>> set gpio : BT_RESET_N[%d]=> 1 \n",BT_RESET_N);
	gpio_request(BT_RESET_N, "BT RESET N");
	gpio_direction_output(BT_RESET_N, 0);
	msleep(5);
    //printk(">>>>> set gpio : BT_RESET_N => 1 \n");
	gpio_direction_output(BT_RESET_N, 1);	

	//ENET_Rest_N
    pr_info(">>>>> set gpio : ENET_RST[%d]=> 1 \n",ENET_RST);
	gpio_request(ENET_RST, "ENET Rest N");
	gpio_direction_output(ENET_RST, 0);
	msleep(5);
    gpio_direction_output(ENET_RST, 1);

    //UART_EN1 to High
    pr_info(">>>>> set gpio : UART_EN1[%d]=> 0 \n",UART_EN1);
	gpio_request(UART_EN1, "UART EN1");
	gpio_direction_output(UART_EN1, 1);		
	gpio_free(UART_EN1);

    //UART_EN2 to High
    pr_info(">>>>> set gpio : UART_EN2[%d]=> 0 \n",UART_EN2);
	gpio_request(UART_EN2, "UART EN2");
	gpio_direction_output(UART_EN2, 1);	
	gpio_free(UART_EN2);	

    pr_warn(">>>>> %s, %s: +%d()--\n", __FILE__, __FUNCTION__, __LINE__);
}

static void __init imx6ul_enet_clk_init(void)
{
	struct regmap *gpr;

	gpr = syscon_regmap_lookup_by_compatible("fsl,imx6ul-iomuxc-gpr");
	if (!IS_ERR(gpr))
		regmap_update_bits(gpr, IOMUXC_GPR1, IMX6UL_GPR1_ENET_CLK_DIR,
				   IMX6UL_GPR1_ENET_CLK_OUTPUT);
	else
		pr_err("failed to find fsl,imx6ul-iomux-gpr regmap\n");

}

static int ksz8081_phy_fixup(struct phy_device *dev)
{
	if (dev && dev->interface == PHY_INTERFACE_MODE_MII) {
		phy_write(dev, 0x1f, 0x8110);
		phy_write(dev, 0x16, 0x201);
	} else if (dev && dev->interface == PHY_INTERFACE_MODE_RMII) {
		phy_write(dev, 0x1f, 0x8190);
		phy_write(dev, 0x16, 0x202);
	}

	return 0;
}

static void __init imx6ul_enet_phy_init(void)
{
	if (IS_BUILTIN(CONFIG_PHYLIB)) {
		/*
		 * i.MX6UL EVK board RevA, RevB, RevC all use KSZ8081
		 * Silicon revision 00, the PHY ID is 0x00221560, pass our
		 * test with the phy fixup.
		 */
		phy_register_fixup(PHY_ANY_ID, PHY_ID_KSZ8081, 0xffffffff,
				   ksz8081_phy_fixup);

		/*
		 * i.MX6UL EVK board RevC1 board use KSZ8081
		 * Silicon revision 01, the PHY ID is 0x00221561.
		 * This silicon revision still need the phy fixup setting.
		 */
		#define PHY_ID_KSZ8081_MNRN61	0x00221561
		phy_register_fixup(PHY_ANY_ID, PHY_ID_KSZ8081_MNRN61,
				   0xffffffff, ksz8081_phy_fixup);
	}
}

#define OCOTP_CFG3			0x440
#define OCOTP_CFG3_SPEED_SHIFT		16
#define OCOTP_CFG3_SPEED_696MHZ		0x2
#define OCOTP_CFG3_SPEED_900MHZ		0x3

static void __init imx6ul_opp_check_speed_grading(struct device *cpu_dev)
{
	struct device_node *np;
	void __iomem *base;
	u32 val;

	if (cpu_is_imx6ul())
		np = of_find_compatible_node(NULL, NULL, "fsl,imx6ul-ocotp");
	else
		np = of_find_compatible_node(NULL, NULL, "fsl,imx6ull-ocotp");

	if (!np) {
		pr_warn("failed to find ocotp node\n");
		return;
	}

	base = of_iomap(np, 0);
	if (!base) {
		pr_warn("failed to map ocotp\n");
		goto put_node;
	}

	/*
	 * Speed GRADING[1:0] defines the max speed of ARM:
	 * 2b'00: Reserved;
	 * 2b'01: 528000000Hz;
	 * 2b'10: 700000000Hz(i.MX6UL), 800000000Hz(i.MX6ULL);
	 * 2b'11: 900000000Hz(i.MX6ULL);
	 * We need to set the max speed of ARM according to fuse map.
	 */
	val = readl_relaxed(base + OCOTP_CFG3);
	val >>= OCOTP_CFG3_SPEED_SHIFT;
	val &= 0x3;
	if (cpu_is_imx6ul()) {
		if (val < OCOTP_CFG3_SPEED_696MHZ) {
			if (dev_pm_opp_disable(cpu_dev, 696000000))
				pr_warn("Failed to disable 696MHz OPP\n");
		}
	}

	if (cpu_is_imx6ull()) {
		if (val != OCOTP_CFG3_SPEED_696MHZ) {
			if (dev_pm_opp_disable(cpu_dev, 792000000))
				pr_warn("Failed to disable 792MHz OPP\n");
		}

		if (val != OCOTP_CFG3_SPEED_900MHZ) {
			if(dev_pm_opp_disable(cpu_dev, 900000000))
				pr_warn("Failed to disable 900MHz OPP\n");
		}
	}
	iounmap(base);

put_node:
	of_node_put(np);
}

static void __init imx6ul_opp_init(void)
{
	struct device_node *np;
	struct device *cpu_dev = get_cpu_device(0);

	if (!cpu_dev) {
		pr_warn("failed to get cpu0 device\n");
		return;
	}
	np = of_node_get(cpu_dev->of_node);
	if (!np) {
		pr_warn("failed to find cpu0 node\n");
		return;
	}

	if (dev_pm_opp_of_add_table(cpu_dev)) {
		pr_warn("failed to init OPP table\n");
		goto put_node;
	}

	imx6ul_opp_check_speed_grading(cpu_dev);

put_node:
	of_node_put(np);
}

static inline void imx6ul_enet_init(void)
{
	imx6ul_enet_clk_init();
	imx6ul_enet_phy_init();
	if (cpu_is_imx6ul())
		imx6_enet_mac_init("fsl,imx6ul-fec", "fsl,imx6ul-ocotp");
	else
		imx6_enet_mac_init("fsl,imx6ul-fec", "fsl,imx6ull-ocotp");
}



static void __init imx6ul_init_machine(void)
{
	struct device *parent;

	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

	of_platform_default_populate(NULL, NULL, parent);
	imx6ul_enet_init();
	imx_anatop_init();
	imx6ul_pm_init();

	

}

static void __init imx6ul_init_irq(void)
{
	imx_gpc_check_dt();
	imx_init_revision_from_anatop();
	imx_src_init();
	irqchip_init();
	imx6_pm_ccm_init("fsl,imx6ul-ccm");
}

static void __init imx6ul_init_late(void)
{
	if (IS_ENABLED(CONFIG_ARM_IMX6Q_CPUFREQ)) {
		imx6ul_opp_init();
		platform_device_register_simple("imx6q-cpufreq", -1, NULL, 0);
	}

	imx6ul_cpuidle_init();
	DMS_SE25_init();
}

static void __init imx6ul_map_io(void)
{
	debug_ll_io_init();
	imx6_pm_map_io();
	imx_busfreq_map_io();
}

static const char * const imx6ul_dt_compat[] __initconst = {
	"fsl,imx6ul",
	"fsl,imx6ull",
	NULL,
};

DT_MACHINE_START(IMX6UL, "Freescale i.MX6 UltraLite (Device Tree)")
	.map_io		= imx6ul_map_io,
	.init_irq	= imx6ul_init_irq,
	.init_machine	= imx6ul_init_machine,
	.init_late	= imx6ul_init_late,
	.dt_compat	= imx6ul_dt_compat,
MACHINE_END
