/*
 * Copyright (c) 2020 Trunexa Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3
 * as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; If not, see <http://www.gnu.org/licenses/>.
 */

#include <linux/can/platform/flexcan.h>
#include <linux/fec.h>
#include <linux/gpio.h>
#include <linux/irqchip.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/imx6q-iomuxc-gpr.h>
#include <linux/netdevice.h>
#include <linux/of_address.h>
#include <linux/of_gpio.h>
#include <linux/of_platform.h>
#include <linux/phy.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>
#include <asm/mach/arch.h>
#include <asm/mach/map.h>
#include <linux/micrel_phy.h>

#include "common.h"
#include "cpuidle.h"
#include "hardware.h"

#define HW_OCOTP_CFGn(n)                        (0x00000410 + (n) * 0x10)
#define BSP_VERSION             "TRUX-iMX6UL-Q01-Linux4.1.15-V.1.0.0"

static struct flexcan_platform_data flexcan_pdata;
static int flexcan0_en_gpio;
/*
 * @file mach-imx6sm.c
 *
 * @brief Enet clk enable
 *
 * @ingroup ENET
 */
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

/*
 * @file mach-imx6sm.c
 *
 * @brief Enet phy fixup
 *
 * @ingroup ENET
 */
static int ksz8081_phy_fixup(struct phy_device *dev)
{
	if (dev && dev->interface == PHY_INTERFACE_MODE_MII) {
		phy_write(dev, 0x1f, 0x8110);
		phy_write(dev, 0x16, 0x201);
	} else if (dev && dev->interface == PHY_INTERFACE_MODE_RMII) {
		phy_write(dev, 0x1f, 0x0190);
		phy_write(dev, 0x16, 0x202);
	}

	return 0;
}

/*
 * i.MX6UL EVK board RevA, RevB, RevC all use KSZ8081
 * Silicon revision 00, the PHY ID is 0x00221560, pass our
 * test with the phy fixup.
 */
#define PHY_ID_KSZ8081_MNRN60	0x00221560
/*
 * i.MX6UL EVK board RevC1 board use KSZ8081
 * Silicon revision 01, the PHY ID is 0x00221561.
 * This silicon revision still need the phy fixup setting.
 */
#define PHY_ID_KSZ8081_MNRN61	0x00221561
static void __init imx6ul_enet_phy_init(void)
{
	phy_register_fixup(PHY_ANY_ID, PHY_ID_KSZ8081_MNRN60, 0xffffffff, ksz8081_phy_fixup);
	phy_register_fixup(PHY_ANY_ID, PHY_ID_KSZ8081_MNRN61, 0xffffffff, ksz8081_phy_fixup);
}

#define OCOTP_CFG3			0x440
#define OCOTP_CFG3_SPEED_SHIFT		16
#define OCOTP_CFG3_SPEED_696MHZ		0x2

static void __init imx6ul_opp_check_speed_grading(struct device *cpu_dev)
{
	struct device_node *np;
	void __iomem *base;
	u32 val;

	np = of_find_compatible_node(NULL, NULL, "fsl,imx6ul-ocotp");
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
	 * 2b'10: 700000000Hz;
	 * 2b'11: Reserved;
	 * We need to set the max speed of ARM according to fuse map.
	 */
	val = readl_relaxed(base + OCOTP_CFG3);
	val >>= OCOTP_CFG3_SPEED_SHIFT;
	val &= 0x3;

	if (val != OCOTP_CFG3_SPEED_696MHZ) {
		if (dev_pm_opp_disable(cpu_dev, 696000000))
			pr_warn("Failed to disable 696MHz OPP\n");
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

	if (of_init_opp_table(cpu_dev)) {
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

static void imx6ul_flexcan0_switch(int enable)
{
        if (enable)
        /* Active low enables the CAN tranceiver */
                gpio_set_value_cansleep(flexcan0_en_gpio, 0);
        else
                gpio_set_value_cansleep(flexcan0_en_gpio, 1);
}

static int __init imx6ul_flexcan_fixup(void)
{
        struct device_node *np;
        enum of_gpio_flags en_flags;

        np = of_find_node_by_path("/soc/aips-bus@02000000/can@02090000");
        if (!np)
                return -ENODEV;

        flexcan0_en_gpio = of_get_named_gpio_flags(np, "trx-en-gpio", 0, &en_flags);

        if (gpio_is_valid(flexcan0_en_gpio) &&
                !gpio_request_one(flexcan0_en_gpio, GPIOF_DIR_OUT, "flexcan-trx-en")) {
                /* flexcan 0 & 1 are using the same GPIOs for transceiver */
                flexcan_pdata.transceiver_switch = imx6ul_flexcan0_switch;
        }

        return 0;
}

/*
 * @file mach-imx6sm.c
 *
 * @brief printing board information
 *
 */
static int __init print_board_info (void)
{
        struct device_node *np;
        unsigned int unique_id1, unique_id2;
        void __iomem *base;

	if (cpu_is_imx6ul())
        	np = of_find_compatible_node(NULL, NULL, "fsl,imx6ul-ocotp");
	else
        	np = of_find_compatible_node(NULL, NULL, "fsl,imx6ull-ocotp");
        if (!np) {
                pr_warn("failed to find ocotp node\n");
                return 0;
        }

        base = of_iomap(np, 0);
        if (!base) {
                pr_warn("failed to map ocotp\n");
                goto put_node;
        }

        unique_id1 = readl_relaxed(base + HW_OCOTP_CFGn(0));
        unique_id2 = readl_relaxed(base + HW_OCOTP_CFGn(1));

        printk ("\n");
        printk ("Board Info:\n");
        printk ("\tBSP Version     : %s\n", BSP_VERSION);
        printk ("\tSOM Version     : TRUX-iMX6UL-Q01\n");
        printk ("\tCPU Unique ID   : 0x%08x%08x \n", unique_id2, unique_id1);
        printk ("\n");

        iounmap(base);

	
put_node:
        of_node_put(np);
        return 0;
}

/* Add auxdata to pass platform data */
static const struct of_dev_auxdata imx6_truxq01_auxdata_lookup[] __initdata = { 
        OF_DEV_AUXDATA("fsl,imx6ul-flexcan", 0x02090000, NULL, &flexcan_pdata), 
        { /* sentinel */  }
};

static void __init imx6ul_init_machine(void)
{
	struct device *parent;

	parent = imx_soc_device_init();
	if (parent == NULL)
		pr_warn("failed to initialize soc device\n");

      	of_platform_populate(NULL, of_default_bus_match_table,
                              imx6_truxq01_auxdata_lookup, NULL);
	imx6ul_enet_init();
	imx_anatop_init();
	imx6ul_pm_init();
	print_board_info();
}

static void __init imx6ul_init_irq(void)
{
	imx_gpc_check_dt();
	imx_init_revision_from_anatop();
	imx_src_init();
	irqchip_init();
}

static void __init imx6ul_init_late(void)
{
	if (IS_ENABLED(CONFIG_ARM_IMX6Q_CPUFREQ)) {
		if (cpu_is_imx6ul())
			imx6ul_opp_init();
		platform_device_register_simple("imx6q-cpufreq", -1, NULL, 0);
	}

	imx6ul_cpuidle_init();

	imx6ul_flexcan_fixup();
}

static void __init imx6ul_map_io(void)
{
	debug_ll_io_init();
	imx6_pm_map_io();
	imx_busfreq_map_io();
}

static const char *imx6ul_dt_compat[] __initconst = {
	"trux,ul_truxq01_som",
	"trux,ull_truxq01_som",
	NULL,
};

DT_MACHINE_START(IMX6UL, "Freescale i.MX6 UL/ULL (Device Tree)")
	.map_io		= imx6ul_map_io,
	.init_irq	= imx6ul_init_irq,
	.init_machine	= imx6ul_init_machine,
	.init_late	= imx6ul_init_late,
	.dt_compat	= imx6ul_dt_compat,
MACHINE_END
