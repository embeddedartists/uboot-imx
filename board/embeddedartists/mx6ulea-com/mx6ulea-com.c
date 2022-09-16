/*
 * Copyright (C) 2015 Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <init.h>
#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/crm_regs.h>
#if defined(MX6UL)
#include <asm/arch/mx6ul_pins.h>
#elif defined(MX6ULL)
#include <asm/arch/mx6ull_pins.h>
#endif
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/mach-imx/boot_mode.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <asm/io.h>
#include <common.h>
#include <env.h>
#include <fsl_esdhc_imx.h>
#include <i2c.h>
#include <linux/sizes.h>
#include <linux/fb.h>
#include <miiphy.h>
#include <mmc.h>
#include <mxsfb.h>
#include <netdev.h>
#include <usb.h>
#include <usb/ehci-ci.h>
#include <power/pmic.h>
#include <power/pfuze3000_pmic.h>

#include <dm.h>
#include <fdt_support.h>
#include <linux/delay.h>

#include "../common/ea_eeprom.h"
#include "../common/ea_common.h"
#include "../common/ea_gpio_expander.h"

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
	PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define PERI_PWR_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_DSE_40ohm)

iomux_v3_cfg_t const peri_pwr_pads[] = {
	(MX6_PAD_SNVS_TAMPER2__GPIO5_IO02 | MUX_PAD_CTRL(PERI_PWR_PAD_CTRL)),
};

int dram_init(void)
{
	gd->ram_size = imx_ddr_size();

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_UART1_TX_DATA__UART1_DCE_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_UART1_RX_DATA__UART1_DCE_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

#ifdef CONFIG_FEC_MXC
/*
 * pin conflicts for fec1 and fec2, GPIO1_IO06 and GPIO1_IO07 can only
 * be used for ENET1 or ENET2, cannot be used for both.
 */

static iomux_v3_cfg_t const enet_pwr_pads[] = {
	/* enet pwr en */
	MX6_PAD_SNVS_TAMPER3__GPIO5_IO03 | MUX_PAD_CTRL(NO_PAD_CTRL),
};


#endif

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FSL_ESDHC

int mmc_map_to_kernel_blk(int dev_no)
{
	/* eMMC device is available at mmcblk1 in Linux */
	return 1;
}

int board_mmc_get_env_dev(int devno)
{
	int no = devno;
	bool is_v2 = true;
	ea_config_t *ea_conf = (ea_config_t *)EA_SHARED_CONFIG_MEM;

	if (ea_conf->magic == EA_CONFIG_MAGIC) {
		is_v2 = ea_conf->is_carrier_v2;
	}

	/*
	 * This function is used to get the MMC device used for
	 * the u-boot environment. The devno argument specifies the
	 * boot device as defined in BOOT_CFG2 (see SD/eSD Boot Fusemap in
	 * NXP's User's Manual).
	 *
	 * For the iMX6 Ultralite COM board eMMC is on USDHC2 which gives devno=1.
	 *
	 * When using the Device Module (dts) USDHC devices are added
	 * in the order they are defined in the dts file. Depending on
	 * which carrier board being used USDHC1 might also be enabled
	 * (in board_fix_fdt)
	 * This however effects which device number being assigned to eMMC.
	 * When USDHC1 is enabled eMMC will be on mmc1 (since USDHC1
	 * is on mmc0). When USDHC1 isn't enabled eMMC will be on mmc0.
	 */

	if (is_v2) {
		no = 0;
	}
	else {
		no = 1;
	}

	return no;

}

#endif

#ifdef CONFIG_FEC_MXC

static int setup_fec(void)
{
	struct iomuxc *const iomuxc_regs = (struct iomuxc *)IOMUXC_BASE_ADDR;
	int ret;

	/*
	 * Use 50M anatop loopback REF_CLK1 for ENET1,
	 * clear gpr1[13], set gpr1[17]
	 */
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC1_MASK,
			IOMUX_GPR1_FEC1_CLOCK_MUX1_SEL_MASK);
	clrsetbits_le32(&iomuxc_regs->gpr[1], IOMUX_GPR1_FEC2_MASK,
			IOMUX_GPR1_FEC2_CLOCK_MUX1_SEL_MASK);

	ret = enable_fec_anatop_clock(0, ENET_50MHZ);
	if (ret)
		return ret;


	if (!check_module_fused(MODULE_ENET2)) {
		ret = enable_fec_anatop_clock(1, ENET_50MHZ);
		if (ret)
			return ret;
	}

	enable_enet_clk(1);

	imx_iomux_v3_setup_multiple_pads(enet_pwr_pads, ARRAY_SIZE(enet_pwr_pads));

	/* enet pwr en */
	gpio_request(IMX_GPIO_NR(5, 3), "enet pwr");
	gpio_direction_output(IMX_GPIO_NR(5, 3) , 0);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{

	phy_write(phydev, MDIO_DEVAD_NONE, 0x1f, 0x8190);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x16, 0x202);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}

#endif

int power_init_board(void)
{
	struct udevice *dev;
	int ret, dev_id, rev_id;

	ret = pmic_get("pfuze3000@8", &dev);
	if (ret == -ENODEV)
		return 0;
	if (ret != 0)
		return ret;

	dev_id = pmic_reg_read(dev, PFUZE3000_DEVICEID);
	rev_id = pmic_reg_read(dev, PFUZE3000_REVID);
	printf("PMIC: PFUZE3000 DEV_ID=0x%x REV_ID=0x%x\n", dev_id, rev_id);

	return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
	unsigned int value;
	u32 vddarm;
	struct udevice *dev;
	int ret;

	ret = pmic_get("pfuze3000@8", &dev);
	if (ret == -ENODEV) {
		printf("No PMIC found!\n");
		return;
	}

	/* switch to ldo_bypass mode */
	if (ldo_bypass) {
		prep_anatop_bypass();
		/* decrease VDDARM to 1.275V */
		value = pmic_reg_read(dev, PFUZE3000_SW1BVOLT);
		value &= ~0x1f;
		value |= PFUZE3000_SW1AB_SETP(12750);
		pmic_reg_write(dev, PFUZE3000_SW1BVOLT, value);

		set_anatop_bypass(1);
		vddarm = PFUZE3000_SW1AB_SETP(11750);

		value = pmic_reg_read(dev, PFUZE3000_SW1BVOLT);
		value &= ~0x1f;
		value |= vddarm;
		pmic_reg_write(dev, PFUZE3000_SW1BVOLT, value);

		finish_anatop_bypass();

		printf("switch to ldo_bypass mode!\n");
	}
}
#endif

int board_early_init_f(void)
{
	return 0;
}

int board_init_common(void)
{

	/* configure and enable pwr on carrier board*/
	imx_iomux_v3_setup_multiple_pads(peri_pwr_pads,
			ARRAY_SIZE(peri_pwr_pads));
	gpio_request(IMX_GPIO_NR(5, 2), "peri 3.3  pwr");
	gpio_direction_output(IMX_GPIO_NR(5, 2), 1);

#ifndef CONFIG_EA_NO_UART_FLUSH
	/*
	 * Empty UART RX FIFO 5ms after PERI_PWR_ENABLE goes high.
	 *
	 * Needed to avoid u-boot to stop booting (boot delay) due to
	 * data being available in the uart buffer. This problem is
	 * available on COM Carrier board rev A to rev D. The problem
	 * doesn't exist on rev E (also known as COM Carrier Board V2)
	 */
        udelay(5000);
        while (tstc()) {
                (void)getchar();
        }
#endif

	setup_iomux_uart();

	return 0;
}

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

	board_init_common();

#ifdef	CONFIG_FEC_MXC
	setup_fec();
#endif

	ea_print_board();

	return 0;
}

int board_late_init(void)
{

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_env_init();
#endif

#ifdef CONFIG_FEC_MXC
	/*
	 * Loading ethernet addresses must be done in late_init
	 * since they update the environment (env_set). The
	 * environment isn't loaded and ready at board_init.
	 */
	if (ea_load_ethaddr()) {
		printf("Failed to load MAC addresses\n");
	}
#endif

#ifdef CONFIG_SYS_I2C_MXC
	ea_gpio_exp_configure(1);
#endif
	set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);

	ea_board_info_to_env();

	return 0;
}


int board_fix_fdt(void* rw_fdt_blob)
{
	int ret = 0;

	bool is_v2 = true;
	ea_config_t *ea_conf = (ea_config_t *)EA_SHARED_CONFIG_MEM;

	if (ea_conf->magic == EA_CONFIG_MAGIC) {
		is_v2 = ea_conf->is_carrier_v2;
	}

	/*
	 * On Carrier board V2 there isn't any SD card interface available
	 * since this interface is connected to M.2 connector.
	 * On V1 boards USDHC1 is used.
	 */
	if (!is_v2) {
		ret = fdt_status_okay_by_alias(rw_fdt_blob, "usdhc1");
	}

	if (ret) {
		printf("%s: failed to enable usdhc node, ret=%d\n", __func__, ret);
	}

	return ret;
}
