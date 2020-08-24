/*
 * Copyright 2017 NXP
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <spl.h>
#include <asm/io.h>
#include <errno.h>
#include <asm/io.h>
#include <asm/mach-imx/iomux-v3.h>
#include <asm/arch/ddr.h>
#include <asm/arch/imx8mq_pins.h>
#include <asm/arch/sys_proto.h>
#include <power/pmic.h>
#include <power/bd71837.h>
#include <asm/arch/clock.h>
#include <asm/mach-imx/gpio.h>
#include <asm/mach-imx/mxc_i2c.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <asm/arch/imx8m_ddr.h>

#include "../common/ea_common.h"
#include "../common/ea_eeprom.h"
#include "../common/ea_gpio_expander.h"

DECLARE_GLOBAL_DATA_PTR;

#define I2C_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE)
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = IMX8MQ_PAD_I2C1_SCL__I2C1_SCL | PC,
		.gpio_mode = IMX8MQ_PAD_I2C1_SCL__GPIO5_IO14 | PC,
		.gp = IMX_GPIO_NR(5, 14),
	},
	.sda = {
		.i2c_mode = IMX8MQ_PAD_I2C1_SDA__I2C1_SDA | PC,
		.gpio_mode = IMX8MQ_PAD_I2C1_SDA__GPIO5_IO15 | PC,
		.gp = IMX_GPIO_NR(5, 15),
	},
};

struct i2c_pads_info i2c_pad_info2 = {
        .scl = {
                .i2c_mode = IMX8MQ_PAD_I2C2_SCL__I2C2_SCL | PC,
                .gpio_mode = IMX8MQ_PAD_I2C2_SCL__GPIO5_IO16 | PC,
                .gp = IMX_GPIO_NR(5, 16),
        },
        .sda = {
                .i2c_mode = IMX8MQ_PAD_I2C2_SDA__I2C2_SDA | PC,
                .gpio_mode = IMX8MQ_PAD_I2C2_SDA__GPIO5_IO17 | PC,
                .gp = IMX_GPIO_NR(5, 17),
        },
};

#define USDHC_PAD_CTRL	(PAD_CTL_DSE6 | PAD_CTL_HYS | PAD_CTL_PUE | \
			 PAD_CTL_FSEL2)

static iomux_v3_cfg_t const usdhc1_pads[] = {
	IMX8MQ_PAD_SD1_CLK__USDHC1_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_CMD__USDHC1_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA0__USDHC1_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA1__USDHC1_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA2__USDHC1_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA3__USDHC1_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA4__USDHC1_DATA4 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA5__USDHC1_DATA5 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA6__USDHC1_DATA6 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_DATA7__USDHC1_DATA7 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	IMX8MQ_PAD_SD1_RESET_B__GPIO2_IO10 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#include "lpddr4_timing.c"

enum ea_ddr_field {
	EA_DDR_DDRC   = 1,
	EA_DDR_DDRPHY,
	EA_DDR_DDRPHY_TRAINED,
	EA_DDR_PHY_PIE,
	EA_DDR_FSP_INFO,
	EA_DDR_FSP0,
	EA_DDR_FSP1,
	EA_DDR_FSP2,
	EA_DDR_FSP3,
};

struct dram_fsp_msg ea_ddr_dram_fsp_msg[4] = {{1}};

#define EA_DBUF_SZ (16384)
#define EA_GZBUF_SZ (6144)
static unsigned char ea_dbuf[EA_DBUF_SZ] = {1};
static unsigned char ea_gzbuf[EA_GZBUF_SZ] = {1};

static void spl_ddr_map_array(enum ea_ddr_field idx, struct dram_cfg_param* a, int sz)
{
	switch(idx) {
	case EA_DDR_DDRC:
		dram_timing.ddrc_cfg = a;
		dram_timing.ddrc_cfg_num = sz;

		break;
	case EA_DDR_DDRPHY:
		dram_timing.ddrphy_cfg = a;
		dram_timing.ddrphy_cfg_num = sz;

		break;
	case EA_DDR_DDRPHY_TRAINED:
		dram_timing.ddrphy_trained_csr = a;
		dram_timing.ddrphy_trained_csr_num = sz;

		break;
	case EA_DDR_PHY_PIE:
		dram_timing.ddrphy_pie = a;
		dram_timing.ddrphy_pie_num = sz;

		break;
	case EA_DDR_FSP_INFO:
		/*
		 * [0].reg = size of the fsp table
		 * [1].reg = fsp_table[0]
		 * [1].val = fsp_table[1]
		 * [2].reg = fsp_table[2]
		 * [2].val = fsp_table[3]
		 */
		dram_timing.fsp_msg_num = a[0].reg;
		dram_timing.fsp_msg = ea_ddr_dram_fsp_msg;
		dram_timing.fsp_table[0] = a[1].reg;
		dram_timing.fsp_table[1] = a[1].val;
		dram_timing.fsp_table[2] = a[2].reg;
		dram_timing.fsp_table[3] = a[2].val;
		break;
	case EA_DDR_FSP0:
		/*
		 * First pair conatins drate and fw_type
		 */
		ea_ddr_dram_fsp_msg[0].drate   = a[0].reg;
		ea_ddr_dram_fsp_msg[0].fw_type = a[0].val;
		ea_ddr_dram_fsp_msg[0].fsp_cfg = &a[1];

		/* sz also contains the drate and fw_type pair -> remove one */
		ea_ddr_dram_fsp_msg[0].fsp_cfg_num = sz-1;

		break;
	case EA_DDR_FSP1:
		ea_ddr_dram_fsp_msg[1].drate   = a[0].reg;
		ea_ddr_dram_fsp_msg[1].fw_type = a[0].val;
		ea_ddr_dram_fsp_msg[1].fsp_cfg = &a[1];
		ea_ddr_dram_fsp_msg[1].fsp_cfg_num = sz-1;

		break;
	case EA_DDR_FSP2:
		ea_ddr_dram_fsp_msg[2].drate   = a[0].reg;
		ea_ddr_dram_fsp_msg[2].fw_type = a[0].val;
		ea_ddr_dram_fsp_msg[2].fsp_cfg = &a[1];
		ea_ddr_dram_fsp_msg[2].fsp_cfg_num = sz-1;

		break;
	case EA_DDR_FSP3:
		ea_ddr_dram_fsp_msg[3].drate   = a[0].reg;
		ea_ddr_dram_fsp_msg[3].fw_type = a[0].val;
		ea_ddr_dram_fsp_msg[3].fsp_cfg = &a[1];
		ea_ddr_dram_fsp_msg[3].fsp_cfg_num = sz-1;

		break;
	default:
		printf("Invalid ddr field index (%d). Invalid data in eeprom?\n", idx);
		break;
	}
}

static int spl_ddr_unpack_data(ea_eeprom_config_t* cfg)
{
	int ret;
	int offset;
	int nread=0;
	unsigned long len;
	struct dram_cfg_param* p;

	/* data_size is in this case the size of the gzipped data */
	len = cfg->data_size;

	ret = ea_eeprom_read_all_data(ea_gzbuf, EA_GZBUF_SZ, &nread);
	if (ret) {
		printf("Failed to read ddr data from eeprom %d\n", ret);
		return ret;
	}

	ret = gunzip(ea_dbuf, EA_DBUF_SZ, ea_gzbuf, &len);
	if (ret) {
		printf("Failed to unpack ddr data %d\n", ret);
		return ret;
	}

	p = (struct dram_cfg_param*)&ea_dbuf[0];

	offset = 0;
	while(offset*sizeof(struct dram_cfg_param) < len) {
		spl_ddr_map_array(p[offset].reg, &p[offset+1], p[offset].val);
		offset += (p[offset].val+1);
	}

	return ret;
}

static void spl_dram_init(uint32_t *size)
{
	ea_eeprom_config_t cfg;
	int ret;

        /* set default value, will be replaced if  eeprom cfg is valid */
        *size = (PHYS_SDRAM_SIZE >> 20);

#ifdef CONFIG_EA_IMX_PTP
	/* Skip reading eeprom */
	(void)cfg;
	(void)ret;
#else
        ret = ea_eeprom_get_config(&cfg);

        /* If eeprom is valid read ddr config; otherwise use default */
        if (!ret) {
                *size = cfg.ddr_size;

		/*
		 * timing values might exist in eeprom as gzipped data
		 */
		if (cfg.data_type == EA_EEPROM_DATA_TYPE_GZIP) {
			printf("EA: Using gzipped ddr data from eeprom\n");
			ret = spl_ddr_unpack_data(&cfg);
		}
        }
#endif

	ddr_init(&dram_timing);
}

void spl_board_init(void)
{
#ifndef CONFIG_SPL_USB_SDP_SUPPORT
	/* Serial download mode */
	if (is_usb_boot()) {
		puts("Back to ROM, SDP\n");
		restore_boot_params();
	}
#endif

	init_usb_clk();

	puts("Normal Boot\n");
}

#define USDHC1_PWR_GPIO IMX_GPIO_NR(2, 10)

static struct fsl_esdhc_cfg usdhc_cfg[1] = {
	{USDHC1_BASE_ADDR, 0, 8},
};

int board_mmc_getcd(struct mmc *mmc)
{
	/* we always boot from eMMC, which is always connected */
	return 1;
}

int board_mmc_init(bd_t *bis)
{

	/* we always boot from eMMC so only one mmc device is supported */

	usdhc_cfg[0].sdhc_clk = mxc_get_clock(USDHC1_CLK_ROOT);
	imx_iomux_v3_setup_multiple_pads(
		usdhc1_pads, ARRAY_SIZE(usdhc1_pads));
	gpio_request(USDHC1_PWR_GPIO, "usdhc1_reset");
	gpio_direction_output(USDHC1_PWR_GPIO, 0);
	udelay(500);
	gpio_direction_output(USDHC1_PWR_GPIO, 1);

	return fsl_esdhc_initialize(bis, &usdhc_cfg[0]);
}

#ifdef CONFIG_POWER
#define I2C_PMIC	0
int power_init_board(void)
{
	struct pmic *p;
	int ret;

	ret = power_bd71837_init(I2C_PMIC);
	if (ret)
		printf("power init failed");

	p = pmic_get("BD71837");
	pmic_probe(p);

	/* decrease RESET key long push time from the default 10s to 10ms */
	pmic_reg_write(p, BD71837_PWRONCONFIG1, 0x0);

	/* unlock the PMIC regs */
	pmic_reg_write(p, BD71837_REGLOCK, 0x1);

	/* increase VDD_DRAM to 0.9v for 3Ghz DDR */
	pmic_reg_write(p, BD71837_BUCK5_VOLT, 0x2);

#ifndef CONFIG_IMX8M_LPDDR4
	/* increase NVCC_DRAM_1V2 to 1.2v for DDR4 */
	pmic_reg_write(p, BD71837_BUCK8_VOLT, 0x28);
#endif

	/* lock the PMIC regs */
	pmic_reg_write(p, BD71837_REGLOCK, 0x11);

	return 0;
}
#endif

#ifdef CONFIG_SPL_LOAD_FIT
int board_fit_config_name_match(const char *name)
{
	/* Just empty function now - can't decide what to choose */
	debug("%s: %s\n", __func__, name);

	return 0;
}
#endif

void board_init_f(ulong dummy)
{
	uint32_t size;
	ea_config_t *ea_conf = (ea_config_t *)EA_SHARED_CONFIG_MEM;
	int ret;

	/* Clear the BSS. */
	memset(__bss_start, 0, __bss_end - __bss_start);

	arch_cpu_init();

	init_uart_clk(0); /* Init UART0 clock */

	timer_init();

	board_early_init_f();

	preloader_console_init();


	ret = spl_init();
	if (ret) {
		debug("spl_init() failed: %d\n", ret);
		hang();
	}

	enable_tzc380();

	/* Setup I2C for PMIC/eeprom amd gpio expander */
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

	/* Adjust pmic voltage to 1.0V for 800M */
	power_init_board();

	/* DDR initialization */
	spl_dram_init(&size);

	/*
	 * Setting configuration data that will be shared with u-boot.
	 *
	 * SPL is able to use i2c to detect gpio expander or read data
	 * from eeprom. U-boot is not able to do this before relocation
	 * when the device model (CONFIG_DM) is enabled.
	 */
        ea_conf->magic = EA_CONFIG_MAGIC;
        ea_conf->is_carrier_v2 = ea_is_carrier_v2(1);
        ea_conf->ddr_size = size;

	board_init_r(NULL, 0);
}
