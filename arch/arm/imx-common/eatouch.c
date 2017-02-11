/*
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/errno.h>
#include <asm/gpio.h>
#include <asm/imx-common/eatouch.h>
#include <asm/io.h>
#include <div64.h>
#include <i2c.h>
#include <malloc.h>
#include <stdlib.h>

/*
 * The board file should call eatouch_init() to
 * load the selected touch controller information.
 *
 * The eatouch command supports enabling/disabling each available touch
 * controller for each of the display interface connectors on the board.
 *
 * The eatouch command does this by setting environment variables (XX stand for
 * rgb,lvds0,lvds1):
 * - cmd_ts_rgb          used by cmd_ts to modify device tree for the rgb connector
 * - cmd_ts_lvds0        used by cmd_ts to modify device tree for the lvds0 connector
 * - cmd_ts_lvds1        used by cmd_ts to modify device tree for the lvds1 connector
 * - cmd_ts              main command, will be executed by bootscript prior to boot
 */

#define CONNECTOR_RGB    0
#define CONNECTOR_LVDS0  1
#define CONNECTOR_LVDS1  2

#if defined(CONFIG_MX6SX)
  #define CONNECTORS        "(rgb|lvds0)"
  #define NUM_CONNECTORS    2  /* Only have 1x RGB and 1x LVDS on SoloX */
#elif defined(CONFIG_MX6Q)
  #define CONNECTORS        "(rgb|lvds0|lvds1)"
  #define NUM_CONNECTORS    3
#elif defined(CONFIG_MX6DL)
  #define CONNECTORS        "(rgb|lvds0|lvds1)"
  #define NUM_CONNECTORS    3
#elif defined(CONFIG_MX6UL)
  #define CONNECTORS        "rgb"
  #define NUM_CONNECTORS    1  /* Only have 1x RGB on UltraLite */
#elif defined(CONFIG_MX7D)
  #define CONNECTORS        "rgb"
  #define NUM_CONNECTORS    1  /* Only have 1x RGB on 7 Dual */
#else
  #error Unsupported hardware
#endif

static const char *const cmd_names[] = {
[CONNECTOR_RGB] = "cmd_ts_rgb",
[CONNECTOR_LVDS0] = "cmd_ts_lvds0",
[CONNECTOR_LVDS1] = "cmd_ts_lvds1",
};

static const char *const connector_names[] = {
[CONNECTOR_RGB] = "rgb",
[CONNECTOR_LVDS0] = "lvds0",
[CONNECTOR_LVDS1] = "lvds1",
};

static const char *const short_names[] = {
[TOUCH_AR1021] = "ar1021",
[TOUCH_EGALAX] = "egalax",
[TOUCH_FT5X06] = "ft5x06",
[TOUCH_ILITEK] = "ilitek",
[TOUCH_SITRONIX] = "sitronix",
[TOUCH_MXT1664] = "mxt1664",
};

static struct touch_info_t controllers[NUM_CONNECTORS][TOUCH_COUNT] = {
	{ EATOUCH_ALL_CONTROLLERS(rgb)   },
#if NUM_CONNECTORS > 1
	{ EATOUCH_ALL_CONTROLLERS(lvds0) },
#if NUM_CONNECTORS > 2
	{ EATOUCH_ALL_CONTROLLERS(lvds1) },
#endif
#endif
};

#define TOUCH_DISABLED  (-1)
#define TOUCH_ENV_CMD   "cmd_ts"
#define TOUCH_ENV_SEL   "eatouch_selected"


static void print_current_config(void)
{
	int i, j;

	printf("\nCurrent Setup:\n");
	printf("\t%10s\t", "");
	for (i = 0; i < NUM_CONNECTORS; i++) {
		printf("  %s conn.\t", connector_names[i]);
	}
	printf("\n");

	for (i = 0; i < TOUCH_COUNT; i++) {
		printf("\t%10s", short_names[i]);
		for (j = 0; j < NUM_CONNECTORS; j++) {
			if (controllers[j][i].enabled) {
				printf("\tEnabled 0x%02x", controllers[j][i].addr);
			} else {
				printf("\t  Disabled");
			}
		}
		printf("\n");
	}
}

static void print_status(void)
{
	int i;

	printf("\nAvailable Touch Controllers:\n");
	for (i = 0; i < TOUCH_COUNT; i++) {
		printf("   %2d) %s\n", i+1, short_names[i]);
	}

	print_current_config();
}

static void load_selection(void)
{
	int i, j;
	int valid = 0;
	const char* env = getenv(TOUCH_ENV_SEL);
	if (env) {
		valid = 1;
		for (i = 0; i < NUM_CONNECTORS; i++) {
			for (j = 0; j < TOUCH_COUNT; j++) {
				if (*env == 'E') {
					controllers[i][j].enabled = 1;
					env++;
				} else if (*env == '-') {
					controllers[i][j].enabled = 0;
					env++;
				} else {
					printf("Illegal touch configuration, turning all off\n");
					setenv(TOUCH_ENV_SEL, NULL);
					valid = 0;
					break;
				}
			}
			if (!valid) {
				break;
			}
		}
	}

	if (!valid) {
		for (i = 0; i < NUM_CONNECTORS; i++) {
			for (j = 0; j < TOUCH_COUNT; j++) {
				controllers[i][j].enabled = 0;
			}
		}
	}
}

static void save_selection(void)
{
	char buf[NUM_CONNECTORS * TOUCH_COUNT + 1];
	int i, j;
	char* p = &buf[0];

	for (i = 0; i < NUM_CONNECTORS; i++) {
		for (j = 0; j < TOUCH_COUNT; j++) {
			if (controllers[i][j].enabled) {
				*p++ = 'E';
			} else {
				*p++ = '-';
			}
		}
	}
	*p = '\0';
	setenv(TOUCH_ENV_SEL, buf);
}

static void update_commands(void)
{
	char* buf = malloc(4096);
	char* p;
	int i, j, count;
	int size;

	if (!buf) {
		printf("Failed to allocate memory for eatouch command\n");
	}

	for (i = 0; i < NUM_CONNECTORS; i++) {
		p = buf;
		size = 4096;
		for (j = 0; j < TOUCH_COUNT; j++) {
			if (controllers[i][j].enabled) {
				count = snprintf(p, size, "fdt set %s status okay; fdt set %s reg <0x%x>; ", controllers[i][j].alias, controllers[i][j].alias, controllers[i][j].addr);
			} else {
				count = snprintf(p, size, "fdt set %s status disabled; ", controllers[i][j].alias);
			}
			p += count;
			size -= count;
		}
		setenv(cmd_names[i], buf);
	}
	p = buf;
	size = 4096;
	for (i = 0; i < NUM_CONNECTORS; i++) {
		count = snprintf(p, size, "run %s; ", cmd_names[i]);
		p += count;
		size -= count;
	}
	setenv(TOUCH_ENV_CMD, buf);

	free(buf);
}

static int do_eatouch(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	int i;
	const char *cmd;

	if (argc == 1) {
		print_status();
		return CMD_RET_SUCCESS;
	}
	if (argc < 3) {
		return CMD_RET_USAGE;
	}

	cmd = argv[1];

	if (!strcmp("enable", cmd) || !strcmp("disable", cmd)) {
		int enable = !strcmp("enable", cmd);
		if (argc == 4) {
			for (i = 0; i < NUM_CONNECTORS; i++) {
				if (!strcmp(argv[2], connector_names[i])) {
					int idx = simple_strtoul(argv[3], NULL, 10);
					//printf("'%s', '%s', '%s', '%s' (%d)\n", argv[0], argv[1], argv[2], argv[3], idx);
					if (idx >= 1 && idx <= TOUCH_COUNT) {
						controllers[i][idx-1].enabled = enable;
					} else {
						printf("Invalid controller number '%s', should be 1<=num<=%d\n", argv[3], TOUCH_COUNT);
					}
					save_selection();
					update_commands();
					print_current_config();
					return CMD_RET_SUCCESS;
				}
			}
			printf("Invalid display interface connector name '%s'\n", argv[2]);
		}
		return CMD_RET_USAGE;
	}

	// Hidden command to change i2c address for a controller. Will not survive a reboot/reset
	// so the mod command must be followed by a 'boot' command to take effect.
	if (!strcmp("mod", cmd)) {
		if (argc == 4) {
			int ctrl = simple_strtoul(argv[2], NULL, 10);
			if (ctrl >= 1 && ctrl <= TOUCH_COUNT) {
				ctrl--;
				int addr = simple_strtoul(argv[3], NULL, 10);
				if (addr >= 0x01 && addr <= 0xff) {
					int conn;
					printf("Changing I2C address of %s to 0x%x\n", short_names[ctrl], addr);
					for (conn = 0; conn < NUM_CONNECTORS; conn++) {
						controllers[conn][ctrl].addr = addr;
					}
					save_selection();
					update_commands();
					print_current_config();
					return CMD_RET_SUCCESS;
				} else {
					printf("Invalid i2c addr '%s', should be 0x01<=num<=0xff\n", argv[3]);
				}
			} else {
				printf("Invalid controller number '%s', should be 1<=num<=%d\n", argv[2], TOUCH_COUNT);
			}
		}
		printf("usage:\n\teatouch mod num newaddr\n");
		return CMD_RET_FAILURE;
	}

	return CMD_RET_USAGE;
}


U_BOOT_CMD(eatouch, 4, 0, do_eatouch,
		   "Configure Touch Controller Support for each display interface connector",
		   " - Show current configuration\n"
		   "eatouch disable "CONNECTORS" num - Disable touch controller from list for connector\n"
		   "eatouch enable "CONNECTORS" num - Enable touch controller from list for connector\n");


void eatouch_init(void)
{
	load_selection();
	update_commands();
}
