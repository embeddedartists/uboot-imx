/*
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __EATOUCH_H_
#define __EATOUCH_H_


struct touch_info_t {
	int     enabled;
	int	    addr;

#define TOUCH_AR1021    0
#define TOUCH_ILITEK    1
#define TOUCH_SITRONIX  2
#define TOUCH_EGALAX    3
#define TOUCH_FT5X06    4
#define TOUCH_MXT1664   5
#define TOUCH_COUNT     6
	int	    type;

	const char* alias;
};

void eatouch_init(void);

#define EATOUCH_CONTROLLER(_conn, _addr, _type, _tname) \
{ \
	.enabled = 0, \
	.addr	 = _addr, \
	.type	 = _type,\
	.alias	 = "ts_con_" #_conn "/" #_tname "_" #_conn \
}

#define EATOUCH_ALL_CONTROLLERS(_conn) \
	EATOUCH_CONTROLLER(_conn, 0x4d, TOUCH_AR1021,   ar1021),\
	EATOUCH_CONTROLLER(_conn, 0x41, TOUCH_ILITEK,   ilitek_aim),\
	EATOUCH_CONTROLLER(_conn, 0x55, TOUCH_SITRONIX, sitronix),\
	EATOUCH_CONTROLLER(_conn, 0x04, TOUCH_EGALAX,   egalax_ts),\
	EATOUCH_CONTROLLER(_conn, 0x38, TOUCH_FT5X06,   edt-ft5x06), \
	EATOUCH_CONTROLLER(_conn, 0x4b, TOUCH_MXT1664,   mxt1664_ts)

#endif /* __EATOUCH_H_ */
