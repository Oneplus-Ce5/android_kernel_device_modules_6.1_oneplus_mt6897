// SPDX-License-Identifier: GPL-2.0-only
/*
 * mt6379-charger.c -- Mediatek MT6379/MT6720 compatible Charger Driver
 *
 * Copyright (c) 2023 MediaTek Inc.
 *
 * Author: SHIH CHIA CHANG <jeff_chang@richtek.com>
 */

#include <dt-bindings/power/mtk-charger.h>
#include <linux/bitfield.h>
#include <linux/devm-helpers.h>
#include <linux/init.h>
#include <linux/linear_range.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/property.h>

#include "mt6379-charger.h"
#include "ufcs_class.h"

#ifdef OPLUS_FEATURE_CHG_BASIC
#include <linux/time.h>
#include <linux/jiffies.h>
#include <linux/sched/clock.h>
#include <linux/regmap.h>
#include <linux/alarmtimer.h>
#include <linux/regulator/machine.h>
#include <mt-plat/mtk_boot_common.h>
#include <soc/oplus/system/oplus_project.h>

#ifdef CONFIG_OPLUS_PD_EXT_SUPPORT
#include "../oplus/pd_ext/inc/tcpm.h"
#include "../oplus/pd_ext/inc/tcpci.h"
#else
#include <tcpm.h>
#include <tcpci.h>
#endif
#endif /* OPLUS_FEATURE_CHG_BASIC */

#ifdef OPLUS_FEATURE_CHG_BASIC
/* oplus add for hvdcp charging */
#define OPLUS_HVDCP_DISABLE_INTERVAL round_jiffies_relative(msecs_to_jiffies(15000))
#define OPLUS_HVDCP_DETECT_TO_DETACH_TIME 90
#define HVDCP_EXIT_NORMAL	0
#define HVDCP_EXIT_ABNORMAL	1
#define POWER_SUPPLY_TYPE_USB_HVDCP 13
#define POWER_SUPPLY_TYPE_USB_HVDCP_3 14
#define MT6379_BC12_RETRY_CNT	3
#endif /* OPLUS_FEATURE_CHG_BASIC */

struct mt6379_charger_data *oplus_cdata;
bool is_mtksvooc_project = false;
static int mt6379_charger_enable_bc12(struct mt6379_charger_data *cdata, bool en);

#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
int mt6379_get_hvdcp_status(struct mt6379_charger_data *cdata, int val);
int mt6379_set_hvdcp_detect_enable(struct mt6379_charger_data *cdata);
int mt6379_set_hvdcp_detect_disable(struct mt6379_charger_data *cdata);
int mt6379_set_hvdcp_to_5v(void);
int mt6379_set_hvdcp_to_9v(void);
int mt6379_reset_hvdcp_reg(struct mt6379_charger_data *cdata);
void oplus_set_hvdcp_flag_clear(void);
#endif

#define DEFAULT_PMIC_UVLO_MV	2000
#define DPDM_OV_THRESHOLD_MV	3850

/*
 * modify MT6379_IMPLEMENT_ICC_LOAD if you implement icc save/load to/from Phone.
 */
#define  MT6379_IMPLEMENT_ICC_LOAD	0

/* check for do calibration or not */
#define MT6379_CHECK_ICC_TIME		10000	/* 10s */
/* check for icc is good or not */
#define MT6379_ICC_CHECK_TIME		500	/* 500ms */
#define MT6379_SENSING_OFFSET_MAX	250000	/* 250 mA */
#define MT6379_ICC_DIFF_MAX		50000	/* 50 mA */

/*
 * For Fsw Control
 */
#define MT6379_FSW_CONTROL_TIME		0			/* 0s */
#define MT6379_FSW_CONTROL_TIME_NS	(10 * 1000000)		/* 10 ms */
#define MT6379_FSW_CHECK_NR	100

#define MT6379_FSW_CONTROL_TIME_2	10			/* 10s */
#define MT6379_FSW_CONTROL_TIME_NS_2	(0 * 1000000)		/* 0 ms */

unsigned int dbg_log_level = 1;
module_param(dbg_log_level, uint, 0644);

unsigned int fast_fsw_control = 1;
unsigned int fast_fsw_cnt = MT6379_FSW_CHECK_NR;
unsigned int fsw_ctrl_time = MT6379_FSW_CONTROL_TIME;
unsigned int fsw_ctrl_time_ns = MT6379_FSW_CONTROL_TIME_NS;
unsigned int fsw_ctrl_time_2 = MT6379_FSW_CONTROL_TIME_2;
unsigned int fsw_ctrl_time_ns_2 = MT6379_FSW_CONTROL_TIME_NS_2;
unsigned int support_fsw_wakeup = 1;

module_param(support_fsw_wakeup, uint, 0644);
module_param(fast_fsw_control, uint, 0644);
module_param(fast_fsw_cnt, uint, 0644);
module_param(fsw_ctrl_time, uint, 0644);
module_param(fsw_ctrl_time_ns, uint, 0644);
module_param(fsw_ctrl_time_2, uint, 0644);
module_param(fsw_ctrl_time_ns_2, uint, 0644);

static int icc_check_time = MT6379_ICC_CHECK_TIME;
module_param(icc_check_time, int, 0644);

static int test_icc = -1;
static int test_aicr = -1;
static int test_mivr = -1;
module_param(test_icc, int, 0644);
module_param(test_aicr, int, 0644);
module_param(test_mivr, int, 0644);

static const struct rt_charger_data mt6379_data = {
	.name = "mt6379",
	.id = CHARGER_ID_MT6379,
};

static const struct rt_charger_data mt6720_data = {
	.name = "mt6720",
	.id = CHARGER_ID_MT6720,
};

enum {
	CHG_STAT_SLEEP,
	CHG_STAT_VBUS_RDY,
	CHG_STAT_TRICKLE,
	CHG_STAT_PRE,
	CHG_STAT_FAST,
	CHG_STAT_EOC,
	CHG_STAT_BKGND,
	CHG_STAT_DONE,
	CHG_STAT_FAULT,
	CHG_STAT_OTG = 15,
	CHG_STAT_MAX,
};

struct mt6379_charger_field {
	const char *name;
	const struct linear_range *range;
	struct reg_field field;
	const u32 *table;
	u32 tbl_num;
	bool inited;
};

enum mt6379_charger_dtprop_type {
	DTPROP_U32,
	DTPROP_BOOL,
};

struct mt6379_charger_dtprop {
	const char *name;
	size_t offset;
	enum mt6379_charger_reg_field field;
	enum mt6379_charger_dtprop_type type;
};

#define MT6379_CHG_DTPROP(_name, _member, _field, _type)		  \
{									  \
	.name = _name,							  \
	.field = _field,						  \
	.type = _type,							  \
	.offset = offsetof(struct mt6379_charger_platform_data, _member),\
}

#define MT6379_CHARGER_FIELD(_fd, _reg, _lsb, _msb)			\
[_fd] = {								\
	.name = #_fd,							\
	.range = NULL,							\
	.field = REG_FIELD(_reg, _lsb, _msb),				\
	.inited = true,							\
}

#define MT6379_CHARGER_FIELD_RANGE(_fd, _reg, _lsb, _msb)		\
[_fd] = {								\
	.name = #_fd,							\
	.range = &mt6379_charger_ranges[MT6379_RANGE_##_fd],		\
	.field = REG_FIELD(_reg, _lsb, _msb),				\
	.inited = true,							\
}

#define MT6379_CHARGER_FIELD_TABLE(_fd, _reg, _lsb, _msb, _tbl)		\
[_fd] = {								\
	.name = #_fd,							\
	.table = _tbl,							\
	.tbl_num = ARRAY_SIZE(_tbl),					\
	.field = REG_FIELD(_reg, _lsb, _msb),				\
	.inited = true,							\
}

enum {
	PORT_STAT_NOINFO = 0,
	PORT_STAT_APPLE_10W = 8,
	PORT_STAT_SS_TA,
	PORT_STAT_APPLE_5W,
	PORT_STAT_APPLE_12W,
	PORT_STAT_UNKNOWN_TA,
	PORT_STAT_SDP,
	PORT_STAT_CDP,
	PORT_STAT_DCP,
};

enum {
	MT6379_RANGE_F_BATINT = 0,
	MT6379_RANGE_F_IBUS_AICR,
	MT6379_RANGE_F_WLIN_AICR,
	MT6379_RANGE_F_VBUS_MIVR,
	MT6379_RANGE_F_WLIN_MIVR,
	MT6379_RANGE_F_VREC,
	MT6379_RANGE_F_CV,
	MT6379_RANGE_F_CC,
	MT6379_RANGE_F_CHG_TMR,
	MT6379_RANGE_F_IEOC,
	MT6379_RANGE_F_EOC_TIME,
	MT6379_RANGE_F_VSYSOV,
	MT6379_RANGE_F_VSYSMIN,
	MT6379_RANGE_F_PE20_CODE,
	MT6379_RANGE_F_IPREC,
	MT6379_RANGE_F_AICC_RPT,
	MT6379_RANGE_F_OTG_LBP,
	MT6379_RANGE_F_OTG_OCP,
	MT6379_RANGE_F_OTG_CC,
	MT6379_RANGE_F_IRCMP_R,
	MT6379_RANGE_F_IRCMP_V,
	MT6379_RANGE_F_CHRD_UV,
#ifdef OPLUS_FEATURE_CHG_BASIC
	/* oplus add for uvlo */
	MT6379_RANGE_F_CHRD_OV,
#endif
	MT6379_RANGE_F_MAX,
};

enum {
	MT6379_DP_LDO_VSEL_600MV,
	MT6379_DP_LDO_VSEL_650MV,
	MT6379_DP_LDO_VSEL_700MV,
	MT6379_DP_LDO_VSEL_750MV,
	MT6379_DP_LDO_VSEL_1800MV,
	MT6379_DP_LDO_VSEL_2800MV,
	MT6379_DP_LDO_VSEL_3300MV,
};

enum {
	MT6379_DP_PULL_RSEL_1_2_K,
	MT6379_DP_PULL_RSEL_10_K,
	MT6379_DP_PULL_RSEL_15_K,
};

enum {
	MT6379_CHGIN_OV_4_7_V,
	MT6379_CHGIN_OV_5_8_V,
	MT6379_CHGIN_OV_6_5_V,
	MT6379_CHGIN_OV_11_V,
	MT6379_CHGIN_OV_14_5_V,
	MT6379_CHGIN_OV_18_V,
	MT6379_CHGIN_OV_22_5_V,
};

#ifdef OPLUS_FEATURE_CHG_BASIC
#define CHRD_OV_TABLE_MAX 15
#define OPLUS_CHRD_UV_THD 2600000
#define OPLUS_CHRD_OV_THD 12500000
#endif
static const u32 mt6379_chg_chrd_ov[] = {
	6500000, 7000000, 7500000, 8500000, 9500000, 10500000, 11500000, 12500000, 14500000,
	16500000, 18500000, 20500000, 22500000, 24500000, 28500000, 32500000,
};

static const u32 mt6720_chg_chrd_ov[] = {
	6000000, 6500000, 70000000, 7500000, 8500000, 9500000, 10500000, 11500000,
	12500000, 14500000,
};

static const struct linear_range mt6379_charger_ranges[MT6379_RANGE_F_MAX] = {
	LINEAR_RANGE_IDX(MT6379_RANGE_F_BATINT, 3900000, 0x0, 0x51, 10000),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_IBUS_AICR, 100000, 0x0, 0xA7, 25000),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_WLIN_AICR, 100000, 0x0, 0x7F, 25000),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_VBUS_MIVR, 3900000, 0x0, 0xB5, 100000),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_WLIN_MIVR, 3900000, 0x0, 0xB5, 100000),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_VREC, 100000, 0x0, 0x1, 100000),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_CV, 3900000, 0x0, 0x51, 10000),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_CC, 300000, 0x6, 0x50, 50000),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_CHG_TMR, 5, 0x0, 0x3, 5),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_IEOC, 100000, 0x0, 0x3A, 50000),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_EOC_TIME, 0, 0x0, 0x3, 15),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_VSYSOV, 4600000, 0, 7, 100000),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_VSYSMIN, 3200000, 0, 0xF, 100000),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_PE20_CODE, 5500000, 0, 0x1D, 500000),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_IPREC, 50000, 0, 0x27, 50000),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_AICC_RPT, 100000, 0x0, 0xA7, 25000), /* same as aicr */
	LINEAR_RANGE_IDX(MT6379_RANGE_F_OTG_LBP, 2700000, 0x0, 0x7, 100000),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_OTG_OCP, 3500000, 0x0, 0x3, 1000000),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_OTG_CC, 500000, 0x0, 0x6, 3000000),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_IRCMP_R, 0, 0x0, 0xA, 5),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_IRCMP_V, 0, 0x0, 0x14, 10),
	LINEAR_RANGE_IDX(MT6379_RANGE_F_CHRD_UV, 2600000, 0x0, 0xB, 100000),
};


static struct mt6379_charger_field mt6379_charger_fields[F_MAX] = {
	MT6379_CHARGER_FIELD(F_MREN, MT6379_REG_CORE_CTRL0, 4, 4),
	MT6379_CHARGER_FIELD(F_REG_RST, MT6379_REG_RST1, 0, 0),
	MT6379_CHARGER_FIELD(F_BATPROTECT_SOURCE, MT6379_REG_CHG_BATPRO_SLE, 5, 5),
	MT6379_CHARGER_FIELD(F_SHIP_RST_DIS, MT6379_REG_CORE_CTRL2, 0, 0),
	MT6379_CHARGER_FIELD(F_PD_MDEN, MT6379_REG_CORE_CTRL2, 1, 1),
	MT6379_CHARGER_FIELD(F_PREUV_EN, MT6379_REG_BB_VOUT_SEL, 7, 7),
	MT6379_CHARGER_FIELD(F_ST_PWR_RDY, MT6379_REG_CHG_STAT0, 0, 0),
	MT6379_CHARGER_FIELD(F_ST_MIVR, MT6379_REG_CHG_STAT1, 7, 7),
	MT6379_CHARGER_FIELD(F_ST_AICC_DONE, MT6379_REG_CHG_STAT2, 2, 2),
	MT6379_CHARGER_FIELD(F_CHRDET_EXT, MT6379_REG_CHRDET_STAT, 2, 2),
	MT6379_CHARGER_FIELD(F_ST_USBID, MT6379_REG_USBID_STAT, 0, 0),
	MT6379_CHARGER_FIELD_RANGE(F_BATINT, MT6379_REG_CHG_BATPRO, 0, 6),
	MT6379_CHARGER_FIELD(F_BATPROTECT_EN, MT6379_REG_CHG_BATPRO, 7, 7),
	MT6379_CHARGER_FIELD(F_PP_PG_FLAG, MT6379_REG_CHG_TOP1, 7, 7),
	MT6379_CHARGER_FIELD(F_BATFET_DIS, MT6379_REG_CHG_TOP1, 6, 6),
	MT6379_CHARGER_FIELD(F_QON_RST_EN, MT6379_REG_CHG_TOP1, 3, 3),
	MT6379_CHARGER_FIELD(F_UUG_FULLON, MT6379_REG_CHG_TOP2, 3, 3),
	MT6379_CHARGER_FIELD(F_CHG_OCP, MT6379_REG_CHG_TOP2, 2, 2),
	MT6379_CHARGER_FIELD(F_CHG_BYPASS, MT6379_REG_CHG_TOP2, 1, 1),
	MT6379_CHARGER_FIELD_RANGE(F_IBUS_AICR, MT6379_REG_CHG_IBUS_AICR, 0, 7),
	MT6379_CHARGER_FIELD(F_ILIM_EN, MT6379_REG_CHG_WLIN_AICR, 7, 7),
	MT6379_CHARGER_FIELD_RANGE(F_WLIN_AICR, MT6379_REG_CHG_WLIN_AICR, 0, 6),
	MT6379_CHARGER_FIELD_RANGE(F_VBUS_MIVR, MT6379_REG_CHG_VBUS_MIVR, 0, 7),
	MT6379_CHARGER_FIELD_RANGE(F_WLIN_MIVR, MT6379_REG_CHG_WLIN_MIVR, 0, 7),
	MT6379_CHARGER_FIELD_RANGE(F_VREC, MT6379_REG_CHG_VCHG, 7, 7),
	MT6379_CHARGER_FIELD_RANGE(F_CV, MT6379_REG_CHG_VCHG, 0, 6),
	MT6379_CHARGER_FIELD_RANGE(F_CC, MT6379_REG_CHG_ICHG, 0, 6),
	MT6379_CHARGER_FIELD(F_CHG_TMR_EN, MT6379_REG_CHG_TMR, 7, 7),
	MT6379_CHARGER_FIELD(F_CHG_TMR_2XT, MT6379_REG_CHG_TMR, 6, 6),
	MT6379_CHARGER_FIELD_RANGE(F_CHG_TMR, MT6379_REG_CHG_TMR, 4, 5),
	MT6379_CHARGER_FIELD_RANGE(F_IEOC, MT6379_REG_CHG_EOC1, 0, 5),
	MT6379_CHARGER_FIELD(F_WLIN_FST, MT6379_REG_CHG_EOC2, 7, 7),
	MT6379_CHARGER_FIELD(F_CHGIN_OV, MT6379_REG_CHG_EOC2, 4, 6),
	MT6379_CHARGER_FIELD_RANGE(F_EOC_TIME, MT6379_REG_CHG_EOC2, 2, 3),
	MT6379_CHARGER_FIELD(F_TE, MT6379_REG_CHG_EOC2, 1, 1),
	MT6379_CHARGER_FIELD(F_EOC_RST, MT6379_REG_CHG_EOC2, 0, 0),
	MT6379_CHARGER_FIELD(F_DISCHARGE_EN, MT6379_REG_CHG_VSYS, 7, 7),
	MT6379_CHARGER_FIELD_RANGE(F_VSYSOV, MT6379_REG_CHG_VSYS, 4, 6),
	MT6379_CHARGER_FIELD_RANGE(F_VSYSMIN, MT6379_REG_CHG_VSYS, 0, 3),
	MT6379_CHARGER_FIELD(F_HZ, MT6379_REG_CHG_WDT, 6, 6),
	MT6379_CHARGER_FIELD(F_BUCK_EN, MT6379_REG_CHG_WDT, 5, 5),
	MT6379_CHARGER_FIELD(F_CHG_EN, MT6379_REG_CHG_WDT, 4, 4),
	MT6379_CHARGER_FIELD(F_WDT_EN, MT6379_REG_CHG_WDT, 3, 3),
	MT6379_CHARGER_FIELD(F_WDT_RST, MT6379_REG_CHG_WDT, 2, 2),
	MT6379_CHARGER_FIELD(F_WDT_TIME, MT6379_REG_CHG_WDT, 0, 1),
	MT6379_CHARGER_FIELD(F_PE_EN, MT6379_REG_CHG_PUMPX, 7, 7),
	MT6379_CHARGER_FIELD(F_PE_SEL, MT6379_REG_CHG_PUMPX, 6, 6),
	MT6379_CHARGER_FIELD(F_PE10_INC, MT6379_REG_CHG_PUMPX, 5, 5),
	MT6379_CHARGER_FIELD_RANGE(F_PE20_CODE, MT6379_REG_CHG_PUMPX, 0, 4),
	MT6379_CHARGER_FIELD(F_AICC_EN, MT6379_REG_CHG_AICC, 7, 7),
	MT6379_CHARGER_FIELD(F_AICC_ONESHOT, MT6379_REG_CHG_AICC, 6, 6),
	MT6379_CHARGER_FIELD_RANGE(F_IPREC, MT6379_REG_CHG_IPREC, 0, 5),
	MT6379_CHARGER_FIELD_RANGE(F_AICC_RPT, MT6379_REG_CHG_AICC_RPT, 0, 7),
	MT6379_CHARGER_FIELD(F_DIG_THREG_EN, MT6379_REG_THR_REGU1, 6, 6),
	MT6379_CHARGER_FIELD_RANGE(F_OTG_LBP, MT6379_REG_CHG_OTG_LBP, 0, 2),
	MT6379_CHARGER_FIELD(F_SEAMLESS_OTG, MT6379_REG_CHG_OTG_C, 7, 7),
	MT6379_CHARGER_FIELD(F_OTG_THERMAL_EN, MT6379_REG_CHG_OTG_C, 6, 6),
	MT6379_CHARGER_FIELD_RANGE(F_OTG_OCP, MT6379_REG_CHG_OTG_C, 4, 5),
	MT6379_CHARGER_FIELD(F_OTG_WLS, MT6379_REG_CHG_OTG_C, 3, 3),
	MT6379_CHARGER_FIELD_RANGE(F_OTG_CC, MT6379_REG_CHG_OTG_C, 0, 2),
	MT6379_CHARGER_FIELD(F_IRCMP_EN, MT6379_REG_CHG_COMP1, 7, 7),
	MT6379_CHARGER_FIELD_RANGE(F_IRCMP_R, MT6379_REG_CHG_COMP1, 3, 6),
	MT6379_CHARGER_FIELD_RANGE(F_IRCMP_V, MT6379_REG_CHG_COMP2, 0, 4),
	MT6379_CHARGER_FIELD(F_BATFET_DISDLY, MT6379_REG_CHG_COMP2, 5, 7),
	MT6379_CHARGER_FIELD(F_IC_STAT, MT6379_REG_CHG_STAT, 0, 3),
	MT6379_CHARGER_FIELD(F_FORCE_VBUS_SINK, MT6379_REG_CHG_HD_TOP1, 5, 5),
	MT6379_CHARGER_FIELD(F_CHG_RAMP_UP_COMP, MT6379_REG_CHG_HD_BUBO5, 6, 7),
	MT6379_CHARGER_FIELD(F_IEOC_FLOW_RB, MT6379_REG_CHG_HD_TRIM6, 4, 4),
	MT6379_CHARGER_FIELD(F_VBAT_MON_EN, MT6379_REG_ADC_CONFG1, 5, 5),
	MT6379_CHARGER_FIELD(F_VBAT_MON2_EN, MT6379_REG_ADC_CONFG1, 4, 4),
	MT6379_CHARGER_FIELD(F_IS_TDET, MT6379_REG_USBID_CTRL1, 2, 4),
	MT6379_CHARGER_FIELD(F_ID_RUPSEL, MT6379_REG_USBID_CTRL1, 5, 6),
	MT6379_CHARGER_FIELD(F_USBID_EN, MT6379_REG_USBID_CTRL1, 7, 7),
	MT6379_CHARGER_FIELD(F_USBID_FLOATING, MT6379_REG_USBID_CTRL2, 1, 1),
	MT6379_CHARGER_FIELD_RANGE(F_CHRD_UV, MT6379_REG_CHRD_CTRL2, 4, 7),
	MT6379_CHARGER_FIELD_TABLE(F_CHRD_OV, MT6379_REG_CHRD_CTRL2, 0, 3, mt6379_chg_chrd_ov),
	MT6379_CHARGER_FIELD(F_WAFER_ID, MT6379_REG_WAFER_ID, 0, 4),
	MT6379_CHARGER_FIELD(F_ICC_ORIGIN, MT6379_REG_TM_SAVED_ICC_ORIGIN, 0, 7),
	MT6379_CHARGER_FIELD(F_ICC_TRIMMED, MT6379_REG_TM_TBTAD, 7, 7),
	MT6379_CHARGER_FIELD(F_ICC_OFFSET, MT6379_REG_TM_ICC_OFFSET, 0, 7),
	MT6379_CHARGER_FIELD(F_PD_SWRST, MT6379_REG_PD_SYS_CTRL3, 0, 0),
	MT6379_CHARGER_FIELD(F_PD_OTP_HWEN, MT6379_REG_TYPECOTP_CTRL, 0, 0),
	MT6379_CHARGER_FIELD(F_BC12_EN, MT6379_REG_BC12_FUNC, 7, 7),
	MT6379_CHARGER_FIELD(F_PORT_STAT, MT6379_REG_BC12_STAT, 0, 3),
	MT6379_CHARGER_FIELD(F_MANUAL_MODE, MT6379_REG_DPDM_CTRL1, 7, 7),
	MT6379_CHARGER_FIELD(F_DPDM_SW_VCP_EN, MT6379_REG_DPDM_CTRL1, 5, 5),
	MT6379_CHARGER_FIELD(F_DP_DET_EN, MT6379_REG_DPDM_CTRL1, 1, 1),
	MT6379_CHARGER_FIELD(F_DM_DET_EN, MT6379_REG_DPDM_CTRL1, 0, 0),
	MT6379_CHARGER_FIELD(F_DP_LDO_EN, MT6379_REG_DPDM_CTRL2, 7, 7),
	MT6379_CHARGER_FIELD(F_DP_LDO_VSEL, MT6379_REG_DPDM_CTRL2, 4, 6),
	MT6379_CHARGER_FIELD(F_DP_PULL_REN, MT6379_REG_DPDM_CTRL4, 7, 7),
	MT6379_CHARGER_FIELD(F_DP_PULL_RSEL, MT6379_REG_DPDM_CTRL4, 5, 6),
};

#ifdef OPLUS_FEATURE_CHG_BASIC
extern int tcpci_notify_bc12_complete_state(struct tcpc_device *tcpc, bool bc12_complete_state);
extern int tcpci_notify_chrdet_state(struct tcpc_device *tcpc, bool chrdet_state);
extern int tcpci_notify_hvdcp_detect_dn(struct tcpc_device *tcpc, bool hvdcp_detect_dn);
#endif

static int mt6379_charger_init_rmap_fields(struct mt6379_charger_data *cdata)
{
	const struct mt6379_charger_field *fds = mt6379_charger_fields;
	int i = 0;

	for (i = 0; i < F_MAX; i++) {
		if (!fds[i].inited)
			continue;
		cdata->rmap_fields[i] = devm_regmap_field_alloc(cdata->dev, cdata->rmap,
								fds[i].field);
		if (IS_ERR(cdata->rmap_fields[i])) {
			dev_info(cdata->dev, "%s, Failed to allocate regmap fields[%s]\n",
				 __func__, fds[i].name);
			return PTR_ERR(cdata->rmap_fields[i]);
		}
	}

	return 0;
}

int mt6379_charger_field_get(struct mt6379_charger_data *cdata, enum mt6379_charger_reg_field fd,
			     u32 *val)
{
	u32 regval = 0, idx = fd;
	int ret = 0;

	if (!mt6379_charger_fields[idx].inited) {
		dev_info(cdata->dev, "%s, %s is not support\n", __func__, mt6379_charger_fields[idx].name);
		return -EOPNOTSUPP;
	}

	ret = regmap_field_read(cdata->rmap_fields[idx], &regval);
	if (ret)
		return ret;
	if (mt6379_charger_fields[idx].table) {
		if (regval <= mt6379_charger_fields[idx].tbl_num) {
			*val = mt6379_charger_fields[idx].table[regval];
			return 0;
		}
		return -EINVAL;
	}

	if (mt6379_charger_fields[idx].range)
		return linear_range_get_value(mt6379_charger_fields[idx].range, regval, val);

	*val = regval;
	return 0;
}

int mt6379_charger_field_set(struct mt6379_charger_data *cdata, enum mt6379_charger_reg_field fd,
			     unsigned int val)
{
	const struct linear_range *r;
	u32 idx = fd, tbl_n;
	const u32 *table = NULL;
	int ret = 0, i = 0;
	bool f;

	if (!mt6379_charger_fields[idx].inited) {
		dev_info(cdata->dev, "%s, %s is not support\n", __func__, mt6379_charger_fields[idx].name);
		return -EOPNOTSUPP;
	}

	if (mt6379_charger_fields[idx].table) {
		table = mt6379_charger_fields[idx].table;
		tbl_n = mt6379_charger_fields[idx].tbl_num;
		if (val <= table[0])
			return regmap_field_write(cdata->rmap_fields[idx], 0);
		for (i = 0; i < tbl_n - 1; i++) {
			if (val == table[i])
				return regmap_field_write(cdata->rmap_fields[idx], i);
			if (val > table[i] && val < table[i + 1])
				return regmap_field_write(cdata->rmap_fields[idx], i + 1);
		}
		return regmap_field_write(cdata->rmap_fields[idx], tbl_n - 1);
	}

	if (mt6379_charger_fields[idx].range) {
		r = mt6379_charger_fields[idx].range;

		/* MIVR should get high selector */
		if (idx == F_VBUS_MIVR || idx == F_WLIN_MIVR) {
			ret = linear_range_get_selector_high(r, val, &val, &f);
			if (ret)
				val = r->max_sel;
		} else
			linear_range_get_selector_within(r, val, &val);
	}

	return regmap_field_write(cdata->rmap_fields[idx], val);
}

static const struct mt6379_charger_platform_data mt6379_charger_pdata_def = {
	.aicr = 3225,
	.mivr = 4400,
	.ichg = 2000,
	.ieoc = 150,
	.cv = 4200,
	.wdt_time = MT6379_WDT_TIME_40S,
	.ircmp_v = 0,
	.ircmp_r = 0,
	.vrec = 100,
	.chgin_ov = MT6379_CHGIN_OV_22_5V,
	.chg_tmr = 10,
	.nr_port = 1,
	.wdt_en = false,
	.te_en = true,
	.chg_tmr_en = true,
	.chgdev_name = "primary_chg",
	.usb_killer_detect = false,
};

static const u32 mt6379_otg_cc_ma[] = {
	500000, 800000, 1100000, 1400000, 1700000, 2000000, 2300000,
};

static int mt6379_set_voltage_sel(struct regulator_dev *rdev, unsigned int sel)
{
	const struct regulator_desc *desc = rdev->desc;
	struct regmap *rmap = rdev_get_regmap(rdev);

	sel <<= ffs(desc->vsel_mask) - 1;
	sel = cpu_to_be16(sel);

	return regmap_bulk_write(rmap, desc->vsel_reg, &sel, 2);
}

static int mt6379_get_voltage_sel(struct regulator_dev *rdev)
{
	const struct regulator_desc *desc = rdev->desc;
	struct regmap *rmap = rdev_get_regmap(rdev);
	unsigned int val = 0;
	int ret = 0;

	ret = regmap_bulk_read(rmap, desc->vsel_reg, &val, 2);
	if (ret)
		return ret;

	val = be16_to_cpu(val);
	val &= desc->vsel_mask;
	val >>= ffs(desc->vsel_mask) - 1;
	return val;
}

static bool mt6379_charger_is_usb_killer(struct mt6379_charger_data *cdata)
{
	struct mt6379_charger_platform_data *pdata = dev_get_platdata(cdata->dev);
	int i = 0, ret = 0, vdp = 0, vdm = 0;
	static const u32 vdiff = 200;
	bool killer = false;
	static const struct {
		enum mt6379_charger_reg_field fd;
		u32 val;
	} settings[] = {
		{ F_MANUAL_MODE, 1 },
		{ F_DPDM_SW_VCP_EN, 1 },
		{ F_DP_DET_EN, 1 },
		{ F_DM_DET_EN, 1 },
		{ F_DP_LDO_VSEL, MT6379_DP_LDO_VSEL_1800MV },
		{ F_DP_LDO_EN, 1 },
		{ F_DP_PULL_RSEL, MT6379_DP_PULL_RSEL_1_2_K },
		{ F_DP_PULL_REN, 1 },
	};

	if (!pdata->usb_killer_detect) {
		dev_info(cdata->dev, "%s, usb killer is not set\n", __func__);
		return false;
	}

	/* Turn on usb dp 1.8V */
	for (i = 0; i < ARRAY_SIZE(settings); i++) {
		ret = mt6379_charger_field_set(cdata, settings[i].fd, settings[i].val);
		if (ret < 0)
			goto recover;
	}
	--i;

	/* check usb DPDM */
	ret = iio_read_channel_processed(&cdata->iio_adcs[ADC_CHAN_USBDP], &vdp);
	if (ret) {
		dev_info(cdata->dev, "%s, Failed to read usb DP voltage\n", __func__);
		goto recover;
	}

	ret = iio_read_channel_processed(&cdata->iio_adcs[ADC_CHAN_USBDM], &vdm);
	if (ret) {
		dev_info(cdata->dev, "%s, Failed to read usb DM voltage\n", __func__);
		goto recover;
	}

	vdp = U_TO_M(vdp);
	vdm = U_TO_M(vdm);
	dev_info(cdata->dev, "%s, dp = %dmV, dm = %dmV, vdiff = %dmV\n",
		 __func__, vdp, vdm, abs(vdp - vdm));

	if (abs(vdp - vdm) < vdiff) {
		dev_info(cdata->dev, "%s, suspect usb killer\n", __func__);
		killer = true;
	}

recover:
	for (; i >= 0; i--) { /* set to default value */
		if (mt6379_charger_field_set(cdata, settings[i].fd, 0))
			dev_notice(cdata->dev, "%s: Failed to recover %d setting\n",
				   __func__, i);
	}

	return killer;
}

static int mt6379_otg_regulator_enable(struct regulator_dev *rdev)
{
	struct mt6379_charger_data *cdata = rdev->reg_data;
	int ret = 0;

	if (mt6379_charger_is_usb_killer(cdata))
		return -EIO;

	if (cdata->id != CHARGER_ID_MT6379)
		return regulator_enable_regmap(rdev);

	/* disable PP_CV_FLOW_IDLE */
	ret = mt6379_enable_tm(cdata, true);
	if (ret) {
		dev_info(cdata->dev, "%s, Failed to enable tm(ret:%d)\n", __func__, ret);
		return ret;
	}

	ret = regmap_update_bits(cdata->rmap, MT6379_REG_CHG_HD_PP7, BIT(5), 0);
	if (ret) {
		ret = mt6379_enable_tm(cdata, false);
		if (ret)
			dev_info(cdata->dev, "%s, Failed to disable tm(ret:%d)\n", __func__, ret);

		return -EINVAL;
	}

	ret = mt6379_enable_tm(cdata, false);
	if (ret) {
		dev_info(cdata->dev, "%s, Failed to disable tm(ret:%d)\n", __func__, ret);
		return ret;
	}

	return regulator_enable_regmap(rdev);
}

static int mt6379_otg_regulator_disable(struct regulator_dev *rdev)
{
	struct mt6379_charger_data *cdata = rdev->reg_data;
	int ret = 0;

	if (cdata->id != CHARGER_ID_MT6379)
		return regulator_disable_regmap(rdev);

	/* enable PP_CV_FLOW_IDLE */
	ret = mt6379_enable_tm(cdata, true);
	if (ret) {
		dev_info(cdata->dev, "%s, Failed to enable tm(ret:%d)\n", __func__, ret);
		return ret;
	}

	ret = regmap_update_bits(cdata->rmap, MT6379_REG_CHG_HD_PP7, BIT(5), BIT(5));
	if (ret) {
		ret = mt6379_enable_tm(cdata, false);
		if (ret)
			dev_info(cdata->dev, "%s, Failed to disable tm(ret:%d)\n", __func__, ret);

		return -EINVAL;
	}

	ret = mt6379_enable_tm(cdata, false);
	if (ret) {
		dev_info(cdata->dev, "%s, Failed to disable tm(ret:%d)\n", __func__, ret);
		return ret;
	}

	return regulator_disable_regmap(rdev);
}

static int mt6379_otg_set_current_limit(struct regulator_dev *rdev, int min_uA, int max_uA)
{
	struct mt6379_charger_data *cdata = rdev->reg_data;
	const struct regulator_desc *desc = rdev->desc;
	int i, shift = ffs(desc->csel_mask) - 1;

	for (i = 0; i < ARRAY_SIZE(mt6379_otg_cc_ma); i++) {
		if (min_uA <= mt6379_otg_cc_ma[i])
			break;
	}

	if (i == ARRAY_SIZE(mt6379_otg_cc_ma)) {
		dev_notice(cdata->dev, "%s, %dmA is out of valid current range\n",
			   __func__, min_uA);
		return -EINVAL;
	}

	dev_info(cdata->dev, "%s, select otg_cc = %dmA\n", __func__, mt6379_otg_cc_ma[i]);
	return regmap_update_bits(cdata->rmap, desc->csel_reg, desc->csel_mask, i << shift);
}

static const struct regulator_ops mt6379_otg_regulator_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.enable = mt6379_otg_regulator_enable,
	.disable = mt6379_otg_regulator_disable,
	.is_enabled = regulator_is_enabled_regmap,
	.set_voltage_sel = mt6379_set_voltage_sel,
	.get_voltage_sel = mt6379_get_voltage_sel,
	.set_current_limit = mt6379_otg_set_current_limit,
	.get_current_limit = regulator_get_current_limit_regmap,
};

static const struct regulator_desc mt6720_charger_otg_rdesc = {
	.of_match = "usb-otg-vbus-regulator",
	.name = "mt6720-usb-otg-vbus",
	.ops = &mt6379_otg_regulator_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.min_uV = 4850000,
	.uV_step = 25000,
	.n_voltages = 31,
	.linear_min_sel = 0,
	.vsel_reg = MT6379_REG_CHG_OTG_CV_MSB,
	.vsel_mask = 0x1F,
	.enable_reg = MT6379_REG_CHG_TOP2,
	.enable_mask = BIT(0),
	.curr_table = mt6379_otg_cc_ma,
	.n_current_limits = ARRAY_SIZE(mt6379_otg_cc_ma),
	.csel_reg = MT6379_REG_CHG_OTG_C,
	.csel_mask = GENMASK(2, 0),
};

static const struct regulator_desc mt6379_charger_otg_rdesc = {
	.of_match = "usb-otg-vbus-regulator",
	.name = "mt6379-usb-otg-vbus",
	.ops = &mt6379_otg_regulator_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.min_uV = 4850000,
	.uV_step = 25000,
	.n_voltages = 287,
	.linear_min_sel = 20,
	.vsel_reg = MT6379_REG_CHG_OTG_CV_MSB,
	.vsel_mask = 0x1FF,
	.enable_reg = MT6379_REG_CHG_TOP2,
	.enable_mask = BIT(0),
	.curr_table = mt6379_otg_cc_ma,
	.n_current_limits = ARRAY_SIZE(mt6379_otg_cc_ma),
	.csel_reg = MT6379_REG_CHG_OTG_C,
	.csel_mask = GENMASK(2, 0),
};

#ifdef OPLUS_FEATURE_CHG_BASIC
static const struct regulator_init_data mt6379_otg_init_data = {
	.constraints = {
		.valid_ops_mask = REGULATOR_CHANGE_VOLTAGE | REGULATOR_CHANGE_CURRENT,
		.min_uA = 500000,
		.max_uA = 2300000,
	},
};
#endif

static int mt6379_init_otg_regulator(struct mt6379_charger_data *cdata)
{
	struct regulator_config config = {
		.dev = cdata->dev,
		.regmap = cdata->rmap,
		.driver_data = cdata,
#ifdef OPLUS_FEATURE_CHG_BASIC
		.init_data = &mt6379_otg_init_data,
#endif
	};

	dev_info(cdata->dev, "%s, charger id = %d\n", __func__, cdata->id);
	if (cdata->id == CHARGER_ID_MT6720)
		cdata->rdev = devm_regulator_register(cdata->dev, &mt6720_charger_otg_rdesc, &config);
	else
		cdata->rdev = devm_regulator_register(cdata->dev, &mt6379_charger_otg_rdesc, &config);
	return PTR_ERR_OR_ZERO(cdata->rdev);
}

static char *mt6379_psy_supplied_to[] = {
	"battery",
	"mtk-master-charger",
};

static enum power_supply_usb_type mt6379_charger_psy_usb_types[] = {
	POWER_SUPPLY_USB_TYPE_UNKNOWN,
	POWER_SUPPLY_USB_TYPE_SDP,
	POWER_SUPPLY_USB_TYPE_CDP,
	POWER_SUPPLY_USB_TYPE_DCP,
};

static enum power_supply_property mt6379_charger_properties[] = {
	POWER_SUPPLY_PROP_MANUFACTURER,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_PRECHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_USB_TYPE,
	POWER_SUPPLY_PROP_CURRENT_MAX,
	POWER_SUPPLY_PROP_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_CALIBRATE,
	POWER_SUPPLY_PROP_ENERGY_EMPTY,
	POWER_SUPPLY_PROP_TYPE,
};

static const char *const mt6379_attach_trig_names[] = {
	"ignore", "pwr_rdy", "typec",
};

static void __maybe_unused mt6379_charger_check_dpdm_ov(struct mt6379_charger_data *cdata,
							int attach)
{
	struct chgdev_notify *mtk_chg_noti = &(cdata->chgdev->noti);
	int ret = 0, vdp = 0, vdm = 0;

	if (attach == ATTACH_TYPE_NONE)
		return;

	/* Check if USB DPDM is Over Voltage */
	ret = iio_read_channel_processed(&cdata->iio_adcs[ADC_CHAN_USBDP], &vdp);
	if (ret < 0)
		dev_notice(cdata->dev, "%s, Failed to read USB DP voltage\n", __func__);

	ret = iio_read_channel_processed(&cdata->iio_adcs[ADC_CHAN_USBDM], &vdm);
	if (ret < 0)
		dev_notice(cdata->dev, "%s, Failed to read USB DM voltage\n", __func__);

	vdp = U_TO_M(vdp);
	vdm = U_TO_M(vdm);
	if (vdp >= DPDM_OV_THRESHOLD_MV || vdm >= DPDM_OV_THRESHOLD_MV) {
		dev_notice(cdata->dev, "%s, USB DPDM OV! valid: %dmV vdp: %dmV,vdm: %dmV\n",
			   __func__, DPDM_OV_THRESHOLD_MV, vdp, vdm);
		mtk_chg_noti->dpdmov_stat = true;
		charger_dev_notify(cdata->chgdev, CHARGER_DEV_NOTIFY_DPDM_OVP);
	}
}

static enum power_supply_type mt6379_charger_get_psy_type(struct mt6379_charger_data *cdata,
							  int idx)
{
	return POWER_SUPPLY_TYPE_USB;
}

static int mt6379_charger_set_online(struct mt6379_charger_data *cdata,
				     enum mt6379_attach_trigger trig, int attach)
{
	struct mt6379_charger_platform_data *pdata = dev_get_platdata(cdata->dev);
	int i = 0, idx = ONLINE_GET_IDX(attach);
	int active_idx = 0, pre_active_idx = 0;
#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
	int ret;
#endif

	mt_dbg(cdata->dev, "%s, trig: %s, attach: 0x%x\n",
	       __func__, mt6379_attach_trig_names[trig], attach);

	/* if attach trigger is not match, ignore it */
	if (pdata->attach_trig != trig) {
		mt_dbg(cdata->dev, "%s, trig: %s ignored!\n",
		       __func__, mt6379_attach_trig_names[trig]);
		return 0;
	}

	attach = ONLINE_GET_ATTACH(attach);

#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
	ret = mt6379_reset_hvdcp_reg(cdata);
	if (ret < 0)
		dev_dbg(cdata->dev, "%s: fail to write hvdcp_device_type\n", __func__);

	if (!attach) {
		cancel_delayed_work_sync(&cdata->hvdcp_work);
		cancel_delayed_work_sync(&cdata->hvdcp_result_check_work);
		cdata->hvdcp_type = POWER_SUPPLY_TYPE_UNKNOWN;
		if (true == is_mtksvooc_project) {
			printk(KERN_ERR "!!! %s: the hvdcp_detach_time:%llu %llu %d %d\n",
				__func__, cdata->hvdcp_detach_time, cdata->hvdcp_detect_time,
				OPLUS_HVDCP_DETECT_TO_DETACH_TIME, cdata->hvdcp_cfg_9v_done);
			if (cdata->hvdcp_cfg_9v_done &&
					(cdata->hvdcp_detach_time - cdata->hvdcp_detect_time
					<= OPLUS_HVDCP_DETECT_TO_DETACH_TIME)) {
				cdata->hvdcp_exit_stat = HVDCP_EXIT_ABNORMAL;

			} else {
				cdata->hvdcp_exit_stat = HVDCP_EXIT_NORMAL;
			}
			cdata->hvdcp_detect_time = 0;
			cdata->hvdcp_detach_time = 0;
			cdata->hvdcp_cfg_9v_done = false;
		}
	}
#endif

	mutex_lock(&cdata->attach_lock);
	if (attach == ATTACH_TYPE_NONE) {
		cdata->bc12_dn[idx] = false;
		/* reset calibrated when plug out */
		cdata->icc_calibrated = false;
	}

	if (!cdata->bc12_dn[idx])
		atomic_set(&cdata->attach[idx], attach);

	active_idx = cdata->active_idx;
	pre_active_idx = active_idx;
	for (i = 0; i < pdata->nr_port; i++) {
		if (atomic_read(&cdata->attach[i]) > ATTACH_TYPE_NONE) {
			active_idx = i;
			break;
		}
	}

	if (pdata->nr_port > 1 && attach == ATTACH_TYPE_TYPEC && !cdata->bc12_dn[idx]) {
		cdata->psy_type[idx] = mt6379_charger_get_psy_type(cdata, idx);
		cdata->bc12_dn[idx] = true;
		switch (cdata->psy_type[idx]) {
		case POWER_SUPPLY_TYPE_USB:
			cdata->psy_usb_type[idx] = POWER_SUPPLY_USB_TYPE_SDP;
			break;
		case POWER_SUPPLY_TYPE_USB_DCP:
		case POWER_SUPPLY_TYPE_APPLE_BRICK_ID:
			cdata->psy_type[idx] = POWER_SUPPLY_TYPE_USB_DCP;
			cdata->psy_usb_type[idx] = POWER_SUPPLY_USB_TYPE_DCP;
			break;
		case POWER_SUPPLY_TYPE_USB_CDP:
			cdata->psy_usb_type[idx] = POWER_SUPPLY_USB_TYPE_CDP;
			break;
		default:
			cdata->psy_type[idx] = POWER_SUPPLY_TYPE_USB;
			cdata->psy_usb_type[idx] = POWER_SUPPLY_USB_TYPE_DCP;
			break;
		}
	}

	cdata->psy_desc.type = cdata->psy_type[active_idx];
	cdata->active_idx = active_idx;
	if ((attach > ATTACH_TYPE_PD && cdata->bc12_dn[idx]) ||
	    (active_idx == pre_active_idx && idx != active_idx)) {
		mutex_unlock(&cdata->attach_lock);
		return 0;
	}

	mutex_unlock(&cdata->attach_lock);

	if (!queue_work(cdata->wq, &cdata->bc12_work))
		dev_notice(cdata->dev, "%s, bc12 work already queued\n", __func__);

	return 0;
}

static inline int mt6379_charger_get_online(struct mt6379_charger_data *cdata, int *online_status)
{
#ifdef OPLUS_FEATURE_CHG_BASIC
	int ret;
	u32 val;
	int attach;

	ret = mt6379_charger_field_get(cdata, F_CHRDET_EXT, &val);
	attach = atomic_read(&cdata->attach[cdata->active_idx]);
	if (!ret && !val && attach) {
		*online_status = val;
		dev_info(cdata->dev,"%s, chrdet:%d attach:%d. \n", __func__, val, attach);
	} else {
		*online_status = attach;
	}
#else
	*online_status = atomic_read(&cdata->attach[cdata->active_idx]);
#endif /* OPLUS_FEATURE_CHG_BASIC */
	return 0;
}

static int mt6379_get_charger_status(struct mt6379_charger_data *cdata, int *psy_status)
{
	u32 stat = 0, chg_en = 0;
	int ret = 0, online = 0;
	const char *attach_name;

	/* Default PSY status */
	*psy_status = POWER_SUPPLY_STATUS_DISCHARGING;

	ret = mt6379_charger_get_online(cdata, &online);
	if (ret) {
		dev_info(cdata->dev, "%s, Failed to get online/attach_type status(ret:%d)\n",
			 __func__, ret);
		return ret;
	}

	/* Not attach anything! */
	if (!online)
		return 0;

	attach_name = get_attach_type_name(online);
	ret = mt6379_charger_field_get(cdata, F_CHG_EN, &chg_en);
	if (ret) {
		dev_info(cdata->dev, "%s, Failed to get CHG_EN(ret:%d)\n", __func__, ret);
		return ret;
	}

	ret = mt6379_charger_field_get(cdata, F_IC_STAT, &stat);
	if (ret) {
		dev_info(cdata->dev, "%s, Failed to get IC_STAT(ret:%d)\n", __func__, ret);
		return ret;
	}

	mt_dbg(cdata->dev, "%s, online/attach_type: %d(%s), ic_stat: %d, chg_en: %d\n",
	       __func__, online, attach_name, stat, chg_en);

	switch (stat) {
	case CHG_STAT_OTG:
		*psy_status = POWER_SUPPLY_STATUS_DISCHARGING;
		return 0;
	case CHG_STAT_SLEEP:
	case CHG_STAT_VBUS_RDY...CHG_STAT_BKGND:
		*psy_status = chg_en ?
			      POWER_SUPPLY_STATUS_CHARGING : POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	case CHG_STAT_DONE:
		*psy_status = POWER_SUPPLY_STATUS_FULL;
		return 0;
	case CHG_STAT_FAULT:
		*psy_status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		return 0;
	default:
		*psy_status = POWER_SUPPLY_STATUS_UNKNOWN;
		return 0;
	};
}

static int mt6379_get_vbat_monitor(struct mt6379_charger_data *cdata, enum mt6379_batpro_src src,
				   u32 *vbat_mon)
{
	u32 vbat_mon_en_field = 0, adc_chan = 0, reg_val = 0, stat;
	int ret = 0;

	if (cdata->id != CHARGER_ID_MT6379 && src == MT6379_BATPRO_SRC_VBAT_MON2) {
		dev_info(cdata->dev, "%s, MT6720 no support dual BAT\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&cdata->cv_lock);

	/* Check if 6pin battery charging is enabled */
	vbat_mon_en_field = src == MT6379_BATPRO_SRC_VBAT_MON2 ? F_VBAT_MON2_EN : F_VBAT_MON_EN;
	ret = mt6379_charger_field_get(cdata, vbat_mon_en_field, &reg_val);
	if (reg_val || ret) {
		if (reg_val)
			dev_info(cdata->dev, "%s, 6pin battery charging is enabled!\n", __func__);
		else
			dev_info(cdata->dev, "%s, Failed to get vbat_mon%s stat(ret:%d)!\n",
				 __func__, src == MT6379_BATPRO_SRC_VBAT_MON2 ? "2" : "", ret);

		ret = -EBUSY;
		goto out;
	}

	/* Enable vbat mon */
	ret = mt6379_charger_field_set(cdata, vbat_mon_en_field, 1);
	if (ret) {
		dev_info(cdata->dev, "%s, Failed to enable vbat_mon%s(ret:%d)!\n",
			 __func__, src == MT6379_BATPRO_SRC_VBAT_MON2 ? "2" : "", ret);
		goto out;
	}

	/* Read vbat mon adc by chg_adc */
	adc_chan = src == MT6379_BATPRO_SRC_VBAT_MON2 ? ADC_CHAN_VBATMON2 : ADC_CHAN_VBATMON;
	ret = iio_read_channel_processed(&cdata->iio_adcs[adc_chan], vbat_mon);
	if (ret) {
		*vbat_mon = 0;
		dev_info(cdata->dev, "%s, Failed to read vbat_mon%s(ret:%d)\n",
			 __func__, src == MT6379_BATPRO_SRC_VBAT_MON2 ? "2" : "", ret);
	}

	*vbat_mon /= 1000;

	/* Disable vbat mon */
	ret = mt6379_charger_field_set(cdata, vbat_mon_en_field, 0);
	if (ret)
		dev_info(cdata->dev, "%s, Failed to disable vbat_mon%s(ret:%d)!\n",
			 __func__, src == MT6379_BATPRO_SRC_VBAT_MON2 ? "2" : "", ret);

	ret = mt6379_charger_field_get(cdata, vbat_mon_en_field, &stat);
	if (ret)
		dev_info(cdata->dev, "%s, Failed to get vbat_mon%s stat(ret:%d)!\n",
			 __func__, src == MT6379_BATPRO_SRC_VBAT_MON2 ? "2" : "", ret);

	dev_info(cdata->dev, "%s, Disable vbat_mon%s (current stat: %d)\n",
		 __func__, src == MT6379_BATPRO_SRC_VBAT_MON2 ? "2" : "", stat);
out:
	mutex_unlock(&cdata->cv_lock);

	return ret;
}

static inline int mt6379_get_bat1_vbat_monitor(struct mt6379_charger_data *cdata, int *vbat_mon_val)
{
	return mt6379_get_vbat_monitor(cdata, MT6379_BATPRO_SRC_VBAT_MON, vbat_mon_val);
}

static inline int __maybe_unused mt6379_get_bat2_vbat_monitor(struct mt6379_charger_data *cdata,
							      int *vbat_mon_val)
{
	return mt6379_get_vbat_monitor(cdata, MT6379_BATPRO_SRC_VBAT_MON2, vbat_mon_val);
}

static const struct linear_range mt6720_ibus_aicr_range;
static const struct linear_range mt6720_vbus_mivr_range;

static int mt6379_charger_get_property(struct power_supply *psy, enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct mt6379_charger_data *cdata = power_supply_get_drvdata(psy);

	if (!cdata)
		return -ENODEV;

	switch (psp) {
	case POWER_SUPPLY_PROP_MANUFACTURER:
		val->strval = "Mediatek";
		return 0;
	case POWER_SUPPLY_PROP_ONLINE:
		return mt6379_charger_get_online(cdata, &val->intval);
	case POWER_SUPPLY_PROP_STATUS:
		return mt6379_get_charger_status(cdata, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		return mt6379_charger_field_get(cdata, F_CC, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		if (cdata->id == CHARGER_ID_MT6720)
			val->intval = linear_range_get_max_value(&mt6720_ibus_aicr_range);
		else
			val->intval = linear_range_get_max_value(&mt6379_charger_ranges[MT6379_RANGE_F_CC]);
		return 0;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		return mt6379_charger_field_get(cdata, F_CV, &val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		if (cdata->id == CHARGER_ID_MT6720)
			val->intval = linear_range_get_max_value(&mt6720_vbus_mivr_range);
		else
			val->intval = linear_range_get_max_value(&mt6379_charger_ranges[MT6379_RANGE_F_CV]);
		return 0;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		return mt6379_charger_field_get(cdata, F_IBUS_AICR, &val->intval);
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		return mt6379_charger_field_get(cdata, F_VBUS_MIVR, &val->intval);
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		return mt6379_charger_field_get(cdata, F_IPREC, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		return mt6379_charger_field_get(cdata, F_IEOC, &val->intval);
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = cdata->psy_usb_type[cdata->active_idx];
		return 0;
	case POWER_SUPPLY_PROP_CURRENT_MAX:
		switch (cdata->psy_usb_type[cdata->active_idx]) {
		case POWER_SUPPLY_USB_TYPE_DCP:
			val->intval = 3225000;	/* 3225 mA */
			return 0;
		case POWER_SUPPLY_USB_TYPE_CDP:
			val->intval = 1500000;	/* 1500 mA */
			return 0;
		case POWER_SUPPLY_USB_TYPE_SDP:
		default:
			val->intval = 500000;	/* 500 mA */
			return 0;
		}
	case POWER_SUPPLY_PROP_VOLTAGE_MAX:
		if (cdata->psy_usb_type[cdata->active_idx] == POWER_SUPPLY_USB_TYPE_DCP)
			val->intval = 22000000;	/* 2200 mV */
		else
			val->intval = 5000000;	/* 500 mV */

		return 0;
	case POWER_SUPPLY_PROP_TYPE:
		val->intval = cdata->psy_desc.type;
		return 0;
	case POWER_SUPPLY_PROP_CALIBRATE: /* for Gauge */
		return mt6379_get_bat1_vbat_monitor(cdata, &val->intval);
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		val->intval = cdata->vbat0_flag;
		return 0;
	default:
		return -EINVAL;
	}
}

static int mt6379_charger_set_property(struct power_supply *psy, enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct mt6379_charger_data *cdata = power_supply_get_drvdata(psy);
	int ret = 0;
	u32 value = 0;
	unsigned int aicr;

	if (!cdata)
		return -ENODEV;

	mt_dbg(cdata->dev, "%s, psp:%d, val = %d\n", __func__, psp, val->intval);
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		return mt6379_charger_set_online(cdata, ATTACH_TRIG_TYPEC, val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		if (test_icc != -1)
			return mt6379_charger_field_set(cdata, F_CC, test_icc);
		return mt6379_charger_field_set(cdata, F_CC, val->intval);
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		return mt6379_charger_field_set(cdata, F_CV, val->intval);
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		if (cdata->bypass_mode_entered)
			return 0;
		ret = mt6379_charger_field_get(cdata, F_IBUS_AICR, &value);
		if (ret)
			dev_info(cdata->dev, "%s, get F_IBUS_AICR failed\n", __func__);

		if (test_aicr != -1)
			aicr = test_aicr;
		else
			aicr = val->intval;
		ret = mt6379_charger_field_set(cdata, F_IBUS_AICR, aicr);
		if (ret)
			dev_info(cdata->dev, "%s, set F_IBUS_AICR failed\n", __func__);
		if (value != aicr && (value < 500000 || aicr < 500000)) {
			dev_info(cdata->dev, "%s, new aicr = %d, old aicr = %d\n",
				 __func__, aicr, value);
			cdata->fsw_check_nr = 0;
		}
		return ret;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		if (cdata->bypass_mode_entered)
			return 0;
		if (test_mivr != -1)
			return mt6379_charger_field_set(cdata, F_VBUS_MIVR, test_mivr);
		return mt6379_charger_field_set(cdata, F_VBUS_MIVR, val->intval);
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		return mt6379_charger_field_set(cdata, F_IPREC, val->intval);
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		return mt6379_charger_field_set(cdata, F_IEOC, val->intval);
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		cdata->vbat0_flag = val->intval;
		return 0;
	default:
		return -EINVAL;
	};
}

static int mt6379_charger_property_is_writeable(struct power_supply *psy,
						enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_ENERGY_EMPTY:
		return 1;
	default:
		return 0;
	};
}

static const struct power_supply_desc mt6379_charger_psy_desc = {
	.name = "mt6379-charger",
	.type = POWER_SUPPLY_TYPE_USB,
	.usb_types = mt6379_charger_psy_usb_types,
	.num_usb_types = ARRAY_SIZE(mt6379_charger_psy_usb_types),
	.properties = mt6379_charger_properties,
	.num_properties = ARRAY_SIZE(mt6379_charger_properties),
	.get_property = mt6379_charger_get_property,
	.set_property = mt6379_charger_set_property,
	.property_is_writeable = mt6379_charger_property_is_writeable,
};

#ifdef OPLUS_FEATURE_CHG_BASIC
bool mt6379_int_chrdet_attach(void)
{
	struct mt6379_charger_data *cdata = oplus_cdata;
	int ret;
	u32 val;

	if (NULL == oplus_cdata) {
		return -EINVAL;
	}

	ret = mt6379_charger_field_get(oplus_cdata, F_CHRDET_EXT, &val);
	if (ret)
		dev_info(cdata->dev, "%s, Failed to get F_CHRDET_EXT\n", __func__);

	if (!val) {
		mt_dbg(cdata->dev, "int chrdet dettach, val = %d\n", val);
		return false;
	} else {
		mt_dbg(cdata->dev, "int chrdet attach, val = %d\n", val);
		return true;
	}
}
EXPORT_SYMBOL(mt6379_int_chrdet_attach);
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
int mt6379_set_shipping_mode(struct mt6379_charger_data *cdata)
#else
static int mt6379_set_shipping_mode(struct mt6379_charger_data *cdata)
#endif
{
	int ret = 0;

	ret = mt6379_charger_field_set(cdata, F_SHIP_RST_DIS, 1);
	if (ret) {
		dev_info(cdata->dev, "%s, Failed to disable ship reset\n", __func__);
		return ret;
	}

#ifdef OPLUS_FEATURE_CHG_BASIC
	ret = mt6379_charger_field_set(cdata, F_BATFET_DISDLY, 1);
#else
	ret = mt6379_charger_field_set(cdata, F_BATFET_DISDLY, 0);
#endif
	if (ret) {
		dev_info(cdata->dev, "%s, Failed to disable ship mode delay\n", __func__);
		return ret;
	}

	/* To shutdown system even with TA, disable buck_en here */
	ret = mt6379_charger_field_set(cdata, F_BUCK_EN, 0);
	if (ret) {
		dev_info(cdata->dev, "%s, Failed to disable chg buck en\n", __func__);
		return ret;
	}

	/*
	 * MT6379 if using multi key, please set only pwrkey before enter shipping mode
	 */
	if (cdata->id == CHARGER_ID_MT6379) {
		dev_info(cdata->dev, "%s, MT6379 need to disable homekey rst before enter shipping\n",
			 __func__);
		ret = regmap_update_bits(cdata->rmap, MT6379_REG_CHG_TOP1, 0x06, 0);
		if (ret)
			dev_info(cdata->dev, "%s, disable homekey rst en failend\n", __func__);
	}

	return mt6379_charger_field_set(cdata, F_BATFET_DIS, 1);
}

static ssize_t shipping_mode_store(struct device *dev, struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct mt6379_charger_data *cdata = power_supply_get_drvdata(to_power_supply(dev));
	unsigned long magic = 0;
	int ret = 0;

	ret = kstrtoul(buf, 0, &magic);
	if (ret) {
		dev_info(dev, "%s, Failed to parse number\n", __func__);
		return ret;
	}

	if (magic != 5526789)
		return -EINVAL;

	ret = mt6379_set_shipping_mode(cdata);
	return ret < 0 ? ret : count;
}
static DEVICE_ATTR_WO(shipping_mode);

static int mt6379_get_ipeak(struct mt6379_charger_data *cdata)
{
	u32 ibus = 0, vbus = 0, vsys = 0, ipeak = 0;
	struct device *dev = cdata->dev;
	int ret = 0;

	ret = iio_read_channel_processed(&cdata->iio_adcs[ADC_CHAN_CHGVIN], &vbus);
	if (ret) {
		dev_info(dev, "%s get vbus failed\n", __func__);
		return ret;
	}

	ret = iio_read_channel_processed(&cdata->iio_adcs[ADC_CHAN_VSYS], &vsys);
	if (ret) {
		dev_info(dev, "%s get vsys failed\n", __func__);
		return ret;
	}

	ret = iio_read_channel_processed(&cdata->iio_adcs[ADC_CHAN_IBUS], &ibus);
	if (ret) {
		dev_info(dev, "%s get ibus failed\n", __func__);
		return ret;
	}

	vbus = U_TO_M(vbus);
	ibus = U_TO_M(ibus);
	vsys = U_TO_M(vsys);

	ipeak = (vbus * ibus) * 9 / (vsys * 10) + (vbus - vsys) * vsys / (2 * vbus);

	dev_info(cdata->dev, "%s ipeak = %d, ibus = %d, vbus = %d, vsys = %d\n",
		 __func__, ipeak, ibus, vbus, vsys);
	return ipeak;
}

/*
 * Turn off battery device to make sure ibat is only from mt6379 to battery
 */
static void customer_turn_off_battery_device(void)
{
}

/*
 * Turn on battery device after calibration done
 */
static void customer_turn_on_battery_device(void)
{
}

static int mt6379_get_gauge_ibat_ua(struct mt6379_charger_data *cdata)
{
	int ibat = 0, i = 0, samples = 5, ret = 0;
	union power_supply_propval cic1 = {};

	if (!cdata->bat)
		cdata->bat = devm_power_supply_get_by_phandle(cdata->dev, "gauge");
	if (IS_ERR_OR_NULL(cdata->bat)) {
		dev_info(cdata->dev, "%s, Can't Get battery power supply\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < samples; i++) {
		ret = power_supply_get_property(cdata->bat, POWER_SUPPLY_PROP_CURRENT_NOW, &cic1);
		if (ret) {
			dev_info(cdata->dev, "%s, get gauge ibat failed\n", __func__);
			return -EINVAL;
		}
		dev_info(cdata->dev, "%s, gauge ibat = %d\n", __func__, cic1.intval);
		ibat += cic1.intval;
		mdelay(2);
	}

	return ibat / samples;
}

static int mt6379_icc_check(struct mt6379_charger_data *cdata)
{
	struct device *dev = cdata->dev;
	int ret = 0, ibat = 0;
	u32 icc = 0;

	ret = mt6379_charger_field_get(cdata, F_CC, &icc);
	if (ret) {
		dev_info(dev, "%s, get CC setting failed\n", __func__);
		return -EINVAL;
	}

	ibat = mt6379_get_gauge_ibat_ua(cdata);
	if (ibat < 0) {
		dev_info(dev, "%s get gauge ibat failed\n", __func__);
		return -EINVAL;
	}

	if (abs(ibat - icc) < MT6379_ICC_DIFF_MAX) {
		dev_info(dev, "%s, Pass ibat - icc = %duA\n", __func__, ibat - icc);
		return 0;
	}
	dev_info(dev, "%s, Failed ibat -icc = %duA\n", __func__, ibat - icc);
	return -EINVAL;
}

/*
 * Load/Save ICC Sensing Offset Value.
 * This value is from NVRAM or SET_ENV or other method, to avoid "cannot enter calibration
 * flow"
 *
 * Please implement it at your platform
 */
static int __maybe_unused mt6379_load_save_icc_offset(void)
{
	/* Load value from phone, 0 is example */
	return 0;
}

static int mt6379_store_save_icc_offset(int offset)
{
	/* Store value to phone */

	pr_info("%s, offset = %d\n", __func__, offset);
	return 0;
}

static int mt6379_trim_icc_offset(struct mt6379_charger_data *cdata, int offset, bool check)
{
	int ret = 0, result = 0;
	u32 val = 0;

	ret = mt6379_charger_field_get(cdata, F_ICC_OFFSET, &val);
	if (ret) {
		dev_info(cdata->dev, "%s get icc offset failed\n", __func__);
		return ret;
	}

	result = val + offset;
	if (result > 0xFF)
		result = 0xFF;
	else if (result < 0)
		result = 0;

	ret = mt6379_enable_tm(cdata, true);
	if (ret) {
		dev_info(cdata->dev, "%s enter tm failed\n", __func__);
		return ret;
	}

	if (!cdata->icc_trimmed) {
		ret = mt6379_charger_field_set(cdata, F_ICC_ORIGIN, val);
		if (ret)
			dev_info(cdata->dev, "%s, set icc sensing data origin failed\n", __func__);
	}

	if (offset) {
		ret = mt6379_charger_field_set(cdata, F_ICC_OFFSET, result);
		if (ret)
			dev_info(cdata->dev, "%s set icc offset failed\n", __func__);
	}

	ret = mt6379_charger_field_set(cdata, F_ICC_TRIMMED, 1);
	if (ret)
		dev_info(cdata->dev, "%s, set icc trimmed failed\n", __func__);
	else {
		ret = mt6379_charger_field_get(cdata, F_ICC_ORIGIN, &val);
		if (ret) {
			dev_info(cdata->dev, "%s, get F_ICC_ORGIN failed\n", __func__);
			goto failed;
		}
		cdata->icc_trimmed = true;
		cdata->icc_offset = result - val;
		if (check)
			schedule_delayed_work(&cdata->icc_check_work,
					      msecs_to_jiffies(icc_check_time));
	}

	dev_info(cdata->dev, "%s, offset = %d, icc_offset = %d, result = 0x%02x\n",
		 __func__, offset, cdata->icc_offset, result);
failed:
	ret = mt6379_enable_tm(cdata, false);
	if (ret)
		dev_info(cdata->dev, "%s exit tm failed\n", __func__);
	return ret;
}

static int mt6379_do_icc_calibrate(struct mt6379_charger_data *cdata, int offset_ua)
{
	int ret = 0, offset = 0;

	dev_info(cdata->dev, "%s offset_ua = %d\n", __func__, offset_ua);

	if (offset_ua > MT6379_SENSING_OFFSET_MAX)
		offset = (MT6379_SENSING_OFFSET_MAX / 10000) - cdata->saved_icc_offset;
	else if (offset_ua < -MT6379_SENSING_OFFSET_MAX)
		offset = -(MT6379_SENSING_OFFSET_MAX / 10000) - cdata->saved_icc_offset;
	else
		offset = offset_ua > 0 ? (offset_ua + 5000) / (10 * 1000) :
			(offset_ua - 5000) / (10 * 1000);

	dev_info(cdata->dev, "%s offset = %d\n", __func__, offset);
	ret = mt6379_trim_icc_offset(cdata, offset, true);
	if (ret)
		dev_info(cdata->dev, "%s, trim icc offset failed\n", __func__);
	else /* calibration only once when plug in */
		cdata->icc_calibrated = true;
	return ret;
}

static int mt6379_icc_calibrate(struct mt6379_charger_data *cdata)
{
	u32 val = 0, cv = 0, vrec = 0, vbat = 0, chg_ocp = 0, icc = 0, ibat = 0;
	struct device *dev = cdata->dev;
	int ret = 0, ipeak = 0;

	if (cdata->icc_calibrated)
		return 0;

	/* check ic stat is fast chrging */
	ret = mt6379_charger_field_get(cdata, F_IC_STAT, &val);
	if (ret) {
		dev_info(cdata->dev, "%s, Failed to get IC_STAT(ret:%d)\n", __func__, ret);
		goto not_finished;
	}
	if (val != CHG_STAT_FAST) {
		dev_info(cdata->dev, "%s, ic_stat:%d is not fast charging\n", __func__, val);
		goto not_finished;
	}

	/* check vsys_min */
	ret = regmap_read(cdata->rmap, MT6379_REG_CHG_STAT2, &val);
	if (ret) {
		dev_info(dev, "%s, Failed to read chg stat2\n", __func__);
		goto not_finished;
	}
	if (val & BIT(1)) {
		dev_info(dev, "%s, VSYS < VSYS_MIN, %x\n", __func__, val);
		ret = -EAGAIN;
		goto not_finished;
	}

	/* check AICR, MIVR, THERMAL Loop */
	ret = regmap_read(cdata->rmap, MT6379_REG_CHG_STAT1, &val);
	if (ret) {
		dev_info(dev, "%s, Failed to read chg stat1\n", __func__);
		goto not_finished;
	}
	if (val & GENMASK(7, 5)) {
		dev_info(dev, "%s, Charger in AICR, MIVR, THERMAL Loop %x\n", __func__, val);
		ret = -EAGAIN;
		goto not_finished;
	}

	ret = mt6379_charger_field_get(cdata, F_CV, &cv);
	if (ret) {
		dev_info(dev, "%s, get CV Failed\n", __func__);
		goto not_finished;
	}

	ret = mt6379_charger_field_get(cdata, F_VREC, &vrec);
	if (ret) {
		dev_info(dev, "%s, get VREC Failed\n", __func__);
		goto not_finished;
	}

	ret = iio_read_channel_processed(&cdata->iio_adcs[ADC_CHAN_VBAT], &vbat);
	if (ret) {
		dev_info(dev, "%s, get vbat failed\n", __func__);
		goto not_finished;
	}

	if (cdata->vbat >= cv - vrec) {
		dev_info(dev, "%s, VBAT >= CV - VREC, (%d, %d, %d)\n",
			 __func__, vbat, cv, vrec);
		ret = -EAGAIN;
		goto not_finished;
	}

	ret = mt6379_charger_field_get(cdata, F_CHG_OCP, &chg_ocp);
	if (ret) {
		dev_info(dev, "%s, get chg ocp failed\n", __func__);
		goto not_finished;
	}

	chg_ocp = chg_ocp ? 8000 : 6000;
	ipeak = mt6379_get_ipeak(cdata);

	/* mA */
	if (ipeak >= chg_ocp || ipeak < 0) {
		dev_info(dev, "%s, IL_PEAK >= CHG_OCP or invalid, ipeak = %d\n", __func__, ipeak);
		ret = -EAGAIN;
		goto not_finished;
	}

	ret = mt6379_charger_field_get(cdata, F_CC, &icc);
	if (ret) {
		dev_info(dev, "%s, get CC setting failed\n", __func__);
		goto not_finished;
	}

	customer_turn_off_battery_device();
	/* MT6379 or 3rd Party Gauge Ibat Read */
	ibat = mt6379_get_gauge_ibat_ua(cdata);
	if (ibat < 0) {
		ret = -EAGAIN;
		customer_turn_on_battery_device();
		goto not_finished;
	}

	ret = mt6379_do_icc_calibrate(cdata, ibat - icc);
	if (ret)
		dev_info(dev, "%s, mt6379 do icc calibrate failed\n", __func__);

	customer_turn_on_battery_device();
not_finished:
	/* if calibration is not finish, schedule work again */
	if (!cdata->icc_calibrated)
		schedule_delayed_work(&cdata->icc_cali_work,
				      msecs_to_jiffies(MT6379_CHECK_ICC_TIME));
	return ret;
}

/*
 * IF POR occurred, trimmed bit will be reset to 0
 * This bit stored trimmed already or not. Even offset is 0
 */
static bool mt6379_get_icc_trimmed(struct mt6379_charger_data *cdata)
{
	int ret = 0;
	u32 val = 0;

	ret = mt6379_charger_field_get(cdata, F_ICC_TRIMMED, &val);
	if (ret)
		dev_info(cdata->dev, "%s get icc trimmed failed\n", __func__);

	return ret == 0 ? !!val : false;
}

static ssize_t icc_cali_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct mt6379_charger_data *cdata = power_supply_get_drvdata(to_power_supply(dev));
	unsigned long magic = 0;
	int ret = 0;

	ret = kstrtoul(buf, 0, &magic);
	if (ret) {
		dev_info(dev, "%s, Failed to parse number\n", __func__);
		return ret;
	}

	if (magic == 0)
		cancel_delayed_work_sync(&cdata->icc_cali_work);
	else if (magic == 5526789 && cdata->id == CHARGER_ID_MT6379)
		ret = mt6379_icc_calibrate(cdata);

	return ret < 0 ? ret : count;
}

static ssize_t icc_cali_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mt6379_charger_data *cdata = power_supply_get_drvdata(to_power_supply(dev));
	int ret = 0;
	u32 val = 0;

	ret = mt6379_charger_field_get(cdata, F_ICC_TRIMMED, &val);
	if (ret)
		dev_info(dev, "%s, Failed to get icc trimmed field\n", __func__);

	return sysfs_emit(buf, "%d\n", val);
}
static DEVICE_ATTR_RW(icc_cali);

static const u16 mt6xxx_charger_bypass_iq[] = {
	MT6379_REG_CHG_BYPASS_IQ,
	MT6720_REG_CHG_BYPASS_IQ,
};

static ssize_t bypass_iq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mt6379_charger_data *cdata = power_supply_get_drvdata(to_power_supply(dev));
	unsigned int val = 0;
	u8 id = cdata->id;
	int ret = 0;

	if (id >= ARRAY_SIZE(mt6xxx_charger_bypass_iq))
		return -EINVAL;
	ret = regmap_read(cdata->rmap, mt6xxx_charger_bypass_iq[id], &val);
	if (ret)
		return ret;

	dev_info(dev, "%s, reg = %x, val = 0x%02x\n", __func__, mt6xxx_charger_bypass_iq[id], val);
	return sysfs_emit(buf, "%d uA\n", 20 * (1 + val));
}
static DEVICE_ATTR_RO(bypass_iq);

static ssize_t bypass_mode_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct mt6379_charger_data *cdata = power_supply_get_drvdata(to_power_supply(dev));

	if (cdata->bypass_mode_entered == 1)
		return sysfs_emit(buf, "bypass mode\n");
	if (cdata->bypass_mode_entered >= 2)
		return sysfs_emit(buf, "bypass mode with reset\n");

	return sysfs_emit(buf, "bypass mode off\n");
}

static ssize_t bypass_mode_store(struct device *dev, struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct mt6379_charger_data *cdata = power_supply_get_drvdata(to_power_supply(dev));
	u8 rst_code[2] = { 0xA9, 0x96 };
	unsigned long enter = 0;
	int ret = 0;

	ret = kstrtoul(buf, 0, &enter);
	if (ret)
		return ret;

	if (enter == cdata->bypass_mode_entered)
		return count;

	if (enter) {
		ret = mt6379_charger_field_set(cdata, F_VBUS_MIVR, 3900000);
		if (ret) {
			dev_info(cdata->dev, "%s, Failed to set MIVR to 3900mV\n", __func__);
			return ret;
		}

		ret = mt6379_charger_field_set(cdata, F_IBUS_AICR, 3000000);
		if (ret) {
			dev_info(cdata->dev, "%s, Failed to set AICR to 3000mA\n", __func__);
			return ret;
		}

		ret = mt6379_charger_field_set(cdata, F_CHG_BYPASS, 1);
		if (ret) {
			dev_info(cdata->dev, "%s, Failed to enable bypass mode\n", __func__);
			return ret;
		}

		if (enter >= 2) {
			ret = mt6379_charger_field_set(cdata, F_MREN, 0);
			if (ret) {
				dev_info(cdata->dev, "%s, Failed to disable MRSTB\n", __func__);
				return ret;
			}

			/* enter reset pas code */
			ret = regmap_bulk_write(cdata->rmap, MT6379_REG_RST_PAS_CODE1, rst_code, 2);
			if (ret) {
				dev_info(cdata->dev, "%s, Failed to reset pas code\n", __func__);
				return ret;
			}

			/* REG_RST */
			ret = mt6379_charger_field_set(cdata, F_REG_RST, 1);
			if (ret) {
				dev_info(cdata->dev, "%s, Failed to reset pmu rg\n", __func__);
				return ret;
			}

			/* PD SW_RESET */
			ret = mt6379_charger_field_set(cdata, F_PD_SWRST, 1);
			if (ret) {
				dev_info(cdata->dev, "%s, Failed to reset pmu rg\n", __func__);
				return ret;
			}

			mdelay(2);
			/* PD disable OTP_HW_EN */
			ret = mt6379_charger_field_set(cdata, F_PD_OTP_HWEN, 0);
			if (ret) {
				dev_info(cdata->dev, "%s, Failed to disable TYPEC OTP_HW_EN\n",
					 __func__);
				return ret;
			}
		}
	} else {
		ret = mt6379_charger_field_set(cdata, F_MREN, 1);
		if (ret) {
			dev_info(cdata->dev, "%s, Failed to enable MRSTB\n", __func__);
			return ret;
		}

		ret = mt6379_charger_field_set(cdata, F_CHG_BYPASS, 0);
		if (ret) {
			dev_info(cdata->dev, "%s, Failed to disable bypass mode\n", __func__);
			return ret;
		}
	}

	cdata->bypass_mode_entered = enter;
	return count;
}
static DEVICE_ATTR_RW(bypass_mode);

static ssize_t test_mode_store(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct mt6379_charger_data *cdata = power_supply_get_drvdata(to_power_supply(dev));
	int ret = 0;
	bool enter_state = false;

	ret = kstrtobool(buf, &enter_state);
	if (ret)
		return ret;

	ret = mt6379_enable_tm(cdata, enter_state);
	if (ret)
		return ret;
	return count;
}
static DEVICE_ATTR_WO(test_mode);

static struct attribute *mt6379_charger_psy_sysfs_attrs[] = {
	&dev_attr_bypass_mode.attr,
	&dev_attr_shipping_mode.attr,
	&dev_attr_bypass_iq.attr,
	&dev_attr_icc_cali.attr,
	&dev_attr_test_mode.attr,
	NULL
};
ATTRIBUTE_GROUPS(mt6379_charger_psy_sysfs); /* mt6379_charger_psy_sysfs_groups */

static int mt6379_charger_init_psy(struct mt6379_charger_data *cdata)
{
	struct mt6379_charger_platform_data *pdata = dev_get_platdata(cdata->dev);
	struct power_supply_config config = {
		.drv_data = cdata,
		.of_node = dev_of_node(cdata->dev),
		.supplied_to = mt6379_psy_supplied_to,
		.num_supplicants = ARRAY_SIZE(mt6379_psy_supplied_to),
		.attr_grp = mt6379_charger_psy_sysfs_groups,
	};

	memcpy(&cdata->psy_desc, &mt6379_charger_psy_desc, sizeof(cdata->psy_desc));
	cdata->psy_desc.name = pdata->chgdev_name;
	cdata->psy = devm_power_supply_register(cdata->dev, &cdata->psy_desc, &config);

	return PTR_ERR_OR_ZERO(cdata->psy);
}

static const char *const mt6379_port_stat_names[] = {
	[PORT_STAT_NOINFO] = "No Info",
	[PORT_STAT_APPLE_10W] = "Apple 10W",
	[PORT_STAT_SS_TA] = "SS",
	[PORT_STAT_APPLE_5W] = "Apple 5W",
	[PORT_STAT_APPLE_12W] = "Apple 12W",
	[PORT_STAT_UNKNOWN_TA] = "Unknown TA",
	[PORT_STAT_SDP] = "SDP",
	[PORT_STAT_CDP] = "CDP",
	[PORT_STAT_DCP] = "DCP",
};

enum mt6379_usbsw {
	USBSW_CHG = 0,
	USBSW_USB,
};

#define PHY_MODE_BC11_SET 1
#define PHY_MODE_BC11_CLR 2
static int mt6379_charger_set_usbsw(struct mt6379_charger_data *cdata, enum mt6379_usbsw usbsw)
{
	int ret = 0, mode = (usbsw == USBSW_CHG) ? PHY_MODE_BC11_SET : PHY_MODE_BC11_CLR;
	struct phy *phy;

	mt_dbg(cdata->dev, "%s, usbsw = %d\n", __func__, usbsw);

	phy = phy_get(cdata->dev, "usb2-phy");
	if (IS_ERR_OR_NULL(phy)) {
		dev_info(cdata->dev, "%s, Failed to get usb2-phy\n", __func__);
		return -ENODEV;
	}

	ret = phy_set_mode_ext(phy, PHY_MODE_USB_DEVICE, mode);
	if (ret)
		dev_info(cdata->dev, "%s, Failed to set phy ext mode\n", __func__);

	phy_put(cdata->dev, phy);
	return ret;
}

static bool mt6379_charger_is_usb_rdy(struct device *dev)
{
	struct device_node *node;
	bool ready = true;

	node = of_parse_phandle(dev->of_node, "usb", 0);
	if (node) {
		ready = !of_property_read_bool(node, "cdp-block");
		mt_dbg(dev, "%s, usb ready = %d\n", __func__,  ready);
	} else
		mt_dbg(dev, "%s, usb node missing or invalid\n", __func__);

	return ready;
}

#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
int mt6379_get_hvdcp_type(void)
{
	if (!oplus_cdata) {
		printk(KERN_ERR "%s NULL\n", __func__);
		return POWER_SUPPLY_TYPE_USB_DCP;
	}

	return oplus_cdata->hvdcp_type;
}
EXPORT_SYMBOL(mt6379_get_hvdcp_type);

void mt6379_enable_hvdcp_detect(void)
{
	int ret = 0;

	if (!oplus_cdata) {
		printk(KERN_ERR "%s oplusmpci NULL\n", __func__);
		return ;
	}

	dev_err(oplus_cdata->dev, "%s\n", __func__);
	oplus_cdata->oplus_hvdcp_detect = true;
	printk("%s: enable hvdcp detect is_mtksvooc_project = %d %d\n",
		__func__, is_mtksvooc_project, oplus_cdata->hvdcp_exit_stat);

	if (true == is_mtksvooc_project) {
		if (HVDCP_EXIT_NORMAL == oplus_cdata->hvdcp_exit_stat){
			regmap_update_bits(oplus_cdata->rmap, MT6379_REG_DPDM_CTRL1, 0xff, 0x00);
			goto enable_hvdcp;
		} else {
			printk("%s: HVDCP_EXIT_ABNORMAL not enable hvdcp \n", __func__);
			return;
		}
	}

enable_hvdcp:
	mt6379_charger_enable_bc12(oplus_cdata, false);
	mt6379_charger_enable_bc12(oplus_cdata, true);

	msleep(1000);

	if (oplus_cdata->oplus_get_hvdcp_bc12_result == true) {
		printk("%s HVDCP retry bc12 get result is DCP!\n", __func__);
		ret = mt6379_reset_hvdcp_reg(oplus_cdata);
		if (ret < 0)
			printk("cannot reset all the hvdcp reg!\n");
		ret = mt6379_set_hvdcp_detect_enable(oplus_cdata);
		if (ret < 0)
			printk("%s: fail to enable hvdcp detect\n", __func__);
		/* mt6379_chg_set_usbsw(oplus_cdata, USBSW_CHG); */
		schedule_delayed_work(&oplus_cdata->hvdcp_result_check_work, msecs_to_jiffies(1000));
		printk(" %s: start hvdcp_result_check_work\n", __func__);
	} else {
		printk("HVDCP retry bc12 get result is not DCP!\n");
	}

	oplus_cdata->oplus_get_hvdcp_bc12_result = false;
}
EXPORT_SYMBOL(mt6379_enable_hvdcp_detect);

void oplus_set_hvdcp_flag_clear(void)
{
	if (oplus_cdata == NULL)
		return;

	oplus_cdata->oplus_get_hvdcp_bc12_result = false;
	oplus_cdata->oplus_hvdcp_detect = false;
}
EXPORT_SYMBOL(oplus_set_hvdcp_flag_clear);
#endif

static int mt6379_charger_enable_bc12(struct mt6379_charger_data *cdata, bool en)
{
	static const int max_wait_cnt = 250;
	struct device *dev = cdata->dev;
	int i = 0, ret = 0, attach = 0;

	mt_dbg(dev, "%s, en = %d%s\n", __func__, en, en ? "check CDP block" : "");
	if (en) {
		/* CDP port specific process */
		for (i = 0; i < max_wait_cnt; i++) {
			if (mt6379_charger_is_usb_rdy(cdata->dev))
				break;

			attach = atomic_read(&cdata->attach[0]);
			if (attach == ATTACH_TYPE_PWR_RDY || attach == ATTACH_TYPE_TYPEC)
				msleep(100);
			else {
				mt_dbg(dev, "%s, change attach:%d, disable bc12\n",
				       __func__, attach);
				en = false;
				break;
			}
		}

		if (i == max_wait_cnt)
			mt_dbg(dev, "%s, CDP timeout\n", __func__);
		else
			mt_dbg(dev, "%s, CDP free\n", __func__);
	}

#ifdef OPLUS_FEATURE_CHG_BASIC
/* oplus add for dp after bc12 */
	if (cdata->psy_desc.type == POWER_SUPPLY_TYPE_USB_DCP &&
		cdata->psy_usb_type[cdata->active_idx] == POWER_SUPPLY_USB_TYPE_DCP)
		ret = mt6379_charger_set_usbsw(cdata, USBSW_CHG);
	else
 		ret = mt6379_charger_set_usbsw(cdata, en ? USBSW_CHG : USBSW_USB);
#else
	ret = mt6379_charger_set_usbsw(cdata, en ? USBSW_CHG : USBSW_USB);
#endif
	if (ret) {
		mt_dbg(dev, "%s, Failed to set usbsw\n", __func__);
		return ret;
	}

	return mt6379_charger_field_set(cdata, F_BC12_EN, en);
}

static int mt6379_charger_toggle_bc12(struct mt6379_charger_data *cdata)
{
	int ret = 0;

	ret = mt6379_charger_enable_bc12(cdata, false);
	if (ret) {
		dev_info(cdata->dev, "%s, Failed to disable bc12(ret:%d)\n", __func__, ret);
		return ret;
	}

	return mt6379_charger_enable_bc12(cdata, true);
}

static void mt6379_charger_icc_check_work_func(struct work_struct *work)
{
	struct mt6379_charger_data *cdata = container_of(work, struct mt6379_charger_data,
							 icc_check_work.work);
	int ret = 0;

	ret = mt6379_icc_check(cdata);
	if (ret) {
		dev_info(cdata->dev, "%s, icc check failed, do not write to phone\n", __func__);
		cdata->icc_calibrated = false;
		schedule_delayed_work(&cdata->icc_cali_work,
				      msecs_to_jiffies(MT6379_CHECK_ICC_TIME));
	} else {
		dev_info(cdata->dev, "%s, icc is good now\n", __func__);
		ret = mt6379_store_save_icc_offset(cdata->icc_offset);
		if (ret)
			dev_info(cdata->dev, "%s, save icc offset to phone failed\n", __func__);
	}
}

static void mt6379_charger_icc_cali_work_func(struct work_struct *work)
{
	struct mt6379_charger_data *cdata = container_of(work, struct mt6379_charger_data,
							 icc_cali_work.work);
	int online = 0, ret = 0;

	dev_info(cdata->dev, "%s, calibrated = %d\n", __func__, cdata->icc_calibrated);

	ret = mt6379_charger_get_online(cdata, &online);
	if (ret)
		dev_info(cdata->dev, "%s, get online failed, ret(%d)\n", __func__, ret);

	if (online) {
		if (mt6379_icc_calibrate(cdata))
			dev_info(cdata->dev, "%s icc calibrate not excute\n", __func__);
	} else
		schedule_delayed_work(&cdata->icc_cali_work,
				      msecs_to_jiffies(MT6379_CHECK_ICC_TIME));
}

static void mt6379_charger_bc12_work_func(struct work_struct *work)
{
	struct mt6379_charger_data *cdata = container_of(work, struct mt6379_charger_data,
							 bc12_work);
	struct mt6379_charger_platform_data *pdata = dev_get_platdata(cdata->dev);
	bool bc12_ctrl = !(pdata->nr_port > 1), bc12_en = false, rpt_psy = true;
	int ret = 0, attach = ATTACH_TYPE_NONE, active_idx = 0;
	const char *attach_name;
	u32 val = 0;

#ifdef OPLUS_FEATURE_CHG_BASIC
	struct tcpc_device *tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL)
		pr_info("%s: get type_c_port0 fail\n", __func__);
#endif

	mutex_lock(&cdata->attach_lock);

	active_idx = cdata->active_idx;
	attach = atomic_read(&cdata->attach[cdata->active_idx]);
	attach_name = get_attach_type_name(attach);
	mt_dbg(cdata->dev, "%s, active_idx: %d, attach: %d(%s)\n",
	       __func__, active_idx, attach, attach_name);

	if (attach > ATTACH_TYPE_NONE && pdata->boot_mode == 5) {
		/* skip bc12 to speed up ADVMETA_BOOT */
		mt_dbg(cdata->dev, "%s, Force SDP in meta mode\n", __func__);
		cdata->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		cdata->psy_type[active_idx] = POWER_SUPPLY_TYPE_USB;
		cdata->psy_usb_type[active_idx] = POWER_SUPPLY_USB_TYPE_SDP;
		goto out;
	}

	switch (attach) {
	case ATTACH_TYPE_NONE:
#ifdef OPLUS_FEATURE_CHG_BASIC
		/* oplus add for bc1.2 retry */
		cdata->bc12_retry = 0;
		oplus_cdata->oplus_hvdcp_detect = false;
		oplus_cdata->oplus_get_hvdcp_bc12_result = false;
#endif
		/* Put UFCS detach event */
		if (active_idx == 0 && cdata->psy_desc.type == POWER_SUPPLY_TYPE_USB_DCP) {
			cdata->wait_for_ufcs_attach = false;
			ufcs_attach_change(cdata->ufcs, false);
		}
#ifdef OPLUS_FEATURE_CHG_BASIC
		/* oplus add to solve the charging icon intermittently */
		cdata->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
		cdata->psy_type[active_idx] = POWER_SUPPLY_TYPE_UNKNOWN;
#else
		cdata->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		cdata->psy_type[active_idx] = POWER_SUPPLY_TYPE_USB;
#endif
		cdata->psy_usb_type[active_idx] = POWER_SUPPLY_USB_TYPE_UNKNOWN;
#ifdef OPLUS_FEATURE_CHG_BASIC
		/* oplus add for charging icon disappears slowly */
		if (oplus_cdata->oplus_hvdcp_detect == false) {
			power_supply_changed(cdata->psy);
			if (tcpc)
				tcpci_notify_bc12_complete_state(tcpc, true);
		}
#endif

		goto out;
	case ATTACH_TYPE_TYPEC:
		if (pdata->nr_port > 1)
			goto out;
		fallthrough;
	case ATTACH_TYPE_PWR_RDY:
		if (!cdata->bc12_dn[active_idx]) {
			bc12_en = true;
			rpt_psy = false;
#ifdef OPLUS_FEATURE_CHG_BASIC
			/* oplus add for pd become usb port */
			cdata->psy_desc.type = POWER_SUPPLY_TYPE_UNKNOWN;
			cdata->psy_usb_type[active_idx] = POWER_SUPPLY_USB_TYPE_UNKNOWN;
#endif
			goto out;
		}

		ret = mt6379_charger_field_get(cdata, F_PORT_STAT, &val);
		if (ret) {
			dev_info(cdata->dev, "%s, Failed to get bc12 port stat\n", __func__);
			rpt_psy = false;
			goto out;
		}

		break;
	case ATTACH_TYPE_PD_SDP:
		val = PORT_STAT_SDP;
		break;
	case ATTACH_TYPE_PD_DCP:
		val = PORT_STAT_DCP;
		break;
	case ATTACH_TYPE_PD_NONSTD:
		val = PORT_STAT_UNKNOWN_TA;
		break;
	default:
		mt_dbg(cdata->dev, "%s, Invalid attach_type: %d\n", __func__, attach);
		break;
	}

	switch (val) {
	case PORT_STAT_NOINFO:
		bc12_ctrl = false;
		rpt_psy = false;
#ifdef OPLUS_FEATURE_CHG_BASIC
		/* oplus add for bc1.2 retry */
		cdata->bc12_retry = 0;
#endif
		mt_dbg(cdata->dev, "%s, No bc12 port info\n", __func__);
		goto out;
	case PORT_STAT_APPLE_5W:
	case PORT_STAT_APPLE_10W:
	case PORT_STAT_APPLE_12W:
	case PORT_STAT_SS_TA:
	case PORT_STAT_DCP:
		cdata->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
		cdata->psy_type[active_idx] = POWER_SUPPLY_TYPE_USB_DCP;
		cdata->psy_usb_type[active_idx] = POWER_SUPPLY_USB_TYPE_DCP;
#ifndef OPLUS_FEATURE_CHG_BASIC
		/* oplus add for dp after bc12 done*/
		bc12_en = true;
#endif
#ifdef OPLUS_FEATURE_CHG_BASIC
		/* oplus add for bc1.2 retry */
		cdata->bc12_retry = 0;
#endif
#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
		if (oplus_cdata->oplus_hvdcp_detect == true) {
			oplus_cdata->oplus_get_hvdcp_bc12_result = true;
#ifdef OPLUS_FEATURE_CHG_BASIC
			/* oplus add for dp bc12 done */
			bc12_en = true;
#endif
		}
#ifdef OPLUS_FEATURE_CHG_BASIC
		/* oplus add for dp bc12 done */
		if (oplus_cdata->oplus_hvdcp_detect == false)
			regmap_update_bits(oplus_cdata->rmap, MT6379_REG_DPDM_CTRL1, 0xff, 0x80);
#endif
		printk("%s: enable hvdcp detect is_mtksvooc_project = %d\n", __func__, is_mtksvooc_project);
		if (true == is_mtksvooc_project) {
			ret = mt6379_reset_hvdcp_reg(cdata);
			if (ret < 0)
				printk("cannot reset all the hvdcp reg!\n");
			printk("vooc dpdm drop gnd\n");
		} else {
			ret = mt6379_reset_hvdcp_reg(cdata);
			if (ret < 0)
				printk("cannot reset all the hvdcp reg!\n");
			ret = mt6379_set_hvdcp_detect_enable(cdata);
			if (ret < 0)
				printk("%s: fail to enable hvdcp detect\n", __func__);
			schedule_delayed_work(&cdata->hvdcp_result_check_work, msecs_to_jiffies(3000));
		}
#endif
		/* Put UFCS attach event */
		if (active_idx == 0 && val == PORT_STAT_DCP) {
			cdata->wait_for_ufcs_attach = true;
			ufcs_attach_change(cdata->ufcs, true);
		}

		break;
	case PORT_STAT_SDP:
#ifdef OPLUS_FEATURE_CHG_BASIC
		/* oplus add for bc1.2 retry and otg recon */
		if (attach == ATTACH_TYPE_PD_SDP)
			cdata->bc12_retry = 0;
		else
			cdata->bc12_retry++;
#endif
#ifndef OPLUS_FEATURE_CHG_BASIC
		/* oplus add for pd become usb port */
		cdata->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		cdata->psy_type[active_idx] = POWER_SUPPLY_TYPE_USB;
		cdata->psy_usb_type[active_idx] = POWER_SUPPLY_USB_TYPE_SDP;
#endif
		break;
	case PORT_STAT_CDP:
#ifdef OPLUS_FEATURE_CHG_BASIC
		/* oplus add for bc1.2 retry */
		cdata->bc12_retry++;
#endif
#ifndef OPLUS_FEATURE_CHG_BASIC
		/* oplus add for pd become usb port */
		cdata->psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
		cdata->psy_type[active_idx] = POWER_SUPPLY_TYPE_USB_CDP;
		cdata->psy_usb_type[active_idx] = POWER_SUPPLY_USB_TYPE_CDP;
#endif
		break;
	case PORT_STAT_UNKNOWN_TA:
#ifdef OPLUS_FEATURE_CHG_BASIC
		/* oplus add for bc1.2 retry */
		bc12_ctrl = false;
		rpt_psy = false;
		cdata->bc12_retry = 0;
#endif
#ifndef OPLUS_FEATURE_CHG_BASIC
		/* oplus modify for charging */
		cdata->psy_desc.type = POWER_SUPPLY_TYPE_USB;
#else
		cdata->psy_desc.type = POWER_SUPPLY_TYPE_USB_DCP;
#endif
		cdata->psy_type[active_idx] = POWER_SUPPLY_TYPE_USB;
		cdata->psy_usb_type[active_idx] = POWER_SUPPLY_USB_TYPE_DCP;
		break;
	default:
		bc12_ctrl = false;
		rpt_psy = false;
		mt_dbg(cdata->dev, "%s, Invalid port stat(%d)\n", __func__, val);
		goto out;
	}
	dev_info(cdata->dev, "port stat = %s\n", mt6379_port_stat_names[val]);
#ifdef OPLUS_FEATURE_CHG_BASIC
	/* oplus add for qc bc1.2 retry */
	if (oplus_cdata->oplus_hvdcp_detect == true)
		cdata->bc12_retry = 0;
#endif
#ifdef OPLUS_FEATURE_CHG_BASIC
	if (cdata->bc12_retry && cdata->bc12_retry < MT6379_BC12_RETRY_CNT) {
		dev_info(cdata->dev, "bc12_retry = %d\n", cdata->bc12_retry);
		bc12_en = true;
		mutex_unlock(&cdata->attach_lock);
		cdata->bc12_dn[0] = false;
		mt6379_charger_enable_bc12(cdata, false);
		goto retry_bc12;
	} else if (cdata->bc12_retry == 0) {
		dev_info(cdata->dev, "report power supply changed, bc12_retry = %d\n", cdata->bc12_retry);
		if (oplus_cdata->oplus_hvdcp_detect == false) {
			power_supply_changed(cdata->psy);
			if (tcpc)
				tcpci_notify_bc12_complete_state(tcpc, true);
		}
	}
#endif
out:
#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
	if (!attach) {
		cancel_delayed_work_sync(&cdata->hvdcp_work);
		cancel_delayed_work_sync(&cdata->hvdcp_result_check_work);
		cdata->hvdcp_type = POWER_SUPPLY_TYPE_UNKNOWN;

		ret = mt6379_reset_hvdcp_reg(cdata);
		if (ret < 0)
			printk("%s: fail to write dpdm_ctrl\n", __func__);
	}
#endif
	mutex_unlock(&cdata->attach_lock);
	if (bc12_ctrl) {
		// mt6379_charger_check_dpdm_ov(cdata, attach);
		if (mt6379_charger_enable_bc12(cdata, bc12_en) < 0)
			dev_info(cdata->dev, "%s, Failed to set bc12 = %d\n", __func__, bc12_en);
	}
#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
	if (rpt_psy && oplus_cdata->oplus_hvdcp_detect == false)
		power_supply_changed(cdata->psy);
#else
	if (rpt_psy) {
		mt_dbg(cdata->dev, "%s power_supply_changed, port stat: %d(%s), attach: %d(%s)\n",
		       __func__, val, mt6379_port_stat_names[val], attach, attach_name);
		power_supply_changed(cdata->psy);
	}
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
	/* oplus add for pd become usb port */
	if ((attach == ATTACH_TYPE_PD_DCP || attach == ATTACH_TYPE_PD_SDP) &&
	     oplus_cdata->oplus_hvdcp_detect == false) {
		if (val == PORT_STAT_SDP)
			cdata->psy_desc.type = POWER_SUPPLY_TYPE_USB;
		power_supply_changed(cdata->psy);
		if (tcpc)
			tcpci_notify_bc12_complete_state(tcpc, true);
		dev_err(cdata->dev, "bc12 work pd_dcp flow\n");
		return;
	}
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
retry_bc12:
	if (bc12_ctrl && (mt6379_charger_enable_bc12(cdata, bc12_en) < 0))
		dev_err(cdata->dev, "failed to set bc12 = %d\n", bc12_en);
	if (rpt_psy) {
		if (cdata->bc12_retry == 1 &&
		    oplus_cdata->oplus_hvdcp_detect == false &&
		    val == PORT_STAT_UNKNOWN_TA) {
			power_supply_changed(cdata->psy);
			if (tcpc)
				tcpci_notify_bc12_complete_state(tcpc, true);
		}

		/*Add for SDP report*/
		if (cdata->bc12_retry == 3 &&
		    oplus_cdata->oplus_hvdcp_detect == false &&
		    val == PORT_STAT_SDP) {
			cdata->psy_desc.type = POWER_SUPPLY_TYPE_USB;
			cdata->psy_usb_type[active_idx] = POWER_SUPPLY_USB_TYPE_SDP;
			power_supply_changed(cdata->psy);
			if (tcpc)
				tcpci_notify_bc12_complete_state(tcpc, true);
		}

		/*Add for CDP report*/
		if (cdata->bc12_retry == 3 &&
		    oplus_cdata->oplus_hvdcp_detect == false &&
		    val == PORT_STAT_CDP) {
			cdata->psy_desc.type = POWER_SUPPLY_TYPE_USB_CDP;
			cdata->psy_usb_type[active_idx] = POWER_SUPPLY_USB_TYPE_CDP;
			power_supply_changed(cdata->psy);
			if (tcpc)
				tcpci_notify_bc12_complete_state(tcpc, true);
		}
	}
#endif
}

int mt6379_charger_set_non_switching_setting(struct mt6379_charger_data *cdata)
{
	struct device *dev = cdata->dev;
	int ret = 0;

	mutex_lock(&cdata->ramp_lock);
	ret = mt6379_enable_tm(cdata, true);
	if (ret)
		dev_info(dev, "%s, Failed to enter tm\n", __func__);
	ret = mt6379_charger_field_set(cdata, F_CHG_RAMP_UP_COMP, 3);
	if (ret)
		dev_info(dev, "%s, Failed to set chg_ramp_up_comp to 240kOhm\n", __func__);

	ret = mt6379_enable_tm(cdata, false);
	if (ret)
		dev_info(dev, "%s, Failed to disable tm\n", __func__);
	cdata->non_switching = true;
	mutex_unlock(&cdata->ramp_lock);
	return ret;
}

static int mt6379_ibus_compensate_check(int vbus, int vsys)
{
	u32 result = 0;
	int diff_vbus_vsys = 0;

	if (!vbus || !vsys)
		return -EINVAL;

	diff_vbus_vsys = vbus - vsys;

	vbus = (vbus + 50) / 100;
	vsys = (vsys + 50) / 100;

	result = 11 * diff_vbus_vsys * vsys * vsys / (18 * vbus * vbus);
	return result;
}

static bool mt6379_check_ramp_need_update(struct mt6379_charger_data *cdata, bool lv)
{
	int ret = 0;
	u32 val = 0;

	if (lv) {
		ret = mt6379_charger_field_get(cdata, F_CHG_RAMP_UP_COMP, &val);
		if (ret || val != 3)
			return true;

		ret = mt6379_charger_field_get(cdata, F_IEOC_FLOW_RB, &val);
		if (ret || val != 0)
			return true;
	} else {
		ret = mt6379_charger_field_get(cdata, F_CHG_RAMP_UP_COMP, &val);
		if (ret || val != 1)
			return true;

		ret = mt6379_charger_field_get(cdata, F_IEOC_FLOW_RB, &val);
		if (ret || val != 1)
			return true;
	}

	return false;
}

static int mt6379_update_comp_ramp(struct mt6379_charger_data *cdata, bool hv)
{
	struct device *dev = cdata->dev;
	u32 ramp_comp = 0, ieoc_flow_rb = 0;
	int ret = 0;

	dev_dbg(dev, "%s, hv = %d\n", __func__, hv);
	if (hv && mt6379_check_ramp_need_update(cdata, false)) {
		ret = mt6379_enable_tm(cdata, true);
		if (ret)
			dev_info(dev, "%s, Failed to enable tm\n", __func__);

		ret = mt6379_charger_field_set(cdata, F_CHG_RAMP_UP_COMP, 1);
		if (ret)
			dev_info(dev, "%s, Failed to set chg_ramp_up_comp to 400kOhm\n",
				 __func__);

		ret = mt6379_charger_field_set(cdata, F_IEOC_FLOW_RB, 1);
		if (ret)
			dev_info(dev, "%s, Failed to set fccm in eoc flow\n", __func__);

		ret = mt6379_charger_field_get(cdata, F_CHG_RAMP_UP_COMP, &ramp_comp);
		if (ret)
			dev_info(dev, "%s, Failed to get chg_ramp_up_comp\n", __func__);
		ret = mt6379_charger_field_get(cdata, F_IEOC_FLOW_RB, &ieoc_flow_rb);
		if (ret)
			dev_info(dev, "%s, Failed to get no fccm in eoc flow\n", __func__);

		dev_info(dev, "%s, update: chg_ramp_up_comp = %d, ieoc_flow_rb = %d\n",
			 __func__, ramp_comp, ieoc_flow_rb);

		ret = mt6379_enable_tm(cdata, false);
		if (ret)
			dev_info(dev, "%s, Failed to disable tm\n", __func__);

	} else if (!hv && mt6379_check_ramp_need_update(cdata, true)) {
		ret = mt6379_enable_tm(cdata, true);
		if (ret)
			dev_info(dev, "%s, Failed to enable tm\n", __func__);
		ret = mt6379_charger_field_set(cdata, F_CHG_RAMP_UP_COMP, 3);
		if (ret)
			dev_info(dev, "%s, Failed to set chg_ramp_up_comp to 240kOhm\n",
				 __func__);

		ret = mt6379_charger_field_set(cdata, F_IEOC_FLOW_RB, 0);
		if (ret)
			dev_info(dev, "%s, Failed to set no fccm in eoc flow\n", __func__);

		ret = mt6379_charger_field_get(cdata, F_CHG_RAMP_UP_COMP, &ramp_comp);
		if (ret)
			dev_info(dev, "%s, Failed to get chg_ramp_up_comp\n", __func__);
		ret = mt6379_charger_field_get(cdata, F_IEOC_FLOW_RB, &ieoc_flow_rb);
		if (ret)
			dev_info(dev, "%s, Failed to get no fccm in eoc flow\n", __func__);

		dev_info(dev, "%s, update: chg_ramp_up_comp = %d, ieoc_flow_rb = %d\n",
			 __func__, ramp_comp, ieoc_flow_rb);

		ret = mt6379_enable_tm(cdata, false);
		if (ret)
			dev_info(dev, "%s, Failed to disable tm\n", __func__);
	}

	return ret;
}

int mt6379_charger_fsw_control(struct mt6379_charger_data *cdata)
{
	struct device *dev = cdata->dev;
	int vbus = 0, ibus = 0, vsys = 0, ibus_check = 0;
	u32 val = 0;
	bool hv_comp = false;
	int ret = 0;

	mutex_lock(&cdata->ramp_lock);

	ret = mt6379_charger_field_get(cdata, F_BUCK_EN, &val);
	if (ret) {
		dev_info(dev, "%s, failed to get buck_en\n", __func__);
		goto out;
	}

	if (val) { /* if buck en is 1 */
		ret = mt6379_charger_field_get(cdata, F_IC_STAT, &val);
		if (ret) {
			dev_info(dev, "%s, failed to get ic stat\n", __func__);
			goto out;
		}

		dev_dbg(cdata->dev, "%s, IC STAT = %d\n", __func__, val);
		switch (val) {
		case CHG_STAT_TRICKLE:
		case CHG_STAT_PRE:
		case CHG_STAT_FAST:
		case CHG_STAT_EOC:
		case CHG_STAT_BKGND:
		case CHG_STAT_DONE:
			cdata->non_switching = false;
			break;
		case CHG_STAT_SLEEP:
		case CHG_STAT_VBUS_RDY:
		case CHG_STAT_FAULT:
		case CHG_STAT_OTG:
		default:
			break;
		}
	}

	if (cdata->non_switching) {
		dev_dbg(dev, "%s, non switching case, do not fsw control\n", __func__);
		goto out;
	}

	ret = iio_read_channel_processed(&cdata->iio_adcs[ADC_CHAN_CHGVIN], &vbus);
	if (ret < 0) {
		dev_info(dev, "%s, get vbus failed\n", __func__);
		goto out;
	}

	/* u -> m */
	vbus = U_TO_M(vbus);

	if (vbus > 5500) {
		hv_comp = true;
		dev_dbg(dev, "%s, vbus = %d mv\n", __func__, vbus);
	} else {
		ret = iio_read_channel_processed(&cdata->iio_adcs[ADC_CHAN_IBUS], &ibus);
		if (ret < 0) {
			dev_info(dev, "%s, get ibus failed\n", __func__);
			goto out;
		}

		ret = iio_read_channel_processed(&cdata->iio_adcs[ADC_CHAN_VSYS], &vsys);
		if (ret < 0) {
			dev_info(dev, "%s, get vsys failed\n", __func__);
			goto out;
		}

		/* u -> m */
		ibus = U_TO_M(ibus);
		vsys = U_TO_M(vsys);

		ibus_check = mt6379_ibus_compensate_check(vbus, vsys);
		if (ibus_check < 0) {
			dev_info(dev, "%s, ibus check failed\n", __func__);
			goto out;
		}
		dev_dbg(dev, "%s, vbus = %d mv, ibus = %d ma, vsys = %d mv, ibus_check = %d\n",
			__func__, vbus, ibus, vsys, ibus_check);

		if (ibus >= ibus_check)
			hv_comp = false;
		else
			hv_comp = true;
	}

	ret = mt6379_update_comp_ramp(cdata, hv_comp);
	if (ret)
		dev_info(dev, "%s, update comp ramp failed\n", __func__);
out:
	mutex_unlock(&cdata->ramp_lock);
	return ret;
}

static void mt6379_charger_switching_work_func(struct work_struct *work)
{
	struct mt6379_charger_data *cdata = container_of(work, struct mt6379_charger_data,
							 switching_work.work);
	int ret = 0;

	mutex_lock(&cdata->ramp_lock);
	cdata->non_switching = false;
	mutex_unlock(&cdata->ramp_lock);
	ret = mt6379_charger_fsw_control(cdata);
	if (ret)
		dev_info(cdata->dev, "%s, fsw control failed\n", __func__);
}

const struct mt6379_charger_dtprop mt6379_charger_dtprops[] = {
	MT6379_CHG_DTPROP("chg-tmr", chg_tmr, F_CHG_TMR, DTPROP_U32),
	MT6379_CHG_DTPROP("chg-tmr-en", chg_tmr_en, F_CHG_TMR_EN, DTPROP_BOOL),
	MT6379_CHG_DTPROP("ircmp-v", ircmp_v, F_IRCMP_V, DTPROP_U32),
	MT6379_CHG_DTPROP("ircmp-r", ircmp_r, F_IRCMP_R, DTPROP_U32),
	MT6379_CHG_DTPROP("wdt-time", wdt_time, F_WDT_TIME, DTPROP_U32),
	MT6379_CHG_DTPROP("wdt-en", wdt_en, F_WDT_EN, DTPROP_BOOL),
	MT6379_CHG_DTPROP("te-en", te_en, F_TE, DTPROP_BOOL),
	MT6379_CHG_DTPROP("mivr", mivr, F_VBUS_MIVR, DTPROP_U32),
	MT6379_CHG_DTPROP("aicr", aicr, F_IBUS_AICR, DTPROP_U32),
	MT6379_CHG_DTPROP("ichg", ichg, F_CC, DTPROP_U32),
	MT6379_CHG_DTPROP("ieoc", ieoc, F_IEOC, DTPROP_U32),
	MT6379_CHG_DTPROP("cv", cv, F_CV, DTPROP_U32),
	MT6379_CHG_DTPROP("vrec", vrec, F_VREC, DTPROP_U32),
	MT6379_CHG_DTPROP("chgin-ov", chgin_ov, F_CHGIN_OV, DTPROP_U32),
	MT6379_CHG_DTPROP("nr-port", nr_port, F_MAX, DTPROP_U32),
};

void mt6379_charger_parse_dt_helper(struct device *dev, void *pdata,
				    const struct mt6379_charger_dtprop *dp)
{
	void *val = pdata + dp->offset;
	int ret = 0;

	if (dp->type == DTPROP_BOOL)
		*((bool *)val) = device_property_read_bool(dev, dp->name);
	else {
		ret = device_property_read_u32(dev, dp->name, val);
		if (ret < 0)
			mt_dbg(dev, "%s, Failed to get \"%s\" property\n", __func__, dp->name);
	}
}

static int mt6379_charger_get_pdata(struct device *dev)
{
	struct mt6379_charger_platform_data *pdata = dev_get_platdata(dev);
	struct device_node *np = dev->of_node, *boot_np, *pmic_uvlo_np;
	struct mt6379_charger_data *cdata = dev_get_drvdata(dev);
	u32 val = 0;
	int i = 0;
	const struct {
		u32 size;
		u32 tag;
		u32 boot_mode;
		u32 boot_type;
	} *tag;

	if (!np)
		return -ENODEV;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata)
		return -ENOMEM;

	memcpy(pdata, &mt6379_charger_pdata_def, sizeof(*pdata));

	for (i = 0; i < ARRAY_SIZE(mt6379_charger_dtprops); i++)
		mt6379_charger_parse_dt_helper(dev, pdata, &mt6379_charger_dtprops[i]);

	pdata->usb_killer_detect = device_property_read_bool(dev, "usb-killer-detect");

	if (of_property_read_string(np, "chgdev-name", &pdata->chgdev_name))
		dev_info(dev, "%s, Failed to get \"chgdev-name\" property\n", __func__);

	boot_np = of_parse_phandle(np, "boot-mode", 0);
	if (!boot_np) {
		dev_info(dev, "%s, Failed to get \"boot-mode\" phandle\n", __func__);
		return -ENODEV;
	}

	tag = of_get_property(boot_np, "atag,boot", NULL);
	if (!tag) {
		dev_info(dev, "%s, Failed to get \"atag,boot\" property\n", __func__);
		return -EINVAL;
	}

	pdata->boot_mode = tag->boot_mode;
	pdata->boot_type = tag->boot_type;
	dev_info(dev, "%s, sz:0x%x tag:0x%x mode:0x%x type:0x%x\n",
		 __func__, tag->size, tag->tag, tag->boot_mode, tag->boot_type);

	if (of_property_read_u32(np, "bc12-sel", &val) < 0 &&
	    of_property_read_u32(np, "bc12_sel", &val) < 0) {
		dev_info(dev, "%s, Failed to get \"bc12-sel/bc12_sel\" property\n", __func__);
		return -EINVAL;
	}

	if (val != MTK_CTD_BY_SUBPMIC && val != MTK_CTD_BY_SUBPMIC_PWR_RDY)
		pdata->attach_trig = ATTACH_TRIG_IGNORE;
	else if (IS_ENABLED(CONFIG_TCPC_CLASS) && val == MTK_CTD_BY_SUBPMIC)
		pdata->attach_trig = ATTACH_TRIG_TYPEC;
	else
		pdata->attach_trig = ATTACH_TRIG_PWR_RDY;

	pmic_uvlo_np = of_parse_phandle(np, "pmic-uvlo", 0);
	if (!pmic_uvlo_np)
		dev_info(dev, "%s, Failed to get \"pmic-uvlo\" phandle\n", __func__);

	if (of_property_read_u32(pmic_uvlo_np, "uvlo-level", &val) < 0)
		dev_info(dev, "%s, Failed to get \"uvlo-level\", use default uvlo level:%dmV\n",
			 __func__, DEFAULT_PMIC_UVLO_MV);

	if (val != 0)
		pdata->pmic_uvlo = val;
	else
		pdata->pmic_uvlo = DEFAULT_PMIC_UVLO_MV;

	if (of_property_read_u32(np, "enable-fsw-control", &val) < 0) {
		dev_info(dev, "%s, Failed to get enable-fsw-control, default enable\n", __func__);
		cdata->enable_fsw = true;
	} else {
		cdata->enable_fsw = !!val;
		dev_info(dev, "%s, enable_fsw = %d\n", __func__, cdata->enable_fsw);
	}

	dev->platform_data = pdata;

	return 0;
}

static u32 pdata_get_val(void *pdata, const struct mt6379_charger_dtprop *dp)
{
	if (dp->type == DTPROP_BOOL)
		return *((bool *)(pdata + dp->offset));

	return *((u32 *)(pdata + dp->offset));
}

static int mt6379_charger_apply_pdata(struct mt6379_charger_data *cdata)
{
	const struct mt6379_charger_dtprop *dp;
	u32 val = 0;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(mt6379_charger_dtprops); i++) {
		dp = &mt6379_charger_dtprops[i];
		if (dp->field >= F_MAX)
			continue;

		val = pdata_get_val(dev_get_platdata(cdata->dev), dp);
		dev_info(cdata->dev, "%s, dp-name = %s, val = %d\n", __func__, dp->name, val);

		ret = mt6379_charger_field_set(cdata, dp->field, val);
		if (ret == -EOPNOTSUPP) {
			dev_info(cdata->dev, "%s, dp-name = %s not support\n", __func__, dp->name);
			continue;
		} else if (ret < 0) {
			dev_info(cdata->dev, "%s, Failed to apply pdata %s\n", __func__, dp->name);
			return ret;
		}
	}

	return 0;
}

static int mt6379_charger_init_setting(struct mt6379_charger_data *cdata)
{
	struct mt6379_charger_platform_data *pdata = dev_get_platdata(cdata->dev);
	struct device *dev = cdata->dev;
	enum mt6379_chip_rev rev;
	int ret = 0;

	ret = mt6379_enable_tm(cdata, true);
	if (ret) {
		dev_info(dev, "%s, Failed to enable tm(ret:%d)\n", __func__, ret);
		return ret;
	}

	rev = mt6379_charger_get_chip_rev(cdata);
	cdata->rev = rev;

	ret = mt6379_charger_field_get(cdata, F_WAFER_ID, &cdata->waferid);
	if (ret)
		dev_info(dev, "%s get wafer id failed\n", __func__);

	dev_info(dev, "%s, using setting rev: %d, waferid = %d\n", __func__, rev, cdata->waferid);

	if (cdata->id == CHARGER_ID_MT6379) {
		/* Turn on Force base function */
		if (rev >= MT6379_CHIP_REV_E2 && rev <= MT6379_CHIP_REV_E4) {
			ret = regmap_update_bits(cdata->rmap, MT6379_REG_VDDA_SUPPLY, BIT(7), BIT(7));
			if (ret)
				dev_info(dev, "%s, Failed to turn on force base function\n", __func__);
		}

	/* Enable pre-UV function */
		if (rev == MT6379_CHIP_REV_E2) {
			ret = regmap_update_bits(cdata->rmap, MT6379_REG_CHG_HD_DIG2, BIT(1), BIT(1));
			if (ret)
				dev_info(dev, "%s, Failed to enable pre-UV vsys_intb\n", __func__);
		} else {
			ret = mt6379_charger_field_set(cdata, F_PREUV_EN, 1);
			if (ret)
				dev_info(dev, "%s, Failed to enable pre-UV vsys_intb\n", __func__);
		}
	} else {
		ret = mt6379_charger_field_set(cdata, F_PREUV_EN, 1);
		if (ret)
			dev_info(dev, "%s, Failed to enable pre-UV vsys_intb\n", __func__);
	}

	ret = mt6379_enable_tm(cdata, false);
	if (ret) {
		dev_info(dev, "%s, Failed to disable tm(ret:%d)\n", __func__, ret);
		return ret;
	}

#ifdef OPLUS_FEATURE_CHG_BASIC
/* oplus add for uvlo & ov */
	ret = mt6379_charger_field_set(cdata, F_CHRD_UV, OPLUS_CHRD_UV_THD);
	if (ret < 0) {
		dev_err(cdata->dev, "failed to set chrdet uv level\n");
		return ret;
	}
	ret = mt6379_charger_field_set(cdata, F_CHRD_OV, OPLUS_CHRD_OV_THD);
	if (ret < 0) {
		dev_err(cdata->dev, "failed to set chrdet ov level\n");
		return ret;
}
#endif

	ret = mt6379_charger_field_set(cdata, F_AICC_ONESHOT, 1);
	if (ret) {
		dev_info(dev, "%s, Failed to set aicc oneshot\n", __func__);
		return ret;
	}

	/* Disable BC12 */
	ret = mt6379_charger_field_set(cdata, F_BC12_EN, 0);
	if (ret) {
		dev_info(dev, "%s, Failed to disable bc12\n", __func__);
		return ret;
	}

	/* OTG LBP set 2.8 V */
	ret = mt6379_charger_field_set(cdata, F_OTG_LBP, 2800000);
	if (ret) {
		dev_info(dev, "%s, Failed to set otb lbp 2.8V\n", __func__);
		return ret;
	}

	/* set aicr = 200mA in 1:META_BOOT 5:ADVMETA_BOOT */
	if (pdata->boot_mode == 1 || pdata->boot_mode == 5) {
		ret = mt6379_charger_field_set(cdata, F_IBUS_AICR, 200000);
		if (ret) {
			dev_info(dev, "%s, Failed to set aicr 200mA\n", __func__);
			return ret;
		}
	}

	ret = mt6379_charger_apply_pdata(cdata);
	if (ret) {
		dev_info(dev, "%s, Failed to apply charger pdata\n", __func__);
		return ret;
	}

	ret = mt6379_charger_field_get(cdata, F_CV, &cdata->cv);
	if (ret) {
		dev_info(dev, "%s, Failed to get CV after apply pdata\n", __func__);
		cdata->cv = 0;
	}

	/* Disable input current limit */
	ret = mt6379_charger_field_set(cdata, F_ILIM_EN, 0);
	if (ret) {
		dev_info(dev, "%s, Failed to disable input current limit\n", __func__);
		return ret;
	}

#ifdef OPLUS_FEATURE_CHG_BASIC
	if (get_eng_version() == HIGH_TEMP_AGING) {
		ret = mt6379_charger_field_set(cdata, F_DIG_THREG_EN, 0);
		if (ret < 0) {
			dev_err(cdata->dev, "failed to disable thermal dig en\n");
			return ret;
		}
	}
#endif

	/*
	 * disable WDT to save 1mA power consumption
	 * it will be turned back on later
	 * if it is enabled int dt property ant TA attached
	 */
	ret = mt6379_charger_field_set(cdata, F_WDT_EN, 0);
	if (ret)
		dev_info(dev, "%s, Failed to disable WDT\n", __func__);

	return ret;
}

static int mt6379_fsw_control(struct mt6379_charger_data *cdata)
{
	int ret = 0;
	u32 val = 0;
	ktime_t time = ktime_set(0, 0);

	if (!cdata->enable_fsw)
		return 0;

	ret = mt6379_charger_field_get(cdata, F_ST_PWR_RDY, &val);
	if (ret)
		dev_info(cdata->dev, "%s, failed to get pwr_rdy\n", __func__);

	if (val) {
		if (fast_fsw_control && cdata->fsw_check_nr < fast_fsw_cnt) {
			time = ktime_set(fsw_ctrl_time, fsw_ctrl_time_ns);
			dev_dbg(cdata->dev, "%s, fsw check nr = %d\n", __func__, cdata->fsw_check_nr);
			cdata->fsw_check_nr++;
		} else
			time = ktime_set(fsw_ctrl_time_2, fsw_ctrl_time_ns_2);

		alarm_start_relative(&cdata->alarm, time);

		ret = mt6379_charger_fsw_control(cdata);
		if (ret)
			dev_info(cdata->dev, "%s, fsw control failed\n", __func__);
	}
	return ret;
}

static void mt6379_fsw_control_func(struct work_struct *work)
{
	struct mt6379_charger_data *cdata = container_of(work,
							 struct mt6379_charger_data,
							 fsw_control_work);
	mt6379_fsw_control(cdata);
}

static enum alarmtimer_restart mt6379_chg_alarm_call(struct alarm *alarm, ktime_t now)
{
	struct mt6379_charger_data *cdata = container_of(alarm, struct mt6379_charger_data , alarm);

	schedule_work(&cdata->fsw_control_work);
	return ALARMTIMER_NORESTART;
}

static void mt6379_charger_check_pwr_rdy(struct mt6379_charger_data *cdata)
{
	struct device *dev = cdata->dev;
	int ret = 0, attach_type = 0;
	u32 val = 0, ramp_comp = 0, ieoc_flow_rb = 0;
	ktime_t time = ktime_set(0, 0);

	ret = mt6379_charger_field_get(cdata, F_ST_PWR_RDY, &val);
	if (ret)
		return;

	attach_type = val ? ATTACH_TYPE_PWR_RDY : ATTACH_TYPE_NONE;
	if (cdata->enable_fsw) {
		if (val) {
			cdata->fsw_check_nr = 0;
			ret = mt6379_fsw_control(cdata);
			if (ret < 0)
				dev_info(cdata->dev, "%s, fsw control failed\n", __func__);
			if (fast_fsw_control)
				time = ktime_set(fsw_ctrl_time, fsw_ctrl_time_ns);
			else
				time = ktime_set(fsw_ctrl_time_2, fsw_ctrl_time_ns_2);
			alarm_start_relative(&cdata->alarm, time);
			dev_info(cdata->dev, "%s, start timer\n", __func__);
		} else {
			alarm_cancel(&cdata->alarm);
			dev_info(cdata->dev, "%s, stop timer\n", __func__);
		}
	}

	if (attach_type == ATTACH_TYPE_NONE) {
		mutex_lock(&cdata->ramp_lock);
		if (!mt6379_check_ramp_need_update(cdata, true)) {
			mutex_unlock(&cdata->ramp_lock);
			goto ramp_no_update;
		}
		ret = mt6379_enable_tm(cdata, true);
		if (ret)
			dev_info(dev, "%s, Failed to enable tm(ret:%d)\n", __func__, ret);

		ret = mt6379_charger_field_set(cdata, F_CHG_RAMP_UP_COMP, 3);
		if (ret)
			dev_info(dev, "%s, Failed to set chg_ramp_up_comp to 240kOhm\n", __func__);

		ret = mt6379_charger_field_set(cdata, F_IEOC_FLOW_RB, 0);
		if (ret)
			dev_info(dev, "%s, Failed to set no fccm in eoc flow\n", __func__);

		ret = mt6379_charger_field_get(cdata, F_CHG_RAMP_UP_COMP, &ramp_comp);
		if (ret)
			dev_info(dev, "%s, Failed to get chg_ramp_up_comp\n", __func__);
		ret = mt6379_charger_field_get(cdata, F_IEOC_FLOW_RB, &ieoc_flow_rb);
		if (ret)
			dev_info(dev, "%s, Failed to get no fccm in eoc flow\n", __func__);

		dev_info(dev, "%s, update: chg_rmp_up_comp = %d, ieoc_flow_rb = %d\n",
			 __func__, ramp_comp, ieoc_flow_rb);
		ret = mt6379_enable_tm(cdata, false);
		if (ret)
			dev_info(dev, "%s, Failed to disable tm(ret:%d)\n", __func__, ret);
		mutex_unlock(&cdata->ramp_lock);
	}

ramp_no_update:
	ret = mt6379_charger_set_online(cdata, ATTACH_TRIG_PWR_RDY, attach_type);
	if (ret)
		dev_info(dev, "%s, Failed to online status\n", __func__);
}

static irqreturn_t mt6379_fl_pwr_rdy_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	queue_delayed_work(system_freezable_wq, &cdata->pwr_rdy_dwork, 0);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_detach_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	queue_delayed_work(system_freezable_wq, &cdata->pwr_rdy_dwork, 0);
	return IRQ_HANDLED;
}

#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
int mt6379_get_hvdcp_status(struct mt6379_charger_data *cdata, int val)
{
	if (!cdata) {
		return -EINVAL;
	} else {
		regmap_read(cdata->rmap, MT6379_HVDCP_TRIGGER, &val);
		return val;
	}
}

int mt6379_set_hvdcp_detect_enable(struct mt6379_charger_data *cdata)
{
	if (!cdata) {
		return -EINVAL;
	} else {
		regmap_update_bits(cdata->rmap, MT6379_REG_BC12_STAT, 0x20, 0x20);
		return 0;
	}
}

int mt6379_set_hvdcp_detect_disable(struct mt6379_charger_data *cdata)
{
	if (!cdata) {
		return -EINVAL;
	} else {
		regmap_update_bits(cdata->rmap, MT6379_HVDCP_TRIGGER, 0x40, 0x00);
		return 0;
	}
}

int mt6379_set_hvdcp_to_5v(void)
{
	if (!oplus_cdata) {
		return -EINVAL;
	} else {
		regmap_update_bits(oplus_cdata->rmap, MT6379_REG_DPDM_CTRL2, 0xff, 0x80);
		regmap_update_bits(oplus_cdata->rmap, MT6379_HVDCP_SETTING_CTRL1, 0xff, 0x02);
		regmap_update_bits(oplus_cdata->rmap, MT6379_REG_DPDM_CTRL4, 0xff, 0x0c);
		regmap_update_bits(oplus_cdata->rmap, MT6379_HVDCP_SETTING_CTRL2, 0xff, 0x00);
		return 0;
	}
}
EXPORT_SYMBOL(mt6379_set_hvdcp_to_5v);

int mt6379_set_hvdcp_to_9v(void)
{
	if (!oplus_cdata) {
		return -EINVAL;
	} else {
		regmap_update_bits(oplus_cdata->rmap, MT6379_REG_DPDM_CTRL1, 0xff, 0xA3);
		regmap_update_bits(oplus_cdata->rmap, MT6379_REG_DPDM_CTRL2, 0xff, 0xe8);
		regmap_update_bits(oplus_cdata->rmap, MT6379_HVDCP_SETTING_CTRL1, 0xff, 0x00);
		regmap_update_bits(oplus_cdata->rmap, MT6379_REG_DPDM_CTRL4, 0xff, 0x80);
		regmap_update_bits(oplus_cdata->rmap, MT6379_HVDCP_SETTING_CTRL2, 0xff, 0x80);
		return 0;
	}
}
EXPORT_SYMBOL(mt6379_set_hvdcp_to_9v);

int mt6379_reset_hvdcp_reg(struct mt6379_charger_data *cdata)
{
	if (!cdata) {
		return -EINVAL;
	} else {
		regmap_update_bits(cdata->rmap, MT6379_REG_DPDM_CTRL1, 0xff, 0x00);
		regmap_update_bits(cdata->rmap, MT6379_REG_DPDM_CTRL2, 0xff, 0x00);
		regmap_update_bits(cdata->rmap, MT6379_HVDCP_SETTING_CTRL1, 0xff, 0x00);
		regmap_update_bits(cdata->rmap, MT6379_REG_DPDM_CTRL4, 0xff, 0x00);
		regmap_update_bits(cdata->rmap, MT6379_HVDCP_SETTING_CTRL2, 0xff, 0x00);
		return 0;
	}
}

static void mt6379_hvdcp_result_check_work(struct work_struct *work)
{
	int ret = 0;
	int val = 0;
	struct mt6379_charger_data *cdata = container_of(work, struct mt6379_charger_data, hvdcp_result_check_work.work);
	struct tcpc_device *tcpc;

	dev_info(cdata->dev, "%s\n", __func__);

	tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		pr_info("%s: get type_c_port0 fail\n", __func__);
	}

	ret = mt6379_get_hvdcp_status(cdata, val);
	if (ret < 0) {
		printk("%s: fail to read device_type\n", __func__);
	}

	printk("%s: device type: %d\n", __func__, ret);

	if (ret & BIT(6)) {
		printk("%s: hvdcp detect\n", __func__);
		cdata->hvdcp_type = POWER_SUPPLY_TYPE_USB_HVDCP;
#ifdef OPLUS_FEATURE_CHG_BASIC
		if (tcpc)
			tcpci_notify_hvdcp_detect_dn(tcpc, true);
#endif
		ret = mt6379_set_hvdcp_to_5v();
		if (ret < 0)
			printk("%s: fail to write dpdm_ctrl\n", __func__);
	} else {
		cdata->hvdcp_type = POWER_SUPPLY_TYPE_USB_DCP;
#ifdef OPLUS_FEATURE_CHG_BASIC
		if (tcpc)
			tcpci_notify_hvdcp_detect_dn(tcpc, true);
#endif
		printk("%s: hvdcp not detect\n", __func__);
	}
	cdata->oplus_hvdcp_detect = false;
}

void oplus_notify_hvdcp_detect_stat(void)
{
	struct mt6379_charger_data *cdata = oplus_cdata;

	if (cdata && true == is_mtksvooc_project) {
		cdata->hvdcp_cfg_9v_done = true;
#ifdef OPLUS_FEATURE_CHG_BASIC
		cdata->hvdcp_detect_time = local_clock() / 1000000;
#else
		cdata->hvdcp_detect_time = cpu_clock(smp_processor_id()) / 1000000;
#endif
		printk(KERN_ERR "oplus_notify_hvdcp_detect_stat hvdcp2 detect: %d, the detect time: %llu\n",
				cdata->hvdcp_cfg_9v_done, cdata->hvdcp_detect_time);
	}
}
EXPORT_SYMBOL(oplus_notify_hvdcp_detect_stat);

static void mt6379_hvdcp_work(struct work_struct *work)
{
	int ret = 0;
	int val = 0;
	struct mt6379_charger_data *cdata = container_of(work, struct mt6379_charger_data, hvdcp_work.work);

	dev_info(cdata->dev, "%s\n", __func__);

	ret = mt6379_get_hvdcp_status(cdata, val);
	if (ret < 0) {
		printk("%s: fail to read device_type\n", __func__);
	}

	printk("%s: device type: %d\n", __func__, ret);

	if (val & BIT(6)) {
		printk("%s: hvdcp detect\n", __func__);
		cdata->hvdcp_type = POWER_SUPPLY_TYPE_USB_HVDCP;
		ret = mt6379_set_hvdcp_to_5v();
		if (ret < 0)
			printk("%s: fail to write dpdm_ctrl\n", __func__);
	} else {
		cdata->hvdcp_type = POWER_SUPPLY_TYPE_USB_DCP;
		printk("%s: hvdcp not detect\n", __func__);
	}
}
#endif

#ifdef OPLUS_FEATURE_CHG_BASIC
/* oplus add for charging icon disappears slowly */
void oplus_mt6379_get_wd0_stat(bool wd0_status)
{
	oplus_cdata->wd0_status = wd0_status;
}
EXPORT_SYMBOL(oplus_mt6379_get_wd0_stat);
#endif

static irqreturn_t mt6379_fl_rechg_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_chg_done_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_bk_chg_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_ieoc_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_bus_chg_rdy_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	if (cdata->id != CHARGER_ID_MT6379)
		return IRQ_HANDLED;
	queue_delayed_work(system_freezable_wq, &cdata->non_switch_dwork, 0);
	schedule_delayed_work(&cdata->switching_work, msecs_to_jiffies(1 * 1000));
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_wlin_chg_rdy_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	if (cdata->id != CHARGER_ID_MT6379)
		return IRQ_HANDLED;
	queue_delayed_work(system_freezable_wq, &cdata->non_switch_dwork, 0);
	schedule_delayed_work(&cdata->switching_work, msecs_to_jiffies(1 * 1000));
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_vbus_ov_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	charger_dev_notify(cdata->chgdev, CHARGER_DEV_NOTIFY_VBUS_OVP);

	if (cdata->id != CHARGER_ID_MT6379)
		return IRQ_HANDLED;
	queue_delayed_work(system_freezable_wq, &cdata->non_switch_dwork, 0);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_chg_batov_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_chg_sysov_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	if (cdata->id != CHARGER_ID_MT6379)
		return IRQ_HANDLED;
	queue_delayed_work(system_freezable_wq, &cdata->non_switch_dwork, 0);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_chg_tout_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	charger_dev_notify(cdata->chgdev, CHARGER_DEV_NOTIFY_SAFETY_TIMEOUT);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_chg_busuv_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_chg_threg_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_chg_aicr_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_chg_mivr_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_aicc_done_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_pe_done_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_pp_pgb_evt_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_wdt_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;
	int ret = 0;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	ret = mt6379_charger_field_set(cdata, F_WDT_RST, 1);
	if (ret)
		dev_info(cdata->dev, "%s, Failed to kick wdt\n", __func__);

	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_otg_fault_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_otg_lbp_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_otg_cc_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_batpro_done_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;
	int ret = 0;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);

	ret = charger_dev_enable_6pin_battery_charging(cdata->chgdev, false);
	if (ret)
		dev_info(cdata->dev, "%s, Failed to disable 6pin\n", __func__);

	charger_dev_notify(cdata->chgdev, CHARGER_DEV_NOTIFY_BATPRO_DONE);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_otg_clear_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_dcd_done_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_bc12_hvdcp_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_fl_bc12_dn_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;
	bool toggle_by_ufcs = false;
	const char *attach_name;
	int attach = 0;


	mutex_lock(&cdata->attach_lock);
	attach = atomic_read(&cdata->attach[0]);
	attach_name = get_attach_type_name(attach);
	mt_dbg(cdata->dev, "%s, attach = %d(%s)\n", __func__, attach, attach_name);
	if (attach == ATTACH_TYPE_NONE) {
		cdata->bc12_dn[0] = false;
		mutex_unlock(&cdata->attach_lock);
		return IRQ_HANDLED;
	}
#ifndef CONFIG_OPLUS_HVDCP_SUPPORT
	/* If UFCS detect fail, BC12 will be retoggled to support HVDCP */
	if (attach < ATTACH_TYPE_PD && cdata->psy_desc.type == POWER_SUPPLY_TYPE_USB_DCP)
		toggle_by_ufcs = true;
#endif
	cdata->bc12_dn[0] = true;
	mutex_unlock(&cdata->attach_lock);

	if (!toggle_by_ufcs && attach < ATTACH_TYPE_PD) {
		if (!queue_work(cdata->wq, &cdata->bc12_work))
			dev_info(cdata->dev, "%s, bc12 work already queued\n", __func__);
	}

	return IRQ_HANDLED;
}

#ifdef OPLUS_FEATURE_CHG_BASIC
static irqreturn_t mt6379_int_chrdet_ext_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;
	int ret;
	u32 val;
	int pdvbus;

	struct tcpc_device *tcpc = tcpc_dev_get_by_name("type_c_port0");
	if (tcpc == NULL) {
		pr_info("%s: get type_c_port0 fail\n", __func__);
		return IRQ_HANDLED;
	}

	dev_info(cdata->dev, "++\n");

	ret = iio_read_channel_processed(&cdata->iio_adcs[ADC_CHAN_PDVBUS], &pdvbus);
	if (ret)
		dev_err(cdata->dev, "[%s] Failed to read pdvbus voltage\n", __func__);
	dev_info(cdata->dev, "[%s] pd_vbus = %d", __func__, pdvbus);
	ret = mt6379_charger_field_get(data, F_CHRDET_EXT, &val);
	if (ret < 0)
		return ret;
	if (!val) {
		dev_info(cdata->dev, "chrdet dettach\n");
		tcpci_notify_chrdet_state(tcpc, false);

		if (pdvbus > OPLUS_CHRD_UV_THD) {
			dev_info(cdata->dev, "[%s] hardware ovp\n", __func__);
			cdata->pdvbus_ovp_status = true;
		} else {
			cdata->pdvbus_ovp_status = false;
		}
		if (cdata->hvdcp_cfg_9v_done == true)
			cdata->hvdcp_detach_time = local_clock() / 1000000;
		/* oplus add for charging icon disappears slowly */
		if (!oplus_cdata->wd0_status) {
			cdata->psy_desc.type = POWER_SUPPLY_TYPE_USB;
			cdata->psy_usb_type[cdata->active_idx] = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		}
		mt6379_charger_set_online(cdata, ATTACH_TRIG_TYPEC, val);
	} else {
		dev_info(cdata->dev, "chrdet attach\n");
		/* oplus add for bc12_dn flag */
		//cdata->bc12_dn[ddata->active_idx] = false;
		tcpci_notify_chrdet_state(tcpc, true);
		if (cdata->pdvbus_ovp_status) {
			dev_info(cdata->dev, "[%s] hardware ovp before, set online\n", __func__);
			mt6379_charger_set_online(cdata, ATTACH_TRIG_TYPEC, val);
			cdata->pdvbus_ovp_status = false;
		}
	}
	return IRQ_HANDLED;
}
#endif

static irqreturn_t mt6379_adc_vbat_mon_ov_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;
	union power_supply_propval val;
	int ret = 0;

	ret = power_supply_get_property(cdata->psy, POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
					&val);
	if (ret)
		mt_dbg(cdata->dev, "%s, Failed to get CV\n", __func__);
	else
		mt_dbg(cdata->dev, "%s, cv = %dmV\n", __func__, U_TO_M(val.intval));

	return IRQ_HANDLED;
}

static irqreturn_t mt6379_usbid_evt_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;
	u32 val = 0;
	int ret = 0;

	if (cdata->id == CHARGER_ID_MT6720) {
		dev_info(cdata->dev, "%s, MT6720 not support usbid\n", __func__);
		return IRQ_HANDLED;
	}

	ret = mt6379_charger_field_get(cdata,  F_ST_USBID, &val);
	if (ret)
		mt_dbg(cdata->dev, "%s, Failed to get USBID stat\n", __func__);
	else
		mt_dbg(cdata->dev, "%s, USBID stat = %d\n", __func__, val);

	return IRQ_HANDLED;
}

static irqreturn_t mt6379_otp1_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	if (cdata->id != CHARGER_ID_MT6379)
		return IRQ_HANDLED;
	queue_delayed_work(system_freezable_wq, &cdata->non_switch_dwork, 0);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_otp2_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	if (cdata->id != CHARGER_ID_MT6379)
		return IRQ_HANDLED;
	queue_delayed_work(system_freezable_wq, &cdata->non_switch_dwork, 0);
	return IRQ_HANDLED;
}

static irqreturn_t mt6379_otp0_handler(int irq, void *data)
{
	struct mt6379_charger_data *cdata = data;

	mt_dbg(cdata->dev, "%s, irq = %d\n", __func__, irq);
	if (cdata->id != CHARGER_ID_MT6379)
		return IRQ_HANDLED;
	queue_delayed_work(system_freezable_wq, &cdata->non_switch_dwork, 0);
	return IRQ_HANDLED;
}

#define DDATA_DEVM_KCALLOC(member)						\
	(cdata->member = devm_kcalloc(cdata->dev, pdata->nr_port,		\
				      sizeof(*cdata->member), GFP_KERNEL))	\

static int mt6379_chg_init_multi_ports(struct mt6379_charger_data *cdata)
{
	struct mt6379_charger_platform_data *pdata = dev_get_platdata(cdata->dev);
	int i = 0;

	if (pdata->nr_port < 1)
		return -EINVAL;

	if (pdata->nr_port > 1 && pdata->attach_trig != ATTACH_TRIG_TYPEC)
		return -EPERM;

	DDATA_DEVM_KCALLOC(psy_type);
	DDATA_DEVM_KCALLOC(psy_usb_type);
	DDATA_DEVM_KCALLOC(attach);
	DDATA_DEVM_KCALLOC(bc12_dn);
	if (!cdata->psy_type || !cdata->psy_usb_type || !cdata->attach || !cdata->bc12_dn)
		return -ENOMEM;

	cdata->active_idx = 0;
	for (i = 0; i < pdata->nr_port; i++) {
		cdata->psy_type[i] = POWER_SUPPLY_TYPE_USB;
		cdata->psy_usb_type[i] = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		atomic_set(&cdata->attach[i], ATTACH_TYPE_NONE);
		cdata->bc12_dn[i] = false;
	}

	return 0;
}

#define MT6379_CHARGER_IRQ(_name)					\
{									\
	.name = #_name,							\
	.handler = mt6379_##_name##_handler,				\
}

static int mt6379_charger_init_irq(struct mt6379_charger_data *cdata)
{
	struct device *dev = cdata->dev;
	int i = 0, ret = 0;
	const struct {
		char *name;
		irq_handler_t handler;
	} mt6379_charger_irqs[] = {
		MT6379_CHARGER_IRQ(fl_pwr_rdy),
		MT6379_CHARGER_IRQ(fl_detach),
		MT6379_CHARGER_IRQ(fl_rechg),
		MT6379_CHARGER_IRQ(fl_chg_done),
		MT6379_CHARGER_IRQ(fl_bk_chg),
		MT6379_CHARGER_IRQ(fl_ieoc),
		MT6379_CHARGER_IRQ(fl_bus_chg_rdy),
		MT6379_CHARGER_IRQ(fl_wlin_chg_rdy),
		MT6379_CHARGER_IRQ(fl_vbus_ov),
		MT6379_CHARGER_IRQ(fl_chg_batov),
		MT6379_CHARGER_IRQ(fl_chg_sysov),
		MT6379_CHARGER_IRQ(fl_chg_tout),
		MT6379_CHARGER_IRQ(fl_chg_busuv),
		MT6379_CHARGER_IRQ(fl_chg_threg),
		MT6379_CHARGER_IRQ(fl_chg_aicr),
		MT6379_CHARGER_IRQ(fl_chg_mivr),
		MT6379_CHARGER_IRQ(fl_aicc_done),
		MT6379_CHARGER_IRQ(fl_pe_done),
		MT6379_CHARGER_IRQ(pp_pgb_evt),
		MT6379_CHARGER_IRQ(fl_wdt),
		MT6379_CHARGER_IRQ(fl_otg_fault),
		MT6379_CHARGER_IRQ(fl_otg_lbp),
		MT6379_CHARGER_IRQ(fl_otg_cc),
		MT6379_CHARGER_IRQ(fl_batpro_done),
		MT6379_CHARGER_IRQ(fl_otg_clear),
		MT6379_CHARGER_IRQ(fl_dcd_done),
		MT6379_CHARGER_IRQ(fl_bc12_hvdcp),
#ifdef OPLUS_FEATURE_CHG_BASIC
		MT6379_CHARGER_IRQ(int_chrdet_ext),
#endif
		MT6379_CHARGER_IRQ(fl_bc12_dn),
		MT6379_CHARGER_IRQ(adc_vbat_mon_ov),
		MT6379_CHARGER_IRQ(usbid_evt),
		MT6379_CHARGER_IRQ(otp0),
		MT6379_CHARGER_IRQ(otp1),
		MT6379_CHARGER_IRQ(otp2),
	};

	if (ARRAY_SIZE(mt6379_charger_irqs) > MT6379_IRQ_MAX) {
		dev_info(dev, "%s, irq number out of MT6379_IRQ_MAX\n", __func__);
		return -EINVAL;
	}

	for (i = 0; i < ARRAY_SIZE(mt6379_charger_irqs); i++) {
		ret = platform_get_irq_byname(to_platform_device(cdata->dev),
					      mt6379_charger_irqs[i].name);
		if (ret < 0) /* not declare in dts file */
			continue;

		mt_dbg(dev, "%s, request \"%s\", irq number:%d\n",
		       __func__, mt6379_charger_irqs[i].name, ret);

		cdata->irq_nums[i] = ret;
		ret = devm_request_threaded_irq(cdata->dev, ret, NULL,
						mt6379_charger_irqs[i].handler,
						IRQF_TRIGGER_FALLING,
						mt6379_charger_irqs[i].name, cdata);
		if (ret) {
			dev_info(dev, "%s, Failed to request irq %s\n",
				 __func__, mt6379_charger_irqs[i].name);
			return ret;
		}
	}

	return 0;
}

static void mt6379_charger_destroy_wq(void *data)
{
	struct workqueue_struct *wq = data;

	flush_workqueue(wq);
	destroy_workqueue(wq);
}

static void mt6379_charger_destroy_attach_lock(void *data)
{
	struct mutex *attach_lock = data;

	mutex_destroy(attach_lock);
}

static void mt6379_charger_destroy_cv_lock(void *data)
{
	struct mutex *cv_lock = data;

	mutex_destroy(cv_lock);
}

static void mt6379_charger_destroy_tm_lock(void *data)
{
	struct mutex *tm_lock = data;

	mutex_destroy(tm_lock);
}

static void mt6379_charger_destroy_pe_lock(void *data)
{
	struct mutex *pe_lock = data;

	mutex_destroy(pe_lock);
}

static void mt6379_charger_destroy_ramp_lock(void *data)
{
	struct mutex *ramp_lock = data;

	mutex_destroy(ramp_lock);
}

static int mt6379_charger_init_mutex(struct mt6379_charger_data *cdata)
{
	struct device *dev = cdata->dev;
	int ret = 0;

	/* init mutex */
	mutex_init(&cdata->attach_lock);
	ret = devm_add_action_or_reset(dev, mt6379_charger_destroy_attach_lock,
				       &cdata->attach_lock);
	if (ret) {
		dev_info(dev, "%s, Failed to init attach lock\n", __func__);
		return ret;
	}

	mutex_init(&cdata->cv_lock);
	ret = devm_add_action_or_reset(dev, mt6379_charger_destroy_cv_lock, &cdata->cv_lock);
	if (ret) {
		dev_info(dev, "%s, Failed to init cv lock\n", __func__);
		return ret;
	}

	mutex_init(&cdata->tm_lock);
	ret = devm_add_action_or_reset(dev, mt6379_charger_destroy_tm_lock, &cdata->tm_lock);
	if (ret) {
		dev_info(dev, "%s, Failed to init tm lock\n", __func__);
		return ret;
	}

	mutex_init(&cdata->pe_lock);
	ret = devm_add_action_or_reset(dev, mt6379_charger_destroy_pe_lock, &cdata->pe_lock);
	if (ret) {
		dev_info(dev, "%s, Failed to init pe lock\n", __func__);
		return ret;
	}

	mutex_init(&cdata->ramp_lock);
	ret = devm_add_action_or_reset(dev, mt6379_charger_destroy_ramp_lock, &cdata->ramp_lock);
	if (ret) {
		dev_info(dev, "%s, Failed to init ramp lock\n", __func__);
		return ret;
	}

	/* init atomic */
	atomic_set(&cdata->eoc_cnt, 0);
	atomic_set(&cdata->no_6pin_used, 0);
	atomic_set(&cdata->tchg, 0);

	return 0;
}

static int ufcs_port_notifier_call(struct notifier_block *nb, unsigned long action, void *data)
{
	struct mt6379_charger_data *cdata = container_of(nb, struct mt6379_charger_data, ufcs_noti);
	int ret = 0, attach = 0;

	if (!cdata->wait_for_ufcs_attach)
		return NOTIFY_DONE;

	cdata->wait_for_ufcs_attach = false;

	mutex_lock(&cdata->attach_lock);

	attach = atomic_read(&cdata->attach[0]);

	if (action == UFCS_NOTIFY_ATTACH_FAIL &&
	    (attach == ATTACH_TYPE_TYPEC || attach == ATTACH_TYPE_PWR_RDY)) {
		ret = mt6379_charger_toggle_bc12(cdata);
		if (ret)
			dev_info(cdata->dev, "%s, Failed to toggle bc12\n", __func__);
	}

	mutex_unlock(&cdata->attach_lock);

	return NOTIFY_DONE;
}

static void mt6379_release_ufcs_port(void *d)
{
	struct mt6379_charger_data *cdata = d;

	unregister_ufcs_dev_notifier(cdata->ufcs, &cdata->ufcs_noti);
	ufcs_port_put(cdata->ufcs);
}

static void mt6379_charger_destroy_switching_work(void *d)
{
	struct delayed_work *work = d;

	cancel_delayed_work(work);
}

static void mt6379_charger_destroy_icc_cali_work(void *d)
{
	struct delayed_work *work = d;

	cancel_delayed_work(work);
}

static void mt6379_charger_destroy_icc_check_work(void *d)
{
	struct delayed_work *work = d;

	cancel_delayed_work(work);
}

static void mt6379_charger_destroy_pwr_rdy_work(void *d)
{
	struct delayed_work *work = d;

	cancel_delayed_work(work);
}

static void mt6379_charger_destroy_non_switch_work(void *d)
{
	struct delayed_work *work = d;

	cancel_delayed_work(work);
}

static int mt6379_charger_get_iio_adc(struct mt6379_charger_data *cdata)
{
	int ret = 0;

	cdata->iio_adcs = devm_iio_channel_get_all(cdata->dev);
	if (IS_ERR(cdata->iio_adcs))
		return PTR_ERR(cdata->iio_adcs);

	ret = iio_read_channel_processed(&cdata->iio_adcs[ADC_CHAN_ZCV], &cdata->zcv);
	if (ret)
		dev_info(cdata->dev, "%s, Failed to read ZCV voltage\n", __func__);

	dev_info(cdata->dev, "%s, zcv = %d mV\n", __func__, cdata->zcv);
	return 0;
}

/*
 * MT6720 range and filed
 */
static const struct linear_range mt6720_batint_range = {
	.min = 3900000,
	.min_sel = 0,
	.max_sel = 0x5F,
	.step = 10000,
};

static const struct linear_range mt6720_ibus_aicr_range = {
	.min = 100000,
	.min_sel = 0x2,
	.max_sel = 0x7F,
	.step = 25000,
};

static const struct reg_field mt6720_ibus_aicr_field = {
	.reg = MT6720_REG_CHG_IBUS_AICR,
	.lsb = 0,
	.msb = 6,
};

static const struct linear_range mt6720_vbus_mivr_range = {
	.min = 3900000,
	.min_sel = 0x0,
	.max_sel = 0x5F,
	.step = 100000,
};

static const struct reg_field mt6720_vbus_mivr_field = {
	.reg = MT6379_REG_CHG_VBUS_MIVR,
	.lsb = 0,
	.msb = 6,
};

static const struct linear_range mt6720_cv_range = {
	.min = 3900000,
	.min_sel = 0x0,
	.max_sel = 0x5F,
	.step = 10000,
};

static const struct linear_range mt6720_cc_range = {
	.min = 300000,
	.min_sel = 0x6,
	.max_sel = 0x3F,
	.step = 50000,
};

static const struct reg_field mt6720_cc_field = {
	.reg = MT6379_REG_CHG_ICHG,
	.lsb = 0,
	.msb = 5,
};

static const struct linear_range mt6720_ieoc_range = {
	.min = 50000,
	.min_sel = 0x0,
	.max_sel = 0xF,
	.step = 50000,
};

static const struct reg_field mt6720_ieoc_field = {
	.reg = MT6379_REG_CHG_EOC1,
	.lsb = 0,
	.msb = 3,
};

static const struct linear_range mt6720_vsysmin_range = {
	.min = 3000000,
	.min_sel = 0x0,
	.max_sel = 0xA,
	.step = 100000,
};

static const struct linear_range mt6720_iprec_range = {
	.min = 50000,
	.min_sel = 0x0,
	.max_sel = 0x1F,
	.step = 50000,
};

static const struct reg_field mt6720_iprec_field = {
	.reg = MT6379_REG_CHG_IPREC,
	.lsb = 0,
	.msb = 4,
};

static const struct reg_field mt6720_aicc_rpt_field = {
	.reg = MT6379_REG_CHG_AICC_RPT,
	.lsb = 0,
	.msb = 6,
};

static const struct reg_field mt6720_preuv_en_field = {
	.reg = MT6720_REG_VSYS_INTB,
	.lsb = 7,
	.msb = 7,
};

static const int mt6720_remove_field_table[] = {
	F_BATPROTECT_SOURCE,
	F_ST_USBID,
	F_WLIN_AICR,
	F_WLIN_MIVR,
	F_WLIN_FST,
	F_CHGIN_OV,
	F_SEAMLESS_OTG,
	F_OTG_THERMAL_EN,
	F_OTG_OCP,
	F_OTG_WLS,
	F_IRCMP_EN,
	F_IRCMP_R,
	F_IRCMP_V,
	F_VBAT_MON2_EN,
	F_ICC_ORIGIN,
	F_ICC_TRIMMED,
	F_ICC_OFFSET,
};

static void mt6xxx_charger_compatible_map(struct mt6379_charger_data *cdata)
{
	int i = 0;

	dev_info(cdata->dev, "%s charger %s\n", __func__, cdata->charger_data->name);
	if (cdata->id == CHARGER_ID_MT6720) {
		/* BATINT */
		mt6379_charger_fields[F_BATINT].range = &mt6720_batint_range;
		/* AICR */
		mt6379_charger_fields[F_IBUS_AICR].range = &mt6720_ibus_aicr_range;
		mt6379_charger_fields[F_IBUS_AICR].field = mt6720_ibus_aicr_field;
		/* MIVR */
		mt6379_charger_fields[F_VBUS_MIVR].range = &mt6720_vbus_mivr_range;
		mt6379_charger_fields[F_VBUS_MIVR].field = mt6720_vbus_mivr_field;
		/* CV */
		mt6379_charger_fields[F_CV].range = &mt6720_cv_range;
		/* CC */
		mt6379_charger_fields[F_CC].range = &mt6720_cc_range;
		mt6379_charger_fields[F_CC].field = mt6720_cc_field;
		/* IEOC */
		mt6379_charger_fields[F_IEOC].range = &mt6720_ieoc_range;
		mt6379_charger_fields[F_IEOC].field = mt6720_ieoc_field;
		/* VSYSMIN */
		mt6379_charger_fields[F_VSYSMIN].range = &mt6720_vsysmin_range;
		/* IPREC */
		mt6379_charger_fields[F_IPREC].range = &mt6720_iprec_range;
		mt6379_charger_fields[F_IPREC].field = mt6720_iprec_field;
		/* AICC_RPT */
		mt6379_charger_fields[F_AICC_RPT].field = mt6720_aicc_rpt_field;
		/* PREUV_EN */
		mt6379_charger_fields[F_PREUV_EN].field = mt6720_preuv_en_field;

		/* CHRD_OV */
		mt6379_charger_fields[F_CHRD_OV].table = mt6720_chg_chrd_ov;
		mt6379_charger_fields[F_CHRD_OV].tbl_num = ARRAY_SIZE(mt6720_chg_chrd_ov);

		/* remove part , set inited = false */
		for (i = 0; i < ARRAY_SIZE(mt6720_remove_field_table); i++)
			mt6379_charger_fields[mt6720_remove_field_table[i]].inited = false;
	}
}

static void mt6379_charger_pwr_rdy_dwork_func(struct work_struct *work)
{
	struct mt6379_charger_data *cdata = container_of(work, struct mt6379_charger_data,
							 pwr_rdy_dwork.work);

	mt6379_charger_check_pwr_rdy(cdata);
}

static void mt6379_charger_non_switch_dwork_func(struct work_struct *work)
{
	struct mt6379_charger_data *cdata = container_of(work, struct mt6379_charger_data,
							 non_switch_dwork.work);
	int ret = 0;

	ret = mt6379_charger_set_non_switching_setting(cdata);
	if (ret)
		dev_info(cdata->dev, "%s set non switching setting failed\n", __func__);
}

static int mt6379_charger_probe(struct platform_device *pdev)
{
	struct mt6379_charger_data *cdata;
	struct device *dev = &pdev->dev;
	const struct platform_device_id *id;
	int ret = 0;

	cdata = devm_kzalloc(dev, sizeof(*cdata), GFP_KERNEL);
	if (!cdata)
		return -ENOMEM;

	cdata->rmap = dev_get_regmap(dev->parent, NULL);
	if (!cdata->rmap) {
		dev_info(dev, "%s, Failed to get regmap\n", __func__);
		return -ENODEV;
	}
	cdata->dev = dev;
#ifdef OPLUS_FEATURE_CHG_BASIC
	oplus_cdata = cdata;
	is_mtksvooc_project = true;
#endif

	if (pdev->dev.of_node)
		cdata->charger_data = of_device_get_match_data(&pdev->dev);
	else {
		id = platform_get_device_id(pdev);
		cdata->charger_data = (struct rt_charger_data *)id->driver_data;
	}

	if (!cdata->charger_data) {
		dev_info(&pdev->dev, "failed to get charger_data\n");
		return -ENODEV;
	}
	cdata->id = cdata->charger_data->id;

	mt6xxx_charger_compatible_map(cdata);

	ret = mt6379_charger_init_rmap_fields(cdata);
	if (ret) {
		dev_info(dev, "%s, Failed to init regmap field\n", __func__);
		return ret;
	}

	platform_set_drvdata(pdev, cdata);

	ret = mt6379_charger_get_pdata(dev);
	if (ret) {
		dev_info(dev, "%s, Failed to get platform data\n", __func__);
		return ret;
	}

	ret = mt6379_charger_init_mutex(cdata);
	if (ret) {
		dev_info(dev, "%s, Failed to init mutex\n", __func__);
		return ret;
	}

	cdata->wq = create_singlethread_workqueue(dev_name(cdata->dev));
	if (!cdata->wq) {
		dev_info(dev, "%s, Failed to create WQ\n", __func__);
		return -ENOMEM;
	}

	ret = devm_add_action_or_reset(dev, mt6379_charger_destroy_wq, cdata->wq);
	if (ret) {
		dev_info(dev, "%s, Failed to init WQ\n", __func__);
		return ret;
	}

	INIT_DELAYED_WORK(&cdata->switching_work, mt6379_charger_switching_work_func);
	ret = devm_add_action_or_reset(dev, mt6379_charger_destroy_switching_work,
				       &cdata->switching_work);
	if (ret) {
		dev_info(dev, "%s, Failed to add fsw control action\n", __func__);
		return ret;
	}
	cdata->non_switching = false;

	INIT_DELAYED_WORK(&cdata->icc_cali_work, mt6379_charger_icc_cali_work_func);
	ret = devm_add_action_or_reset(dev, mt6379_charger_destroy_icc_cali_work,
				       &cdata->icc_cali_work);
	if (ret) {
		dev_info(dev, "%s, Failed to add icc cali action\n", __func__);
		return ret;
	}

	INIT_DELAYED_WORK(&cdata->icc_check_work, mt6379_charger_icc_check_work_func);
	ret = devm_add_action_or_reset(dev, mt6379_charger_destroy_icc_check_work,
				       &cdata->icc_check_work);
	if (ret) {
		dev_info(dev, "%s, Failed to add icc check action\n", __func__);
		return ret;
	}

	INIT_DELAYED_WORK(&cdata->pwr_rdy_dwork, mt6379_charger_pwr_rdy_dwork_func);
	ret = devm_add_action_or_reset(dev, mt6379_charger_destroy_pwr_rdy_work,
					   &cdata->pwr_rdy_dwork);
	if (ret) {
		dev_info(dev, "%s, Failed to add pwr rdy action\n", __func__);
		return ret;
	}

	INIT_DELAYED_WORK(&cdata->non_switch_dwork, mt6379_charger_non_switch_dwork_func);
	ret = devm_add_action_or_reset(dev, mt6379_charger_destroy_non_switch_work,
					   &cdata->non_switch_dwork);
	if (ret) {
		dev_info(dev, "%s, Failed to add non switch action\n", __func__);
		return ret;
	}

	ret = devm_work_autocancel(dev, &cdata->bc12_work, mt6379_charger_bc12_work_func);
	if (ret) {
		dev_info(dev, "%s, Failed to init bc12 work\n", __func__);
		return ret;
	}

#ifdef OPLUS_FEATURE_CHG_BASIC
	platform_set_drvdata(pdev, cdata);
#endif

	ret = mt6379_charger_init_setting(cdata);
	if (ret) {
		dev_info(dev, "%s, Failed to init mt6379 charger\n", __func__);
		return ret;
	}

	ret = mt6379_charger_get_iio_adc(cdata);
	if (ret) {
		dev_info(dev, "%s, Failed to get iio adc\n", __func__);
		return ret;
	}

	ret = mt6379_chg_init_multi_ports(cdata);
	if (ret) {
		dev_info(dev, "%s, Failed to init multi ports\n", __func__);
		return ret;
	}

	ret = mt6379_charger_init_psy(cdata);
	if (ret) {
		dev_info(dev, "%s, Failed to init psy\n", __func__);
		return ret;
	}

	ret = mt6379_init_otg_regulator(cdata);
	if (ret) {
		dev_info(dev, "%s, Failed to init OTG regulator\n", __func__);
		return ret;
	}

#ifdef OPLUS_FEATURE_CHG_BASIC
	/* get OTG regulator */
	cdata->otg_regu = devm_regulator_get(dev, "mt6379-usb-otg-vbus");
	if (IS_ERR(cdata->otg_regu)) {
		dev_err(dev, "Failed to get OTG regulator\n");
		return PTR_ERR(cdata->otg_regu);
	}
#endif
	INIT_WORK(&cdata->fsw_control_work, mt6379_fsw_control_func);
	alarm_init(&cdata->alarm, ALARM_REALTIME, mt6379_chg_alarm_call);

	ret = mt6379_charger_init_chgdev(cdata);
	if (ret) {
		dev_info(dev, "%s, Failed to init chgdev\n", __func__);
		return ret;
	}

	ret = mt6379_charger_init_irq(cdata);
	if (ret) {
		dev_info(dev, "%s, Failed to init irq\n", __func__);
		return ret;
	}

	cdata->ufcs = ufcs_port_get_by_name("port.0");
	if (!cdata->ufcs) {
		dev_info(dev, "%s, Failed to get ufcs port\n", __func__);
		return -ENODEV;
	}

	cdata->ufcs_noti.notifier_call = ufcs_port_notifier_call;
	register_ufcs_dev_notifier(cdata->ufcs, &cdata->ufcs_noti);

	ret = devm_add_action_or_reset(dev, mt6379_release_ufcs_port, cdata);
	if (ret) {
		dev_info(dev, "%s, Failed to add ufcs action\n", __func__);
		return ret;
	}

	cdata->icc_trimmed = mt6379_get_icc_trimmed(cdata);
	dev_info(dev, "%s, icc trimmed = %d\n", __func__, cdata->icc_trimmed);
#if MT6379_IMPLEMENT_ICC_LOAD
	if (!cdata->icc_trimmed) { /* POR occurred */
		cdata->saved_icc_offset = mt6379_load_save_icc_offset();
		dev_info(dev, "%s, saved icc offset = %d\n", __func__, cdata->saved_icc_offset);
		ret = mt6379_trim_icc_offset(cdata, cdata->saved_icc_offset, false);
		if (ret)
			dev_info(dev, "%s trim icc offset failed\n", __func__);
	}
#endif /* MT6379_IMPLEMENT_ICC_LOAD */

#ifdef CONFIG_OPLUS_HVDCP_SUPPORT
	INIT_DELAYED_WORK(&cdata->hvdcp_work, mt6379_hvdcp_work);
	INIT_DELAYED_WORK(&cdata->hvdcp_result_check_work, mt6379_hvdcp_result_check_work);
	cdata->hvdcp_detect_time = 0;
	cdata->hvdcp_detach_time = 0;
	cdata->hvdcp_cfg_9v_done = false;
	cdata->hvdcp_exit_stat = HVDCP_EXIT_NORMAL;
	cdata->hvdcp_type = POWER_SUPPLY_TYPE_UNKNOWN;
	cdata->oplus_get_hvdcp_bc12_result = false;
	cdata->oplus_hvdcp_detect = false;
#endif
#ifdef OPLUS_FEATURE_CHG_BASIC
/* oplus add for pd become usb port */
	cdata->bc12_retry = 0;

/* oplus add for identify ovp on dx3++ */
	cdata->pdvbus_ovp_status = false;
#endif
	mt6379_charger_check_pwr_rdy(cdata);
	return 0;
}

static int mt6379_charger_remove(struct platform_device *pdev)
{
	struct mt6379_charger_data *cdata = platform_get_drvdata(pdev);
#ifdef OPLUS_FEATURE_CHG_BASIC
	/* Release OTG regulator resource */
	devm_regulator_put(cdata->otg_regu);
#endif
	charger_device_unregister(cdata->chgdev);
	return 0;
}

static void mt6379_charger_shutdown(struct platform_device *pdev)
{
	struct mt6379_charger_data *cdata = platform_get_drvdata(pdev);

	alarm_cancel(&cdata->alarm);
}

static int __maybe_unused mt6379_charger_suspend(struct device *dev)
{
	struct mt6379_charger_data *cdata = dev_get_drvdata(dev);

	/*
	 * If customer need to sleep while charging, set support_fsw_wakeup = 1
	 * We will wake up system via alarm timer to do fsw control
	 */
	if (!support_fsw_wakeup)
		alarm_cancel(&cdata->alarm);
	return 0;
}

static int __maybe_unused mt6379_charger_resume(struct device *dev)
{
	struct mt6379_charger_data *cdata = dev_get_drvdata(dev);
	int ret = 0;
	u32 val = 0;

	if (!cdata->enable_fsw)
		return 0;

	ret = mt6379_charger_field_get(cdata, F_ST_PWR_RDY, &val);
	if (ret < 0)
		dev_info(cdata->dev, "%s, get pwr rdy stat failed\n", __func__);
	if (val) {
		if (fast_fsw_control && cdata->fsw_check_nr < fast_fsw_cnt)
			alarm_start_relative(&cdata->alarm,
					     ktime_set(fsw_ctrl_time, fsw_ctrl_time_ns));
		else
			alarm_start_relative(&cdata->alarm,
					     ktime_set(fsw_ctrl_time_2, fsw_ctrl_time_ns_2));
	} else
		dev_info(cdata->dev, "%s, pwr is not rdy\n", __func__);
	return 0;
}

static const struct dev_pm_ops mt6379_charger_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(mt6379_charger_suspend, mt6379_charger_resume)
};

static const struct of_device_id mt6379_charger_of_match[] = {
	{ .compatible = "mediatek,mt6379-charger", .data = &mt6379_data },
	{ .compatible = "mediatek,mt6720-charger", .data = &mt6720_data },
	{ },
};
MODULE_DEVICE_TABLE(of, mt6379_charger_of_match);

static const struct platform_device_id charger_id[] = {
	{ "mt6379-charger", (kernel_ulong_t)&mt6379_data},
	{ "mt6720-charger", (kernel_ulong_t)&mt6720_data},
	{ }
};
MODULE_DEVICE_TABLE(platform, charger_id);

static struct platform_driver mt6379_charger_driver = {
	.probe = mt6379_charger_probe,
	.remove = mt6379_charger_remove,
	.shutdown = mt6379_charger_shutdown,
	.id_table = charger_id,
	.driver = {
		.name = "mt6379-charger",
		.of_match_table = mt6379_charger_of_match,
		.pm = &mt6379_charger_pm_ops,
	},
};
module_platform_driver(mt6379_charger_driver);

MODULE_AUTHOR("SHIH CHIA CHANG <jeff_chang@richtek.com>");
MODULE_DESCRIPTION("MT6379 Charger Driver");
MODULE_LICENSE("GPL");
