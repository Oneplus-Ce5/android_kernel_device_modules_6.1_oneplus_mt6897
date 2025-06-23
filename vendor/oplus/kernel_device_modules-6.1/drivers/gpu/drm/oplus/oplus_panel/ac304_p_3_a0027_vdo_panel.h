#ifndef PANEL_AC304_P_3_A0027_H
#define PANEL_AC304_P_3_A0027_H

#define REGFLAG_CMD				0xFFFA
#define REGFLAG_DELAY			0xFFFC
#define REGFLAG_UDELAY			0xFFFB
#define REGFLAG_END_OF_TABLE	0xFFFD

#define BRIGHTNESS_HALF         3515
#define BRIGHTNESS_MAX          4094

enum MODE_ID {
	FHD_SDC60 = 0,
	FHD_SDC90 = 1,
	FHD_SDC120 = 2,
};

struct ba {
	u32 brightness;
	u32 alpha;
};

struct LCM_setting_table {
	unsigned int cmd;
	unsigned int count;
	unsigned char para_list[256];
};

/* pmic VS2 voter */
#define VS_VOTER_EN_LO 0x0
#define VS_VOTER_EN_LO_SET 0x1
#define VS_VOTER_EN_LO_CLR 0x2
struct hw_vsvoter {
	struct device *dev;
	struct regmap *vsv;
	u32 vsv_reg;
	u32 vsv_mask;
	u32 vsv_vers;
};

/* -------------------------doze mode setting start------------------------- */

struct LCM_setting_table lcm_lhbm_on_setting[] = {
	{REGFLAG_CMD,9, {0xAE, 0x82, 0x1C, 0xA0, 0x08, 0x00, 0x80, 0x08, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

struct LCM_setting_table lcm_lhbm_off_setting[] = {
	{REGFLAG_CMD,9, {0xAE, 0x02, 0x1C, 0xA0, 0x08, 0x00, 0x80, 0x08, 0x00}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

//1~12:DBV <= 0x481
struct LCM_setting_table lcm_set_demura_offset1[] = {
	{REGFLAG_CMD,6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00}},
	{REGFLAG_CMD,2, {0x6F, 0x01}},
	{REGFLAG_CMD,2, {0xC7, 0x04}},
	{REGFLAG_CMD,2, {0x6F, 0x2E}},
	{REGFLAG_CMD,3, {0xC0, 0x31, 0x20}},
	{REGFLAG_CMD,2, {0x6F, 0x30}},
	{REGFLAG_CMD,4, {0xC0, 0x04, 0x44, 0x00}},
	{REGFLAG_CMD,6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x04}},
	{REGFLAG_CMD,2, {0x6F, 0x01}},
	{REGFLAG_CMD,6, {0xCB, 0x08, 0x10, 0x1E, 0x78, 0xE1}},
	{REGFLAG_CMD,2, {0x6F, 0x06}},
	{REGFLAG_CMD,10, {0xCB, 0x00, 0x08, 0xE9, 0x12, 0x72, 0x24, 0x34, 0x05,0x81}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

//13~22:0x481< DBV<= 0xFFE
struct LCM_setting_table lcm_set_demura_offset2[] = {
	{REGFLAG_CMD,6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00}},
	{REGFLAG_CMD,2, {0x6F, 0x01}},
	{REGFLAG_CMD,2, {0xC7, 0x64}},
	{REGFLAG_CMD,2, {0x6F, 0x2E}},
	{REGFLAG_CMD,3, {0xC0, 0x78, 0x90}},
	{REGFLAG_CMD,6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x04}},
	{REGFLAG_CMD,2, {0x6F, 0x01}},
	{REGFLAG_CMD,6, {0xCB, 0x03, 0x08, 0x1E, 0x78, 0xE1}},
	{REGFLAG_CMD,2, {0x6F, 0x06}},
	{REGFLAG_CMD,10, {0xCB, 0x46, 0x82, 0x67, 0x7C, 0x4E, 0x0C, 0xDF, 0xBB, 0xFE}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

//23~34: DBV = 0xFFF
struct LCM_setting_table lcm_set_demura_offset3[] = {
	{REGFLAG_CMD,6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x00}},
	{REGFLAG_CMD,2, {0x6F, 0x01}},
	{REGFLAG_CMD,2, {0xC7, 0x54}},
	{REGFLAG_CMD,2, {0x6F, 0x2E}},
	{REGFLAG_CMD,3, {0xC0, 0x55, 0x50}},
	{REGFLAG_CMD,6, {0xF0, 0x55, 0xAA, 0x52, 0x08, 0x04}},
	{REGFLAG_CMD,2, {0x6F, 0x01}},
	{REGFLAG_CMD,6, {0xCB, 0x03, 0x08, 0x1E, 0x78, 0xE1}},
	{REGFLAG_CMD,2, {0x6F, 0x06}},
	{REGFLAG_CMD,10, {0xCB, 0x46, 0x82, 0x67, 0x7C, 0x4E, 0x0C, 0xDF, 0xBB, 0xFE}},
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

struct LCM_setting_table AOD_off_setting[] = {
	{REGFLAG_END_OF_TABLE, 0x00, {}}
};

struct LCM_setting_table AOD_on_setting[] = {
	{REGFLAG_CMD, 3, {0x51, 0x03, 0xFF}},
};

struct LCM_setting_table aod_high_bl_level[] = {
	{REGFLAG_CMD, 3, {0x51, 0x03, 0xFF}},
};

struct LCM_setting_table aod_low_bl_level[] = {
	{REGFLAG_CMD, 3, {0x51, 0x01, 0xFF}},
};

struct LCM_setting_table hbm_on_cmd[] = {
	{REGFLAG_CMD, 3, {0x51, 0x0F, 0xFE}},
};

struct LCM_setting_table hbm_off_cmd[] = {
	{REGFLAG_CMD, 3, {0x51, 0x0D, 0xBB}},
};
/* -------------------------doze mode setting end------------------------- */

/* -------------------------esd_check_multipage start------------------------- */
static struct LCM_setting_table esd_check_multipage_pre[] = {
	{REGFLAG_CMD, 3, {0xF0, 0x5A, 0x5A}},
	{REGFLAG_CMD, 2, {0xB0, 0xA4}},
};
/* -------------------------esd_check_multipage end------------------------- */
#endif
