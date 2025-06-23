// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#include <linux/backlight.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_panel.h>
#include <drm/drm_modes.h>
#include <linux/delay.h>
#include <drm/drm_connector.h>
#include <drm/drm_device.h>

#include <linux/gpio/consumer.h>
#include <linux/regulator/consumer.h>

#include <video/mipi_display.h>
#include <video/of_videomode.h>
#include <video/videomode.h>

#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/of_graph.h>
#include <linux/platform_device.h>
#include <soc/oplus/device_info.h>
#include "../../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"

#include "../../oplus/oplus_display_mtk_debug.h"
#define CONFIG_MTK_PANEL_EXT
#include "../../mediatek/mediatek_v2/mtk_panel_ext.h"
#include "../../mediatek/mediatek_v2/mtk_drm_graphics_base.h"
//#include "../../../../misc/mediatek/include/mt-plat/mtk_boot_common.h"
#include "../../mediatek/mediatek_v2/mtk_dsi.h"
#include "../../mediatek/mediatek_v2/mtk-cmdq-ext.h"

#ifdef OPLUS_FEATURE_DISPLAY
#include "../../oplus/oplus_drm_disp_panel.h"
#include "../../oplus/oplus_display_temp_compensation.h"
#include "../../oplus/oplus_display_mtk_debug.h"
#endif /* OPLUS_FEATURE_DISPLAY */

#ifdef OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT
#include "../../oplus/oplus_display_onscreenfingerprint.h"
#endif /* OPLUS_FEATURE_DISPLAY_ONSCREENFINGERPRINT */

#include "include/aa597_data_hw_roundedpattern.h"
#include "aa597_p_7_a0025_dsi_vdo_panel.h"

static unsigned int nt37703a_vdo_dphy_buf_thresh[14] ={896, 1792, 2688, 3584, 4480,
	5376, 6272, 6720, 7168, 7616, 7744, 7872, 8000, 8064};
static unsigned int nt37703a_vdo_dphy_range_min_qp[15] ={0, 4, 5, 5, 7, 7, 7, 7, 7,
	7, 9, 9, 9, 11, 23};
static unsigned int nt37703a_vdo_dphy_range_max_qp[15] ={8, 8, 9, 10, 11, 11, 11,
	12, 13, 14, 15, 16, 17, 17, 19};
static int nt37703a_vdo_dphy_range_bpg_ofs[15] ={2, 0, 0, -2, -4, -6, -8, -8, -8,
	-10, -10, -12, -12, -12, -12};

struct lcm {
	struct device *dev;
	struct drm_panel panel;
	struct backlight_device *backlight;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *vddr1p2_enable_gpio;
	struct gpio_desc *vci3p0_enable_gpio;
	struct drm_display_mode *m;
	bool prepared;
	bool enabled;
	bool hbm_en;
	bool hbm_wait;
	int error;
};

extern unsigned int oplus_display_brightness;
extern unsigned int oplus_max_normal_brightness;
extern unsigned long seed_mode;
extern char regs1[AC178_GAMMA_COMPENSATION_READ_LENGTH];
extern char regs2[AC178_GAMMA_COMPENSATION_READ_LENGTH];
extern char regs3[AC178_GAMMA_COMPENSATION_READ_LENGTH];
extern int oplus_serial_number_probe(struct device *dev);
static int current_fps = 120;
static unsigned int temp_seed_mode = 0;
static int mode_id = -1;
extern atomic_t esd_pending;
extern void mipi_dsi_dcs_write_gce(struct mtk_dsi *dsi, struct cmdq_pkt *handle,
				const void *data, size_t len);
//static bool aod_state = false;
static unsigned int lhbm_last_backlight = 0;

#define MAX_NORMAL_BRIGHTNESS   3515
#define LCM_BRIGHTNESS_TYPE 2
#define FINGER_HBM_BRIGHTNESS 3730

extern void lcdinfo_notify(unsigned long val, void *v);
extern int oplus_display_panel_dbv_probe(struct device *dev);

#define lcm_dcs_write_seq(ctx, seq...) \
({\
	const u8 d[] = { seq };\
	BUILD_BUG_ON_MSG(ARRAY_SIZE(d) > 128, "DCS sequence too big for stack");\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})

#define lcm_dcs_write_seq_static(ctx, seq...) \
({\
	static const u8 d[] = { seq };\
	lcm_dcs_write(ctx, d, ARRAY_SIZE(d));\
})
static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle);

static inline struct lcm *panel_to_lcm(struct drm_panel *panel)
{
	return container_of(panel, struct lcm, panel);
}

static void lcm_dcs_write(struct lcm *ctx, const void *data, size_t len)
{
	struct mipi_dsi_device *dsi = to_mipi_dsi_device(ctx->dev);
	ssize_t ret;
	char *addr;

	if (ctx->error < 0)
		return;

	addr = (char *)data;
	if ((int)*addr < 0xB0)
		ret = mipi_dsi_dcs_write_buffer(dsi, data, len);
	else
		ret = mipi_dsi_generic_write(dsi, data, len);
	if (ret < 0) {
		dev_info(ctx->dev, "error %zd writing seq: %ph\n", ret, data);
		ctx->error = ret;
	}
}

static int panel_send_pack_hs_cmd(void *dsi, struct LCM_setting_table *table, unsigned int lcm_cmd_count, dcs_write_gce_pack cb, void *handle)
{
	unsigned int i = 0;
	struct mtk_ddic_dsi_cmd send_cmd_to_ddic;

	if (lcm_cmd_count > MAX_TX_CMD_NUM_PACK) {
		pr_err("out of mtk_ddic_dsi_cmd \n");
		return 0;
	}

	for (i = 0; i < lcm_cmd_count; i++) {
		send_cmd_to_ddic.mtk_ddic_cmd_table[i].cmd_num = table[i].count;
		send_cmd_to_ddic.mtk_ddic_cmd_table[i].para_list = table[i].para_list;
	}
	send_cmd_to_ddic.is_hs = 1;
	send_cmd_to_ddic.is_package = 1;
	send_cmd_to_ddic.cmd_count = lcm_cmd_count;
	cb(dsi, handle, &send_cmd_to_ddic);

	return 0;
}

static void push_table(struct lcm *ctx, struct LCM_setting_table *table, unsigned int count)
{
	unsigned int i;
	unsigned int cmd;

	for (i = 0; i < count; i++) {
		cmd = table[i].cmd;
		switch (cmd) {
		case REGFLAG_DELAY:
			usleep_range(table[i].count*1000, table[i].count*1000 + 100);
			break;
		case REGFLAG_UDELAY:
			usleep_range(table[i].count, table[i].count + 100);
			break;
		case REGFLAG_END_OF_TABLE:
			break;
		default:
			lcm_dcs_write(ctx, table[i].para_list, table[i].count);
			break;
		}
	}
}

static int get_mode_enum(struct drm_display_mode *m)
{
	int ret = 0;
	int m_vrefresh = 0;

	if (m == NULL) {
		DISP_ERR("aa597_p_7_a0025 %s display_mode m = NULL!\n", __func__);
		return -EINVAL;
	}

	m_vrefresh = drm_mode_vrefresh(m);

	if (m_vrefresh == 60) {
		ret = FHD_SDC60;
	} else if (m_vrefresh == 90) {
		ret = FHD_SDC90;
	} else if (m_vrefresh == 120) {
		ret = FHD_SDC120;
	} else if (m_vrefresh == 30) {
		ret = FHD_SDC30;
		DISP_INFO("fps == 30\n");
	} else {
		DISP_ERR("Invalid fps\n");
	}
	return ret;
}

static void lcm_panel_init(struct lcm *ctx)
{
	DISP_ERR("aa597_p_7_a0025 %s +, mode id=%d\n", __func__, mode_id);
	switch (mode_id) {
	case FHD_SDC60:
		push_table(ctx, init_setting_60Hz, sizeof(init_setting_60Hz)/sizeof(struct LCM_setting_table));
		break;
	case FHD_SDC90:
		push_table(ctx, init_setting_90Hz, sizeof(init_setting_90Hz)/sizeof(struct LCM_setting_table));
		break;
	case FHD_SDC120:
		push_table(ctx, init_setting_120Hz, sizeof(init_setting_120Hz)/sizeof(struct LCM_setting_table));
		break;
	default:
		push_table(ctx, init_setting_120Hz, sizeof(init_setting_120Hz)/sizeof(struct LCM_setting_table));
		break;
	}

	DISP_ERR("%s,aa597_p_7_a0025 restore seed_mode:%d\n", __func__, temp_seed_mode);
	if (temp_seed_mode == NATURAL){
		push_table(ctx, dsi_set_seed_natural, sizeof(dsi_set_seed_natural) / sizeof(struct LCM_setting_table));
	} else if (temp_seed_mode == EXPERT){
		push_table(ctx, dsi_set_seed_expert, sizeof(dsi_set_seed_expert) / sizeof(struct LCM_setting_table));
	} else if (temp_seed_mode == VIVID){
		push_table(ctx, dsi_set_seed_vivid, sizeof(dsi_set_seed_vivid) / sizeof(struct LCM_setting_table));
	}

	DISP_ERR("aa597_p_7_a0025 %s -\n", __func__);
}

static int lcm_disable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (!ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_POWERDOWN;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = false;

	return 0;
}

static int lcm_unprepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	DISP_ERR("%s:prepared=%d\n", __func__, ctx->prepared);

	if (!ctx->prepared)
		return 0;

	lcm_dcs_write_seq_static(ctx, 0x28);
	usleep_range(10000, 11000);
	lcm_dcs_write_seq_static(ctx, 0x10);
	usleep_range(150*1000, 151*1000);

	ctx->error = 0;
	ctx->prepared = false;
	//ctx->hbm_en = false;
	DISP_ERR("%s:success\n", __func__);

	return 0;
}

static int lcm_prepare(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	DISP_ERR("%s:prepared=%d\n", __func__, ctx->prepared);
	if (ctx->prepared)
		return 0;

	lcm_panel_init(ctx);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	ctx->prepared = true;
	DISP_ERR("%s:success\n", __func__);
	return ret;
}

static int lcm_enable(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->enabled)
		return 0;

	if (ctx->backlight) {
		ctx->backlight->props.power = FB_BLANK_UNBLANK;
		backlight_update_status(ctx->backlight);
	}

	ctx->enabled = true;

	return 0;
}

#define HFP_30HZ                (2044)
#define HBP_30HZ                (18)
#define HSA_30HZ                (8)
#define VFP_30HZ                (84)
#define VBP_30HZ                (18)
#define VSA_30HZ                (2)
static const struct drm_display_mode disp_mode_30Hz = {
	.clock = ((FRAME_WIDTH + HFP_30HZ + HBP_30HZ + HSA_30HZ) * (FRAME_HEIGHT + VFP_30HZ + VBP_30HZ + VSA_30HZ) * 30) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP_30HZ,
	.hsync_end = FRAME_WIDTH + HFP_30HZ + HSA_30HZ,
	.htotal = FRAME_WIDTH + HFP_30HZ + HSA_30HZ + HBP_30HZ,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + VFP_30HZ,
	.vsync_end = FRAME_HEIGHT + VFP_30HZ + VSA_30HZ,
	.vtotal = FRAME_HEIGHT + VFP_30HZ + VSA_30HZ + VBP_30HZ,
};

static const struct drm_display_mode disp_mode_60Hz = {
	.clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_60HZ + VBP + VSA) * 60) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + VFP_60HZ,
	.vsync_end = FRAME_HEIGHT + VFP_60HZ + VSA,
	.vtotal = FRAME_HEIGHT + VFP_60HZ + VSA + VBP,
};

static const struct drm_display_mode disp_mode_90Hz = {
	.clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_90HZ + VBP + VSA) * 90) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + VFP_90HZ,
	.vsync_end = FRAME_HEIGHT + VFP_90HZ + VSA,
	.vtotal = FRAME_HEIGHT + VFP_90HZ + VSA + VBP,
};

static const struct drm_display_mode disp_mode_120Hz = {
	.clock = ((FRAME_WIDTH + HFP + HBP + HSA) * (FRAME_HEIGHT + VFP_120HZ + VBP + VSA) * 120) / 1000,
	.hdisplay = FRAME_WIDTH,
	.hsync_start = FRAME_WIDTH + HFP,
	.hsync_end = FRAME_WIDTH + HFP + HSA,
	.htotal = FRAME_WIDTH + HFP + HSA + HBP,
	.vdisplay = FRAME_HEIGHT,
	.vsync_start = FRAME_HEIGHT + VFP_120HZ,
	.vsync_end = FRAME_HEIGHT + VFP_120HZ + VSA,
	.vtotal = FRAME_HEIGHT + VFP_120HZ + VSA + VBP,
};

static struct mtk_panel_params ext_params_30Hz = {
	.pll_clk = MIPI_CLK,
	.data_rate = DATA_RATE,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 30,
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	.vdo_mix_mode_en = false,
	// .lcm_esd_check_table[1] = {
	// 	.cmd = 0x05, .count = 1, .para_list[0] = 0x00,
	// },

	.vdo_mix_mode_en = false,
	.vdo_per_frame_lp_enable = 0,

	.vendor = "AA597_P_7_A0025",
	.manufacture = "P_7",

	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.oplus_uiready_before_time = 17,
	.color_vivid_status = true,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.seed_sync = true,

	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2392,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 187,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 3255,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = nt37703a_vdo_dphy_buf_thresh,
			.range_min_qp = nt37703a_vdo_dphy_range_min_qp,
			.range_max_qp = nt37703a_vdo_dphy_range_max_qp,
			.range_bpg_ofs = nt37703a_vdo_dphy_range_bpg_ofs,
			},
	},
	.panel_bpp = 10,
	.before_power_down = true,
	.set_backlight_delay = true,
};

static struct mtk_panel_params ext_params_60Hz = {
	.pll_clk = MIPI_CLK,
	.data_rate = DATA_RATE,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 60,
		.dfps_cmd_table[0] = {0, 2 , {0x2F, 0x03}},
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0xAB, .count = 2, .para_list[0] = 0x0F, .para_list[1] = 0x07, .revert_flag = 1,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .revert_flag = 0,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0x0E, .count = 1, .para_list[0] = 0x81, .revert_flag = 1,
	},
	.vdo_mix_mode_en = false,
	.vdo_per_frame_lp_enable = 0,

	.vendor = "AA597_P_7_A0025",
	.manufacture = "P_7",

	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.oplus_uiready_before_time = 17,
	.color_vivid_status = true,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.seed_sync = true,

	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2392,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 187,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 3255,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = nt37703a_vdo_dphy_buf_thresh,
			.range_min_qp = nt37703a_vdo_dphy_range_min_qp,
			.range_max_qp = nt37703a_vdo_dphy_range_max_qp,
			.range_bpg_ofs = nt37703a_vdo_dphy_range_bpg_ofs,
			},
	},
	.panel_bpp = 10,
	.before_power_down = true,
	.set_backlight_delay = true,
};

static struct mtk_panel_params ext_params_90Hz = {
	.pll_clk = MIPI_CLK,
	.data_rate = DATA_RATE,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 90,
		.dfps_cmd_table[0] = {0, 2 , {0x2F, 0x02}},
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0xAB, .count = 2, .para_list[0] = 0x0F, .para_list[1] = 0x07, .revert_flag = 1,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .revert_flag = 0,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0x0E, .count = 1, .para_list[0] = 0x81, .revert_flag = 1,
	},
	.vdo_mix_mode_en = false,
	.vdo_per_frame_lp_enable = 0,

	.vendor = "AA597_P_7_A0025",
	.manufacture = "P_7",

	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.oplus_uiready_before_time = 17,
	.color_vivid_status = true,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.seed_sync = true,

	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2392,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 187,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 3255,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = nt37703a_vdo_dphy_buf_thresh,
			.range_min_qp = nt37703a_vdo_dphy_range_min_qp,
			.range_max_qp = nt37703a_vdo_dphy_range_max_qp,
			.range_bpg_ofs = nt37703a_vdo_dphy_range_bpg_ofs,
			},
	},
	.panel_bpp = 10,
	.before_power_down = true,
	.set_backlight_delay = true,
};


static struct mtk_panel_params ext_params_120Hz = {
	.pll_clk = MIPI_CLK,
	.data_rate = DATA_RATE,
	.change_fps_by_vfp_send_cmd = 1,
	.dyn_fps = {
		.switch_en = 1,
		.vact_timing_fps = 120,
		.dfps_cmd_table[0] = {0, 2 , {0x2F, 0x00}},
	},
	.output_mode = MTK_PANEL_DSC_SINGLE_PORT,

	.cust_esd_check = 1,
	.esd_check_enable = 1,
	.lcm_esd_check_table[0] = {
		.cmd = 0xAB, .count = 2, .para_list[0] = 0x0F, .para_list[1] = 0x07, .revert_flag = 1,
	},
	.lcm_esd_check_table[1] = {
		.cmd = 0x0A, .count = 1, .para_list[0] = 0x9C, .revert_flag = 0,
	},
	.lcm_esd_check_table[2] = {
		.cmd = 0x0E, .count = 1, .para_list[0] = 0x81, .revert_flag = 1,
	},
	.vdo_mix_mode_en = false,
	.vdo_per_frame_lp_enable = 0,

	.vendor = "AA597_P_7_A0025",
	.manufacture = "P_7",

	.oplus_ofp_need_keep_apart_backlight = true,
	.oplus_ofp_hbm_on_delay = 11,
	.oplus_ofp_pre_hbm_off_delay = 2,
	.oplus_ofp_hbm_off_delay = 11,
	.oplus_uiready_before_time = 17,
	.color_vivid_status = true,
	.lcm_color_mode = MTK_DRM_COLOR_MODE_DISPLAY_P3,
	.seed_sync = true,

	.dsc_params = {
		.enable = 1,
		.ver = 17,
		.slice_mode = 1,
		.rgb_swap = 0,
		.dsc_cfg = 40,
		.rct_on = 1,
		.bit_per_channel = 10,
		.dsc_line_buf_depth = 11,
		.bp_enable = 1,
		.bit_per_pixel = 128,
		.pic_height = 2392,
		.pic_width = 1080,
		.slice_height = 8,
		.slice_width = 540,
		.chunk_size = 540,
		.xmit_delay = 512,
		.dec_delay = 526,
		.scale_value = 32,
		.increment_interval = 187,
		.decrement_interval = 7,
		.line_bpg_offset = 12,
		.nfl_bpg_offset = 3511,
		.slice_bpg_offset = 3255,
		.initial_offset = 6144,
		.final_offset = 4336,
		.flatness_minqp = 7,
		.flatness_maxqp = 16,
		.rc_model_size = 8192,
		.rc_edge_factor = 6,
		.rc_quant_incr_limit0 = 15,
		.rc_quant_incr_limit1 = 15,
		.rc_tgt_offset_hi = 3,
		.rc_tgt_offset_lo = 3,
		.ext_pps_cfg = {
			.enable = 1,
			.rc_buf_thresh = nt37703a_vdo_dphy_buf_thresh,
			.range_min_qp = nt37703a_vdo_dphy_range_min_qp,
			.range_max_qp = nt37703a_vdo_dphy_range_max_qp,
			.range_bpg_ofs = nt37703a_vdo_dphy_range_bpg_ofs,
			},
	},
	.panel_bpp = 10,
	.before_power_down = true,
	.set_backlight_delay = true,
};

static int panel_set_seed(void *dsi, dcs_write_gce cb, void *handle, unsigned int mode)
{
	unsigned int i = 0;
	pr_info("[DISP][INFO][%s: mode=%d\n", __func__, mode);
	if (!dsi || !cb) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}
	temp_seed_mode = mode;
	switch(mode) {
		case NATURAL:
			for(i = 0; i < sizeof(dsi_set_seed_natural)/sizeof(struct LCM_setting_table); i++) {
				cb(dsi, handle, dsi_set_seed_natural[i].para_list, dsi_set_seed_natural[i].count);
			}
		break;
		case EXPERT:
			for(i = 0; i < sizeof(dsi_set_seed_expert)/sizeof(struct LCM_setting_table); i++) {
				cb(dsi, handle, dsi_set_seed_expert[i].para_list, dsi_set_seed_expert[i].count);
			}
		break;
		case VIVID:
			for(i = 0; i < sizeof(dsi_set_seed_vivid)/sizeof(struct LCM_setting_table); i++) {
				cb(dsi, handle, dsi_set_seed_vivid[i].para_list, dsi_set_seed_vivid[i].count);
			}
		break;
		default:
		break;
	}
	return 0;
}

static int panel_ata_check(struct drm_panel *panel)
{
	/* Customer test by own ATA tool */
	return 1;
}

static unsigned int demura_tap = 0;
static int lcm_setbacklight_cmdq(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	unsigned int i = 0;
	unsigned char bl_level[] = {0x51, 0x03, 0xFF};

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (level == 0) {
		DISP_ERR("[%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
	}

	if (level == 1) {
		DISP_ERR("[%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
		return 0;
	} else if (level > 4095) {
		level = 4095;
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0){
		level = 2047;
	}

	mapped_level = level;
	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

	bl_level[1] = level >> 8;
	bl_level[2] = level & 0xFF;
	if(!handle) {
		cb(dsi, handle, bl_level, ARRAY_SIZE(bl_level));
	} else {
		mipi_dsi_dcs_write_gce(dsi, handle, bl_level, ARRAY_SIZE(bl_level));
	}
	DISP_ERR("aa597_p_7_a0025 backlight = %d bl_level[1]=%x, bl_level[2]=%x\n", level, bl_level[1], bl_level[2]);
	oplus_display_brightness = level;
	lhbm_last_backlight = level;
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &level);

	if (mapped_level <= 1153 && demura_tap != 1) {
		demura_tap = 1;
		DISP_ERR("aa597_p_7_a0025 backlight send demura1\n");
		for (i = 0; i < sizeof(lcm_set_demura_offset1)/sizeof(struct LCM_setting_table); i++){
			if ((sizeof(lcm_set_demura_offset1)/sizeof(struct LCM_setting_table)-1) == i || !handle) {
				cb(dsi, handle, lcm_set_demura_offset1[i].para_list, lcm_set_demura_offset1[i].count);
			} else {
				mipi_dsi_dcs_write_gce(dsi, handle, lcm_set_demura_offset1[i].para_list, lcm_set_demura_offset1[i].count);
			}
		}
	} else if(mapped_level > 1153 && mapped_level <= 4094 && demura_tap != 2) {
		demura_tap = 2;
		DISP_ERR("aa597_p_7_a0025 backlight send demura2\n");
		for (i = 0; i < sizeof(lcm_set_demura_offset2)/sizeof(struct LCM_setting_table); i++){
			if ((sizeof(lcm_set_demura_offset2)/sizeof(struct LCM_setting_table)-1) == i || !handle) {
				cb(dsi, handle, lcm_set_demura_offset2[i].para_list, lcm_set_demura_offset2[i].count);
			} else {
				mipi_dsi_dcs_write_gce(dsi, handle, lcm_set_demura_offset2[i].para_list, lcm_set_demura_offset2[i].count);
			}
		}
	} else if(mapped_level == 4095 && demura_tap != 3) {
		demura_tap = 3;
		DISP_ERR("aa597_p_7_a0025 backlight send demura3\n");
		for (i = 0; i < sizeof(lcm_set_demura_offset3)/sizeof(struct LCM_setting_table); i++){
			if ((sizeof(lcm_set_demura_offset3)/sizeof(struct LCM_setting_table)-1) == i || !handle) {
				cb(dsi, handle, lcm_set_demura_offset3[i].para_list, lcm_set_demura_offset3[i].count);
			} else {
				mipi_dsi_dcs_write_gce(dsi, handle, lcm_set_demura_offset3[i].para_list, lcm_set_demura_offset3[i].count);
			}
		}
	}

	DISP_ERR("aa597_p_7_a0025 setbacklight finish\n");
	return 0;
}

static int oplus_esd_backlight_recovery(void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int level = oplus_display_brightness;
	unsigned char esd_bl_level[] = {0x51, 0x03, 0xFF};
	int i = 0;

	if (!dsi || !cb) {
		return -EINVAL;
	}

	esd_bl_level[1] = level >> 8;
	esd_bl_level[2] = level & 0xFF;
	cb(dsi, handle, esd_bl_level, ARRAY_SIZE(esd_bl_level));
	lhbm_last_backlight = level;

	DISP_ERR("esd_bl_level[1]=%x, esd_bl_level[2]=%x backlight = %d\n", esd_bl_level[1], esd_bl_level[2], level);

	if (level <= 1153 && demura_tap != 1) {
		demura_tap = 1;
		DISP_ERR("aa597_p_7_a0025 esd send demura1\n");
		for (i = 0; i < sizeof(lcm_set_demura_offset1)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_set_demura_offset1[i].para_list, lcm_set_demura_offset1[i].count);
		}
	} else if(level > 1153 && level <= 4094 && demura_tap != 2) {
		demura_tap = 2;
		DISP_ERR("aa597_p_7_a0025 esd send demura2\n");
		for (i = 0; i < sizeof(lcm_set_demura_offset2)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_set_demura_offset2[i].para_list, lcm_set_demura_offset2[i].count);
		}
	} else if(level == 4095 && demura_tap != 3) {
		demura_tap = 3;
		DISP_ERR("aa597_p_7_a0025 esd send demura3\n");
		for (i = 0; i < sizeof(lcm_set_demura_offset3)/sizeof(struct LCM_setting_table); i++){
			cb(dsi, handle, lcm_set_demura_offset3[i].para_list, lcm_set_demura_offset3[i].count);
		}
	}

	DISP_ERR("aa597_p_7_a0025 esd_backlight_recovery finish\n");
	return 0;
}

/*static int lcm_setbacklight_cmdq_witch_hs(void *dsi, dcs_write_gce_pack cb, void *handle, unsigned int level)
{
	unsigned int mapped_level = 0;
	struct LCM_setting_table bl_level[] = {
		{REGFLAG_CMD, 3, {0x51,0x00,0x00}},
	};

	if (!dsi || !cb) {
		return -EINVAL;
	}

	if (level == 0) {
		DISP_ERR("[%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
	}

	if (level == 1) {
		DISP_ERR("[%s:%d]backlight lvl:%u\n", __func__, __LINE__, level);
		return 0;
	} else if (level > 4094) {
		level = 4094;
	}

	if (get_boot_mode() == KERNEL_POWER_OFF_CHARGING_BOOT && level > 0){
		level = 2047;
	}

	mapped_level = level;
	if (mapped_level > 1) {
		lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &mapped_level);
	}

	bl_level[0].para_list[1] = level >> 8;
	bl_level[0].para_list[2] = level & 0xFF;
	panel_send_pack_hs_cmd(dsi, bl_level, 1, cb, handle);
	OFP_INFO("aa597_p_7_a0025 backlight = %d\n", level);
	oplus_display_brightness = level;
	lcdinfo_notify(LCM_BRIGHTNESS_TYPE, &level);
	return 0;
}*/
static int lcm_set_hbm(void *dsi, dcs_write_gce_pack cb,
		void *handle, unsigned int hbm_mode)
{
	unsigned int lcm_cmd_count = 0;
	unsigned int level = oplus_display_brightness;

	if (!dsi || !cb) {
		pr_err("Invalid params\n");
		return -EINVAL;
	}

	OFP_INFO("%s,oplus_display_brightness=%d, hbm_mode=%u\n", __func__, oplus_display_brightness, hbm_mode);
	if (hbm_mode == 1) {
		lcm_cmd_count = sizeof(hbm_on_cmd) / sizeof(struct LCM_setting_table);
		panel_send_pack_hs_cmd(dsi, hbm_on_cmd, lcm_cmd_count, cb, handle);
		OFP_INFO("Enter hbm mode\n");
		lcdinfo_notify(1, &hbm_mode);
	} else if (hbm_mode == 0) {
		hbm_off_cmd[0].para_list[1] = level >> 8;
		hbm_off_cmd[0].para_list[2] = level & 0xFF;
		lcm_cmd_count = sizeof(hbm_off_cmd) / sizeof(struct LCM_setting_table);
		panel_send_pack_hs_cmd(dsi, hbm_off_cmd, lcm_cmd_count, cb, handle);
		lcdinfo_notify(1, &hbm_mode);
		OFP_INFO("level %x\n", level);
	}

	return 0;
}


static unsigned int getBeta(unsigned int level)
{
	unsigned int Beta = 0;
	if (level <= 0x481) {
		Beta = 0x00;
	} else if (level >= 0x482 && level < 0x74E) {
		Beta = 2048 * (level - 1154) / (1870 - 1154);
	} else if (level >= 0x74E && level <= 0xDBB) {
		Beta = (2304 - 2048) * (level - 1870) / (3515 - 1870) + 2048;
	} else if (level <= 0xFFE) {
		Beta = (2304 - 2048) * (level - 3515) / (4094 - 3515) + 2304;
	}
	return Beta;
}

static unsigned int getAlpha(unsigned int level)
{
	unsigned int Alpha = 0;
	if (level <= 0x481) {
		Alpha = 0xEF6;
	} else if (level >= 0x482 && level < 0xDBB) {
		Alpha = 0xE72;
	} else if (level <= 0xFFE) {
		Alpha = (4095 - 3698) * (level - 3515) / (4095 - 3515) + 3698;
	}
	return Alpha;
}

static int oplus_ofp_set_lhbm_pressed_icon(struct drm_panel *panel, void *dsi,
		dcs_write_gce cb, void *handle, bool en)
{
	unsigned int reg_count = 0;
	unsigned int vrefresh_rate = 0;
	struct lcm *ctx = NULL;
	//struct LCM_setting_table *lhbm_pressed_icon_cmd = NULL;
	int i = 0;
	u32 seed_gain = 0;
	static char r_reg1 = 0;
	static char r_reg2 = 0;
	static char g_reg1 = 0;
	static char g_reg2 = 0;
	static char b_reg1 = 0;
	static char b_reg2 = 0;
	unsigned int Beta = 0;
	unsigned int Alpha = 0;
	OFP_DEBUG("start\n");

	if (!oplus_ofp_local_hbm_is_enabled()) {
		OFP_DEBUG("local hbm is not enabled, should not set lhbm pressed icon\n");
	}

	if (!panel || !dsi || !cb) {
		OFP_ERR("Invalid input params\n");
		return -EINVAL;
	}

	ctx = panel_to_lcm(panel);
	if (!ctx) {
		OFP_ERR("Invalid ctx params\n");
	}

	if (!ctx->m) {
		vrefresh_rate = 120;
		OFP_INFO("default refresh rate is 120hz\n");
	} else {
		vrefresh_rate = drm_mode_vrefresh(ctx->m);
	}

	OFP_INFO("%s,oplus_display_brightness=%d, hbm_mode=%d, refresh_rate:%u modeid %u, lhbm_last_backlight %d\n",
			 __func__, oplus_display_brightness, en, vrefresh_rate, mode_id, lhbm_last_backlight);
	if (en) {

		if (temp_seed_mode == VIVID) {
			seed_gain = 400;
		} else if (temp_seed_mode == EXPERT) {
			seed_gain = 416;
		} else if (temp_seed_mode == NATURAL) {
			seed_gain = 410;
		} else {
			seed_gain = 400;
		}
		r_reg1 = ((((regs1[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
				| regs1[AC178_GAMMA_COMPENSATION_REG_INDEX2]) * seed_gain / 100U) >> 8) & 0xFF;
		r_reg2 = (((regs1[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
				| regs1[AC178_GAMMA_COMPENSATION_REG_INDEX2]) * seed_gain / 100U) & 0xFF;
		g_reg1 = ((((regs2[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
				| regs2[AC178_GAMMA_COMPENSATION_REG_INDEX2]) * seed_gain / 100U) >> 8) & 0xFF;
		g_reg2 = (((regs2[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
				| regs2[AC178_GAMMA_COMPENSATION_REG_INDEX2]) * seed_gain / 100U) & 0xFF;
		b_reg1 = ((((regs3[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
				| regs3[AC178_GAMMA_COMPENSATION_REG_INDEX2]) * seed_gain / 100U) >> 8) & 0xFF;
		b_reg2 = (((regs3[AC178_GAMMA_COMPENSATION_REG_INDEX1] << 8)
				| regs3[AC178_GAMMA_COMPENSATION_REG_INDEX2]) * seed_gain / 100U) & 0xFF;
		OFP_INFO("compensation regs1=[%02X %02X], regs2=[%02X %02X], regs3=[%02X %02X]\n",
				r_reg1, r_reg2, g_reg1, g_reg2, b_reg1, b_reg2);
		if (lhbm_pressed_icon_on_cmd[1].count >= (AC178_GAMMA_COMPENSATION_REG_INDEX2 + 1)) {
			lhbm_pressed_icon_on_cmd[1].para_list[AC178_GAMMA_COMPENSATION_REG_INDEX1 + 1] = r_reg1;
			lhbm_pressed_icon_on_cmd[1].para_list[AC178_GAMMA_COMPENSATION_REG_INDEX2 + 1] = r_reg2;
			lhbm_pressed_icon_on_cmd[1].para_list[AC178_GAMMA_COMPENSATION_REG_INDEX3 + 1] = g_reg1;
			lhbm_pressed_icon_on_cmd[1].para_list[AC178_GAMMA_COMPENSATION_REG_INDEX4 + 1] = g_reg2;
			lhbm_pressed_icon_on_cmd[1].para_list[AC178_GAMMA_COMPENSATION_REG_INDEX5 + 1] = b_reg1;
			lhbm_pressed_icon_on_cmd[1].para_list[AC178_GAMMA_COMPENSATION_REG_INDEX6 + 1] = b_reg2;
		} else {
				OFP_INFO("Invalid modeDSI_CMD_GAMMA_COMPENSATION\n");
		}
		Beta = getBeta(lhbm_last_backlight);
		Alpha = getAlpha(lhbm_last_backlight);

		lhbm_pressed_icon_on_cmd[4].para_list[1] = Beta >> 8;
		lhbm_pressed_icon_on_cmd[4].para_list[2] = Beta & 0xFF;
		lhbm_pressed_icon_on_cmd[4].para_list[3] = Alpha >> 8;
		lhbm_pressed_icon_on_cmd[4].para_list[4] = Alpha & 0xFF;

		if (lhbm_last_backlight > 0x481) {
			lhbm_pressed_icon_on_cmd[5].para_list[6] = 0x06;
		} else {
			lhbm_pressed_icon_on_cmd[5].para_list[6] = 0x16;
		}

		for (i = 0; i < sizeof(lhbm_pressed_icon_on_cmd) / sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, lhbm_pressed_icon_on_cmd[i].para_list, lhbm_pressed_icon_on_cmd[i].count);
		}

		//panel_send_pack_hs_cmd(dsi, lhbm_pressed_icon_cmd, reg_count, cb, handle);
	} else if (en == 0) {
		reg_count = sizeof(lhbm_pressed_icon_off_cmd) / sizeof(struct LCM_setting_table);
		for (i = 0; i < reg_count; i++) {
			cb(dsi, handle, lhbm_pressed_icon_off_cmd[i].para_list, lhbm_pressed_icon_off_cmd[i].count);
		}
		//panel_send_pack_hs_cmd(dsi, lhbm_pressed_icon_off_cmd, reg_count, cb, handle);
		lcm_setbacklight_cmdq(dsi, cb, handle, oplus_display_brightness);
	}
	return 0;
}

static int panel_doze_disable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int cmd;

	if (!panel || !dsi) {
		pr_err("Invalid dsi params\n");
	}

	for (i = 0; i < (sizeof(aod_off_cmd) / sizeof(struct LCM_setting_table)); i++) {

		cmd = aod_off_cmd[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY:
				if (handle == NULL) {
					usleep_range(aod_off_cmd[i].count * 1000, aod_off_cmd[i].count * 1000 + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count * 1000), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_UDELAY:
				if (handle == NULL) {
					usleep_range(aod_off_cmd[i].count, aod_off_cmd[i].count + 100);
				} else {
					cmdq_pkt_sleep(handle, CMDQ_US_TO_TICK(aod_off_cmd[i].count), CMDQ_GPR_R14);
				}
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				cb(dsi, handle, aod_off_cmd[i].para_list, aod_off_cmd[i].count);
		}
	}

	OFP_INFO("%s:success\n", __func__);
	atomic_set(&esd_pending, 0);
	return 0;
}


static int panel_doze_enable(struct drm_panel *panel, void *dsi, dcs_write_gce cb, void *handle)
{
	unsigned int i = 0;
	unsigned int cmd;
	unsigned int reg_count = 0;
	if (!panel || !dsi) {
		pr_err("Invalid dsi params\n");
	}

	if(oplus_ofp_local_hbm_is_enabled() && oplus_ofp_get_hbm_state()) {
		reg_count = sizeof(lhbm_pressed_icon_off_cmd) / sizeof(struct LCM_setting_table);
		for (i = 0; i < reg_count; i++) {
			cb(dsi, handle, lhbm_pressed_icon_off_cmd[i].para_list, lhbm_pressed_icon_off_cmd[i].count);
		}
		OFP_INFO("Should off hbm\n");
	}

	atomic_set(&esd_pending, 1);
	for (i = 0; i < (sizeof(aod_on_cmd)/sizeof(struct LCM_setting_table)); i++) {
		cmd = aod_on_cmd[i].cmd;
		switch (cmd) {
			case REGFLAG_DELAY:
				usleep_range(aod_on_cmd[i].count * 1000, aod_on_cmd[i].count * 1000 + 100);
				break;
			case REGFLAG_UDELAY:
				usleep_range(aod_on_cmd[i].count, aod_on_cmd[i].count + 100);
				break;
			case REGFLAG_END_OF_TABLE:
				break;
			default:
				{
					cb(dsi, handle, aod_on_cmd[i].para_list, aod_on_cmd[i].count);
				}
		}
	}

	OFP_INFO("%s:success\n", __func__);
	return 0;
}

static int panel_set_aod_light_mode(void *dsi, dcs_write_gce cb, void *handle, unsigned int level)
{
	int i = 0;

	if (level == 0) {
		for (i = 0; i < sizeof(aod_high_mode)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_high_mode[i].para_list, aod_high_mode[i].count);
		}
	} else {
		for (i = 0; i < sizeof(aod_low_mode)/sizeof(struct LCM_setting_table); i++) {
			cb(dsi, handle, aod_low_mode[i].para_list, aod_low_mode[i].count);
		}
	}
	OFP_INFO("%s:success %d !\n", __func__, level);

	return 0;
}

static struct vdo_aod_params vdo_aod_on = {
	.porch_change_flag = 0x03,
	.dst_hfp = 2044,
	.dst_vfp = 84, //30fps
	.vdo_aod_cmd_table[0]={1, {0x39} },
	.vdo_aod_cmd_table[1]={2, {0x6F, 0x04} },
	.vdo_aod_cmd_table[2]={3, {0x51, 0x07, 0xFF}},
};


static struct vdo_aod_params vdo_aod_to_120hz = {
	.porch_change_flag = 0x03,
	.dst_hfp = 180,
	.dst_vfp = 84,
	.vdo_aod_cmd_table[0]={1, {0x38}},

};

static struct vdo_aod_params vdo_aod_to_120hz_unlocking = {
        .porch_change_flag = 0x03,
        .dst_hfp = 180,
        .dst_vfp = 84,
        .vdo_aod_cmd_table[0]={1, {0x38}},
        .vdo_aod_cmd_table[1]={3, {0x51, 0x00, 0x00}},

};


static struct vdo_aod_params vdo_aod_to_90hz = {
        .porch_change_flag = 0x03,
        .dst_hfp = 180,
        .dst_vfp = 900,
        .vdo_aod_cmd_table[0]={1, {0x38}},

};

static struct vdo_aod_params vdo_aod_to_90hz_unlocking = {
        .porch_change_flag = 0x03,
        .dst_hfp = 180,
        .dst_vfp = 900,
        .vdo_aod_cmd_table[0]={1, {0x38}},
	.vdo_aod_cmd_table[1]={3, {0x51, 0x00, 0x00}},

};

static struct vdo_aod_params vdo_aod_to_60hz = {
        .porch_change_flag = 0x03,
        .dst_hfp = 180,
        .dst_vfp = 2580,
        .vdo_aod_cmd_table[0]={1, {0x38}},

};

static struct vdo_aod_params vdo_aod_to_60hz_unlocking = {
        .porch_change_flag = 0x03,
        .dst_hfp = 180,
        .dst_vfp = 2580,
        .vdo_aod_cmd_table[0]={1, {0x38}},
        .vdo_aod_cmd_table[1]={3, {0x51, 0x00, 0x00}},

};

static int mtk_get_vdo_aod_param(int aod_en, struct vdo_aod_params **vdo_aod_param)
{
	static int mode_id_before_aod = 2;
	if(aod_en) {
		atomic_set(&esd_pending, 1);
		*vdo_aod_param = &vdo_aod_on;
		mode_id_before_aod = mode_id;
	} else {
		if(mode_id_before_aod == FHD_SDC60) {
			if(oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_60hz_unlocking;
			else {
				*vdo_aod_param = &vdo_aod_to_60hz;
				OFP_INFO("%s:before mode_id %d\n", __func__, mode_id_before_aod);
			}
		}
		else if (mode_id_before_aod == FHD_SDC90) {
			if (oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_90hz_unlocking;
			else {
				*vdo_aod_param = &vdo_aod_to_90hz;
				OFP_INFO("%s:before mode_id %d\n", __func__, mode_id_before_aod);
			}
		}
		else {
			if(oplus_ofp_get_aod_unlocking())
				*vdo_aod_param = &vdo_aod_to_120hz_unlocking;
			else {
				*vdo_aod_param = &vdo_aod_to_120hz;
				OFP_INFO("%s:before mode_id %d\n", __func__, mode_id_before_aod);
			}
		}
		atomic_set(&esd_pending, 0);
	}
	OFP_INFO("%s:aod_en %d, mode_id %d, unlocking =%d\n", __func__, aod_en, mode_id, oplus_ofp_get_aod_unlocking());
	return 0;
}


static int panel_ext_reset(struct drm_panel *panel, int on)
{
	// struct lcm *ctx = panel_to_lcm(panel);

	// ctx->reset_gpio =
	// 	devm_gpiod_get(ctx->dev, "reset", GPIOD_OUT_HIGH);
	// if (IS_ERR(ctx->reset_gpio)) {
	// 	dev_info(ctx->dev, "%s: cannot get reset_gpio %ld\n",
	// 		__func__, PTR_ERR(ctx->reset_gpio));
	// 	return PTR_ERR(ctx->reset_gpio);
	// }
	// gpiod_set_value(ctx->reset_gpio, on);
	// devm_gpiod_put(ctx->dev, ctx->reset_gpio);

	return 0;
}

static struct regulator *vrf18_aif;
static int lcm_panel_1p8_ldo_regulator_init(struct device *dev)
{
	static int regulator_1p8_inited;
	int ret = 0;

	if (regulator_1p8_inited)
		return ret;

	/* please only get regulator once in a driver */
	vrf18_aif = devm_regulator_get(dev, "1p8");
	if (IS_ERR_OR_NULL(vrf18_aif)) { /* handle return value */
		ret = PTR_ERR(vrf18_aif);
		pr_err("get vrf18_aif fail, error: %d\n", ret);
	}
		regulator_1p8_inited = 1;
		pr_info("get lcm_panel_1p8_ldo_regulator_init\n");
		return ret; /* must be 0 */

}

static int lcm_panel_1p8_ldo_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_1p8_ldo_regulator_init(dev);

	/* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(vrf18_aif)) {
		ret = regulator_set_voltage(vrf18_aif, 1800000, 1800000);
		if (ret < 0)
			pr_err("set voltage vrf18_aif fail, ret = %d\n", ret);
		retval |= ret;
	}
	/* enable regulator */
	if (!IS_ERR_OR_NULL(vrf18_aif)) {
		ret = regulator_enable(vrf18_aif);
		if (ret < 0)
			pr_err("enable regulator vrf18_aif fail, ret = %d\n", ret);
		retval |= ret;
	}
	pr_info("get lcm_panel_1p8_ldo_enable\n");

	return retval;
}

static int lcm_panel_1p8_ldo_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;

	lcm_panel_1p8_ldo_regulator_init(dev);

	if (!IS_ERR_OR_NULL(vrf18_aif)) {
		ret = regulator_disable(vrf18_aif);
		if (ret < 0)
			pr_err("disable regulator vrf18_aif fail, ret = %d\n", ret);
		retval |= ret;
	}
	return ret;
}

static struct regulator *mt6319_8_vbuck4;
static int vddr8_buck4_regulator_init(struct device *dev)
{
	static int regulator_inited;
	int ret = 0;

	if (regulator_inited)
		return ret;

	DISP_ERR("get vddr8_buck4_regulator_init +\n");

/* please only get regulator once in a driver */
	mt6319_8_vbuck4 = devm_regulator_get_optional(dev, "8_vbuck4");
	if (IS_ERR_OR_NULL(mt6319_8_vbuck4)) { /* handle return value */
		ret = PTR_ERR(mt6319_8_vbuck4);
		pr_err("get vddr8_buck4_optional fail, error: %d\n", ret);
		//return ret;
	}
	regulator_inited = 1;
	DISP_ERR("get vddr8_buck4_regulator_init -\n");
	return ret; /* must be 0 */
}

static int vddr8_buck4_regulator_enable(struct device *dev)
{
	int ret = 0;
	int retval = 0;
	DISP_ERR("get vddr8_buck4_regulator_enable +\n");
	vddr8_buck4_regulator_init(dev);
	/* set voltage with min & max*/
	if (!IS_ERR_OR_NULL(mt6319_8_vbuck4)) {
		ret = regulator_set_voltage(mt6319_8_vbuck4, 1185000, 1187500);
		if (ret < 0)
			DISP_ERR("set voltage mt6319_8_vbuck4 fail, ret = %d\n", ret);
		retval |= ret;
	}

	/* enable regulator */
	if (!IS_ERR_OR_NULL(mt6319_8_vbuck4)) {
		ret = regulator_enable(mt6319_8_vbuck4);
		if (ret < 0)
			DISP_ERR("enable regulator mt6319_8_vbuck4 fail, ret = %d\n", ret);
		retval |= ret;
	}
	DISP_ERR("get vddr8_buck4_regulator_enable -\n");
	return retval;
}

static int vddr8_buck4_regulator_disable(struct device *dev)
{
	int ret = 0;
	int retval = 0;
	DISP_ERR("get vddr8_buck4_regulator_disable +\n");
	vddr8_buck4_regulator_init(dev);
	if (!IS_ERR_OR_NULL(mt6319_8_vbuck4)) {
		ret = regulator_disable(mt6319_8_vbuck4);
		if (ret < 0)
			DISP_ERR("disable regulator mt6319_8_vbuck4 fail, ret = %d\n", ret);
		retval |= ret;
	}
	DISP_ERR("get vddr8_buck4_regulator_disable -\n");
	return retval;
}

static int lcm_panel_poweron(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	DISP_ERR("%s: aa597_p_7_a0025 poweron Start\n", __func__);
	//iovcc enable 1.8V
	lcm_panel_1p8_ldo_enable(ctx->dev);
	/* Wait > 1ms, actual 5ms */
	usleep_range(5000, 5100);
	// enable VDDR 1P2 6319 vbuck4
	vddr8_buck4_regulator_enable(ctx->dev);
	/* Wait no limits, actual 3ms */
	usleep_range(5000, 5100);
	//enable gpio 3p0
	gpiod_set_value(ctx->vci3p0_enable_gpio, 1);
	/* Wait > 10ms, actual 12ms */
	usleep_range(22000, 22100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);

	DISP_ERR("%s: aa597_p_7_a0025 poweron Successful\n", __func__);
	return 0;
}

static int lcm_panel_poweroff(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);
	int ret;

	if (ctx->prepared)
		return 0;

	DISP_ERR("%s: aa597_p_7_a0025 poweroff Start\n", __func__);
	usleep_range(10000, 10100);
	gpiod_set_value(ctx->reset_gpio, 0);
	/* Wait > 1ms, actual 5ms */
	usleep_range(5000, 5100);
	//disable gpio 3p0
	gpiod_set_value(ctx->vci3p0_enable_gpio, 0);
	/* Wait no limits, actual 5ms */
	usleep_range(10000, 10100);
	// disable VDDR 1P2 6319 vbuck4
	vddr8_buck4_regulator_disable(ctx->dev);
	/* Wait > 1ms, actual 5ms */
	usleep_range(10000, 10100);
	//set vddi 1.8v
	lcm_panel_1p8_ldo_disable(ctx->dev);
	/* power off Foolproof, actual 70ms*/
	usleep_range(72000, 72100);

	ret = ctx->error;
	if (ret < 0)
		lcm_unprepare(panel);
	demura_tap = 0;
	DISP_ERR("%s: aa597_p_7_a0025 poweroff Successful\n", __func__);
	return 0;
}

static int lcm_panel_reset(struct drm_panel *panel)
{
	struct lcm *ctx = panel_to_lcm(panel);

	if (ctx->prepared)
		return 0;

	pr_info("[LCM]debug for lcd reset :%s, ctx->prepared:%d\n", __func__, ctx->prepared);

	if(IS_ERR(ctx->reset_gpio)){
		pr_err("cannot get reset-gpios %ld\n",PTR_ERR(ctx->reset_gpio));
	}

	gpiod_set_value(ctx->reset_gpio,1);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio,0);
	usleep_range(5000, 5100);
	gpiod_set_value(ctx->reset_gpio, 1);
	usleep_range(15000, 15100);

	return 0;
}

struct drm_display_mode *get_mode_by_id(struct drm_connector *connector,
	unsigned int mode)
{
	struct drm_display_mode *m;
	unsigned int i = 0;

	list_for_each_entry(m, &connector->modes, head) {
		if (i == mode)
			return m;
		i++;
	}
	return NULL;
}

static int mtk_panel_ext_param_set(struct drm_panel *panel,
			struct drm_connector *connector, unsigned int mode)
{
	struct mtk_panel_ext *ext = find_panel_ext(panel);
	int ret = 0;
	int m_vrefresh = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, mode);

	m_vrefresh = drm_mode_vrefresh(m);
	pr_info("%s: mode=%d, vrefresh=%d\n", __func__, mode, drm_mode_vrefresh(m));

	if (m_vrefresh == 60) {
		ext->params = &ext_params_60Hz;
		current_fps = 60;
	} else if (m_vrefresh == 90) {
		ext->params = &ext_params_90Hz;
		current_fps = 90;
	}else if (m_vrefresh == 120) {
		ext->params = &ext_params_120Hz;
		current_fps = 120;
	} else if (m_vrefresh == 30) {
		ext->params = &ext_params_30Hz;
		current_fps = 30;
	}
	else {
		ret = 1;
	}

	return ret;
}

static int mtk_panel_ext_param_get(struct drm_panel *panel,
		struct drm_connector *connector,
		struct mtk_panel_params **ext_param,
		unsigned int id)
{
	int ret = 0;
	struct drm_display_mode *m = get_mode_by_id(connector, id);

	if (get_mode_enum(m) != FHD_SDC30) {
		mode_id = get_mode_enum(m);
	}

	if (mode_id == FHD_SDC60) {
		*ext_param = &ext_params_60Hz;
	} else if (mode_id == FHD_SDC90) {
		*ext_param = &ext_params_90Hz;
	} else if (mode_id == FHD_SDC120) {
		*ext_param = &ext_params_120Hz;
	} else {
		*ext_param = &ext_params_60Hz;
	}

	if (*ext_param)
		DISP_DEBUG("aa597_p_7_a0025 data_rate:%d\n", (*ext_param)->data_rate);
	else
		DISP_ERR("aa597_p_7_a0025 ext_param is NULL;\n");

	return ret;
}

static struct mtk_panel_funcs ext_funcs = {
	.reset = panel_ext_reset,
	.set_backlight_cmdq = lcm_setbacklight_cmdq,
	.panel_poweron = lcm_panel_poweron,
	.panel_poweroff = lcm_panel_poweroff,
	.panel_reset = lcm_panel_reset,
	.ata_check = panel_ata_check,
	.ext_param_get = mtk_panel_ext_param_get,
	.ext_param_set = mtk_panel_ext_param_set,
	//.mode_switch = mode_switch,
	.oplus_set_hbm = lcm_set_hbm,
	.oplus_ofp_set_lhbm_pressed_icon_single = oplus_ofp_set_lhbm_pressed_icon,
	.doze_disable = panel_doze_disable,
	.doze_enable = panel_doze_enable,
	.set_aod_light_mode = panel_set_aod_light_mode,
	.esd_backlight_recovery = oplus_esd_backlight_recovery,
	.set_seed = panel_set_seed,
	.get_vdo_aod_param = mtk_get_vdo_aod_param,
};

static int lcm_get_modes(struct drm_panel *panel,
					struct drm_connector *connector)
{
	struct drm_display_mode *mode[4];

	mode[0] = drm_mode_duplicate(connector->dev, &disp_mode_60Hz);
	if (!mode[0]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_60Hz.hdisplay, disp_mode_60Hz.vdisplay, drm_mode_vrefresh(&disp_mode_60Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[0]);
	mode[0]->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode[0]);
	pr_info("%s clock=%d,htotal=%d,vtotal=%d,hskew=%d,vrefresh=%d\n", __func__, mode[0]->clock, mode[0]->htotal,
		mode[0]->vtotal, mode[0]->hskew, drm_mode_vrefresh(mode[0]));

	mode[1] = drm_mode_duplicate(connector->dev, &disp_mode_90Hz);
	if (!mode[1]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_90Hz.hdisplay, disp_mode_90Hz.vdisplay, drm_mode_vrefresh(&disp_mode_90Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[1]);
	mode[1]->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode[1]);

	mode[2] = drm_mode_duplicate(connector->dev, &disp_mode_120Hz);
	if (!mode[2]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_120Hz.hdisplay, disp_mode_120Hz.vdisplay, drm_mode_vrefresh(&disp_mode_120Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[2]);
	mode[2]->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
	drm_mode_probed_add(connector, mode[2]);

	mode[3] = drm_mode_duplicate(connector->dev, &disp_mode_30Hz);
	if (!mode[3]) {
		pr_info("%s failed to add mode %ux%ux@%u\n", __func__, disp_mode_30Hz.hdisplay, disp_mode_30Hz.vdisplay, drm_mode_vrefresh(&disp_mode_30Hz));
		return -ENOMEM;
	}
	drm_mode_set_name(mode[3]);
	mode[3]->type = DRM_MODE_TYPE_DRIVER;
	drm_mode_probed_add(connector, mode[3]);


	connector->display_info.width_mm = PHYSICAL_WIDTH / 1000;
	connector->display_info.height_mm = PHYSICAL_HEIGHT / 1000;

	return 1;
}

static const struct drm_panel_funcs lcm_drm_funcs = {
	.disable = lcm_disable,
	.unprepare = lcm_unprepare,
	.prepare = lcm_prepare,
	.enable = lcm_enable,
	.get_modes = lcm_get_modes,
};

static int lcm_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct device_node *dsi_node, *remote_node = NULL, *endpoint = NULL;
	struct lcm *ctx;
	struct device_node *backlight;
	int ret;

	pr_info("[LCM] aa597_p_7_a0025 %s START\n", __func__);


	dsi_node = of_get_parent(dev->of_node);
	if (dsi_node) {
		endpoint = of_graph_get_next_endpoint(dsi_node, NULL);
		if (endpoint) {
			remote_node = of_graph_get_remote_port_parent(endpoint);
			if (!remote_node) {
				pr_info("No panel connected,skip probe lcm\n");
				return -ENODEV;
			}
			pr_info("device node name:%s\n", remote_node->name);
		}
	}
	if (remote_node != dev->of_node) {
		pr_info("%s+ skip probe due to not current lcm\n", __func__);
		return -ENODEV;
	}

	ctx = devm_kzalloc(dev, sizeof(struct lcm), GFP_KERNEL);
	if (!ctx)
		return -ENOMEM;

	mipi_dsi_set_drvdata(dsi, ctx);

	ctx->dev = dev;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE
			 | MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_NO_EOT_PACKET
			 | MIPI_DSI_CLOCK_NON_CONTINUOUS;

	backlight = of_parse_phandle(dev->of_node, "backlight", 0);
	if (backlight) {
		ctx->backlight = of_find_backlight_by_node(backlight);
		of_node_put(backlight);

		if (!ctx->backlight)
			return -EPROBE_DEFER;
	}

	ctx->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ctx->reset_gpio)) {
		dev_info(dev, "%s: cannot get reset-gpios %ld\n",
			__func__, PTR_ERR(ctx->reset_gpio));
		return PTR_ERR(ctx->reset_gpio);
	}
	devm_gpiod_put(dev, ctx->reset_gpio);

	usleep_range(5000, 5100);
	lcm_panel_1p8_ldo_enable(ctx->dev);

	usleep_range(5000, 5100);

	vddr8_buck4_regulator_enable(ctx->dev);

	usleep_range(5000, 5100);

	ctx->vci3p0_enable_gpio = devm_gpiod_get(dev, "3p0", GPIOD_OUT_HIGH);
	if (IS_ERR_OR_NULL(ctx->vci3p0_enable_gpio)) {
		pr_err("cannot get vci3p0_enable_gpio %ld\n",
			 PTR_ERR(ctx->vci3p0_enable_gpio));
		return PTR_ERR(ctx->vci3p0_enable_gpio);
	}
    gpiod_set_value(ctx->vci3p0_enable_gpio, 1);

	usleep_range(5000, 5100);

	ctx->prepared = true;
	ctx->enabled = true;

	drm_panel_init(&ctx->panel, dev, &lcm_drm_funcs, DRM_MODE_CONNECTOR_DSI);

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret < 0)
		drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_tch_handle_reg(&ctx->panel);
	ret = mtk_panel_ext_create(dev, &ext_params_60Hz, &ext_funcs, &ctx->panel);
	if (ret < 0)
		return ret;
#endif

	oplus_display_panel_dbv_probe(dev);
	oplus_serial_number_probe(dev);
	register_device_proc("lcd", "AA597_P_7_A0025", "P_7");
	ctx->hbm_en = false;
	oplus_max_normal_brightness = MAX_NORMAL_BRIGHTNESS;
	oplus_ofp_init(dev);


	pr_info("[LCM] %s- lcm, aa597_p_7_a0025, END\n", __func__);


	return ret;
}

static void lcm_remove(struct mipi_dsi_device *dsi)
{
	struct lcm *ctx = mipi_dsi_get_drvdata(dsi);
#if defined(CONFIG_MTK_PANEL_EXT)
	struct mtk_panel_ctx *ext_ctx = find_panel_ctx(&ctx->panel);
#endif

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);

#if defined(CONFIG_MTK_PANEL_EXT)
	mtk_panel_detach(ext_ctx);
	mtk_panel_remove(ext_ctx);
#endif

}

static const struct of_device_id lcm_of_match[] = {
	{
		.compatible = "aa597,p,7,a0025,vdo",
	},
	{ }
};

MODULE_DEVICE_TABLE(of, lcm_of_match);

static struct mipi_dsi_driver lcm_driver = {
	.probe = lcm_probe,
	.remove = lcm_remove,
	.driver = {
		.name = "aa597_p_7_a0025_dsi_vdo",
		.owner = THIS_MODULE,
		.of_match_table = lcm_of_match,
	},
};

module_mipi_dsi_driver(lcm_driver);

MODULE_AUTHOR("oplus");
MODULE_DESCRIPTION("lcm AMOLED Panel Driver");
MODULE_LICENSE("GPL v2");
