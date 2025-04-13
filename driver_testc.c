// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright © 2023 Raspberry Pi Ltd
 *
 * Based on panel-raspberrypi-touchscreen by Broadcom
 */

 #include <linux/delay.h>
 #include <linux/device.h>
 #include <linux/err.h>
 #include <linux/errno.h>
 #include <linux/kernel.h>
 #include <linux/module.h>
 #include <linux/of.h>
 #include <linux/of_device.h>
 #include <linux/gpio/consumer.h>
 #include <linux/regulator/consumer.h>
 #include <linux/of_graph.h>
 
 #include <drm/drm_mipi_dsi.h>
 #include <drm/drm_modes.h>
 #include <drm/drm_panel.h>
 #include <drm/drm_crtc.h>
 #include <video/mipi_display.h>
 #include <linux/backlight.h>
 #include <linux/err.h>
 #include <linux/gpio.h>
 #include <linux/gpio/driver.h>
 #include <linux/module.h>
 #include <linux/regmap.h>
 #include <linux/regulator/driver.h>
 #include <linux/of.h>

/* I2C registers of the microcontroller. */
#define REG_TP				0x94
#define REG_LCD				0x95
#define REG_PWM				0x96
#define REG_SIZE			0x97
#define REG_ID				0x98
#define REG_VERSION			0x99

#define NUM_GPIO	16	/* Treat BL_ENABLE, LCD_RESET, TP_RESET as GPIOs */

struct ws_panel_desc {
	const struct panel_init_cmd *init;
	const struct drm_display_mode *mode;
	const unsigned long mode_flags;
	unsigned int lanes;
	enum mipi_dsi_pixel_format format;
};

struct ws_panel {
	struct drm_panel	panel;
	struct mipi_dsi_device	*dsi;
	const struct ws_panel_desc	*desc;

	struct regulator	*power;
	struct gpio_desc	*reset;

	enum drm_panel_orientation	orientation;
};

struct waveshare_panel_lcd {
	struct mutex	lock;
	struct regmap	*regmap;
    struct ws_panel ctx;
	u16 poweron_state;
	u16 direction_state;

	struct gpio_chip gc;
};

static const struct regmap_config waveshare_panel_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = REG_PWM,
};



enum dsi_cmd_type {
	INIT_DCS_CMD,
	DELAY_CMD,
};

struct panel_init_cmd {
	enum dsi_cmd_type type;
	size_t len;
	const char *data;
};

#define _INIT_DCS_CMD(...) { \
	.type = INIT_DCS_CMD, \
	.len = sizeof((char[]){__VA_ARGS__}), \
	.data = (char[]){__VA_ARGS__} }

#define _INIT_DELAY_CMD(...) { \
	.type = DELAY_CMD,\
	.len = sizeof((char[]){__VA_ARGS__}), \
	.data = (char[]){__VA_ARGS__} }

static const struct panel_init_cmd ws_panel_10_1_a_init[] = {
    _INIT_DCS_CMD(0xE0, 0x00),
    _INIT_DCS_CMD(0xE1, 0x93),
    _INIT_DCS_CMD(0xE2, 0x65),
    _INIT_DCS_CMD(0xE3, 0xF8),
    _INIT_DCS_CMD(0x80, 0x01),
    _INIT_DCS_CMD(0xE0, 0x01),
    _INIT_DCS_CMD(0x00, 0x00),
    _INIT_DCS_CMD(0x01, 0x38),
    _INIT_DCS_CMD(0x03, 0x10),
    _INIT_DCS_CMD(0x04, 0x38),
    _INIT_DCS_CMD(0x0C, 0x74),
    _INIT_DCS_CMD(0x17, 0x00),
    _INIT_DCS_CMD(0x18, 0xAF),
    _INIT_DCS_CMD(0x19, 0x00),
    _INIT_DCS_CMD(0x1A, 0x00),
    _INIT_DCS_CMD(0x1B, 0xAF),
    _INIT_DCS_CMD(0x1C, 0x00),
    _INIT_DCS_CMD(0x35, 0x26),
    _INIT_DCS_CMD(0x37, 0x09),
    _INIT_DCS_CMD(0x38, 0x04),
    _INIT_DCS_CMD(0x39, 0x00),
    _INIT_DCS_CMD(0x3A, 0x01),
    _INIT_DCS_CMD(0x3C, 0x78),
    _INIT_DCS_CMD(0x3D, 0xFF),
    _INIT_DCS_CMD(0x3E, 0xFF),
    _INIT_DCS_CMD(0x3F, 0x7F),
    _INIT_DCS_CMD(0x40, 0x06),
    _INIT_DCS_CMD(0x41, 0xA0),
    _INIT_DCS_CMD(0x42, 0x81),
    _INIT_DCS_CMD(0x43, 0x1E),
    _INIT_DCS_CMD(0x44, 0x0D),
    _INIT_DCS_CMD(0x45, 0x28),
    _INIT_DCS_CMD(0x55, 0x02),
    _INIT_DCS_CMD(0x57, 0x69),
    _INIT_DCS_CMD(0x59, 0x0A),
    _INIT_DCS_CMD(0x5A, 0x2A),
    _INIT_DCS_CMD(0x5B, 0x17),
    _INIT_DCS_CMD(0x5D, 0x7F),
    _INIT_DCS_CMD(0x5E, 0x6A),
    _INIT_DCS_CMD(0x5F, 0x5B),
    _INIT_DCS_CMD(0x60, 0x4F),
    _INIT_DCS_CMD(0x61, 0x4A),
    _INIT_DCS_CMD(0x62, 0x3D),
    _INIT_DCS_CMD(0x63, 0x41),
    _INIT_DCS_CMD(0x64, 0x2A),
    _INIT_DCS_CMD(0x65, 0x44),
    _INIT_DCS_CMD(0x66, 0x43),
    _INIT_DCS_CMD(0x67, 0x44),
    _INIT_DCS_CMD(0x68, 0x62),
    _INIT_DCS_CMD(0x69, 0x52),
    _INIT_DCS_CMD(0x6A, 0x59),
    _INIT_DCS_CMD(0x6B, 0x4C),
    _INIT_DCS_CMD(0x6C, 0x48),
    _INIT_DCS_CMD(0x6D, 0x3A),
    _INIT_DCS_CMD(0x6E, 0x26),
    _INIT_DCS_CMD(0x6F, 0x00),
    _INIT_DCS_CMD(0x70, 0x7F),
    _INIT_DCS_CMD(0x71, 0x6A),
    _INIT_DCS_CMD(0x72, 0x5B),
    _INIT_DCS_CMD(0x73, 0x4F),
    _INIT_DCS_CMD(0x74, 0x4A),
    _INIT_DCS_CMD(0x75, 0x3D),
    _INIT_DCS_CMD(0x76, 0x41),
    _INIT_DCS_CMD(0x77, 0x2A),
    _INIT_DCS_CMD(0x78, 0x44),
    _INIT_DCS_CMD(0x79, 0x43),
    _INIT_DCS_CMD(0x7A, 0x44),
    _INIT_DCS_CMD(0x7B, 0x62),
    _INIT_DCS_CMD(0x7C, 0x52),
    _INIT_DCS_CMD(0x7D, 0x59),
    _INIT_DCS_CMD(0x7E, 0x4C),
    _INIT_DCS_CMD(0x7F, 0x48),
    _INIT_DCS_CMD(0x80, 0x3A),
    _INIT_DCS_CMD(0x81, 0x26),
    _INIT_DCS_CMD(0x82, 0x00),
    _INIT_DCS_CMD(0xE0, 0x02),
    _INIT_DCS_CMD(0x00, 0x42),
    _INIT_DCS_CMD(0x01, 0x42),
    _INIT_DCS_CMD(0x02, 0x40),
    _INIT_DCS_CMD(0x03, 0x40),
    _INIT_DCS_CMD(0x04, 0x5E),
    _INIT_DCS_CMD(0x05, 0x5E),
    _INIT_DCS_CMD(0x06, 0x5F),
    _INIT_DCS_CMD(0x07, 0x5F),
    _INIT_DCS_CMD(0x08, 0x5F),
    _INIT_DCS_CMD(0x09, 0x57),
    _INIT_DCS_CMD(0x0A, 0x57),
    _INIT_DCS_CMD(0x0B, 0x77),
    _INIT_DCS_CMD(0x0C, 0x77),
    _INIT_DCS_CMD(0x0D, 0x47),
    _INIT_DCS_CMD(0x0E, 0x47),
    _INIT_DCS_CMD(0x0F, 0x45),
    _INIT_DCS_CMD(0x10, 0x45),
    _INIT_DCS_CMD(0x11, 0x4B),
    _INIT_DCS_CMD(0x12, 0x4B),
    _INIT_DCS_CMD(0x13, 0x49),
    _INIT_DCS_CMD(0x14, 0x49),
    _INIT_DCS_CMD(0x15, 0x5F),
    _INIT_DCS_CMD(0x16, 0x41),
    _INIT_DCS_CMD(0x17, 0x41),
    _INIT_DCS_CMD(0x18, 0x40),
    _INIT_DCS_CMD(0x19, 0x40),
    _INIT_DCS_CMD(0x1A, 0x5E),
    _INIT_DCS_CMD(0x1B, 0x5E),
    _INIT_DCS_CMD(0x1C, 0x5F),
    _INIT_DCS_CMD(0x1D, 0x5F),
    _INIT_DCS_CMD(0x1E, 0x5F),
    _INIT_DCS_CMD(0x1F, 0x57),
    _INIT_DCS_CMD(0x20, 0x57),
    _INIT_DCS_CMD(0x21, 0x77),
    _INIT_DCS_CMD(0x22, 0x77),
    _INIT_DCS_CMD(0x23, 0x46),
    _INIT_DCS_CMD(0x24, 0x46),
    _INIT_DCS_CMD(0x25, 0x44),
    _INIT_DCS_CMD(0x26, 0x44),
    _INIT_DCS_CMD(0x27, 0x4A),
    _INIT_DCS_CMD(0x28, 0x4A),
    _INIT_DCS_CMD(0x29, 0x48),
    _INIT_DCS_CMD(0x2A, 0x48),
    _INIT_DCS_CMD(0x2B, 0x5F),
    _INIT_DCS_CMD(0x2C, 0x01),
    _INIT_DCS_CMD(0x2D, 0x01),
    _INIT_DCS_CMD(0x2E, 0x00),
    _INIT_DCS_CMD(0x2F, 0x00),
    _INIT_DCS_CMD(0x30, 0x1F),
    _INIT_DCS_CMD(0x31, 0x1F),
    _INIT_DCS_CMD(0x32, 0x1E),
    _INIT_DCS_CMD(0x33, 0x1E),
    _INIT_DCS_CMD(0x34, 0x1F),
    _INIT_DCS_CMD(0x35, 0x17),
    _INIT_DCS_CMD(0x36, 0x17),
    _INIT_DCS_CMD(0x37, 0x37),
    _INIT_DCS_CMD(0x38, 0x37),
    _INIT_DCS_CMD(0x39, 0x08),
    _INIT_DCS_CMD(0x3A, 0x08),
    _INIT_DCS_CMD(0x3B, 0x0A),
    _INIT_DCS_CMD(0x3C, 0x0A),
    _INIT_DCS_CMD(0x3D, 0x04),
    _INIT_DCS_CMD(0x3E, 0x04),
    _INIT_DCS_CMD(0x3F, 0x06),
    _INIT_DCS_CMD(0x40, 0x06),
    _INIT_DCS_CMD(0x41, 0x1F),
    _INIT_DCS_CMD(0x42, 0x02),
    _INIT_DCS_CMD(0x43, 0x02),
    _INIT_DCS_CMD(0x44, 0x00),
    _INIT_DCS_CMD(0x45, 0x00),
    _INIT_DCS_CMD(0x46, 0x1F),
    _INIT_DCS_CMD(0x47, 0x1F),
    _INIT_DCS_CMD(0x48, 0x1E),
    _INIT_DCS_CMD(0x49, 0x1E),
    _INIT_DCS_CMD(0x4A, 0x1F),
    _INIT_DCS_CMD(0x4B, 0x17),
    _INIT_DCS_CMD(0x4C, 0x17),
    _INIT_DCS_CMD(0x4D, 0x37),
    _INIT_DCS_CMD(0x4E, 0x37),
    _INIT_DCS_CMD(0x4F, 0x09),
    _INIT_DCS_CMD(0x50, 0x09),
    _INIT_DCS_CMD(0x51, 0x0B),
    _INIT_DCS_CMD(0x52, 0x0B),
    _INIT_DCS_CMD(0x53, 0x05),
    _INIT_DCS_CMD(0x54, 0x05),
    _INIT_DCS_CMD(0x55, 0x07),
    _INIT_DCS_CMD(0x56, 0x07),
    _INIT_DCS_CMD(0x57, 0x1F),
    _INIT_DCS_CMD(0x58, 0x40),
    _INIT_DCS_CMD(0x5B, 0x30),
    _INIT_DCS_CMD(0x5C, 0x00),
    _INIT_DCS_CMD(0x5D, 0x34),
    _INIT_DCS_CMD(0x5E, 0x05),
    _INIT_DCS_CMD(0x5F, 0x02),
    _INIT_DCS_CMD(0x63, 0x00),
    _INIT_DCS_CMD(0x64, 0x6A),
    _INIT_DCS_CMD(0x67, 0x73),
    _INIT_DCS_CMD(0x68, 0x07),
    _INIT_DCS_CMD(0x69, 0x08),
    _INIT_DCS_CMD(0x6A, 0x6A),
    _INIT_DCS_CMD(0x6B, 0x08),
    _INIT_DCS_CMD(0x6C, 0x00),
    _INIT_DCS_CMD(0x6D, 0x00),
    _INIT_DCS_CMD(0x6E, 0x00),
    _INIT_DCS_CMD(0x6F, 0x88),
    _INIT_DCS_CMD(0x75, 0xFF),
    _INIT_DCS_CMD(0x77, 0xDD),
    _INIT_DCS_CMD(0x78, 0x2C),
    _INIT_DCS_CMD(0x79, 0x15),
    _INIT_DCS_CMD(0x7A, 0x17),
    _INIT_DCS_CMD(0x7D, 0x14),
    _INIT_DCS_CMD(0x7E, 0x82),
    _INIT_DCS_CMD(0xE0, 0x04),
    _INIT_DCS_CMD(0x00, 0x0E),
    _INIT_DCS_CMD(0x02, 0xB3),
    _INIT_DCS_CMD(0x09, 0x61),
    _INIT_DCS_CMD(0x0E, 0x48),
    _INIT_DCS_CMD(0x37, 0x58),
    _INIT_DCS_CMD(0x2B, 0x0F),
    _INIT_DCS_CMD(0xE0, 0x00),
    _INIT_DCS_CMD(0xE6, 0x02),
    _INIT_DCS_CMD(0xE7, 0x0C),
    _INIT_DCS_CMD(0x11),
    _INIT_DELAY_CMD(120),
    _INIT_DCS_CMD(0x29),
    _INIT_DELAY_CMD(60),
    {},
};

static inline struct ws_panel *panel_to_ws(struct drm_panel *panel)
{
	return container_of(panel, struct ws_panel, panel);
}

static int ws_panel_init_dcs_cmd(struct ws_panel *ts)
{
	struct mipi_dsi_device *dsi = ts->dsi;
	struct drm_panel *panel = &ts->panel;
	int i, err = 0;

	if (ts->desc->init) {
		const struct panel_init_cmd *init_cmds = ts->desc->init;

		for (i = 0; init_cmds[i].len != 0; i++) {
			const struct panel_init_cmd *cmd = &init_cmds[i];

			switch (cmd->type) {
			case DELAY_CMD:
				msleep(cmd->data[0]);
				err = 0;
				break;

			case INIT_DCS_CMD:
				err = mipi_dsi_dcs_write(dsi, cmd->data[0],
							 cmd->len <= 1 ? NULL :
							 &cmd->data[1],
							 cmd->len - 1);
				break;

			default:
				err = -EINVAL;
			}

			if (err < 0) {
				dev_err(panel->dev,	"failed to write command %u\n", i);
				return err;
			}
		}
	}
	return 0;
}

static int ws_panel_prepare(struct drm_panel *panel)
{
	struct ws_panel *ctx = panel_to_ws(panel);
	int ret;
	dev_info(panel->dev, "waveshare dsi prepare start: \n");
	/* And reset it */
	if (ctx->reset != NULL) {
		gpiod_set_value_cansleep(ctx->reset, 0);
		msleep(60);
		gpiod_set_value_cansleep(ctx->reset, 1);
		msleep(60);
	}

	ret = ws_panel_init_dcs_cmd(ctx);
	if (ret < 0)
		dev_err(panel->dev, "waveshare dsi failed to init panel: %d\n", ret);

	dev_info(panel->dev, "waveshare dsi prepare: \n");
	return 0;
}

static int ws_panel_enable(struct drm_panel *panel)
{
	dev_info(panel->dev, "waveshare dsi enable: \n");
	return 0;
}

static int ws_panel_disable(struct drm_panel *panel)
{
	dev_info(panel->dev, "waveshare dsi ws_panel_disable: \n");
	return 0;
}

static int ws_panel_unprepare(struct drm_panel *panel)
{
	struct ws_panel *ctx = panel_to_ws(panel);

	mipi_dsi_dcs_set_display_off(ctx->dsi);
	mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	dev_info(panel->dev, "waveshare dsi ws_panel_unprepare: \n");
	if (ctx->reset != NULL)
		gpiod_set_value_cansleep(ctx->reset, 0);

	return 0;
}

static const struct drm_display_mode ws_panel_10_1_a_mode = {
	.clock       = 70000,
	.hdisplay    = 800,
	.hsync_start = 800 + 40,
	.hsync_end   = 800 + 40 + 20,
	.htotal      = 800 + 40 + 20 + 20,
	.vdisplay    = 1280,
	.vsync_start = 1280 + 30,
	.vsync_end   = 1280 + 30 + 10,
	.vtotal      = 1280 + 30 + 10 + 4,
	.width_mm	 = 135,
	.height_mm	 = 216,
};

static int ws_panel_get_modes(struct drm_panel *panel,
    struct drm_connector *connector)
{
    struct ws_panel *ctx = panel_to_ws(panel);
    struct drm_display_mode *mode;
	static const u32 bus_format = MEDIA_BUS_FMT_RGB888_1X24;
    dev_info(panel->dev, "waveshare dsi:get_mode: %s\n",connector->name);
    mode = drm_mode_duplicate(connector->dev, ctx->desc->mode);
    if (!mode) {
        dev_err(&ctx->dsi->dev, "waveshare dsi:failed to add mode %ux%ux@%u\n",
        ctx->desc->mode->hdisplay,
        ctx->desc->mode->vdisplay,
        drm_mode_vrefresh(ctx->desc->mode));
        return -ENOMEM;
    }

    drm_mode_set_name(mode);

    mode->type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED;
    drm_mode_probed_add(connector, mode);

	connector->display_info.bpc = 8;
    connector->display_info.width_mm = mode->width_mm;
    connector->display_info.height_mm = mode->height_mm;

    drm_connector_set_panel_orientation(connector, ctx->orientation);
	drm_display_info_set_bus_formats(&connector->display_info, &bus_format, 1);

    return 1;
}

static const struct drm_panel_funcs ws_panel_funcs = {
    .disable	= ws_panel_disable,
    .unprepare	= ws_panel_unprepare,
    .prepare	= ws_panel_prepare,
    .enable		= ws_panel_enable,
    .get_modes	= ws_panel_get_modes,
};
#define WAVE_DSI_DRIVER_NAME "wave-ts-dsi"

static const struct ws_panel_desc ws_panel_10_1_inch_a_desc = {
	.init = ws_panel_10_1_a_init,
	.mode = &ws_panel_10_1_a_mode,
	.mode_flags =  MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS,
	.lanes = 2,
	.format = MIPI_DSI_FMT_RGB888,
};





static int waveshare_panel_gpio_direction_in(struct gpio_chip *gc,
						unsigned int offset)
{
	struct waveshare_panel_lcd *state = gpiochip_get_data(gc);

	state->direction_state |= (1 << offset);

	return 0;
}

static int waveshare_panel_gpio_direction_out(struct gpio_chip *gc,
								unsigned int offset, int val)
{
	struct waveshare_panel_lcd *state = gpiochip_get_data(gc);
	u16 last_val;

	state->direction_state &= ~(1 << offset);

	last_val = state->poweron_state;
	if (val)
		last_val |= (1 << offset);
	else
		last_val &= ~(1 << offset);

	state->poweron_state = last_val;

	regmap_write(state->regmap, REG_TP, last_val >> 8);
	regmap_write(state->regmap, REG_LCD, last_val & 0xff);

	return 0;
}

static int waveshare_panel_gpio_get_direction(struct gpio_chip *gc,
						unsigned int offset)
{
	struct waveshare_panel_lcd *state = gpiochip_get_data(gc);

	if (state->direction_state & (1 << offset))
		return GPIO_LINE_DIRECTION_IN;
	else
		return GPIO_LINE_DIRECTION_OUT;
}

static int waveshare_panel_gpio_get(struct gpio_chip *gc,
						unsigned int offset)
{
	struct waveshare_panel_lcd *state = gpiochip_get_data(gc);

	if (state->poweron_state & (1 << offset))
		return 1;
	else
		return 0;
}

static void waveshare_panel_gpio_set(struct gpio_chip *gc,
						unsigned int offset, int value)
{
	struct waveshare_panel_lcd *state = gpiochip_get_data(gc);
	u16 last_val;

	if (offset >= NUM_GPIO)
		return;

	mutex_lock(&state->lock);

	last_val = state->poweron_state;
	if (value)
		last_val |= (1 << offset);
	else
		last_val &= ~(1 << offset);

	state->poweron_state = last_val;

	regmap_write(state->regmap, REG_TP, last_val >> 8);
	regmap_write(state->regmap, REG_LCD, last_val & 0xff);

	mutex_unlock(&state->lock);
}

static int waveshare_panel_update_status(struct backlight_device *bl)
{
	struct waveshare_panel_lcd *state = bl_get_data(bl);
	int brightness = bl->props.brightness;
	u16 last_val;

	if (bl->props.power != FB_BLANK_UNBLANK ||
		bl->props.state & (BL_CORE_SUSPENDED | BL_CORE_FBBLANK))
		brightness = 0;

	mutex_lock(&state->lock);

	last_val = state->poweron_state;
	if (brightness)
		last_val |= (1 << 2); // Enable BL_EN
	else
		last_val &= ~(1 << 2); // Disable BL_EN

	state->poweron_state = last_val;

	regmap_write(state->regmap, REG_TP, last_val >> 8);
	regmap_write(state->regmap, REG_LCD, last_val & 0xff);

	mutex_unlock(&state->lock);

	return regmap_write(state->regmap, REG_PWM, brightness);
}

static const struct backlight_ops waveshare_panel_bl = {
	.update_status	= waveshare_panel_update_status,
};

static int waveshare_panel_i2c_read(struct i2c_client *client, u8 reg, unsigned int *buf)
{
	struct i2c_msg msgs[1];
	u8 addr_buf[1] = { reg };
	u8 data_buf[1] = { 0, };
	int ret;

	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = ARRAY_SIZE(addr_buf);
	msgs[0].buf = addr_buf;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	usleep_range(5000, 10000);

	/* Read data from register */
	msgs[0].addr = client->addr;
	msgs[0].flags = I2C_M_RD;
	msgs[0].len = 1;
	msgs[0].buf = data_buf;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*buf = data_buf[0];
	return 0;
}

/*
 * I2C driver interface functions
 */
static int waveshare_panel_i2c_probe(struct i2c_client *i2c, const struct i2c_device_id *dev_id)
{
	struct backlight_properties props = { };
	struct backlight_device *bl;
	struct waveshare_panel_lcd *state;
	struct regmap *regmap;
	unsigned int data;
	int ret;
    struct device *dev = &i2c->dev;
    struct device_node *endpoint, *dsi_host_node;
    struct mipi_dsi_host *host;
    struct mipi_dsi_device_info info = {
        .type = WAVE_DSI_DRIVER_NAME,
        .channel = 0,
        .node = NULL,
    };

	state = devm_kzalloc(&i2c->dev, sizeof(*state), GFP_KERNEL);
	if (!state)
		return -ENOMEM;
	
	dev_info(&i2c->dev, "dsi panel: %s\n", (char *)of_get_property(i2c->dev.of_node, "compatible", NULL));

	mutex_init(&state->lock);
	i2c_set_clientdata(i2c, state);

	regmap = devm_regmap_init_i2c(i2c, &waveshare_panel_regmap_config);
	if (IS_ERR(regmap)) {
		ret = PTR_ERR(regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		goto error;
	}

	ret = waveshare_panel_i2c_read(i2c, REG_ID, &data);
	if (ret == 0)
		dev_info(&i2c->dev, "dsi:waveshare panel hw id = 0x%x\n", data);

	ret = waveshare_panel_i2c_read(i2c, REG_SIZE, &data);
	if (ret == 0)
		dev_info(&i2c->dev, "dsi:waveshare panel size = %d\n", data);

	ret = waveshare_panel_i2c_read(i2c, REG_VERSION, &data);
	if (ret == 0)
		dev_info(&i2c->dev, "dsi:waveshare panel mcu version = 0x%x\n", data);

	state->direction_state = 0;
	state->poweron_state = (1 << 9) | (1 << 8) | (1 << 4) | (1 << 0); // Enable VCC
	regmap_write(regmap, REG_TP, state->poweron_state >> 8);
	regmap_write(regmap, REG_LCD, state->poweron_state & 0xff);
	msleep(20);

	state->regmap = regmap;
	state->gc.parent = &i2c->dev;
	state->gc.label = i2c->name;
	state->gc.owner = THIS_MODULE;
	state->gc.base = -1;
	state->gc.ngpio = NUM_GPIO;

	state->gc.get = waveshare_panel_gpio_get;
	state->gc.set = waveshare_panel_gpio_set;
	state->gc.direction_input = waveshare_panel_gpio_direction_in;
	state->gc.direction_output = waveshare_panel_gpio_direction_out;
	state->gc.get_direction = waveshare_panel_gpio_get_direction;
	state->gc.can_sleep = true;

	ret = devm_gpiochip_add_data(&i2c->dev, &state->gc, state);
	if (ret) {
		dev_err(&i2c->dev, "Failed to create gpiochip: %d\n", ret);
		goto error;
	}

	props.type = BACKLIGHT_RAW;
	props.max_brightness = 255;
	bl = devm_backlight_device_register(&i2c->dev, dev_name(&i2c->dev),
						&i2c->dev, state, &waveshare_panel_bl,
						&props);
	if (IS_ERR(bl))
		return PTR_ERR(bl);

	bl->props.brightness = 255;
// End of i2c part start DSI

	//
    //
	state->ctx.desc = &ws_panel_10_1_inch_a_desc;
    state->ctx.orientation = DRM_MODE_PANEL_ORIENTATION_NORMAL;//DRM_MODE_PANEL_ORIENTATION_NORMAL;
//DRM_MODE_PANEL_ORIENTATION_LEFT_UP
	state->ctx.reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(state->ctx.reset))
		return dev_err_probe(&i2c->dev, PTR_ERR(state->ctx.reset),
					"Couldn't get our reset GPIO\n");

	//ret = drm_panel_of_backlight(&state->ctx.panel);
	//if (ret)
	//	return ret;
		
    //dsi->lanes = state->ctx.desc->lanes;
	 /* Look up the DSI host.  It needs to probe before we do. */
     dev_info(&i2c->dev, "dsi:start0\n");
	 endpoint = of_graph_get_next_endpoint(dev->of_node, NULL);
	 if (!endpoint)
		 return -ENODEV;
 
	 dsi_host_node = of_graph_get_remote_port_parent(endpoint);
	 if (!dsi_host_node)
		 goto error;
    
	 host = of_find_mipi_dsi_host_by_node(dsi_host_node);
	 of_node_put(dsi_host_node);
	 if (!host) {
		 dev_info(&i2c->dev, "dsi:eba\n");
		 of_node_put(endpoint);
		 return -EPROBE_DEFER;
	 }
 
	 info.node = of_graph_get_remote_port(endpoint);
	 if (!info.node)
		 goto error;
 
	 of_node_put(endpoint);
     dev_info(&i2c->dev, "dsi:start1\n");

	 state->ctx.dsi = mipi_dsi_device_register_full(host, &info);
	 
	 //mipi_dsi_set_drvdata(state->ctx.dsi, state->ctx);
	 if (IS_ERR(state->ctx.dsi)) {
		 dev_err(dev, "DSI device registration failed: %ld\n",
			 PTR_ERR(state->ctx.dsi));
		 return PTR_ERR(state->ctx.dsi);
	 }
	 state->ctx.dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |  MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET;
	 //state->ctx.dsi->mode_flags = MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	
	//state->ctx.dsi->mode_flags = (MIPI_DSI_MODE_VIDEO |
	//	MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
	//	MIPI_DSI_MODE_LPM);
	 state->ctx.dsi->format = MIPI_DSI_FMT_RGB888;
	 state->ctx.dsi->lanes = 2;

     dev_info(&i2c->dev, "dsi:start2\n");
	 drm_panel_init(&state->ctx.panel, dev, &ws_panel_funcs,
				DRM_MODE_CONNECTOR_DSI);
 
	 /* This appears last, as it's what will unblock the DSI host
	  * driver's component bind function.
	  */
     dev_info(&i2c->dev, "dsi:start3\n");
	 drm_panel_add(&state->ctx.panel);
     dev_info(&i2c->dev, "dsi:start4\n");
	return 0;

error:
	dev_info(&i2c->dev, "dsi:error\n");
	mutex_destroy(&state->lock);
	return ret;
}

static int waveshare_panel_i2c_remove(struct i2c_client *client)
{
	struct waveshare_panel_lcd *state = i2c_get_clientdata(client);

    mipi_dsi_detach(state->ctx.dsi);

    drm_panel_remove(&state->ctx.panel);

    mipi_dsi_device_unregister(state->ctx.dsi);

	mutex_destroy(&state->lock);
	return 0;
}


static int rpi_touchscreen_dsi_probe(struct mipi_dsi_device *dsi)
{
    int ret;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_BURST |  MIPI_DSI_MODE_LPM | MIPI_DSI_MODE_EOT_PACKET;
    //dsi->mode_flags = MIPI_DSI_MODE_VIDEO_HSE | MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_LPM | MIPI_DSI_CLOCK_NON_CONTINUOUS;
	//dsi->mode_flags = (MIPI_DSI_MODE_VIDEO |
	//			MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
	//			MIPI_DSI_MODE_LPM);
	dsi->format = MIPI_DSI_FMT_RGB888;
    dsi->lanes = 2;
	dev_info(&dsi->dev, "dsi:init\n");
    ret = mipi_dsi_attach(dsi);
   
    if (ret) //EPROBE_DEFER
        dev_err(&dsi->dev, "dsi: failed to attach dsi to host: %d\n", ret);

		dev_info(&dsi->dev, "dsi:success\n");
    return ret;
}

static struct mipi_dsi_driver waveshare_touchscreen_dsi_driver = {
    .driver.name = WAVE_DSI_DRIVER_NAME,
    .probe = rpi_touchscreen_dsi_probe,
};

static const struct of_device_id rpi_touchscreen_of_ids[] = {
    { .compatible = "waveshare,my_driver" },
    { } /* sentinel */
};
MODULE_DEVICE_TABLE(of, rpi_touchscreen_of_ids);

static struct i2c_driver rpi_touchscreen_driver = {
    .driver = {
        .name = "wave_my_dr",
        .of_match_table = rpi_touchscreen_of_ids,
    },
    .probe = waveshare_panel_i2c_probe,
    .remove = waveshare_panel_i2c_remove,
};

static int __init waveshare_touchscreen_init(void)
{
    mipi_dsi_driver_register(&waveshare_touchscreen_dsi_driver);
    return i2c_add_driver(&rpi_touchscreen_driver);
}
module_init(waveshare_touchscreen_init);

static void __exit rpi_touchscreen_exit(void)
{
    i2c_del_driver(&rpi_touchscreen_driver);
    mipi_dsi_driver_unregister(&waveshare_touchscreen_dsi_driver);
}
module_exit(rpi_touchscreen_exit);

MODULE_AUTHOR("Test");
MODULE_DESCRIPTION("Waveshare DSI panel driver");
MODULE_LICENSE("GPL");
