/*
 * Copyright (c) 2020 Trunexa Inc. 
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pinctrl/consumer.h>
#include <linux/regulator/consumer.h>
#include <linux/v4l2-mediabus.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ctrls.h>

#define OV7725_VOLTAGE_ANALOG               2800000
#define OV7725_VOLTAGE_DIGITAL_CORE         1500000
#define OV7725_VOLTAGE_DIGITAL_IO           1800000

#define MIN_FPS 15
#define MAX_FPS 30
#define DEFAULT_FPS 30

#define OV7725_XCLK_MIN 6000000
#define OV7725_XCLK_MAX 24000000

#define OV7725_CHIP_ID_HIGH_BYTE        0x0A
#define OV7725_CHIP_ID_LOW_BYTE         0x0B

enum ov7725_mode {
	ov7725_mode_MIN = 0,
	ov7725_mode_VGA_480_272 = 0,
	ov7725_mode_VGA_640_480 = 1,
	ov7725_mode_VGA_320_240 = 2,
	ov7725_mode_MAX = 2
};

enum ov7725_frame_rate {
	ov7725_30_fps
};

struct ov7725_datafmt {
	u32	code;
	enum v4l2_colorspace		colorspace;
};

struct reg_value {
	u8 u8RegAddr;
	u8 u8Val;
	u8 u8Mask;
	u8 u8Delay_ms;
};

struct ov7725_mode_info {
	enum ov7725_mode mode;
	u32 width;
	u32 height;
	struct reg_value *init_data_ptr;
	u32 init_data_size;
};

struct ov7725 {
	struct v4l2_subdev		subdev;
	struct i2c_client *i2c_client;
	struct v4l2_pix_format pix;
	const struct ov7725_datafmt	*fmt;
	struct v4l2_captureparm streamcap;
	bool on;

	/* control settings */
	int brightness;
	int hue;
	int contrast;
	int saturation;
	int red;
	int green;
	int blue;
	int ae_mode;

	u32 mclk;
	u8 mclk_source;
	struct clk *sensor_clk;
	int csi;

	void (*io_init)(void);
};

/*!
 * Maintains the information on the current state of the sesor.
 */
static struct ov7725 ov7725_data;
static int pwn_gpio, rst_gpio;

static struct reg_value ov7725_setting_30fps_VGA_640_480[] = {
	{0x12, 0x80, 0, 30}, {0x3d, 0x03, 0, 0}, {0x17, 0x26, 0, 0},
	{0x18, 0xa4, 0, 0},  {0x19, 0x07, 0, 0}, {0x1a, 0xf0, 0, 0},
	{0x32, 0x00, 0, 0},  {0x29, 0xa0, 0, 0}, {0x2c, 0xf0, 0, 0},
	{0x2a, 0x00, 0, 0},  {0x11, 0x03, 0, 0}, {0x42, 0x7f, 0, 0},
	{0x4d, 0x09, 0, 0},  {0x63, 0xe0, 0, 0}, {0x64, 0xff, 0, 0},
	{0x65, 0x20, 0, 0},  {0x66, 0x00, 0, 0}, {0x67, 0x48, 0, 0},
	{0x13, 0xf0, 0, 0},  {0x0d, 0x41, 0, 0}, {0x0f, 0xc5, 0, 0},
	{0x14, 0x11, 0, 0},  {0x22, 0x3f, 0, 0}, {0x23, 0x07, 0, 0},
	{0x24, 0x40, 0, 0},  {0x25, 0x30, 0, 0}, {0x26, 0xa1, 0, 0},
	{0x2b, 0x00, 0, 0},  {0x6b, 0xaa, 0, 0}, {0x13, 0xff, 0, 0},
	{0x90, 0x05, 0, 0},  {0x91, 0x01, 0, 0}, {0x92, 0x03, 0, 0},
	{0x93, 0x00, 0, 0},  {0x94, 0xb0, 0, 0}, {0x95, 0x9d, 0, 0},
	{0x96, 0x13, 0, 0},  {0x97, 0x16, 0, 0}, {0x98, 0x7b, 0, 0},
	{0x99, 0x91, 0, 0},  {0x9a, 0x1e, 0, 0}, {0x9b, 0x08, 0, 0},
	{0x9c, 0x20, 0, 0},  {0x9e, 0x81, 0, 0}, {0xa6, 0x04, 0, 0},
	{0x7e, 0x0c, 0, 0},  {0x7f, 0x16, 0, 0}, {0x80, 0x2a, 0, 0},
	{0x81, 0x4e, 0, 0},  {0x82, 0x61, 0, 0}, {0x83, 0x6f, 0, 0},
	{0x84, 0x7b, 0, 0},  {0x85, 0x86, 0, 0}, {0x86, 0x8e, 0, 0},
	{0x87, 0x97, 0, 0},  {0x88, 0xa4, 0, 0}, {0x89, 0xaf, 0, 0},
	{0x8a, 0xc5, 0, 0},  {0x8b, 0xd7, 0, 0}, {0x8c, 0xe8, 0, 0},
	{0x8d, 0x20, 0, 0},  {0x11, 0x01, 0, 0}, {0x22, 0x7f, 0, 0}, /*0x99*/
	{0x23, 0x03, 0, 0},
};

static struct reg_value ov7725_setting_30fps_VGA_480_272[] = {
	{0x12, 0x80, 0, 30}, {0x3d, 0x03, 0, 0}, {0x17, 0x3f, 0, 0},
        {0x18, 0x7c, 0, 0},  {0x19, 0x03, 0, 0}, {0x1a, 0x88, 0, 0},
        {0x32, 0x00, 0, 0},  {0x29, 0x78, 0, 0}, {0x2c, 0x88, 0, 0},
        {0x2a, 0x00, 0, 0},  {0x11, 0x03, 0, 0}, {0x42, 0x7f, 0, 0},
        {0x4d, 0x09, 0, 0},  {0x63, 0xe0, 0, 0}, {0x64, 0xff, 0, 0},
        {0x65, 0x20, 0, 0},  {0x66, 0x00, 0, 0}, {0x67, 0x48, 0, 0},
        {0x13, 0xf0, 0, 0},  {0x0d, 0x41, 0, 0}, {0x0f, 0xc5, 0, 0},
        {0x14, 0x11, 0, 0},  {0x22, 0x3f, 0, 0}, {0x23, 0x07, 0, 0},
        {0x24, 0x40, 0, 0},  {0x25, 0x30, 0, 0}, {0x26, 0xa1, 0, 0},
        {0x2b, 0x00, 0, 0},  {0x6b, 0xaa, 0, 0}, {0x13, 0xff, 0, 0},
        {0x90, 0x05, 0, 0},  {0x91, 0x01, 0, 0}, {0x92, 0x03, 0, 0},
        {0x93, 0x00, 0, 0},  {0x94, 0xb0, 0, 0}, {0x95, 0x9d, 0, 0},
        {0x96, 0x13, 0, 0},  {0x97, 0x16, 0, 0}, {0x98, 0x7b, 0, 0},
        {0x99, 0x91, 0, 0},  {0x9a, 0x1e, 0, 0}, {0x9b, 0x08, 0, 0},
        {0x9c, 0x20, 0, 0},  {0x9e, 0x81, 0, 0}, {0xa6, 0x04, 0, 0},
        {0x7e, 0x0c, 0, 0},  {0x7f, 0x16, 0, 0}, {0x80, 0x2a, 0, 0},
        {0x81, 0x4e, 0, 0},  {0x82, 0x61, 0, 0}, {0x83, 0x6f, 0, 0},
        {0x84, 0x7b, 0, 0},  {0x85, 0x86, 0, 0}, {0x86, 0x8e, 0, 0},
        {0x87, 0x97, 0, 0},  {0x88, 0xa4, 0, 0}, {0x89, 0xaf, 0, 0},
        {0x8a, 0xc5, 0, 0},  {0x8b, 0xd7, 0, 0}, {0x8c, 0xe8, 0, 0},
        {0x8d, 0x20, 0, 0},  {0x11, 0x01, 0, 0}, {0x22, 0x7f, 0, 0}, /*0x99*/
        {0x23, 0x03, 0, 0},  
};

static struct reg_value ov7725_setting_30fps_VGA_320_240[] = {
        {0x12, 0x80, 0, 30}, {0x3d, 0x03, 0, 0}, {0x17, 0x3f, 0, 0},
        {0x18, 0x54, 0, 0},  {0x19, 0x03, 0, 0}, {0x1a, 0x78, 0, 0},
        {0x32, 0x00, 0, 0},  {0x29, 0x50, 0, 0}, {0x2c, 0x78, 0, 0},
        {0x2a, 0x00, 0, 0},  {0x11, 0x03, 0, 0}, {0x42, 0x7f, 0, 0},
        {0x4d, 0x09, 0, 0},  {0x63, 0xe0, 0, 0}, {0x64, 0xff, 0, 0},
        {0x65, 0x20, 0, 0},  {0x66, 0x00, 0, 0}, {0x67, 0x48, 0, 0},
        {0x13, 0xf0, 0, 0},  {0x0d, 0x41, 0, 0}, {0x0f, 0xc5, 0, 0},
        {0x14, 0x11, 0, 0},  {0x22, 0x3f, 0, 0}, {0x23, 0x07, 0, 0},
        {0x24, 0x40, 0, 0},  {0x25, 0x30, 0, 0}, {0x26, 0xa1, 0, 0},
        {0x2b, 0x00, 0, 0},  {0x6b, 0xaa, 0, 0}, {0x13, 0xff, 0, 0},
        {0x90, 0x05, 0, 0},  {0x91, 0x01, 0, 0}, {0x92, 0x03, 0, 0},
        {0x93, 0x00, 0, 0},  {0x94, 0xb0, 0, 0}, {0x95, 0x9d, 0, 0},
        {0x96, 0x13, 0, 0},  {0x97, 0x16, 0, 0}, {0x98, 0x7b, 0, 0},
        {0x99, 0x91, 0, 0},  {0x9a, 0x1e, 0, 0}, {0x9b, 0x08, 0, 0},
        {0x9c, 0x20, 0, 0},  {0x9e, 0x81, 0, 0}, {0xa6, 0x04, 0, 0},
        {0x7e, 0x0c, 0, 0},  {0x7f, 0x16, 0, 0}, {0x80, 0x2a, 0, 0},
        {0x81, 0x4e, 0, 0},  {0x82, 0x61, 0, 0}, {0x83, 0x6f, 0, 0},
        {0x84, 0x7b, 0, 0},  {0x85, 0x86, 0, 0}, {0x86, 0x8e, 0, 0},
        {0x87, 0x97, 0, 0},  {0x88, 0xa4, 0, 0}, {0x89, 0xaf, 0, 0},
        {0x8a, 0xc5, 0, 0},  {0x8b, 0xd7, 0, 0}, {0x8c, 0xe8, 0, 0},
        {0x8d, 0x20, 0, 0},  {0x11, 0x01, 0, 0}, {0x22, 0x7f, 0, 0}, /*0x99*/
        {0x23, 0x03, 0, 0},
};

static struct ov7725_mode_info ov7725_mode_info_data[1][ov7725_mode_MAX + 1] = {
	{
		{ov7725_mode_VGA_480_272,     480,  272,
		ov7725_setting_30fps_VGA_480_272,
		ARRAY_SIZE(ov7725_setting_30fps_VGA_480_272)},
		{ov7725_mode_VGA_640_480,     640,  480,
		ov7725_setting_30fps_VGA_640_480,
		ARRAY_SIZE(ov7725_setting_30fps_VGA_640_480)},
		{ov7725_mode_VGA_320_240,     320,  240,
		ov7725_setting_30fps_VGA_320_240,
		ARRAY_SIZE(ov7725_setting_30fps_VGA_320_240)},
	},
};

static int ov7725_probe(struct i2c_client *adapter,
				const struct i2c_device_id *device_id);
static int ov7725_remove(struct i2c_client *client);

static s32 ov7725_read_reg(u8 reg, u8 *val);
static s32 ov7725_write_reg(u8 reg, u8 val);

static const struct i2c_device_id ov7725_id[] = {
	{"ov7725", 0},
	{},
};

MODULE_DEVICE_TABLE(i2c, ov7725_id);

static struct i2c_driver ov7725_i2c_driver = {
	.driver = {
		  .owner = THIS_MODULE,
		  .name  = "ov7725",
		  },
	.probe  = ov7725_probe,
	.remove = ov7725_remove,
	.id_table = ov7725_id,
};

static const struct ov7725_datafmt ov7725_colour_fmts[] = {
	{MEDIA_BUS_FMT_YUYV8_2X8, V4L2_COLORSPACE_JPEG},
};

static struct ov7725 *to_ov7725(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ov7725, subdev);
}

/* Find a data format by a pixel code in an array */
static const struct ov7725_datafmt
			*ov7725_find_datafmt(u32 code)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ov7725_colour_fmts); i++)
		if (ov7725_colour_fmts[i].code == code)
			return ov7725_colour_fmts + i;

	return NULL;
}

static inline void ov7725_power_down(int enable)
{
	if (gpio_is_valid(pwn_gpio)) {
		gpio_set_value_cansleep(pwn_gpio, enable);

		msleep(2);
	}
}

static inline void ov7725_reset(void)
{
	if (gpio_is_valid(rst_gpio) && 	gpio_is_valid(pwn_gpio)) 
	{
		/* camera reset */
		gpio_set_value_cansleep(rst_gpio, 1);

		/* camera power down */
		gpio_set_value_cansleep(pwn_gpio, 1);
		msleep(5);
		gpio_set_value_cansleep(pwn_gpio, 0);
		msleep(5);
		gpio_set_value_cansleep(rst_gpio, 0);
		msleep(1);
		gpio_set_value_cansleep(rst_gpio, 1);
		msleep(5);
		gpio_set_value_cansleep(pwn_gpio, 1);
	}	
}

static s32 ov7725_write_reg(u8 reg, u8 val)
{
	u8 au8Buf[2] = {0};

	au8Buf[0] = reg & 0xff;
	au8Buf[1] = val;

	if (i2c_master_send(ov7725_data.i2c_client, au8Buf, 2) < 0) {
		pr_err("%s:write reg error:reg=%x,val=%x\n",
			__func__, reg, val);
		return -1;
	}

	return 0;
}

static s32 ov7725_read_reg(u8 reg, u8 *val)
{
	u8 au8RegBuf[1] = {0};
	u8 u8RdVal = 0;

	au8RegBuf[0] = reg & 0xff;

	if (1 != i2c_master_send(ov7725_data.i2c_client, au8RegBuf, 1)) {
		pr_err("%s:write reg error:reg=%x\n",
				__func__, reg);
		return -1;
	}

	if (1 != i2c_master_recv(ov7725_data.i2c_client, &u8RdVal, 1)) {
		pr_err("%s:read reg error:reg=%x,val=%x\n",
				__func__, reg, u8RdVal);
		return -1;
	}

	*val = u8RdVal;

	return u8RdVal;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ov7725_get_register(struct v4l2_subdev *sd,
					struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 val;

	if (reg->reg & ~0xffff)
		return -EINVAL;

	reg->size = 1;

	ret = ov7725_read_reg(reg->reg, &val);
	if (!ret)
		reg->val = (__u64)val;

	return ret;
}

static int ov7725_set_register(struct v4l2_subdev *sd,
					const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (reg->reg & ~0xffff || reg->val & ~0xff)
		return -EINVAL;

	return ov7725_write_reg(reg->reg, reg->val);
}
#endif

static int ov7725_init_mode(enum ov7725_frame_rate frame_rate,
					enum ov7725_mode mode)
{
        struct reg_value *pModeSetting = NULL;
        s32 iModeSettingArySize = 0;
        s32 i = 0;
        register u32 Delay_ms = 0;
        register u8 RegAddr, Mask, Val = 0;
        u8 RegVal = 0;
        int retval = 0;

        if (mode > ov7725_mode_MAX || mode < ov7725_mode_MIN) {
                pr_err("Unsupported ov7725 mode detected!\n");
                return -1;
        }

        pModeSetting = ov7725_mode_info_data[frame_rate][mode].init_data_ptr;
        iModeSettingArySize =
                ov7725_mode_info_data[frame_rate][mode].init_data_size;

        ov7725_data.pix.width = ov7725_mode_info_data[frame_rate][mode].width;
        ov7725_data.pix.height = ov7725_mode_info_data[frame_rate][mode].height;

        if (ov7725_data.pix.width == 0 || ov7725_data.pix.height == 0 ||
            pModeSetting == NULL || iModeSettingArySize == 0)
                return -EINVAL;

        for (i = 0; i < iModeSettingArySize; ++i, ++pModeSetting) {
                Delay_ms = pModeSetting->u8Delay_ms;
                RegAddr = pModeSetting->u8RegAddr;
                Val = pModeSetting->u8Val;
                Mask = pModeSetting->u8Mask;

                if (Mask) {
                        retval = ov7725_read_reg(RegAddr, &RegVal);
                        if (retval < 0)
                                goto err;

                        RegVal &= ~(u8)Mask;
                        Val &= Mask;
                        Val |= RegVal;
                }

                retval = ov7725_write_reg(RegAddr, Val);
                if (retval < 0)
                        goto err;

                if (Delay_ms)
                        msleep(Delay_ms);
        }
err:
        return retval;

}

/*!
 * ov7725_s_power - V4L2 sensor interface handler for VIDIOC_S_POWER ioctl
 * @s: pointer to standard V4L2 device structure
 * @on: indicates power mode (on or off)
 *
 * Turns the power on or off, depending on the value of on and returns the
 * appropriate error code.
 */
static int ov7725_s_power(struct v4l2_subdev *sd, int on)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7725 *sensor = to_ov7725(client);

	if (on)
		clk_enable(ov7725_data.sensor_clk);
	else
		clk_disable(ov7725_data.sensor_clk);

	sensor->on = on;

	return 0;
}

/*!
 * ov7725_g_parm - V4L2 sensor interface handler for VIDIOC_G_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_G_PARM ioctl structure
 *
 * Returns the sensor's video CAPTURE parameters.
 */
static int ov7725_g_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7725 *sensor = to_ov7725(client);
	struct v4l2_captureparm *cparm = &a->parm.capture;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		memset(a, 0, sizeof(*a));
		a->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
		cparm->capability = sensor->streamcap.capability;
		cparm->timeperframe = sensor->streamcap.timeperframe;
		cparm->capturemode = sensor->streamcap.capturemode;
		ret = 0;
		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

	return ret;
}

/*!
 * ov5460_s_parm - V4L2 sensor interface handler for VIDIOC_S_PARM ioctl
 * @s: pointer to standard V4L2 sub device structure
 * @a: pointer to standard V4L2 VIDIOC_S_PARM ioctl structure
 *
 * Configures the sensor to use the input parameters, if possible.  If
 * not possible, reverts to the old parameters and returns the
 * appropriate error code.
 */
static int ov7725_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *a)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7725 *sensor = to_ov7725(client);
	struct v4l2_fract *timeperframe = &a->parm.capture.timeperframe;
	u32 tgt_fps;	/* target frames per secound */
	enum ov7725_frame_rate frame_rate;
	int ret = 0;

	switch (a->type) {
	/* This is the only case currently handled. */
	case V4L2_BUF_TYPE_VIDEO_CAPTURE:
		/* Check that the new frame rate is allowed. */
		if ((timeperframe->numerator == 0) ||
		    (timeperframe->denominator == 0)) {
			timeperframe->denominator = DEFAULT_FPS;
			timeperframe->numerator = 1;
		}

		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps != DEFAULT_FPS) {
			timeperframe->denominator = MAX_FPS;
			timeperframe->numerator = 1;
		} 

		/* Actual frame rate we use */
		tgt_fps = timeperframe->denominator /
			  timeperframe->numerator;

		if (tgt_fps == 30)
			frame_rate = ov7725_30_fps;
		else {
			pr_err(" The camera driver supports only 30 fps!\n");
			goto error;
		}

		ret = ov7725_init_mode(frame_rate,
				a->parm.capture.capturemode);
		if (ret < 0)
			goto error;

		sensor->streamcap.timeperframe = *timeperframe;
		sensor->streamcap.capturemode = a->parm.capture.capturemode;

		break;

	/* These are all the possible cases. */
	case V4L2_BUF_TYPE_VIDEO_OUTPUT:
	case V4L2_BUF_TYPE_VIDEO_OVERLAY:
	case V4L2_BUF_TYPE_VBI_CAPTURE:
	case V4L2_BUF_TYPE_VBI_OUTPUT:
	case V4L2_BUF_TYPE_SLICED_VBI_CAPTURE:
	case V4L2_BUF_TYPE_SLICED_VBI_OUTPUT:
		pr_debug("   type is not " \
			"V4L2_BUF_TYPE_VIDEO_CAPTURE but %d\n",
			a->type);
		ret = -EINVAL;
		break;

	default:
		pr_debug("   type is unknown - %d\n", a->type);
		ret = -EINVAL;
		break;
	}

error:
	return ret;
}

static int ov7725_try_fmt(struct v4l2_subdev *sd,
			  struct v4l2_mbus_framefmt *mf)
{
	const struct ov7725_datafmt *fmt = ov7725_find_datafmt(mf->code);

	if (!fmt) {
		mf->code	= ov7725_colour_fmts[0].code;
		mf->colorspace	= ov7725_colour_fmts[0].colorspace;
	}

	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ov7725_s_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7725 *sensor = to_ov7725(client);

	/* MIPI CSI could have changed the format, double-check */
	if (!ov7725_find_datafmt(mf->code)){
		printk (" Failed to find fmt %s\n", __func__);
		return -EINVAL;
	}

	ov7725_try_fmt(sd, mf);
	sensor->fmt = ov7725_find_datafmt(mf->code);

	return 0;
}

static int ov7725_g_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *mf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov7725 *sensor = to_ov7725(client);

	const struct ov7725_datafmt *fmt = sensor->fmt;

	mf->code	= fmt->code;
	mf->colorspace	= fmt->colorspace;
	mf->field	= V4L2_FIELD_NONE;

	return 0;
}

static int ov7725_enum_fmt(struct v4l2_subdev *sd, unsigned int index,
			   u32 *code)
{
	if (index >= ARRAY_SIZE(ov7725_colour_fmts))
		return -EINVAL;

	*code = ov7725_colour_fmts[index].code;
	return 0;
}

/*!
 * ov7725_enum_framesizes - V4L2 sensor interface handler for
 *			   VIDIOC_ENUM_FRAMESIZES ioctl
 * @s: pointer to standard V4L2 device structure
 * @fsize: standard V4L2 VIDIOC_ENUM_FRAMESIZES ioctl structure
 *
 * Return 0 if successful, otherwise -EINVAL.
 */
static int ov7725_enum_framesizes(struct v4l2_subdev *sd,
		struct v4l2_subdev_pad_config *cfg,
		struct v4l2_subdev_frame_size_enum *fse)
{

	if (fse->index > ov7725_mode_MAX)
		return -EINVAL;

	fse->max_width = ov7725_mode_info_data[0][fse->index].width; 
	fse->min_width = fse->max_width;
	fse->max_height = ov7725_mode_info_data[0][fse->index].height;
	fse->min_height = fse->max_height;
	return 0;               
}

static int ov7725_set_clk_rate(void)
{
	u32 tgt_xclk;	/* target xclk */
	int ret;

	/* mclk */
	tgt_xclk = ov7725_data.mclk;
	tgt_xclk = min(tgt_xclk, (u32)OV7725_XCLK_MAX);
	tgt_xclk = max(tgt_xclk, (u32)OV7725_XCLK_MIN);
	ov7725_data.mclk = tgt_xclk;

	pr_debug("   Setting mclk to %d MHz\n", tgt_xclk / 1000000);
	ret = clk_set_rate(ov7725_data.sensor_clk, ov7725_data.mclk);
	if (ret < 0)
		pr_debug("set rate filed, rate=%d\n", ov7725_data.mclk);
	return ret;
}

/*!
 * dev_init - V4L2 sensor init
 * @s: pointer to standard V4L2 device structure
 *
 */
static int init_device(void)
{
	u32 tgt_xclk;	/* target xclk */
	u32 tgt_fps;	/* target frames per secound */
	enum ov7725_frame_rate frame_rate;
	int ret;

	ov7725_data.on = true;

	/* mclk */
	tgt_xclk = ov7725_data.mclk;

	/* Default camera frame rate is set in probe */
	tgt_fps = ov7725_data.streamcap.timeperframe.denominator /
		  ov7725_data.streamcap.timeperframe.numerator;

	if (tgt_fps == 30)
		frame_rate = ov7725_30_fps;
	else
		return -EINVAL; /* Only support 15fps or 30fps now. */

	ret = ov7725_init_mode(frame_rate,
                                ov7725_data.streamcap.capturemode);

	return ret;
}

static struct v4l2_subdev_video_ops ov7725_subdev_video_ops = {
	.g_parm = ov7725_g_parm,
	.s_parm = ov7725_s_parm,
	.s_mbus_fmt	= ov7725_s_fmt,
	.g_mbus_fmt	= ov7725_g_fmt,
	.try_mbus_fmt	= ov7725_try_fmt,
	.enum_mbus_fmt	= ov7725_enum_fmt,
};

static const struct v4l2_subdev_pad_ops ov7725_subdev_pad_ops = {
        .enum_frame_size       = ov7725_enum_framesizes,
};

static struct v4l2_subdev_core_ops ov7725_subdev_core_ops = {
	.s_power	= ov7725_s_power,
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register	= ov7725_get_register,
	.s_register	= ov7725_set_register,
#endif
};

static struct v4l2_subdev_ops ov7725_subdev_ops = {
	.core	= &ov7725_subdev_core_ops,
	.video	= &ov7725_subdev_video_ops,
	.pad    = &ov7725_subdev_pad_ops,
};

/*!
 * ov7725 I2C probe function
 *
 * @param adapter            struct i2c_adapter *
 * @return  Error code indicating success or failure
 */
static int ov7725_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct pinctrl *pinctrl;
	struct device *dev = &client->dev;
	int retval;
	u8 chip_id_high, chip_id_low;

	/* ov7725 pinctrl */
	pinctrl = devm_pinctrl_get_select_default(dev);
	if (IS_ERR(pinctrl)) {
		dev_err(dev, "setup pinctrl failed\n");
		return PTR_ERR(pinctrl);
	}


	/* request power down pin */
	pwn_gpio = of_get_named_gpio(dev->of_node, "pwn-gpios", 0);
	if (gpio_is_valid(pwn_gpio)) {
		retval = devm_gpio_request_one(dev, pwn_gpio, GPIOF_OUT_INIT_HIGH,
					"ov7725_pwdn");
		if (retval < 0)
			return retval;
	}


	/* request reset pin */
	rst_gpio = of_get_named_gpio(dev->of_node, "rst-gpios", 0);
	if (gpio_is_valid(rst_gpio)) {
		retval = devm_gpio_request_one(dev, rst_gpio, GPIOF_OUT_INIT_HIGH,
					"ov7725_reset");
		if (retval < 0)
			return retval;
	}



	/* Set initial values for the sensor struct. */
	memset(&ov7725_data, 0, sizeof(ov7725_data));
	ov7725_data.sensor_clk = devm_clk_get(dev, "csi_mclk");
	if (IS_ERR(ov7725_data.sensor_clk)) {
		dev_err(dev, "get mclk failed\n");
		return PTR_ERR(ov7725_data.sensor_clk);
	}

	retval = of_property_read_u32(dev->of_node, "mclk",
					&ov7725_data.mclk);
	if (retval) {
		dev_err(dev, "mclk frequency is invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "mclk_source",
					(u32 *) &(ov7725_data.mclk_source));
	if (retval) {
		dev_err(dev, "mclk_source invalid\n");
		return retval;
	}

	retval = of_property_read_u32(dev->of_node, "csi_id",
					&(ov7725_data.csi));
	if (retval) {
		dev_err(dev, "csi_id invalid\n");
		return retval;
	}

	/* Set mclk rate before clk on */
	ov7725_set_clk_rate();

	clk_prepare_enable(ov7725_data.sensor_clk);

	ov7725_data.io_init = ov7725_reset;
	ov7725_data.i2c_client = client;
	ov7725_data.pix.pixelformat = V4L2_PIX_FMT_YUYV;
	ov7725_data.pix.width = 480;
	ov7725_data.pix.height = 272;
	ov7725_data.streamcap.capability = V4L2_MODE_HIGHQUALITY |
					   V4L2_CAP_TIMEPERFRAME;
	ov7725_data.streamcap.capturemode = 0;
	ov7725_data.streamcap.timeperframe.denominator = DEFAULT_FPS;
	ov7725_data.streamcap.timeperframe.numerator = 1;

	ov7725_reset();

	ov7725_power_down(0);

	retval = ov7725_read_reg(OV7725_CHIP_ID_HIGH_BYTE, &chip_id_high);
	if (retval < 0 || chip_id_high != 0x77) {
		clk_disable_unprepare(ov7725_data.sensor_clk);
		pr_warning("camera ov7725 is not found\n");
		return -ENODEV;
	}
	retval = ov7725_read_reg(OV7725_CHIP_ID_LOW_BYTE, &chip_id_low);
	if (retval < 0 || chip_id_low != 0x21) {
		clk_disable_unprepare(ov7725_data.sensor_clk);
		pr_warning("camera ov7725 is not found\n");
		return -ENODEV;
	}

	retval = init_device();
	if (retval < 0) {
		clk_disable_unprepare(ov7725_data.sensor_clk);
		pr_warning("camera ov7725 init failed\n");
		ov7725_power_down(1);
		return retval;
	}

	clk_disable(ov7725_data.sensor_clk);

	v4l2_i2c_subdev_init(&ov7725_data.subdev, client, &ov7725_subdev_ops);

	retval = v4l2_async_register_subdev(&ov7725_data.subdev);
	if (retval < 0)
		dev_err(&client->dev,
					"%s--Async register failed, ret=%d\n", __func__, retval);

	pr_info("camera ov7725, is found\n");
	return retval;
}

/*!
 * ov7725 I2C detach function
 *
 * @param client            struct i2c_client *
 * @return  Error code indicating success or failure
 */
static int ov7725_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	v4l2_async_unregister_subdev(sd);

	clk_unprepare(ov7725_data.sensor_clk);

	ov7725_power_down(1);

	return 0;
}

module_i2c_driver(ov7725_i2c_driver);

MODULE_AUTHOR("Trunexa Inc.");
MODULE_DESCRIPTION("OV7725 Camera Driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
MODULE_ALIAS("CSI");
