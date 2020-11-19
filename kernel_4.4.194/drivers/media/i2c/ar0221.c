// SPDX-License-Identifier: GPL-2.0
/*
 * ar0221 driver
 *
 * Copyright (C) 2017 Fuzhou Rockchip Electronics Co., Ltd.
 */

#include <linux/clk.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <linux/sysfs.h>
#include <media/media-entity.h>
#include <media/v4l2-async.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-subdev.h>
#include <linux/pinctrl/consumer.h>
#include <linux/rk-camera-module.h>

#ifndef V4L2_CID_DIGITAL_GAIN
#define V4L2_CID_DIGITAL_GAIN		V4L2_CID_GAIN
#endif
/////////////////////
#define AR0221_REG_RESET                    0x301a 
#define      AR0221_RESET_SMIA_DIS          (1 << 12) 
#define      AR0221_RESET_FORCE_PLL_ON      (1 << 11) 
#define      AR0221_RESET_RESTART_BAD       (1 << 10) 
#define      AR0221_RESET_MASK_BAD          (1 << 9) 
#define      AR0221_RESET_GPI_EN            (1 << 8) 
#define      AR0221_RESET_LOCK_REG          (1 << 3) 
#define      AR0221_RESET_STREAM            (1 << 2) 
#define      AR0221_RESET_RESTART           (1 << 1) 
#define      AR0221_RESET_RESET             (1 << 0)
#define AR0221_MODE_SELECT              0x301c 
#define      AR0221_MODE_SELECT_STREAM      (1 << 0)  
//#define AR0221_SW_RESET           0x0103
//#define AR0221_REG_LINE_H         0x3500
//#define AR0221_REG_LINE_M         0x3501
//#define AR0221_REG_LINE_L         0x3502
//#define AR0221_EXPOSURE_DEFAULT   0x001000

#define AR0221_LINK_FREQ_300MHZ	    300000000
#define AR0221_LINK_FREQ_150MHZ     150000000
/* pixel rate = link frequency * 2 * lanes / BITS_PER_SAMPLE */
//#define AR0221_PIXEL_RATE		(AR0221_LINK_FREQ_150MHZ * 2 * 2 / 10)
#define AR0221_PIXEL_RATE       (AR0221_LINK_FREQ_300MHZ * 2 * 2 / 10)
#define AR0221_XVCLK_FREQ		     24000000

#define CHIP_ID                    0x0856
#define AR0221_REG_CHIP_ID         0x3000
//#define AR0221_REG_CHIPID_L     0x300B
#define AR0221_REG_SCCB_ID         0x3422
#define AR0221_REG_GREEN_GAIN      0x3056
#define AR0221_VENDOR_I2C_ADDR		0x36

#define AR0221_REG_CTRL_MODE		0x0100
#define AR0221_MODE_SW_STANDBY		0x0
#define AR0221_MODE_STREAMING		BIT(0)

#define AR0221_REG_EXPOSURE		0x3500
#define	AR0221_EXPOSURE_MIN		0x000000 //4   // #define AR0221_EXPOSURE_MIN     0x000000
#define AR0221_EXPOSURE_MAX     0x0fffff
#define	AR0221_EXPOSURE_STEP	1  //#define AR0221_EXPOSURE_STEP    0x01
#define AR0221_EXPOSURE_DEFAULT 0x001000
#define AR0221_VTS_MAX			0x7fff  //#define AR0221_EXPOSURE_MAX     0x0fffff

#define AR0221_REG_GAIN_H		0x350a
#define AR0221_REG_GAIN_L		0x350b
#define AR0221_GAIN_H_MASK		0x07
#define AR0221_GAIN_H_SHIFT		8
#define AR0221_GAIN_L_MASK		0xff
#define AR0221_GAIN_MIN		    0x10   //#define AR0221_ANALOG_GAIN_MIN  0x0000
#define AR0221_GAIN_MAX		    0x03ff //0xf8   // #define AR0221_ANALOG_GAIN_MAX  0x03ff
#define AR0221_GAIN_STEP		1      //#define AR0221_ANALOG_GAIN_STEP 0x01
#define AR0221_GAIN_DEFAULT		0x100 //0x10   //#define AR0221_ANALOG_GAIN_DEFAULT 0x100

#define AR0221_ANALOG_GAIN_MIN  0x0000
#define AR0221_ANALOG_GAIN_MAX  0x03ff
#define AR0221_ANALOG_GAIN_STEP 0x01
#define AR0221_ANALOG_GAIN_DEFAULT 0x100

#define AR0221_REG_TEST_PATTERN	    0x5e00
#define	AR0221_TEST_PATTERN_ENABLE	0x80
#define	AR0221_TEST_PATTERN_DISABLE	0x0

#define AR0221_REG_VTS			0x380e

#define REG_NULL			     0xFFFF

#define AR0221_REG_VALUE_08BIT		1
#define AR0221_REG_VALUE_16BIT		2
#define AR0221_REG_VALUE_24BIT		3

#define AR0221_LANES			     2
#define AR0221_BITS_PER_SAMPLE		10

#define AR0221_CHIP_REVISION_REG	0x302A
//#define AR0221_R1A			0xb1
//#define AR0221_R2A			0xb2

#define OF_CAMERA_PINCTRL_STATE_DEFAULT	"rockchip,camera_default"
#define OF_CAMERA_PINCTRL_STATE_SLEEP	"rockchip,camera_sleep"

#define CONFIG_PRINTK   1
//#define dev_err(fmt,arg...)          printk("<<-AR0221-ERR ->> "fmt"\n",##arg)
//#define dev_info(fmt,arg...)         printk("<<-AR0221-INFO->> "fmt"\n",##arg)

static DEFINE_MUTEX(ar0221_power_mutex);
static int ar0221_power_count;

//static const struct regval *ar0221_global_regs;

static const char * const ar0221_supply_names[] = {
	"avdd",		/* Analog power       1_C7 */
	"dovdd",	/* Digital I/O power  1_C7 */
	"dvdd",		/* Digital core power 1_C6 */
};

#define AR0221_NUM_SUPPLIES ARRAY_SIZE(ar0221_supply_names)

struct regval {
	u16 addr;
	u16 val;
};

/*
struct ar0221_mode {
    u32 width;
    u32 height;
    u32 max_fps;
    u32 hts_def;
    u32 vts_def;
    struct regval *reg_list;
};
*/

struct ar0221_mode {
	u32 width;
	u32 height;
	struct v4l2_fract max_fps;
	u32 hts_def;
	u32 vts_def;
	u32 exp_def;
	const struct regval *reg_list;
    const struct regval *reg_common;
};


struct ar0221 {
	struct i2c_client	     *client;
	struct clk		         *xvclk;
	struct gpio_desc	     *reset_gpio;
	struct gpio_desc	     *pwdn_gpio;
	struct regulator_bulk_data supplies[AR0221_NUM_SUPPLIES];

	struct pinctrl		     *pinctrl;
	struct pinctrl_state	 *pins_default;
	struct pinctrl_state	 *pins_sleep;

	struct v4l2_subdev	     subdev;
	struct media_pad	     pad;
	struct v4l2_ctrl_handler ctrl_handler;
    struct v4l2_ctrl         *link_freq;
    struct v4l2_ctrl         *pixel_rate;
	struct v4l2_ctrl	     *exposure;
	struct v4l2_ctrl	     *anal_gain;
	struct v4l2_ctrl	     *digi_gain;
	struct v4l2_ctrl	     *hblank;
	struct v4l2_ctrl	     *vblank;
	struct v4l2_ctrl	     *test_pattern;
	struct mutex		     mutex;
	bool			streaming;
	bool				power_on;
	const struct ar0221_mode *cur_mode;
	u32			module_index;
	const char		*module_facing;
	const char		*module_name;
	const char		*len_name;
};

static struct ar0221 *ar0221_master;

#define to_ar0221(sd) container_of(sd, struct ar0221, subdev)
/*
static struct regval sensor_oe_disable_regs[] = {
    {0x3000, 0x00},
    {0x3001, 0x00},
    {0x3002, 0x00},
    {REG_NULL, 0x00}
};

static struct regval sensor_oe_enable_regs[] = {
    {0x3000, 0x0f},
    {0x3001, 0xff},
    {0x3002, 0xe4},
    {REG_NULL, 0x00}
};
*/

/*
 * Xclk 24Mhz
 */
#if 0
static const struct regval ar0221_640x480_regs[] = {
    { 0x4800, 0x24},
    { 0x0100, 0x00},  //Nornal mode
    //{ 0x0103, 0x01},  //reset 
    { 0x3035, 0x11},
    { 0x3036, 0x46},
    { 0x303c, 0x11},
    { 0x3821, 0x07},
    { 0x3820, 0x41},
    { 0x370c, 0x03},
    { 0x3612, 0x59},
    { 0x3618, 0x00},
    { 0x5000, 0x06},
    { 0x5003, 0x08},
    { 0x5a00, 0x08},
    { 0x3000, 0xff},
    { 0x3001, 0xff},
    { 0x3002, 0xff},
    { 0x301d, 0xf0},
    { 0x3a18, 0x00},
    { 0x3a19, 0xf8},
    { 0x3c01, 0x80},
    { 0x3b07, 0x0c},
    { 0x380c, 0x07},
    { 0x380d, 0x3c},  //0x68
    { 0x380e, 0x03},
    { 0x380f, 0xf0},  //0xd8
    { 0x3814, 0x71},  //0x31
    { 0x3815, 0x71},  //0x31
    { 0x3708, 0x64},
    { 0x3709, 0x52},
    { 0x3808, 0x02},
    { 0x3809, 0x80},
    { 0x380a, 0x01},
    { 0x380b, 0xe0},
    { 0x3800, 0x00},
    { 0x3801, 0x10},  //0x18
    { 0x3802, 0x00},
    { 0x3803, 0x00},  //0x0e
    { 0x3804, 0x0a},
    { 0x3805, 0x2f},  //0x27
    { 0x3806, 0x07},
    { 0x3807, 0x9f},  //0x95
    { 0x3630, 0x2e},
    { 0x3632, 0xe2},
    { 0x3633, 0x23},
    { 0x3634, 0x44},
    { 0x3620, 0x64},
    { 0x3621, 0xe0},
    { 0x3600, 0x37},
    { 0x3704, 0xa0},
    { 0x3703, 0x5a},
    { 0x3715, 0x78},
    { 0x3717, 0x01},
    { 0x3731, 0x02},
    { 0x370b, 0x60},
    { 0x3705, 0x1a},
    { 0x3f05, 0x02},
    { 0x3f06, 0x10},
    { 0x3f01, 0x0a},
    { 0x3a08, 0x01},
    { 0x3a09, 0x27},
    { 0x3a0a, 0x00},
    { 0x3a0b, 0xf6},
    { 0x3a0d, 0x04},
    { 0x3a0e, 0x03},
    { 0x3a0f, 0x58},
    { 0x3a10, 0x50},
    { 0x3a1b, 0x58},
    { 0x3a1e, 0x50},
    { 0x3a11, 0x60},
    { 0x3a1f, 0x28},
    { 0x4001, 0x02},
    { 0x4004, 0x02},
    { 0x4000, 0x09},
    { 0x0100, 0x01},
    { 0x3000, 0x00},
    { 0x3001, 0x00},
    { 0x3002, 0x00},
    { 0x3017, 0xe0},
    { 0x301c, 0xfc},
    { 0x3636, 0x06},
    { 0x3016, 0x08},
    { 0x3827, 0xec},
    { 0x4800, 0x24},
    { 0x3018, 0x44},
    { 0x3035, 0x21},
    { 0x3106, 0xf5},
    { 0x3034, 0x1a},
    { 0x301c, 0xf8},
    {REG_NULL, 0x00}
};

/*
static const struct regval  ar0221_global_regs[] = {
    // upstream 
    {0x0100, 0x00},
    {0x0103, 0x01},
    {REG_NULL, 0x00}
};
*/

/*
 * Xclk 24Mhz
 * max_framerate 30fps
 * mipi_datarate per lane 600Mbps
 */

static struct regval ar0221_1280x960_regs[] = {
    { 0x4800, 0x24},
    { 0x0100, 0x00},
    //{ 0x0103, 0x01},
    { 0x3035, 0x11},
    { 0x3036, 0x46},
    { 0x303c, 0x11},
    { 0x3821, 0x07},
    { 0x3820, 0x41},
    { 0x370c, 0x03},
    { 0x3612, 0x59},
    { 0x3618, 0x00},
    { 0x5000, 0x06},
    { 0x5003, 0x08},
    { 0x5a00, 0x08},
    { 0x3000, 0xff},
    { 0x3001, 0xff},
    { 0x3002, 0xff},
    { 0x301d, 0xf0},
    { 0x3a18, 0x00},
    { 0x3a19, 0xf8},
    { 0x3c01, 0x80},
    { 0x3b07, 0x0c},
    { 0x380c, 0x07},
    { 0x380d, 0x68},
    { 0x380e, 0x03},
    { 0x380f, 0xd8},
    { 0x3814, 0x31},
    { 0x3815, 0x31},
    { 0x3708, 0x64},
    { 0x3709, 0x52},
    { 0x3808, 0x05},
    { 0x3809, 0x00},
    { 0x380a, 0x03},
    { 0x380b, 0xc0},
    { 0x3800, 0x00},
    { 0x3801, 0x18},
    { 0x3802, 0x00},
    { 0x3803, 0x0e},
    { 0x3804, 0x0a},
    { 0x3805, 0x27},
    { 0x3806, 0x07},
    { 0x3807, 0x95},
    { 0x3630, 0x2e},
    { 0x3632, 0xe2},
    { 0x3633, 0x23},
    { 0x3634, 0x44},
    { 0x3620, 0x64},
    { 0x3621, 0xe0},
    { 0x3600, 0x37},
    { 0x3704, 0xa0},
    { 0x3703, 0x5a},
    { 0x3715, 0x78},
    { 0x3717, 0x01},
    { 0x3731, 0x02},
    { 0x370b, 0x60},
    { 0x3705, 0x1a},
    { 0x3f05, 0x02},
    { 0x3f06, 0x10},
    { 0x3f01, 0x0a},
    { 0x3a08, 0x01},
    { 0x3a09, 0x27},
    { 0x3a0a, 0x00},
    { 0x3a0b, 0xf6},
    { 0x3a0d, 0x04},
    { 0x3a0e, 0x03},
    { 0x3a0f, 0x58},
    { 0x3a10, 0x50},
    { 0x3a1b, 0x58},
    { 0x3a1e, 0x50},
    { 0x3a11, 0x60},
    { 0x3a1f, 0x28},
    { 0x4001, 0x02},
    { 0x4004, 0x02},
    { 0x4000, 0x09},
    { 0x0100, 0x01},
    { 0x3000, 0x00},
    { 0x3001, 0x00},
    { 0x3002, 0x00},
    { 0x3017, 0xe0},
    { 0x301c, 0xfc},
    { 0x3636, 0x06},
    { 0x3016, 0x08},
    { 0x3827, 0xec},
    { 0x4800, 0x24},
    { 0x3018, 0x44},
    { 0x3035, 0x21},
    { 0x3106, 0xf5},
    { 0x3034, 0x1a},
    { 0x301c, 0xf8},
    {REG_NULL, 0x00}
};


static struct regval  ar0221_1920x1080_regs[] = {
    { 0x4800, 0x24},
    { 0x0100, 0x00},  //Nornal mode
    //{ 0x0103, 0x01},  //reset 
    { 0x3035, 0x11},
    { 0x3036, 0x46},
    { 0x303c, 0x11},
    { 0x3821, 0x07},
    { 0x3820, 0x41},
    { 0x370c, 0x03},
    { 0x3612, 0x59},
    { 0x3618, 0x00},
    { 0x5000, 0x06},
    { 0x5003, 0x08},
    { 0x5a00, 0x08},
    { 0x3000, 0xff},
    { 0x3001, 0xff},
    { 0x3002, 0xff},
    { 0x301d, 0xf0},
    { 0x3a18, 0x00},
    { 0x3a19, 0xf8},
    { 0x3c01, 0x80},
    { 0x3b07, 0x0c},
    { 0x380c, 0x09},  //0x07
    { 0x380d, 0x70},  //0x68
    { 0x380e, 0x04},  //0x03
    { 0x380f, 0x50},  //0xd8
    { 0x3814, 0x11},  //0x31
    { 0x3815, 0x11},  //0x31
    { 0x3708, 0x64},
    { 0x3709, 0x52},
    { 0x3808, 0x02},
    { 0x3809, 0x80},
    { 0x380a, 0x01},
    { 0x380b, 0xe0},
    { 0x3800, 0x01},  //0x00
    { 0x3801, 0x5c},  //0x18
    { 0x3802, 0x01},  //0x00
    { 0x3803, 0xb2},  //0x0e
    { 0x3804, 0x08},  //0x0a
    { 0x3805, 0xe3},  //0x27
    { 0x3806, 0x05},  //0x07
    { 0x3807, 0xf1},  //0x95
    { 0x3630, 0x2e},
    { 0x3632, 0xe2},
    { 0x3633, 0x23},
    { 0x3634, 0x44},
    { 0x3620, 0x64},
    { 0x3621, 0xe0},
    { 0x3600, 0x37},
    { 0x3704, 0xa0},
    { 0x3703, 0x5a},
    { 0x3715, 0x78},
    { 0x3717, 0x01},
    { 0x3731, 0x02},
    { 0x370b, 0x60},
    { 0x3705, 0x1a},
    { 0x3f05, 0x02},
    { 0x3f06, 0x10},
    { 0x3f01, 0x0a},
    { 0x3a08, 0x01},
    { 0x3a09, 0x27},
    { 0x3a0a, 0x00},
    { 0x3a0b, 0xf6},
    { 0x3a0d, 0x04},
    { 0x3a0e, 0x03},
    { 0x3a0f, 0x58},
    { 0x3a10, 0x50},
    { 0x3a1b, 0x58},
    { 0x3a1e, 0x50},
    { 0x3a11, 0x60},
    { 0x3a1f, 0x28},
    { 0x4001, 0x02},
    { 0x4004, 0x02},
    { 0x4000, 0x09},
    { 0x0100, 0x01},
    { 0x3000, 0x00},
    { 0x3001, 0x00},
    { 0x3002, 0x00},
    { 0x3017, 0xe0},
    { 0x301c, 0xfc},
    { 0x3636, 0x06},
    { 0x3016, 0x08},
    { 0x3827, 0xec},
    { 0x4800, 0x24},
    { 0x3018, 0x44},
    { 0x3035, 0x21},
    { 0x3106, 0xf5},
    { 0x3034, 0x1a},
    { 0x301c, 0xf8},

 {REG_NULL, 0x00}
};

#endif

static struct regval  ar0221_common_regs[] = {
     { 0x31E0, 0x0018 },// PIX_DEF_ID
     /*PLL Configuration  */
     { 0x302E, 0x0004 },// PRE_PLL_CLK_DIV 1- 64
     { 0x3030, 0x0044 },// PLL_MULTIPLIER  0x90(12bit) max 254
     { 0x302A, 0x000A },// VT_PIX_CLK_DIV   0x0A(27MHz) 0x12(12bit)
     { 0x302C, 0x0001 },// VT_SYS_CLK_DIV
     { 0x3038, 0x0001 },// OP_SYS_CLK_DIV
     { 0x3036, 0x000A },// OP_WORD_CLK_DIV  0x0A(27MHz)
     /* Data Format */
     { 0x31AC, 0x0C0A },// DATA_FORMAT_BITS  //0x0C0C(12bit)
     { 0x31AE, 0x0202 },// SERIAL_FORMAT  2Lanes & MIPI
     /* Mipi Setting */
     { 0x31B0, 0x0087 },// FRAME_PREAMBLE  87 max255
     { 0x31B2, 0x0056 },// LINE_PREAMBLE   56 max255
     { 0x31B4, 0x5247 },// MIPI_TIMING_0
     { 0x31B6, 0x3255 },// MIPI_TIMING_1
     { 0x31B8, 0x804B },// MIPI_TIMING_2
     { 0x31BA, 0x028A },// MIPI_TIMING_3
     { 0x31BC, 0x8D08 },// MIPI_TIMING_4
     { 0x3342, 0x122B },// MIPI_F1_PDT_EDT
     { 0x3346, 0x122B },// MIPI_F2_PDT_EDT
     { 0x334A, 0x122B },// MIPI_F3_PDT_EDT
     { 0x334E, 0x122B },// MIPI_F4_PDT_EDT

     { 0x3082, 0x0001 },// OPERATION_MODE_CTRL  Linear Mode
     { 0x30BA, 0x1100 },// DIGITAL_CTRL
     { 0x3014, 0x0000 },// FINE_INTEGRATION_TIME_
     
     { 0x3366, 0x2020 },// ANALOG_GAIN 0x2020
     { 0x336A, 0xA040 },// ANALOG_GAIN2 0xA020
     { 0x3370, 0x0331 },// DBLC_CONTROL
     { 0x3092, 0x0C24 },// ROW_NOISE_CONTROL
     { 0x351C, 0x0045 },// RESERVED_MFR_351C
     { 0x3522, 0x8840 },// RESERVED_MFR_3522
     { 0x3524, 0x4046 },// RESERVED_MFR_3524
     { 0x3540, 0xC63C },// RESERVED_MFR_3540
     { 0x3542, 0x4640 },// RESERVED_MFR_3542
     { 0x3544, 0x464B },// RESERVED_MFR_3544
     { 0x3546, 0x5653 },// RESERVED_MFR_3546
     { 0x3548, 0x5600 },// RESERVED_MFR_3548
     { 0x337A, 0x0B53 },// DBLC_SCALE0
     { 0x30FE, 0x00A8 },// NOISE_PEDESTAL
     { 0x3222, 0x05DC },// FINE_INTEGRATION_TIME3
     { 0x31DE, 0x0410 },// RESERVED_MFR_31DE
     { 0x31E0, 0x001B },// PIX_DEF_ID
     { 0x3372, 0xF10F },// DBLC_FS0_CONTROL
     { 0x3566, 0xB538 },// RESERVED_MFR_3566
     { 0x30B4, 0x0083 },// TEMPSENS0_CTRL_REG
     /*ESCHOI TEST*/
     { 0x3058, 0x00E0 },// BLUE GAIN  def(0x0080)
    // { 0x3070, 0x0002 },// TEST PATTERN
    // { 0x301A, 0x001E },// RESET_REGISTER
    /////////////
     {REG_NULL, 0x00}
};

static struct regval  ar0221_1920x1080_regs[] = {
     /* Resolution Setting  */
     { 0x3004, 0x0008 },// X_ADDR_START_    8
     { 0x3008, 0x0787 },// X_ADDR_END_      1927
     { 0x3002, 0x0008 },// Y_ADDR_START_    8
     { 0x3006, 0x043F },// Y_ADDR_END_      0x43f(1087)
     { 0x300A, 0x0466 },// FRAME_LENGTH_LINES_ 0x0466(1126)
     { 0x300C, 0x04B6 },// LINE_LENGTH_PCK_ 0x04B6(1206)
     { 0x3238, 0x8222 },// EXPOSURE RATIO
     { 0x3012, 0x0352 },// COARSE_INTEGRATION_TIME_ 0x0152
     { 0x3212, 0x0054 },// COARSE_INTEGRATION_TIME2  0x0054
     { 0x3216, 0x0015 },// COARSE_INTEGRATION_TIME3  0x0015
     { 0x30A2, 0x0001 },// X_ODD_INC_   1=>No skip
     { 0x30A6, 0x0001 },// Y_ODD_INC_   1=>No skip
     {REG_NULL, 0x00}
};

static struct regval  ar0221_1080x1080_regs[] = {
     /* Resolution Setting  */
     { 0x3004, 0x01AC },// X_ADDR_START_    8+420
     { 0x3008, 0x05E3 },// X_ADDR_END_      1087+420
     { 0x3002, 0x0008 },// Y_ADDR_START_    8
     { 0x3006, 0x043F },// Y_ADDR_END_      0x43f(1087)
     { 0x300A, 0x0466 },// FRAME_LENGTH_LINES_ 0x0466(1126)
     { 0x300C, 0x04B6 },// LINE_LENGTH_PCK_ 0x04B6(1206)
     { 0x3238, 0x8222 },// EXPOSURE RATIO
     { 0x3012, 0x0352 },// COARSE_INTEGRATION_TIME_ 0x0152
     { 0x3212, 0x0054 },// COARSE_INTEGRATION_TIME2  0x0054
     { 0x3216, 0x0015 },// COARSE_INTEGRATION_TIME3  0x0015
     { 0x30A2, 0x0001 },// X_ODD_INC_   1=>No skip
     { 0x30A6, 0x0001 },// Y_ODD_INC_   1=>No skip
     {REG_NULL, 0x00}
};

static struct regval  ar0221_540x540_regs[] = {
     /* Resolution Setting  */
     { 0x3004, 0x02BA },// X_ADDR_START_    8 + 690
     { 0x3008, 0x04D5 },// X_ADDR_END_      547 + 690
     { 0x3002, 0x0116 },// Y_ADDR_START_    8 + 270
     { 0x3006, 0x0331 },// Y_ADDR_END_      547 + 270
     { 0x300A, 0x0466 },// FRAME_LENGTH_LINES_ 0x0466(1126)
     { 0x300C, 0x04B6 },// LINE_LENGTH_PCK_ 0x04B6(1206)
     { 0x3238, 0x8222 },// EXPOSURE RATIO
     { 0x3012, 0x0352 },// COARSE_INTEGRATION_TIME_ 0x0152
     { 0x3212, 0x0054 },// COARSE_INTEGRATION_TIME2  0x0054
     { 0x3216, 0x0015 },// COARSE_INTEGRATION_TIME3  0x0015
     { 0x30A2, 0x0001 },// X_ODD_INC_   1=>No skip
     { 0x30A6, 0x0001 },// Y_ODD_INC_   1=>No skip
     {REG_NULL, 0x00}
};


#if 0
static struct regval ar0221_2592x1944_regs[] = {
    {0x0100, 0x00},
    {0x3035, 0x21},
    {0x3036, 0x60},
    {0x303c, 0x11},
    {0x3612, 0x5b},
    {0x3618, 0x04},
    {0x380c, 0x0a},
    {0x380d, 0x8c},
    {0x380e, 0x07},
    {0x380f, 0xb6},
    {0x3814, 0x11},
    {0x3815, 0x11},
    {0x3708, 0x64},
    {0x3709, 0x12},
    {0x3808, 0x0a},
    {0x3809, 0x20},
    {0x380a, 0x07},
    {0x380b, 0x98},
    {0x3800, 0x00},
    {0x3801, 0x0c},
    {0x3802, 0x00},
    {0x3803, 0x04},
    {0x3804, 0x0a},
    {0x3805, 0x33},
    {0x3806, 0x07},
    {0x3807, 0xa3},
    {0x3a08, 0x01},
    {0x3a09, 0x28},
    {0x3a0a, 0x00},
    {0x3a0b, 0xf6},
    {0x3a0d, 0x08},
    {0x3a0e, 0x06},
    {0x4004, 0x04},
    {0x4837, 0x19},
    {0x0100, 0x01},
    {REG_NULL, 0x00}
};

#endif

static const struct ar0221_mode supported_modes[] = {
    {
     .width = 540, //1296,
     .height = 540, //972,
     .max_fps = {  //25,
          .numerator = 10000,
          .denominator = 600000,
      },
     .hts_def = 0x1000,  //4096
     .vts_def = 0x0700,  //1536
     .reg_list = ar0221_540x540_regs,
     .reg_common = ar0221_common_regs,

     },
     
    {
     .width = 1080, //1296,
     .height = 1080, //972,
     .max_fps = {  //25,
          .numerator = 10000,
          .denominator = 400000,
      },
     .hts_def = 0x1000,  //4096
     .vts_def = 0x0700,  //1536
     .reg_list = ar0221_1080x1080_regs,
     .reg_common = ar0221_common_regs,

     },
  
    {
     .width = 1920, //2592,
     .height = 1080, //1944,
     .max_fps = {  //25,
          .numerator = 10000,
          .denominator = 400000,
      },
     .hts_def = 0x1000,  //4096
     .vts_def = 0x0700,  //1536
     .reg_list = ar0221_1920x1080_regs,
     .reg_common = ar0221_common_regs,
     },
};

/*
static const struct ar0221_mode supported_modes[] = {
	{
		.width = 1296, //2112,
		.height = 972, //1568,
        .max_fps = 15,
		//.max_fps = {  //25,
		//	.numerator = 10000,
	    //		.denominator = 300000,
		//},
		//.exp_def = 0x0600,
		.hts_def = 0x0a8c, //0x12c0,
		.vts_def = 0x058c, //0x0680,
		.reg_list = ar0221_1296x972_regs, //ar0221_2112x1568_regs,
	},{
		.width = 2592, //4224,
		.height = 1944, //3136,
        .max_fps = 15,
		//.max_fps = {
		//	.numerator = 20000,
		//	.denominator = 150000,
		//},
		//.exp_def = 0x0600,
		.hts_def = 0x0a8c, //0x12c0,
		.vts_def = 0x07b6, //0x0d00,
		.reg_list = ar0221_2592x1944_regs, //ar0221_4224x3136_regs,
	},
};
*/


static void ar0221_get_module_inf(struct ar0221 *ar0221,
				  struct rkmodule_inf *inf)
{
	struct device *dev = &ar0221->client->dev;

	memset(inf, 0, sizeof(*inf));
	strlcpy(inf->base.sensor, "ar0221", sizeof(inf->base.sensor));
	strlcpy(inf->base.module, ar0221->module_name,
		sizeof(inf->base.module));
	strlcpy(inf->base.lens, ar0221->len_name, sizeof(inf->base.lens));


	dev_info(dev, "DBG get_module_inf %d %s %s %s", ar0221->module_index, ar0221->module_name, ar0221->module_facing, ar0221->len_name);
}

static long ar0221_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	struct ar0221 *ar0221 = to_ar0221(sd);
	long ret = 0;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		ar0221_get_module_inf(ar0221, (struct rkmodule_inf *)arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}

#ifdef CONFIG_COMPAT
static long ar0221_compat_ioctl32(struct v4l2_subdev *sd,
				  unsigned int cmd, unsigned long arg)
{
	void __user *up = compat_ptr(arg);
	struct rkmodule_inf *inf;
	long ret;

	switch (cmd) {
	case RKMODULE_GET_MODULE_INFO:
		inf = kzalloc(sizeof(*inf), GFP_KERNEL);
		if (!inf) {
			ret = -ENOMEM;
			return ret;
		}

		ret = ar0221_ioctl(sd, cmd, inf);
		if (!ret)
			ret = copy_to_user(up, inf, sizeof(*inf));
		kfree(inf);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	return ret;
}
#endif 


static const s64 link_freq_menu_items[] = {
	AR0221_LINK_FREQ_300MHZ
    //AR0221_LINK_FREQ_150MHZ
};

static const char * const ar0221_test_pattern_menu[] = {
	"Disabled",
	"Vertical Color Bar Type 1",
	"Vertical Color Bar Type 2",
	"Vertical Color Bar Type 3",
	"Vertical Color Bar Type 4"
};

/* Write registers up to 4 at a time */
static int ar0221_write_reg(struct i2c_client *client, u16 reg,
			     u32 len, u32 val)
{
	u32 buf_i, val_i;
	u8 buf[6];
	u8 *val_p;
	__be32 val_be;

	if (len > 4)
		return -EINVAL;

	buf[0] = reg >> 8;
	buf[1] = reg & 0xff;

	val_be = cpu_to_be32(val);
	val_p = (u8 *)&val_be;
	buf_i = 2;
	val_i = 4 - len;

	while (val_i < 4)
		buf[buf_i++] = val_p[val_i++];

	if (i2c_master_send(client, buf, len + 2) != len + 2)
		return -EIO;

	return 0;
}

static int ar0221_write_array(struct i2c_client *client,
			       const struct regval *regs)
{
	u32 i;
	int ret = 0;

	for (i = 0; ret == 0 && regs[i].addr != REG_NULL; i++)
    {
		ret = ar0221_write_reg(client, regs[i].addr,
					AR0221_REG_VALUE_16BIT,
					regs[i].val);
    }

	return ret;
}

/* Read registers up to 4 at a time */
static int ar0221_read_reg(struct i2c_client *client, u16 reg,
			    unsigned int len, u32 *val)
{
	struct i2c_msg msgs[2];
	u8 *data_be_p;
	__be32 data_be = 0;
	__be16 reg_addr_be = cpu_to_be16(reg);
	int ret;
    //struct device *dev = &client->dev;

	if (len > 4 || !len)
		return -EINVAL;

	data_be_p = (u8 *)&data_be;
	/* Write register address */
	msgs[0].addr = client->addr;
	msgs[0].flags = 0;
	msgs[0].len = 2;
	msgs[0].buf = (u8 *)&reg_addr_be;

	/* Read data from register */
	msgs[1].addr = client->addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].len = len;
	msgs[1].buf = &data_be_p[4 - len];

    //dev_info(dev, "dev_addr=0x%02x reg_addr0==0x%02x reg_addr01=0x%02x \n",
    //    msgs[0].addr,msgs[0].buf[0],msgs[0].buf[1]);

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
    //dev_info(dev, "r0=0x%02x r1=0x%02x r2=0x%02x r3=0x%02x\n",
    //    msgs[1].buf[0],msgs[1].buf[1],msgs[1].buf[2],msgs[1].buf[3]);

	if (ret != ARRAY_SIZE(msgs))
		return -EIO;

	*val = be32_to_cpu(data_be);

	return 0;
}

static int ar0221_get_reso_dist(const struct ar0221_mode *mode,
				 struct v4l2_mbus_framefmt *framefmt)
{
	return abs(mode->width - framefmt->width) +
	       abs(mode->height - framefmt->height);
}

static const struct ar0221_mode *
ar0221_find_best_fit(struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *framefmt = &fmt->format;
	int dist;
	int cur_best_fit = 0;
	int cur_best_fit_dist = -1;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(supported_modes); i++) {
		dist = ar0221_get_reso_dist(&supported_modes[i], framefmt);
		if (cur_best_fit_dist == -1 || dist < cur_best_fit_dist) {
			cur_best_fit_dist = dist;
			cur_best_fit = i;
		}
	}

	return &supported_modes[cur_best_fit];
}

static int ar0221_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *fmt)
{
	struct ar0221 *ar0221 = to_ar0221(sd);
	const struct ar0221_mode *mode;
	s64 h_blank, vblank_def;

	mutex_lock(&ar0221->mutex);

	mode = ar0221_find_best_fit(fmt);
	fmt->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->format.width = mode->width;
	fmt->format.height = mode->height;
	fmt->format.field = V4L2_FIELD_NONE;
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		*v4l2_subdev_get_try_format(sd, cfg, fmt->pad) = fmt->format;
#else
		mutex_unlock(&ar0221->mutex);
		return -ENOTTY;
#endif
	} else {
		ar0221->cur_mode = mode;
		h_blank = mode->hts_def - mode->width;
		__v4l2_ctrl_modify_range(ar0221->hblank, h_blank,
					 h_blank, 1, h_blank);
		vblank_def = mode->vts_def - mode->height;
		__v4l2_ctrl_modify_range(ar0221->vblank, vblank_def,
					 AR0221_VTS_MAX - mode->height,
					 1, vblank_def);
	}

	mutex_unlock(&ar0221->mutex);

	return 0;
}

static int ar0221_get_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_pad_config *cfg,
			   struct v4l2_subdev_format *fmt)
{
	struct ar0221 *ar0221 = to_ar0221(sd);
	const struct ar0221_mode *mode = ar0221->cur_mode;

	mutex_lock(&ar0221->mutex);
	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY) {
#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
		fmt->format = *v4l2_subdev_get_try_format(sd, cfg, fmt->pad);
#else
		mutex_unlock(&ar0221->mutex);
		return -ENOTTY;
#endif
	} else {
		fmt->format.width = mode->width;
		fmt->format.height = mode->height;
		fmt->format.code = MEDIA_BUS_FMT_SGRBG10_1X10;
		fmt->format.field = V4L2_FIELD_NONE;
	}
	mutex_unlock(&ar0221->mutex);

	return 0;
}

static int ar0221_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_pad_config *cfg,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index != 0)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;

	return 0;
}

static int ar0221_enum_frame_sizes(struct v4l2_subdev *sd,
				    struct v4l2_subdev_pad_config *cfg,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(supported_modes))
		return -EINVAL;

	if (fse->code != MEDIA_BUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	fse->min_width  = supported_modes[fse->index].width;
	fse->max_width  = supported_modes[fse->index].width;
	fse->max_height = supported_modes[fse->index].height;
	fse->min_height = supported_modes[fse->index].height;

	return 0;
}

/*
static int ar0221_enable_test_pattern(struct ar0221 *ar0221, u32 pattern)
{
	u32 val;

	if (pattern)
		val = (pattern - 1) | AR0221_TEST_PATTERN_ENABLE;
	else
		val = AR0221_TEST_PATTERN_DISABLE;

	return ar0221_write_reg(ar0221->client,
				 AR0221_REG_TEST_PATTERN,
				 AR0221_REG_VALUE_08BIT,
				 val);
}
*/

static int ar0221_g_frame_interval(struct v4l2_subdev *sd,
				    struct v4l2_subdev_frame_interval *fi)
{
	struct ar0221 *ar0221 = to_ar0221(sd);
	const struct ar0221_mode *mode = ar0221->cur_mode;

	mutex_lock(&ar0221->mutex);
	fi->interval = mode->max_fps;
	mutex_unlock(&ar0221->mutex);

	return 0;
}

static int __ar0221_start_stream(struct ar0221 *ar0221)
{
	int ret;
    u32 read_val;

    //dev_info(&ar0221->client->dev, "start1_stream_id = 0x%02x \n",ar0221->client->addr);

	//Switch Control

    //ret = ar0221_write_array(ar0221->client, ar0221_global_regs);
	//if (ret)
	//	return ret;

    //gpiod_set_value(ar0221->pwdn_gpio, 1);  //eschoi
    //dev_info(&ar0221->client->dev, "start stream pwdn_gpio_high addr=0x%02x \n",ar0221->client->addr);


	ret = ar0221_write_array(ar0221->client, ar0221->cur_mode->reg_list);
	//if (ret)
	//	return ret;

    ret = ar0221_write_array(ar0221->client, ar0221->cur_mode->reg_common);
    //if (ret)
    //    return ret;

	/* In case these controls are set before streaming */
	mutex_unlock(&ar0221->mutex);
	ret = v4l2_ctrl_handler_setup(&ar0221->ctrl_handler);
	mutex_lock(&ar0221->mutex);
	if (ret)
		return ret;

    dev_info(&ar0221->client->dev, "------Color Order----------\n");

    //ret = ar0221_write_reg(ar0221->client,0x3024,AR0221_REG_VALUE_16BIT, 0x0002);
    ret = ar0221_read_reg(ar0221->client, 0x3040,AR0221_REG_VALUE_16BIT, &read_val);
    dev_info(&ar0221->client->dev, "Read 0x3040 Vallue=0x%04x \n",read_val);
    ret = ar0221_read_reg(ar0221->client, 0x3024,AR0221_REG_VALUE_16BIT, &read_val);
    dev_info(&ar0221->client->dev, "Read 0x3024 Vallue=0x%04x \n",read_val);

    ret = ar0221_write_reg(ar0221->client,AR0221_REG_RESET,AR0221_REG_VALUE_16BIT, 0x001E);
    usleep_range(10000, 20000);

    dev_info(&ar0221->client->dev, "------Stream Start----------\n");
    usleep_range(500000, 600000);
    ret = ar0221_read_reg(ar0221->client, 0x2000,AR0221_REG_VALUE_16BIT, &read_val);
    dev_info(&ar0221->client->dev, "Read 0x2000 Vallue=0x%04x \n",read_val);
    ret = ar0221_read_reg(ar0221->client, 0x2002,AR0221_REG_VALUE_16BIT, &read_val);
    dev_info(&ar0221->client->dev, "Read 0x2002 Vallue=0x%04x \n",read_val);
   
    //usleep_range(4000, 8000);
    /*
    ret = ar0221_read_reg(ar0221->client, 0x2000,AR0221_REG_VALUE_16BIT, &read_val);
    dev_info(&ar0221->client->dev, "Read 0x2000 Vallue=0x%04x \n",read_val);
    ret = ar0221_read_reg(ar0221->client, 0x2002,AR0221_REG_VALUE_16BIT, &read_val);
    dev_info(&ar0221->client->dev, "Read 0x2002 Vallue=0x%04x \n",read_val);
    ret = ar0221_read_reg(ar0221->client, 0x2004,AR0221_REG_VALUE_16BIT, &read_val);
    dev_info(&ar0221->client->dev, "Read 0x2004 Vallue=0x%04x \n",read_val);
    ret = ar0221_read_reg(ar0221->client, 0x2006,AR0221_REG_VALUE_16BIT, &read_val);
    dev_info(&ar0221->client->dev, "Read 0x2006 Vallue=0x%04x \n",read_val);
    ret = ar0221_read_reg(ar0221->client, 0x2008,AR0221_REG_VALUE_16BIT, &read_val);
    dev_info(&ar0221->client->dev, "Read 0x2008 Vallue=0x%04x \n",read_val);
   */

    //ret = ar0221_read_reg(ar0221->client, 0x30B4,AR0221_REG_VALUE_16BIT, &read_val);
    //dev_info(&ar0221->client->dev, "Read 0x30B4 Vallue(0x0083)=0x%04x \n",read_val);

    //ret = ar0221_read_reg(ar0221->client, 0x3372,AR0221_REG_VALUE_16BIT, &read_val);
    //dev_info(&ar0221->client->dev, "Read 0x3372 Vallue(0xF10F)=0x%04x \n",read_val);
     
    //gpiod_set_value_cansleep(ar0221->pwdn_gpio, 0);
    //dev_info(&ar0221->client->dev, "start stream pwdn_gpio_low addr=0x%02x \n",ar0221->client->addr);
     
     return ret;
}

static int __ar0221_stop_stream(struct ar0221 *ar0221)
{
    int ret;

    //gpiod_set_value_cansleep(ar0221->pwdn_gpio, 1);
    //dev_info(&ar0221->client->dev, "stop stream pwdn_gpio_high addr=0x%02x \n",ar0221->client->addr);
    //ret = ar0221_write_reg(ar0221->client,0x301A,AR0221_REG_VALUE_16BIT, 0x00000);
    ret = ar0221_write_reg(ar0221->client,AR0221_MODE_SELECT,AR0221_REG_VALUE_16BIT, 0x00);
    //usleep_range(500, 1000);
    //gpiod_set_value_cansleep(ar0221->pwdn_gpio, 0);
    //dev_info(&ar0221->client->dev, "stop stream pwdn_gpio_low addr=0x%02x \n",ar0221->client->addr);

    return ret;

}

static int ar0221_s_stream(struct v4l2_subdev *sd, int on)
{
	struct ar0221 *ar0221 = to_ar0221(sd);
	struct i2c_client *client = ar0221->client;
	int ret = 0;

	mutex_lock(&ar0221->mutex);
	on = !!on;
	if (on == ar0221->streaming)
		goto unlock_and_return;

	if (on) {
		ret = pm_runtime_get_sync(&client->dev);
		if (ret < 0) {
			pm_runtime_put_noidle(&client->dev);
			goto unlock_and_return;
		}

		ret = __ar0221_start_stream(ar0221);
		if (ret) {
			v4l2_err(sd, "start stream failed while write regs\n");
			pm_runtime_put(&client->dev);
			goto unlock_and_return;
		}
	} else {
		__ar0221_stop_stream(ar0221);
		pm_runtime_put(&client->dev);
	}

	ar0221->streaming = on;

unlock_and_return:
	mutex_unlock(&ar0221->mutex);

	return ret;
}

/* Calculate the delay in us by clock rate and clock cycles */
static inline u32 ar0221_cal_delay(u32 cycles)
{
	return DIV_ROUND_UP(cycles, AR0221_XVCLK_FREQ / 1000 / 1000);
}

static int __ar0221_master_power_on(struct device *dev) {
	struct ar0221 *ar0221 = ar0221_master;
	int ret;

	if (!ar0221) {
		dev_err(dev, "no ar0221 master set\n");
		return -EINVAL;
	}

	ar0221_power_count++;
	if (ar0221_power_count > 1) {
		ret = 0;
		goto err_shortcut;
	}

	if (!IS_ERR_OR_NULL(ar0221->pins_default)) {
		ret = pinctrl_select_state(ar0221->pinctrl,
					   ar0221->pins_default);
		if (ret < 0) {
			dev_err(dev, "could not set pins\n");
			goto err_pins;
		}
	}

	ret = clk_prepare_enable(ar0221->xvclk);
	if (ret < 0) {
		dev_err(dev, "Failed to enable xvclk\n");
		goto err_clk;
	}

	//if (!IS_ERR(ar0221->reset_gpio))
    //{
	//	gpiod_set_value_cansleep(ar0221->reset_gpio, 0);   //eschoi
    //    dev_info(dev, "master power on  pwdn_gpio_low addr=0x%02x \n",ar0221->client->addr);
    //}

	ret = regulator_bulk_enable(AR0221_NUM_SUPPLIES, ar0221->supplies);
	if (ret < 0) {
		dev_err(dev, "Failed to enable regulators\n");
		goto err_regulator;
	}

    usleep_range(10000, 20000);

	//if (!IS_ERR(ar0221->reset_gpio))
    //{
	//	gpiod_set_value_cansleep(ar0221->reset_gpio, 1);  //eschoi
    //    dev_info(dev, "master power on pwdn_gpio_high addr=0x%02x \n",ar0221->client->addr);
    //}

	return 0;

err_regulator:
	clk_disable_unprepare(ar0221->xvclk);
err_clk:
	if (!IS_ERR_OR_NULL(ar0221->pins_sleep)) {
		int _ret;
		_ret = pinctrl_select_state(ar0221->pinctrl,
					   ar0221->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set sleep pins\n");
	}
err_pins:
	ar0221_power_count--;
err_shortcut:
	return ret;
}

static void __ar0221_master_power_off(struct device *dev)
{
	struct ar0221 *ar0221 = ar0221_master;
	int ret;

	if (!ar0221) {
		dev_err(dev, "no ar0221 master set\n");
		return;
	}

	ar0221_power_count--;
	if (ar0221_power_count > 0) {
		return;
	}

	clk_disable_unprepare(ar0221->xvclk);
	//if (!IS_ERR(ar0221->reset_gpio))
    //{
	//	gpiod_set_value_cansleep(ar0221->reset_gpio, 0);
    //    dev_info(dev, "master power off pwdn_gpio_low addr=0x%02x \n",ar0221->client->addr);
    //}
	if (!IS_ERR_OR_NULL(ar0221->pins_sleep)) {
		ret = pinctrl_select_state(ar0221->pinctrl,
					   ar0221->pins_sleep);
		if (ret < 0)
			dev_dbg(dev, "could not set pins\n");
	}
	regulator_bulk_disable(AR0221_NUM_SUPPLIES, ar0221->supplies);

	return;
}

static int __ar0221_power_on(struct ar0221 *ar0221)
{
	int ret;
	u32 delay_us;
	struct device *dev = &ar0221->client->dev;
	struct i2c_client *client = ar0221->client;
	u8  addr,addr2;
    u32 sccb_id;
	u32 reset_init_state;

	mutex_lock(&ar0221_power_mutex);
	ret = __ar0221_master_power_on(dev);
	if (ret) {
		dev_err(dev, "could not power on, error %d\n", ret);
		goto err_power;
	}

	usleep_range(500, 1000);
	if (!IS_ERR(ar0221->pwdn_gpio))
    {
        gpiod_set_value_cansleep(ar0221->pwdn_gpio, 1);  //High
        usleep_range(500, 1000); 
        //gpiod_set_value_cansleep(ar0221->pwdn_gpio, 1);  //High
        //usleep_range(500, 1000); 
        //gpiod_set_value_cansleep(ar0221->pwdn_gpio, 1);  //High
        dev_info(dev, "f_power_on pwdn_gpio_high addr=0x%02x \n",ar0221->client->addr);
    } 

		
    if (!IS_ERR(ar0221->reset_gpio))
    {
        //gpiod_set_value_cansleep(ar0221->reset_gpio, 1);   //eschoi
        //usleep_range(1000, 2000); 
        gpiod_set_value_cansleep(ar0221->reset_gpio, 0);   //eschoi
        usleep_range(1000, 2000);  //1ms 
        gpiod_set_value_cansleep(ar0221->reset_gpio, 1);   //eschoi
        usleep_range(1000, 2000); 
        dev_info(dev, "f_power on  reset_gpio_low->high addr=0x%02x \n",ar0221->client->addr);
    }

	/* 8192 cycles prior to first SCCB transaction */
	delay_us = ar0221_cal_delay(8192);
	usleep_range(delay_us, delay_us * 2);

/*
    

    //ret = ar0221_read(sd, 0x0100, &resetval);
    ret = ar0221_read_reg(client, 0x0100,
                   AR0221_REG_VALUE_16BIT, &resetval);
    if (ret < 0)
        return ret;

    if (!(resetval & 0x01)) {
        dev_err(&client->dev, "Device was in SW standby");
        //ret = ar0221_write(sd, 0x0100, 0x01);
        ret = ar0221_write_reg(client,0x0100,AR0221_REG_VALUE_08BIT, 0x01);
        if (ret < 0)
            return ret;
    }
*/
     /* Change i2c address by programming SCCB_ID */
	
    addr = client->addr;
    addr2 = addr * 2;
	//local_i2c_id_addr = AR0221_VENDOR_I2C_ADDR;
	if (addr != AR0221_VENDOR_I2C_ADDR) {
		client->addr = AR0221_VENDOR_I2C_ADDR;
        //SW Reset
		ret = ar0221_read_reg(client, AR0221_REG_RESET, AR0221_REG_VALUE_16BIT, &sccb_id);
		reset_init_state = sccb_id;
		//dev_info(dev, "init reset  addr=0x%02x r_val = 0x%04x ret=%d \n",client->addr * 2,sccb_id , ret);
		ret = ar0221_write_reg(client, AR0221_REG_RESET,	AR0221_REG_VALUE_16BIT, 0x0010);
		ret = ar0221_read_reg(client, AR0221_REG_RESET, AR0221_REG_VALUE_16BIT, &sccb_id);
		dev_info(dev, "unlock reset  addr=0x%02x r_val = 0x%04x ret=%d \n",client->addr * 2,sccb_id , ret);
		ret = ar0221_write_reg(client, AR0221_REG_SCCB_ID,	AR0221_REG_VALUE_16BIT, 0x6e8c);
        dev_info(dev, "write  addr=0x%02x target = 0x%02x ret=%d \n",client->addr * 2,addr2 , ret);
		if (ret) 	goto err_i2c_addr;		
		//ret = ar0221_read_reg(client, AR0221_REG_SCCB_ID, AR0221_REG_VALUE_16BIT, &sccb_id);
		//dev_info(dev, "read   addr=0x%02x SCCB_ID = 0x%04x ret=%d \n",client->addr * 2,sccb_id , ret);
        client->addr = addr;
		sccb_id = 0;
        ret = ar0221_read_reg(client, AR0221_REG_SCCB_ID+1, AR0221_REG_VALUE_08BIT, &sccb_id);
        dev_info(dev, "check addr=0x%02x READ SCCB_ID = 0x%02x ret=%d \n",client->addr * 2,sccb_id , ret);
		ret = ar0221_write_reg(client, AR0221_REG_RESET,	AR0221_REG_VALUE_16BIT, reset_init_state);
		ret = ar0221_read_reg(client, AR0221_REG_RESET, AR0221_REG_VALUE_16BIT, &sccb_id);
		dev_info(dev, "lock reset  addr=0x%02x r_val = 0x%04x ret=%d \n",client->addr * 2,sccb_id , ret);
		
	}
    //else
    //{
        //ret = ar0221_read_reg(client, AR0221_REG_SCCB_ID, AR0221_REG_VALUE_16BIT, &sccb_id);
        ret = ar0221_read_reg(client, AR0221_REG_SCCB_ID+1, AR0221_REG_VALUE_08BIT, &sccb_id);

        //sccb_id_high = (sccb_id >> 8) & 0xff;
        //sccb_id_low = (sccb_id >> 0) & 0xff;

        dev_info(dev, "addr=0x%02x Read SCCB_ID = 0x%02x ret=%d \n",addr2,sccb_id,ret);

   // }
/*
    ret = ar0221_write_array(client, sensor_oe_enable_regs);
    if (ret < 0) {
        dev_err(&client->dev,
            "write sensor_oe_enable_regs error\n");
        goto err_power;
    }

*/

	mutex_unlock(&ar0221_power_mutex);
	return 0;

err_i2c_addr:
    dev_err(dev, "write SCCB_ID failed\n");
	if (!IS_ERR(ar0221->pwdn_gpio))
    {
		gpiod_set_value_cansleep(ar0221->pwdn_gpio, 0);  //LOW
        dev_info(dev, "SCCB_ID Error pwdn_gpio_low addr=0x%02x \n",ar0221->client->addr);
    }
	__ar0221_master_power_off(dev);

err_power:
	mutex_unlock(&ar0221_power_mutex);
	return ret;

}

static void __ar0221_power_off(struct ar0221 *ar0221)
{
	struct device *dev = &ar0221->client->dev;
    //u32 read_val;

	mutex_lock(&ar0221_power_mutex);

	if (!IS_ERR(ar0221->pwdn_gpio))
    {
		//ar0221_read_reg(ar0221->client, 0x2000,AR0221_REG_VALUE_16BIT, &read_val);
        //dev_info(&ar0221->client->dev, "Read 0x2000 Vallue=0x%04x \n",read_val);
        //ar0221_read_reg(ar0221->client, 0x2002,AR0221_REG_VALUE_16BIT, &read_val);
        //dev_info(&ar0221->client->dev, "Read 0x2002 Vallue=0x%04x \n",read_val);
        gpiod_set_value_cansleep(ar0221->pwdn_gpio, 0);  //LOW
        dev_info(dev, "Power_off pwdn_gpio_low addr=0x%02x \n",ar0221->client->addr);
    }

	__ar0221_master_power_off(dev);

	mutex_unlock(&ar0221_power_mutex);
}

static int ar0221_runtime_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0221 *ar0221 = to_ar0221(sd);

	return __ar0221_power_on(ar0221);
}

static int ar0221_runtime_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0221 *ar0221 = to_ar0221(sd);

	__ar0221_power_off(ar0221);

	return 0;
}

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static int ar0221_open(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh)
{
	struct ar0221 *ar0221 = to_ar0221(sd);
	struct v4l2_mbus_framefmt *try_fmt =
				v4l2_subdev_get_try_format(sd, fh->pad, 0);
	const struct ar0221_mode *def_mode = &supported_modes[0];

	mutex_lock(&ar0221->mutex);
	/* Initialize try_fmt */
	try_fmt->width = def_mode->width;
	try_fmt->height = def_mode->height;
	try_fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	try_fmt->field = V4L2_FIELD_NONE;

	mutex_unlock(&ar0221->mutex);
	/* No crop or compose */

	return 0;
}
#endif

static const struct dev_pm_ops ar0221_pm_ops = {
	SET_RUNTIME_PM_OPS(ar0221_runtime_suspend,
			   ar0221_runtime_resume, NULL)
};

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
static const struct v4l2_subdev_internal_ops ar0221_internal_ops = {
	.open = ar0221_open,
};
#endif

static const struct v4l2_subdev_core_ops ar0221_core_ops = {
	.ioctl = ar0221_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = ar0221_compat_ioctl32,
#endif
};

static const struct v4l2_subdev_video_ops ar0221_video_ops = {
	.s_stream = ar0221_s_stream,
	.g_frame_interval = ar0221_g_frame_interval,
};

static const struct v4l2_subdev_pad_ops ar0221_pad_ops = {
	.enum_mbus_code = ar0221_enum_mbus_code,
	.enum_frame_size = ar0221_enum_frame_sizes,
	.get_fmt = ar0221_get_fmt,
	.set_fmt = ar0221_set_fmt,
};

static const struct v4l2_subdev_ops ar0221_subdev_ops = {
	.core	= &ar0221_core_ops,
	.video	= &ar0221_video_ops,
	.pad	= &ar0221_pad_ops,
};

static int ar0221_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ar0221 *ar0221 = container_of(ctrl->handler,
					     struct ar0221, ctrl_handler);
	struct i2c_client *client = ar0221->client;
	s64 max;
	int ret = 0;

    //gpiod_set_value(ar0221->pwdn_gpio, 1);
    //dev_info(&ar0221->client->dev, "set ctrl pwdn_gpio_high addr=0x%02x \n",ar0221->client->addr);

	/* Propagate change of current control to all related controls */
	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		/* Update max exposure while meeting expected vblanking */
		max = ar0221->cur_mode->height + ctrl->val - 4;
		__v4l2_ctrl_modify_range(ar0221->exposure,
					 ar0221->exposure->minimum, max,
					 ar0221->exposure->step,
					 ar0221->exposure->default_value);
		break;
	}

	if (pm_runtime_get(&client->dev) <= 0)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* 4 least significant bits of expsoure are fractional part */
        dev_warn(&client->dev, "V4L2_CID_EXPOSURE not yet");
		//ret = ar0221_write_reg(ar0221->client,
		//			AR0221_REG_EXPOSURE,
		//			AR0221_REG_VALUE_24BIT,
		//			ctrl->val << 4);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
            dev_warn(&client->dev, "V4L2_CID_ANALOGUE_GAIN not yet");
		/*
        ret = ar0221_write_reg(ar0221->client,
					AR0221_REG_GAIN_H,
					AR0221_REG_VALUE_16BIT,
					(ctrl->val >> AR0221_GAIN_H_SHIFT) &
					AR0221_GAIN_H_MASK);
		ret |= ar0221_write_reg(ar0221->client,
					 AR0221_REG_GAIN_L,
					 AR0221_REG_VALUE_16BIT,
					 ctrl->val & AR0221_GAIN_L_MASK);
                    */
		break;
	case V4L2_CID_VBLANK:
            dev_warn(&client->dev, "V4L2_CID_VBLANK not yet");
		
        //ret = ar0221_write_reg(ar0221->client,
		//			AR0221_REG_VTS,
		//			AR0221_REG_VALUE_16BIT,
		//			ctrl->val + ar0221->cur_mode->height);
		break;
	case V4L2_CID_TEST_PATTERN:
		//ret = ar0221_enable_test_pattern(ar0221, ctrl->val);
        dev_warn(&client->dev, "test pattern not yet");
		break;
	default:
		dev_warn(&client->dev, "%s Unhandled id:0x%x, val:0x%x\n",
			 __func__, ctrl->id, ctrl->val);
		break;
	}

	pm_runtime_put(&client->dev);

    //usleep_range(500, 1000);
    //gpiod_set_value_cansleep(ar0221->pwdn_gpio, 0);
    //dev_info(&ar0221->client->dev, "set ctrl pwdn_gpio_low addr=0x%02x \n",ar0221->client->addr);

	return ret;
}

static const struct v4l2_ctrl_ops ar0221_ctrl_ops = {
	.s_ctrl = ar0221_set_ctrl,
};

static int ar0221_initialize_controls(struct ar0221 *ar0221)
{
	const struct ar0221_mode *mode;
	struct v4l2_ctrl_handler *handler;
	//struct v4l2_ctrl *ctrl;
	//s64 exposure_max, 
    s64 vblank_def;
    //s64 pixel_rate;
	u32 h_blank;
	int ret;


	handler = &ar0221->ctrl_handler;
	mode = ar0221->cur_mode;
	//ret = v4l2_ctrl_handler_init(handler, 8);
    ret = v4l2_ctrl_handler_init(handler, 1);
	if (ret)
		return ret;

	handler->lock = &ar0221->mutex;

	/* link_freq */
    ar0221->link_freq = v4l2_ctrl_new_int_menu(handler, NULL, V4L2_CID_LINK_FREQ,
				      0, 0, link_freq_menu_items);
	if (ar0221->link_freq)
		ar0221->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	/* pixel_rate */
    //pixel_rate = mode->vts_def * mode->hts_def * mode->max_fps.denominator /mode->max_fps.numerator;
    //ar0221->pixel_rate =
    //    v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE, 0, pixel_rate,
    //              1, pixel_rate);
    ar0221->pixel_rate = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_PIXEL_RATE,
			  0, AR0221_PIXEL_RATE, 1, AR0221_PIXEL_RATE);

    /* h_blank */
	h_blank = mode->hts_def - mode->width;
	ar0221->hblank = v4l2_ctrl_new_std(handler, NULL, V4L2_CID_HBLANK,
				h_blank, h_blank, 1, h_blank);
	if (ar0221->hblank)
		ar0221->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

    /* v_blank */
	vblank_def = mode->vts_def - mode->height;
	ar0221->vblank = v4l2_ctrl_new_std(handler, &ar0221_ctrl_ops,V4L2_CID_VBLANK, 
                vblank_def,                         //min
                AR0221_VTS_MAX - mode->height,      //max
				1,                                  //step
                vblank_def);                        //default
    /* exposure */
    ar0221->exposure = v4l2_ctrl_new_std(handler, &ar0221_ctrl_ops,
                         V4L2_CID_EXPOSURE,
                         AR0221_EXPOSURE_MIN,
                         AR0221_EXPOSURE_MAX,
                         AR0221_EXPOSURE_STEP,
                         AR0221_EXPOSURE_DEFAULT);
    /* anal_gain */
    ar0221->anal_gain =
        v4l2_ctrl_new_std(handler, &ar0221_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
                  AR0221_ANALOG_GAIN_MIN, AR0221_ANALOG_GAIN_MAX,
                  AR0221_ANALOG_GAIN_STEP,
                  AR0221_ANALOG_GAIN_DEFAULT);

	//ar0221->test_pattern = v4l2_ctrl_new_std_menu_items(handler,
	//			&ar0221_ctrl_ops, V4L2_CID_TEST_PATTERN,
	//			ARRAY_SIZE(ar0221_test_pattern_menu) - 1,
	//			0, 0, ar0221_test_pattern_menu);

	if (handler->error) {
		ret = handler->error;
		dev_err(&ar0221->client->dev,
			"Failed to init controls(%d)\n", ret);
		goto err_free_handler;
	}
	ar0221->subdev.ctrl_handler = handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(handler);

	return ret;
}

static int ar0221_check_sensor_id(struct ar0221 *ar0221,
				   struct i2c_client *client)
{
	struct device *dev = &ar0221->client->dev;
	u32 id = 0;
    int counter = 5;
	int ret;


    while(counter--)
    {
    	ret = ar0221_read_reg(client, AR0221_REG_CHIP_ID,
    			       AR0221_REG_VALUE_16BIT, &id);
    	
        //dev_info(dev, "Detected OV%06x sensor, REVISION 0x%x\n", CHIP_ID, id);

        if (id != CHIP_ID) {
    		dev_err(dev, "Unexpected sensor id(0x%04x), ret(%d)\n", id, ret);
    		//return ret;
    	}
        else
        {
            dev_info(dev, "CHIP ID Read OK RD=0x%04x\n", id);
            return 0;
        }

        usleep_range(500, 1000);
    }

    return ret;

/*
	ret = ar0221_read_reg(client, AR0221_CHIP_REVISION_REG,
			       AR0221_REG_VALUE_08BIT, &id);
	if (ret) {
		dev_err(dev, "Read chip revision register error\n");
		return ret;
	}

	if (id == AR0221_R2A)
		ar0221_global_regs = ar0221_global_regs_r2a;
	else
		ar0221_global_regs = ar0221_global_regs_r1a;
	dev_info(dev, "Detected OV%06x sensor, REVISION 0x%x\n", CHIP_ID, id);
*/
	
}

static int ar0221_configure_regulators(struct device *dev)
{
	struct ar0221 *ar0221 = ar0221_master;
	unsigned int i;

	if (!ar0221) {
		dev_err(dev, "no ar0221 master set\n");
		return -EINVAL;
	}
	for (i = 0; i < AR0221_NUM_SUPPLIES; i++)
		ar0221->supplies[i].supply = ar0221_supply_names[i];

	return devm_regulator_bulk_get(dev,
				       AR0221_NUM_SUPPLIES,
				       ar0221->supplies);
}

static void ar0221_detach_master(void *data)
{
	if (ar0221_master == data)
		ar0221_master = NULL;
}

static int ar0221_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct device_node *node = dev->of_node;
	struct ar0221 *ar0221;
	struct v4l2_subdev *sd;
	int ret;

	ar0221 = devm_kzalloc(dev, sizeof(*ar0221), GFP_KERNEL);
	if (!ar0221)
		return -ENOMEM;

	ret = of_property_read_u32(node, RKMODULE_CAMERA_MODULE_INDEX,
				   &ar0221->module_index);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_FACING,
				       &ar0221->module_facing);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_MODULE_NAME,
				       &ar0221->module_name);
	ret |= of_property_read_string(node, RKMODULE_CAMERA_LENS_NAME,
				       &ar0221->len_name);
	if (ret) {
		dev_err(dev, "could not get module information!\n");
		return -EINVAL;
	}

	dev_info(dev, "driver version 0.2 : [%s-%d-%s-%s]", ar0221->module_name, ar0221->module_index, ar0221->module_facing, ar0221->len_name);


	ar0221->client = client;
	ar0221->cur_mode = &supported_modes[0];

	if (!ar0221_master) {
		ar0221_master = ar0221;
		devm_add_action(dev, ar0221_detach_master, ar0221);
	}
	if (ar0221_master == ar0221) {
		ar0221->xvclk = devm_clk_get(dev, "xvclk");
		if (IS_ERR(ar0221->xvclk)) {
			dev_err(dev, "Failed to get xvclk\n");
			return -EINVAL;
		}
		ret = clk_set_rate(ar0221->xvclk, AR0221_XVCLK_FREQ);
		if (ret < 0) {
			dev_err(dev, "Failed to set xvclk rate (24MHz)\n");
			return ret;
		}
		if (clk_get_rate(ar0221->xvclk) != AR0221_XVCLK_FREQ)
			dev_warn(dev, "xvclk mismatched, modes are based on 24MHz\n");

		ar0221->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
		if (IS_ERR(ar0221->reset_gpio))
			dev_warn(dev, "Failed to get reset-gpios\n");

		ret = ar0221_configure_regulators(dev);
		if (ret) {
			dev_err(dev, "Failed to get power regulators\n");
			return ret;
		}

		ar0221->pinctrl = devm_pinctrl_get(dev);
		if (!IS_ERR(ar0221->pinctrl)) {
			ar0221->pins_default =
				pinctrl_lookup_state(ar0221->pinctrl,
						     OF_CAMERA_PINCTRL_STATE_DEFAULT);
			if (IS_ERR(ar0221->pins_default))
				dev_err(dev, "could not get default pinstate\n");

			ar0221->pins_sleep =
				pinctrl_lookup_state(ar0221->pinctrl,
						     OF_CAMERA_PINCTRL_STATE_SLEEP);
			if (IS_ERR(ar0221->pins_sleep))
				dev_err(dev, "could not get sleep pinstate\n");
		}
	}

	ar0221->pwdn_gpio = devm_gpiod_get(dev, "pwdn", GPIOD_OUT_LOW);
	if (IS_ERR(ar0221->pwdn_gpio))
		dev_warn(dev, "Failed to get pwdn-gpios\n");
	else
    {
		gpiod_set_value_cansleep(ar0221->pwdn_gpio, 0);
        dev_info(dev, "probe pwdn_gpio_low addr=0x%02x \n",ar0221->client->addr);
    }
    //ar0221->pwdn_gpio Normal => LOW
    //ar0221->pwdn_gpio Sleep mode => High

	mutex_init(&ar0221->mutex);

    //dev_info(dev, "ar0221_probe\n");

	sd = &ar0221->subdev;
	v4l2_i2c_subdev_init(sd, client, &ar0221_subdev_ops);
	ret = ar0221_initialize_controls(ar0221);
	if (ret)
		goto err_destroy_mutex;

    //dev_info(dev, "ar0221_probe_step1\n");
	ret = __ar0221_power_on(ar0221);
	if (ret)
		goto err_free_handler;
    //dev_info(dev, "ar0221_probe_step2\n");

	ret = ar0221_check_sensor_id(ar0221, client);
	if (ret)
		goto err_power_off;
    //dev_info(dev, "ar0221_probe_step3\n");

#ifdef CONFIG_VIDEO_V4L2_SUBDEV_API
	sd->internal_ops = &ar0221_internal_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
#endif
#if defined(CONFIG_MEDIA_CONTROLLER)
	ar0221->pad.flags = MEDIA_PAD_FL_SOURCE;
	sd->entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	ret = media_entity_init(&sd->entity, 1, &ar0221->pad, 0);
	if (ret < 0)
		goto err_power_off;
#endif

	ret = v4l2_async_register_subdev_sensor_common(sd);
	//ret = v4l2_async_register_subdev(sd); // org 
	if (ret) {
		dev_err(dev, "v4l2 async register subdev failed\n");
		goto err_clean_entity;
	}

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);
	pm_runtime_idle(dev);

	return 0;

err_clean_entity:
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
err_power_off:
	__ar0221_power_off(ar0221);
err_free_handler:
	v4l2_ctrl_handler_free(&ar0221->ctrl_handler);
err_destroy_mutex:
	mutex_destroy(&ar0221->mutex);

	return ret;
}

static int ar0221_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0221 *ar0221 = to_ar0221(sd);

	v4l2_async_unregister_subdev(sd);
#if defined(CONFIG_MEDIA_CONTROLLER)
	media_entity_cleanup(&sd->entity);
#endif
	v4l2_ctrl_handler_free(&ar0221->ctrl_handler);
	mutex_destroy(&ar0221->mutex);

	pm_runtime_disable(&client->dev);
	if (!pm_runtime_status_suspended(&client->dev))
		__ar0221_power_off(ar0221);
	pm_runtime_set_suspended(&client->dev);

	return 0;
}

#if IS_ENABLED(CONFIG_OF)
static const struct of_device_id ar0221_of_match[] = {
	{ .compatible = "onsemi,ar0221" },
	{},
};
MODULE_DEVICE_TABLE(of, ar0221_of_match);
#endif

static const struct i2c_device_id ar0221_match_id[] = {
	{ "onsemi,ar0221", 0 },
	{ },
};

static struct i2c_driver ar0221_i2c_driver = {
	.driver = {
		.name = "ar0221",
		.pm = &ar0221_pm_ops,
		.of_match_table = of_match_ptr(ar0221_of_match),
	},
	.probe		= &ar0221_probe,
	.remove		= &ar0221_remove,
	.id_table	= ar0221_match_id,
};

static int __init sensor_mod_init(void)
{
	return i2c_add_driver(&ar0221_i2c_driver);
}

static void __exit sensor_mod_exit(void)
{
	i2c_del_driver(&ar0221_i2c_driver);
}

device_initcall_sync(sensor_mod_init);
module_exit(sensor_mod_exit);

MODULE_DESCRIPTION("OmniVision ar0221 sensor driver");
MODULE_LICENSE("GPL v2");
