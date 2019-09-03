/* Copyright (c) 2014-2015, The Linux Foundation. All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are
* met:
*    * Redistributions of source code must retain the above copyright
*      notice, this list of conditions and the following disclaimer.
*    * Redistributions in binary form must reproduce the above
*      copyright notice, this list of conditions and the following
*      disclaimer in the documentation and/or other materials provided
*      with the distribution.
*    * Neither the name of The Linux Foundation nor the names of its
*      contributors may be used to endorse or promote products derived
*      from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED "AS IS" AND ANY EXPRESS OR IMPLIED
* WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT
* ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS
* BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
* CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
* SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
* BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
* WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE
* OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN
* IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#ifndef _PANEL_ST7789V_VIDEO_H_
#define _PANEL_ST7789V_VIDEO_H_
/*---------------------------------------------------------------------------*/
/* HEADER files                                                              */
/*---------------------------------------------------------------------------*/
#include "panel.h"

/*---------------------------------------------------------------------------*/
/* Panel configuration                                                       */
/*---------------------------------------------------------------------------*/
static struct panel_config st7789v_video_panel_data = {
	"qcom,mdss_dsi_st7759v_parallel_cmd", "dsi:0:", "qcom,mdss-dsi-panel",
	0, 0, "DISPLAY_1", 0, 0, 60, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/*---------------------------------------------------------------------------*/
/* Panel resolution                                                          */
/*---------------------------------------------------------------------------*/
static struct panel_resolution st7789v_video_panel_res = {
	240, 320, 16, 10, 10, 0, 16, 16, 8, 0, 0, 0, 0, 0, 0, 0, 0
};

/*---------------------------------------------------------------------------*/
/* Panel color information                                                   */
/*---------------------------------------------------------------------------*/
static struct color_info st7789v_video_color = {
	16, 0, 0xff, 0, 0, 0 /*Need to change bpp to 16 */
};

/*---------------------------------------------------------------------------*/
/* Panel on commands                                                   */
/*---------------------------------------------------------------------------*/
static char st7789v_video_on_cmd0[] = {
	0x11, 0x00, 0x05, 0x80,
};

static char st7789v_video_on_cmd1[] = {
	0x02, 0x00, 0x39, 0xC0,
	0x36, 0x00, 0xFF, 0xFF,  };

static char st7789v_video_on_cmd2[] = {
	0x02, 0x00, 0x39, 0xC0,
	0x3A, 0x05, 0xFF, 0xFF,  };

static char st7789v_video_on_cmd3[] = {
	0x06, 0x00, 0x39, 0xC0,
	0xB2, 0x0C, 0x0C, 0x00,
	0x33, 0x33, 0xFF, 0xFF,
};

static char st7789v_video_on_cmd4[] = {
	0x02, 0x00, 0x39, 0xC0,
	0xB7, 0x35, 0xFF, 0xFF,  };

static char st7789v_video_on_cmd5[] = {
	0x02, 0x00, 0x39, 0xC0,
	0xBB, 0x55, 0xFF, 0xFF,  };

static char st7789v_video_on_cmd6[] = {
	0x02, 0x00, 0x39, 0xC0,
	0xC0, 0x2C, 0xFF, 0xFF,  };

static char st7789v_video_on_cmd7[] = {
	0x02, 0x00, 0x39, 0xC0,
	0xC2, 0x01, 0xFF, 0xFF,  };

static char st7789v_video_on_cmd8[] = {
	0x02, 0x00, 0x39, 0xC0,
	0xC3, 0x17, 0xFF, 0xFF,  };

static char st7789v_video_on_cmd9[] = {
	0x02, 0x00, 0x39, 0xC0,
	0xC4, 0x20, 0xFF, 0xFF,  };

static char st7789v_video_on_cmd10[] = {
	0x02, 0x00, 0x39, 0xC0,
	0xC6, 0x0F, 0xFF, 0xFF,  };

static char st7789v_video_on_cmd11[] = {
	0x03, 0x00, 0x39, 0xC0,
	0xD0, 0xA4, 0xA1, 0xFF,
};

static char st7789v_video_on_cmd12[] = {
	0x0F, 0x00, 0x39, 0xC0,
	0xE0, 0xD0, 0x00, 0x14,
	0x15, 0x13, 0x2C, 0x42,
	0x43, 0x4E, 0x09, 0x16,
	0x14, 0x18, 0x21, 0xFF,
};

static char st7789v_video_on_cmd13[] = {
	0x0F, 0x00, 0x39, 0xC0,
	0xE1, 0xD0, 0x00, 0x14,
	0x15, 0x13, 0x0B, 0x43,
	0x55, 0x53, 0x0C, 0x17,
	0x14, 0x23, 0x20, 0xFF,
};

static char st7789v_video_on_cmd131[] = {
	0x05, 0x00, 0x39, 0xC0,
	0x2A, 0x00, 0x00, 0x00,
	0xEF, 0xFF, 0xFF, 0xFF,
};

static char st7789v_video_on_cmd132[] = {
	0x05, 0x00, 0x39, 0xC0,
	0x2B, 0x00, 0x00, 0x01,
	0x3F, 0xFF, 0xFF, 0xFF,
};

static char st7789v_video_on_cmd14[] = {
	0x29, 0x00, 0x05, 0x32,
};

static char st7789v_video_on_cmd15[] = {
	0x2C, 0x00, 0x05, 0x32,
};

static struct mipi_dsi_cmd st7789v_video_on_command[] = {
	{0x4, st7789v_video_on_cmd0, 0xc8},
	{0x8, st7789v_video_on_cmd1, 0x00},
	{0x8, st7789v_video_on_cmd2, 0x00},
	{0xc, st7789v_video_on_cmd3, 0x00},
	{0x8, st7789v_video_on_cmd4, 0x00},
	{0x8, st7789v_video_on_cmd5, 0x00},
	{0x8, st7789v_video_on_cmd6, 0x00},
	{0x8, st7789v_video_on_cmd7, 0x00},
	{0x8, st7789v_video_on_cmd8, 0x00},
	{0x8, st7789v_video_on_cmd9, 0x00},
	{0x8, st7789v_video_on_cmd10, 0x00},
	{0x8, st7789v_video_on_cmd11, 0x00},
	{0x14, st7789v_video_on_cmd12, 0x00},
	{0x14, st7789v_video_on_cmd13, 0x00},
	{0xc, st7789v_video_on_cmd131, 0x00},
	{0xc, st7789v_video_on_cmd132, 0x00},
	{0x4, st7789v_video_on_cmd14, 0x32},
	{0x4, st7789v_video_on_cmd15, 0x32},
};

#define ST7789V_VIDEO_ON_COMMAND 18

/*---------------------------------------------------------------------------*/
/* Panel off commands                                                         */
/*---------------------------------------------------------------------------*/
static char st7789v_videooff_cmd0[] = {
	0x2C, 0x00, 0x05, 0xC0,
};

static char st7789v_videooff_cmd1[] = {
	0x28, 0x00, 0x05, 0xC0,
};

static char st7789v_videooff_cmd2[] = {
	0x10, 0x00, 0x05, 0xC0,
};

static struct mipi_dsi_cmd st7789v_video_off_command[] = {
	{0x04, st7789v_videooff_cmd0, 0x32},
	{0x04, st7789v_videooff_cmd1, 0x32},
	{0x04, st7789v_videooff_cmd2, 0x78}
};

#define ST7789V_VIDEO_OFF_COMMAND 3

static struct command_state st7789v_video_state = {
	0, 0
};

/*---------------------------------------------------------------------------*/
/* Command mode panel information                                            */
/*---------------------------------------------------------------------------*/
static struct commandpanel_info st7789v_video_command_panel = {
	0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};

/*---------------------------------------------------------------------------*/
/* Video mode panel information                                              */
/*---------------------------------------------------------------------------*/
static struct videopanel_info st7789v_video_panel = {
	1, 0, 0, 0, 1, 1, 0, 0, 0x9
};

/*---------------------------------------------------------------------------*/
/* Lane configuration                                                        */
/*---------------------------------------------------------------------------*/
static struct lane_configuration st7789v_video_lane_config = {
	1, 0, 1, 0, 0, 0, 0
};

/*---------------------------------------------------------------------------*/
/* Panel timing                                                              */
/*---------------------------------------------------------------------------*/
static const uint32_t st7789v_video_timings[] = {
	0xED, 0x0A, 0x02, 0x00, 0x20, 0x24, 0x06, 0x0C, 0x09, 0x03, 0x04, 0x00
};

static struct panel_timing st7789v_video_timing_info = {
	0, 4, 0x05, 0x36
};

static struct mipi_dsi_cmd st7789v_video_rotation[] = {

};

/*---------------------------------------------------------------------------*/
/* Panel reset sequence                                                      */
/*---------------------------------------------------------------------------*/
static struct panel_reset_sequence st7789v_video_reset_seq = {
	{1, 0, 1, }, {20, 20, 20, }, 2
};

/*---------------------------------------------------------------------------*/
/* Backlight setting                                                         */
/*---------------------------------------------------------------------------*/
static struct backlight st7789v_video_backlight = {
	0, 0, 100, 100, 1, "PMIC_8941"
};

#define ST7789V_VIDEO_SIGNATURE 0xFFFF

#endif /*_PANEL_ST7789V_VIDEO_H_*/
