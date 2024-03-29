/* Copyright (c) 2008-2010, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include "msm_fb.h"
#include "mipi_dsi.h"
#include "mipi_s6e8ab0_wxga.h"
#include "mipi_s6e8ab0_wxga_seq.h"
#include "mdp4_video_enhance.h"
#include "smart_dimming_s6e8ab0.h"

#include <linux/gpio.h>

#ifdef CONFIG_CMC624_P8LTE
#include "pxlte_cmc624.h"
#endif

static void set_backlight(struct msm_fb_data_type *mfd);
#define LCDC_DEBUG
//#define LCD_FACTORY_TEST

//#define MIPI_SINGLE_WRITE

#ifdef LCDC_DEBUG
#define DPRINT(x...)	printk("[Mipi_LCD] " x)
#else
#define DPRINT(x...)	
#endif

static char log_buffer[256] = {0,};
#define LOG_START()	do { log_buffer[0] = 0; } while(0)
#define LOG_CLEAR()	do { log_buffer[0] = 0; } while(0)
#define LOG_ADD(x...)	do { sprintf( log_buffer +strlen(log_buffer), x ); } while(0)
#define LOG_EXIST()	(strlen(log_buffer)>0)
#define LOG_FINISH(x...)	do { if( strlen(log_buffer) != 0 ) DPRINT( "%s\n", log_buffer); LOG_START(); } while(0)

///////////MAX
#define BRIGHTLIMITVALUE 171
#define BRIGHTLIMITVALUE2 151
int BrightLimitValue = BRIGHTLIMITVALUE;
int FlagMaxBrightLimit = 0;
int savLastBrightness = 0;
void set_lcd_MaxLimit();
void set_lcd_MaxLimit2();
void unset_lcd_MaxLimit();
//////////

#define MUTEX_LOCK(a) do { DPRINT("MUTEX LOCK : Line=%d, %x\n", __LINE__, a); mutex_lock(a);} while(0)
#define MUTEX_UNLOCK(a) do { DPRINT("MUTEX unlock : Line=%d, %x\n", __LINE__, a); mutex_unlock(a);} while(0)

#define GPIO_S6E8AB0_RESET (28)

#define LCD_OFF_GAMMA_VALUE (0)
#define DEFAULT_CANDELA_VALUE (150)  // ŷڼ spec: Quincy  150cd

#define ACL_OFF_GAMMA_VALUE (60)
//elvss
#define ELVSS_OFFSET_MAX		0x00
#define ELVSS_OFFSET_2		0x04
#define ELVSS_OFFSET_1		0x08
#define ELVSS_OFFSET_MIN		0x0C
#define ELVSS_OFFSET_DYN_MIN	0x0B

#define DYNAMIC_ELVSS_MIN_VALUE		0x81
#define DYNAMIC_ELVSS_MAX_VALUE_32	0xA1
#define DYNAMIC_ELVSS_MAX_VALUE_24	0xA9

enum lcd_id_value {
	lcd_id_unknown = -1,
	lcd_id_a1_line = 1,
	lcd_id_a2_line,
	lcd_id_material_M2,
	lcd_id_material_M3,
	lcd_id_material_M3_cond_3,
	lcd_id_material_M3_cond_1,
	lcd_id_material_M3_cond_12,
	lcd_id_material_M3_cond_13,
	lcd_id_material_M3_cond_14,
	lcd_id_material_SM2,
	lcd_id_material_SM2_cond_15,
	lcd_id_material_Future,
 	lcd_id_IC_S6E8AB0,
 	lcd_id_lcdDCDC_TPS65138,
 	lcd_id_lcdDCDC_TPS65138A,
 	lcd_id_boardDCDC_TPS65138,
 	lcd_id_boardDCDC_TPS65138A,
 	lcd_id_elvss_normal,
 	lcd_id_elvss_each,
  };


struct lcd_state_type{
	boolean initialized;
	boolean display_on;
	boolean powered_up;
};

#define LCD_ID_BUFFER_SIZE (8)

struct lcd_setting {
	struct device			*dev;
	struct spi_device		*spi;
	unsigned int			power;
	unsigned int			gamma_mode;
	unsigned int			lcd_gamma; // lcd has this value
	unsigned int			stored_gamma; // driver has this value
	unsigned int			gamma_table_count;
	unsigned int			lcd_elvss; // lcd has this value
	unsigned int			stored_elvss; // driver has this value
	unsigned int			beforepower;
	unsigned int			ldi_enable;
	unsigned int 			enabled_acl; // lcd has this value
	unsigned int 			stored_acl; // driver has this value
	unsigned int 			lcd_acl; // lcd has this value
	unsigned int 			goal_brightness;
	struct mutex	lock;
	struct lcd_device		*ld;
	struct backlight_device		*bd;
	struct lcd_platform_data	*lcd_pd;
	struct early_suspend    early_suspend;
	struct lcd_state_type lcd_state;
	char	lcd_id_buffer[LCD_ID_BUFFER_SIZE];
	enum lcd_id_value	factory_id_line;
	enum lcd_id_value	factory_id_material;
	enum lcd_id_value	factory_id_IC;
	enum lcd_id_value	factory_id_elvssType;
	enum lcd_id_value	factory_id_lcdDCDC;
	enum lcd_id_value	factory_id_boardDCDC;
	struct dsi_cmd_desc_LCD *lcd_brightness_table;
	struct dsi_cmd_desc_LCD *lcd_acl_table;
	struct dsi_cmd_desc_LCD *lcd_elvss_table;
	int	eachElvss_value;
	int 	eachElvss_limit;
	int	eachElvss_vector;
	int	IdPannel;

	boolean	isSmartDimming;
	boolean	isSmartDimming_loaded;
	struct str_smart_dim smart;
#ifdef MAPPING_TBL_AUTO_BRIGHTNESS
	unsigned int			auto_brightness;
#endif
};

static struct lcd_setting s6e8ab0_lcd;
//


//void set_lcd_esd_ignore( boolean src );
extern int sec_esd_disable(void); //sec_mipi_lcd_esd_refresh.c
extern int sec_esd_enable(void); //sec_mipi_lcd_esd_refresh.c

extern void cmc_esd_disable(void);// sec_cmc_esd_refresh.c
extern void cmc_esd_enable(void);// sec_cmc_esd_refresh.c
int state_lcd_on = 0; //extern
int lcd_esd_refresh = 0; //extern
static void lcd_set_brightness(struct msm_fb_data_type *mfd, int level);
static void lcd_gamma_apply( struct msm_fb_data_type *mfd, int srcGamma);
static void lcd_gamma_smartDimming_apply( struct msm_fb_data_type *mfd, int srcGamma);
static int lcd_apply_elvss(struct msm_fb_data_type *mfd, int srcElvss, struct lcd_setting *lcd);
//static void lcd_apply_acl(struct msm_fb_data_type *mfd, unsigned int srcAcl, boolean nowOffNeedOn);
static void lcd_set_acl(struct msm_fb_data_type *mfd, struct lcd_setting *lcd);
static int lcd_set_elvss(struct msm_fb_data_type *mfd, struct lcd_setting *lcd);
static void lcd_gamma_ctl(struct msm_fb_data_type *mfd, struct lcd_setting *lcd);

static struct msm_panel_common_pdata *pdata;
static struct msm_fb_data_type *pMFD;

#undef  SmartDimming_16bitBugFix
#define SmartDimming_useSampleValue
#define SmartDimming_CANDELA_UPPER_LIMIT (299)
#define MTP_DATA_SIZE (24)
#define MTP_REGISTER	(0xD3)
unsigned char lcd_mtp_data[MTP_DATA_SIZE+16] = {0,};
#ifdef SmartDimming_useSampleValue
boolean	isFAIL_mtp_load = FALSE;
const unsigned char lcd_mtp_data_default [MTP_DATA_SIZE] = {
0x00, 0x00, 0x00 ,0x04, 0x02, 0x05, 
0x06, 0x05, 0x08, 0x03, 0x02, 0x03, 
0x02, 0x01, 0x01, 0xfa, 0xfe, 0xfa, 
0x00, 0x14, 0x00, 0x0e, 0x00, 0x0d,
};
#endif // SmartDimming_useSampleValue

static struct dsi_buf s6e8ab0_tx_buf;
static struct dsi_buf s6e8ab0_rx_buf;

static char AUTO_POW_ON[3] = { 0xF0, 0x5A, 0x5A };
static char AUTO_POW_ON_2[3] = { 0xF1, 0x5A, 0x5A };

static char ETC_COND_SET_0[7]= { 0xF4, 0x0B, 0x0A, 0x06, 0x0B, 0x33, 0x02 };
static char ETC_COND_SET_1[4] = { 0xF6, 0x04, 0x00, 0x02 };
static char ETC_COND_SET_2[22] = {
	0xD9,
	0x14, 0x5C, 0x20, 0x0C, 0x0F, 0x41, 0x00, 0x10, 0x11, 0x12,
	0xA8, 0xD5, 0x00, 0x00, 0x00, 0x00, 0x80, 0xCB, 0xED, 0x7F,
	0xEF
};

static char ELVSS_DEFAULT[3] = {0xB1, 0x04, 0x00};
static char ELVSS_DYNAMIC[3] = {0xB1, 0x84, 0x15};
static char VREGOUT_SET[12] = {0xD1,0x00, 0x00, 0x00, 0x01, 0x0B, 0x00, 0x00, 0x40, 0x0d, 0x00, 0x00}; //id1~3 


static char ETC_COND_SET_2_1[4] = {0xF6, 0x00, 0x02, 0x00};
static char ETC_COND_SET_2_3[3] = {0xB1, 0x04, 0x00};
static char ETC_COND_SET_2_4[10] = {0xB6, 0x0C, 0x02, 0x03, 0x32, 0xC0, 0x44, 0x44, 0xC0, 0x00};
static char ETC_COND_SET_2_7[8] = {0xF4, 0xCF, 0x0A, 0x12, 0x10, 0x19, 0x33, 0x03 };

static char PANEL_COND_SET[41] = {
	0xF8,
	0x01, 0x8E, 0x00, 0x00, 0x00, 0xAC, 0x00, 0x9E, 0x8D, 0x1F, 
	0x4E, 0x9C, 0x7D, 0x3F, 0x10, 0x00, 0x20, 0x02, 0x10, 0x7D, 
	0x10, 0x00, 0x00, 0x02, 0x08, 0x10, 0x34, 0x34, 0x34, 0xC0, 
	0xC1, 0x01, 0x00, 0xC1, 0x82, 0x00, 0xC8, 0xC1, 0xE3, 0x01
};

static char DISPLAY_COND_SET[4] = { 0xF2, 0xC8, 0x05, 0x0D };

static char GAMMA_COND_SET[25] = {
	0xFA,
	0x1F, 0x00, 0x1F, 0xE5, 0x00, 0xE2, 0xCF, 0xCC, 0xC6, 0xD3,
	0xD7, 0xCF, 0xAE, 0xB6, 0xA7, 0xBF, 0xC4, 0xB9, 0x00, 0x94,
	0x00, 0x9F, 0x00, 0xCE
};
//acl
const unsigned char SEQ_ACL_ON[] = {
	0xC0, 0x01,
};

const unsigned char SEQ_ACL_OFF[] = {
	0xC0, 0x00,
};

const unsigned char SEQ_ACL_CUTOFF_40[] = {
	0xC1,
	0x4D, 0x96, 0x1D, 0x00, 0x00, 0x04, 0xFF, 0x00, 0x00, 0x03,
	0x1F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x21, 0x37,
	0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x37, 0x1D, 0x4D, 0x96, 
};

static char exit_sleep[2] = {0x11, 0x00};
static char display_on[2] = {0x29, 0x00};
static char display_off[2] = {0x28, 0x00};
static char enter_sleep[2] = {0x10, 0x00};
static char gamma_select_cmd[2] = {0x26, 0x01};
static char gamma_update_cmd[2] = {0xF7, 0x01};
static char acl_off_cmd[2] = {0xC0, 0x00};
static char acl_on_cmd[2] = {0xC0, 0x01};
//static char tear_off[2] = {0x34, 0x00};
//static char tear_on [2] = {0x35, 0x00};
//static char all_pixel_off_cmd[2] = {0x22, 0x00};
//static char all_pixel_on_cmd[2] = {0x23, 0x00};
//static char reset_cmd[2] = {0x01, 0x00};

static struct dsi_cmd_desc s6e8ab0_display_off_cmds[] = {
	{DTYPE_DCS_WRITE, 1, 0, 0, 10, sizeof(display_off), display_off},
	{DTYPE_DCS_WRITE, 1, 0, 0, 120, sizeof(enter_sleep), enter_sleep}
};

static struct dsi_cmd_desc s6e8ab0_gamma_update_cmd = {
	DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_update_cmd), gamma_update_cmd};


static struct dsi_cmd_desc s6e8ab0_acl_off_cmd = {
	DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(acl_off_cmd), acl_off_cmd};
static struct dsi_cmd_desc s6e8ab0_acl_on_cmd = {
	DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(acl_on_cmd), acl_on_cmd};

#if 0
static struct dsi_cmd_desc s6e8ab0_display_on_cmds[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(AUTO_POW_ON), AUTO_POW_ON},
    {DTYPE_DCS_WRITE, 1, 0, 0, 10, sizeof(exit_sleep), exit_sleep},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(AUTO_POW_ON_2), AUTO_POW_ON_2},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(PANEL_COND_SET), PANEL_COND_SET},    
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(AUTO_POW_ON_2), AUTO_POW_ON_2},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(DISPLAY_COND_SET), DISPLAY_COND_SET},    
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(GAMMA_COND_SET), GAMMA_COND_SET},    
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_update_cmd), gamma_update_cmd},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(AUTO_POW_ON_2), AUTO_POW_ON_2},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ETC_COND_SET_2_1), ETC_COND_SET_2_1},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ETC_COND_SET_2_3), ETC_COND_SET_2_3},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ETC_COND_SET_2_4), ETC_COND_SET_2_4},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ETC_COND_SET_2_7), ETC_COND_SET_2_7},    
    {DTYPE_GEN_LWRITE, 1, 0, 0, 120, sizeof(ACL_COND_SET_40), ACL_COND_SET_40},    
    {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(display_on), display_on},
};
#endif
static struct dsi_cmd_desc s6e8ab0_wxga_display_on[] ={
	{DTYPE_DCS_WRITE,  1, 0, 0, 0,   sizeof(display_on), display_on},
	{DTYPE_GEN_LWRITE, 1, 0, 0, 0,   sizeof(VREGOUT_SET), VREGOUT_SET}
};

static struct dsi_cmd_desc s6e8ab0_display_on_before_read_id[] = {
    {DTYPE_GEN_LWRITE, 1, 0, 0, 16, sizeof(AUTO_POW_ON), AUTO_POW_ON},
//    {DTYPE_DCS_WRITE, 1, 0, 0, 16, sizeof(exit_sleep), exit_sleep},
};

static struct dsi_cmd_desc s6e8ab0_display_on_before_gamma_cmds[] = {
    {DTYPE_DCS_WRITE,  1, 0, 0, 5, sizeof(exit_sleep), exit_sleep},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(AUTO_POW_ON_2), AUTO_POW_ON_2},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(PANEL_COND_SET), PANEL_COND_SET},    
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(AUTO_POW_ON_2), AUTO_POW_ON_2},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(DISPLAY_COND_SET), DISPLAY_COND_SET},    
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_select_cmd), gamma_select_cmd},
//    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ETC_COND_SET_2_1), ETC_COND_SET_2_1},
//    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ETC_COND_SET_2_3), ETC_COND_SET_2_3},
//    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ETC_COND_SET_2_4), ETC_COND_SET_2_4},
//    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ETC_COND_SET_2_7), ETC_COND_SET_2_7},    
};

static struct dsi_cmd_desc s6e8ab0_display_on_after_gamma_cmds[] = {
	// warning : Don't insert delay time. BrightnessChange in this cmds cannot be apply.
//    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(gamma_update_cmd), gamma_update_cmd},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(AUTO_POW_ON_2), AUTO_POW_ON_2},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ETC_COND_SET_0), ETC_COND_SET_0},    
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ETC_COND_SET_1), ETC_COND_SET_1},
    {DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(ELVSS_DEFAULT), ELVSS_DEFAULT}, //5-1
    {DTYPE_GEN_LWRITE, 1, 0, 0, 120, sizeof(ETC_COND_SET_2), ETC_COND_SET_2},
  //  {DTYPE_DCS_WRITE, 1, 0, 0, 0, sizeof(display_on), display_on},
};

static inline boolean isPANEL_COND_SET( char* src )
{
	if( src == PANEL_COND_SET )
	return TRUE;
	else
	return FALSE;
}

#if 0
char* get_s6e8ab0_id_buffer( void )
{
	return s6e8ab0_lcd.lcd_id_buffer;
}
#endif

static void update_LCD_SEQ_by_id(struct lcd_setting *destLCD)
{
	int	PANEL_COND_SET_dlen_NEW = 0;
	char* PANEL_COND_SET_NEW = NULL;
	int updateCnt = 0;
	int i;

	LOG_START();
	LOG_ADD( "%s :", __func__ );

	updateCnt = 0;

	switch(destLCD->factory_id_material)
	{
	case lcd_id_material_SM2_cond_15:
		destLCD->lcd_brightness_table = lcd_Gamma22_SM2_Meterial15_table;
		LOG_ADD( " G22SM2M15" );
		break;

	case lcd_id_material_M3_cond_14:
		destLCD->lcd_brightness_table = lcd_Gamma22_M3_Meterial14_table;
		LOG_ADD( " G22SM14" );
		break;

	default :		
		destLCD->lcd_brightness_table = lcd_Gamma22_Meterial13_table;
		LOG_ADD( " G22M13" );
		break;
	}
	
	PANEL_COND_SET_dlen_NEW = sizeof(PANEL_COND_SET);
	PANEL_COND_SET_NEW = PANEL_COND_SET;

#if 0
	for( i = 0; i < ARRAY_SIZE( s6e8ab0_display_on_cmds ); i++ )
	{
		if( isPANEL_COND_SET(s6e8ab0_display_on_cmds[i].payload))
		{
			s6e8ab0_display_on_cmds[i].dlen	= PANEL_COND_SET_dlen_NEW;
			s6e8ab0_display_on_cmds[i].payload	= PANEL_COND_SET_NEW;
			updateCnt++;
			LOG_ADD( " c%d", i );
		}
	}
#endif

	for( i = 0; i < ARRAY_SIZE( s6e8ab0_display_on_before_gamma_cmds ); i++ )
	{
		if( isPANEL_COND_SET(s6e8ab0_display_on_before_gamma_cmds[i].payload))
		{
			s6e8ab0_display_on_before_gamma_cmds[i].dlen	= PANEL_COND_SET_dlen_NEW;
			s6e8ab0_display_on_before_gamma_cmds[i].payload	= PANEL_COND_SET_NEW;
			updateCnt++;
			LOG_ADD( " b%d", i );
		}
	}

	// smartDimming data initialiazing
	if( destLCD->isSmartDimming && s6e8ab0_lcd.isSmartDimming_loaded )
	{

		switch(destLCD->factory_id_material)
		{
		case lcd_id_material_SM2:
			init_table_info( &(s6e8ab0_lcd.smart), gamma_300cd_sm2_panel );
			break;

		case lcd_id_material_M3:
			init_table_info( &(s6e8ab0_lcd.smart), gamma_300cd_m3_panel );
			break;

		default:
			init_table_info( &(s6e8ab0_lcd.smart), gamma_300cd_m3_panel );
			break;
		}
		
		calc_voltage_table(&(s6e8ab0_lcd.smart), lcd_mtp_data);
		LOG_ADD( " i" );
	}

	LOG_ADD( "\n" );
	LOG_FINISH();
}


void update_id( char* srcBuffer, struct lcd_setting *destLCD )
{
	char pBuffer[96] = {0,};
	unsigned char checkValue;
	enum lcd_id_value boardDCDC;
	enum lcd_id_value lcdDCDC;
	int	cntBuf;
	int	i;

	LOG_START();
	LOG_ADD( "Update LCD ID:" );
	
	// make read-data valid
	if( srcBuffer[0] == 0x00 )
	{
		for( i = 0; i < LCD_ID_BUFFER_SIZE-2; i ++ ) srcBuffer[i] = srcBuffer[i+2];
	}
	destLCD->isSmartDimming = FALSE;
	
	checkValue = srcBuffer[0];
	switch( checkValue )
	{
		case 0xA1:
			destLCD->factory_id_line = lcd_id_a1_line;
			LOG_ADD( " A1-Line" );
		break;
		case 0xA2:
			destLCD->factory_id_line = lcd_id_a2_line;
			LOG_ADD( " A2-Line" );		
		break;
		default :
			destLCD->factory_id_line = lcd_id_unknown;
			LOG_ADD( " A?-Line" );
		break;				
	} // end switch
	LOG_ADD( "(0x%X)", srcBuffer[0] );

	checkValue = srcBuffer[1];
	switch( checkValue )
	{
		case 0x03:
			destLCD->factory_id_material = lcd_id_material_M2;
			LOG_ADD( " M2" );
		break;
		case 0x25:
			destLCD->factory_id_material = lcd_id_material_M3;
			destLCD->isSmartDimming = true;			
			LOG_ADD( " M3" );
		break;
		case 0x23:
			destLCD->factory_id_material = lcd_id_material_M3_cond_3;
			LOG_ADD( " M3C3" );
		break;
		case 0x33:
			destLCD->factory_id_material = lcd_id_material_M3_cond_1;
			LOG_ADD( " M3C1" );
		break;
		case 0x43:
			destLCD->factory_id_material = lcd_id_material_M3_cond_12;
			LOG_ADD( " M3C12" );
		break;
		case 0x53:
			destLCD->factory_id_material = lcd_id_material_M3_cond_13;
			LOG_ADD( " M3C13" );
		break;
		case 0x63:
			destLCD->factory_id_material = lcd_id_material_M3_cond_14;
			LOG_ADD( " M3C14" );
		break;
		case 0x73:
			destLCD->factory_id_material = lcd_id_material_SM2_cond_15;
			destLCD->isSmartDimming = true;
			LOG_ADD( " SM2C15" );
		break;
		case 0x15:
			destLCD->factory_id_material = lcd_id_material_SM2;
			destLCD->isSmartDimming = true;
			LOG_ADD( " SM2" );
		break;
		case 0x83:
			destLCD->factory_id_material = lcd_id_material_SM2_cond_15;
			destLCD->isSmartDimming = true;
			LOG_ADD( " SM2C15PR" );
		break;
		case 0x93:
			destLCD->factory_id_material = lcd_id_material_SM2_cond_15;
			destLCD->isSmartDimming = true;
			LOG_ADD( " SM2C15PR1" );
		break;
		
		case 0x94 ... 0xFF:
			destLCD->factory_id_material = lcd_id_material_Future;
			LOG_ADD( " MF" );
		break;
		default :
			destLCD->factory_id_material = lcd_id_unknown;
			destLCD->isSmartDimming = true;			
			LOG_ADD( " M?" );
		break;				
	} // end switch
	LOG_ADD( "(0x%X)", srcBuffer[1] );
	destLCD->IdPannel = (int)checkValue;

	checkValue = srcBuffer[2];
	switch( checkValue )
	{

		case 0x44:
			destLCD->factory_id_elvssType = lcd_id_elvss_normal;
			break;
			
		default :
			destLCD->factory_id_elvssType = lcd_id_elvss_each;
			destLCD->eachElvss_value = (checkValue & 0x3F); 
			destLCD->eachElvss_limit = (checkValue & 0xc0) >> 6; ; 			
			LOG_ADD( " Ielvss(%x)", destLCD->eachElvss_value );
			break;
	/*
		case 0x33:
			destLCD->factory_id_elvssType = lcd_id_elvss_normal;
		break;
		case 0x01 ... 0x1F : // case : B6B5 = 00
			destLCD->factory_id_elvssType = lcd_id_elvss_each;
			destLCD->eachElvss_value = checkValue; // ELVSS 2
			LOG_ADD( " Ielvss(%x)", destLCD->eachElvss_value );
		break;
		case 0x41 ... 0x5F : // case : B6B5 = 01
			destLCD->factory_id_elvssType = lcd_id_elvss_each;
			destLCD->eachElvss_value = (checkValue & 0x3F); // ELVSS 2
			LOG_ADD( " Ielvss(%x)", destLCD->eachElvss_value );
		break;
		default :
			destLCD->factory_id_elvssType = lcd_id_elvss_each;
			LOG_ADD( " Ielvss" );
		break;*/
	} // end switch
	// Dynamic elvss configuration
//	destLCD->factory_id_lcdDCDC = (checkValue & 0x80)? lcd_id_lcdDCDC_TPS65138A : lcd_id_lcdDCDC_TPS65138;

#if 0
#if 1
	if( destLCD->factory_id_boardDCDC == lcd_id_unknown ) { // kyNam_110827_
		destLCD->factory_id_elvssType = lcd_id_elvss_normal;
	}
	else {
		destLCD->eachElvss_value = checkValue & 0x3F;

		boardDCDC = destLCD->factory_id_boardDCDC;
		lcdDCDC	= destLCD->factory_id_lcdDCDC;
		if( lcdDCDC == lcd_id_lcdDCDC_TPS65138 && boardDCDC == lcd_id_boardDCDC_TPS65138 ) 
			destLCD->eachElvss_vector = 0;
		else if ( lcdDCDC == lcd_id_lcdDCDC_TPS65138A && boardDCDC == lcd_id_boardDCDC_TPS65138A ) 
			destLCD->eachElvss_vector = 0;
		else if ( lcdDCDC == lcd_id_lcdDCDC_TPS65138 && boardDCDC == lcd_id_boardDCDC_TPS65138A ) 
			destLCD->eachElvss_vector = -10;
		else if ( lcdDCDC == lcd_id_lcdDCDC_TPS65138A && boardDCDC == lcd_id_boardDCDC_TPS65138 ) 
			destLCD->eachElvss_vector = +10;

		destLCD->factory_id_elvssType = lcd_id_elvss_each;
		LOG_ADD( " Elvss:%x_%d", destLCD->eachElvss_value, destLCD->eachElvss_vector );
	}
#else // each DELVSS test code (Forcely)
	{
		destLCD->eachElvss_value = 0x90 & 0x3F;// kyNam_110827_

		boardDCDC = destLCD->factory_id_boardDCDC;
		lcdDCDC	= destLCD->factory_id_lcdDCDC;
		if( lcdDCDC == lcd_id_lcdDCDC_TPS65138 && boardDCDC == lcd_id_boardDCDC_TPS65138 ) 
			destLCD->eachElvss_vector = 0;
		else if ( lcdDCDC == lcd_id_lcdDCDC_TPS65138A && boardDCDC == lcd_id_boardDCDC_TPS65138A ) 
			destLCD->eachElvss_vector = 0;
		else if ( lcdDCDC == lcd_id_lcdDCDC_TPS65138 && boardDCDC == lcd_id_boardDCDC_TPS65138A ) 
			destLCD->eachElvss_vector = -10;
		else if ( lcdDCDC == lcd_id_lcdDCDC_TPS65138A && boardDCDC == lcd_id_boardDCDC_TPS65138 ) 
			destLCD->eachElvss_vector = +10;

		destLCD->factory_id_elvssType = lcd_id_elvss_each;
		LOG_ADD( " Elvss:%x_%d", destLCD->eachElvss_value, destLCD->eachElvss_vector );
	}
#endif 
#endif
	switch( destLCD->factory_id_material )
	{
		// need to addl code 
		default:	// if Error-case, Not Change 
		break;
	}

	LOG_FINISH();
}

void read_reg( char srcReg, int srcLength, char* destBuffer, const int isUseMutex )
{
	const int	one_read_size = 4;
	const int	loop_limit = 16;

	static char read_reg[2] = {0xFF, 0x00}; // first byte = read-register
	static struct dsi_cmd_desc s6e8ab0_read_reg_cmd = 
	{ DTYPE_DCS_READ, 1, 0, 1, 5, sizeof(read_reg), read_reg};

	static char packet_size[] = { 0x04, 0 }; // first byte is size of Register
	static struct dsi_cmd_desc s6e8ab0_packet_size_cmd = 
	{ DTYPE_MAX_PKTSIZE, 1, 0, 0, 0, sizeof(packet_size), packet_size};

	static char reg_read_pos[] = { 0xB0, 0x00 }; // second byte is Read-position
	static struct dsi_cmd_desc s6e8ab0_read_pos_cmd = 
	{ DTYPE_DCS_WRITE1, 1, 0, 0, 0, sizeof(reg_read_pos), reg_read_pos};

	int read_pos;
	int readed_size;
	int show_cnt;

	int i,j;

#ifdef CONFIG_CMC624_P8LTE
	if(p8lte_has_cmc624())
	{
		p8lte_get_reg_data(srcReg, destBuffer, srcLength);
		return;
	}
#endif

	read_reg[0] = srcReg;

	LOG_START();
	LOG_ADD( "read_reg : %X[%d] : ", srcReg, srcLength );

	read_pos = 0;
	readed_size = 0;
	if( isUseMutex ) mutex_lock(&(s6e8ab0_lcd.lock));

//	MIPI_OUTP(MIPI_DSI_BASE + 0x38, 0x14000000); // lp

	packet_size[0] = (char) srcLength;
	mipi_dsi_cmds_tx(pMFD, &s6e8ab0_tx_buf, &(s6e8ab0_packet_size_cmd),	1);

	show_cnt = 0;
	for( j = 0; j < loop_limit; j ++ )
	{
		reg_read_pos[1] = read_pos;
		if( mipi_dsi_cmds_tx(pMFD, &s6e8ab0_tx_buf, &(s6e8ab0_read_pos_cmd), 1) < 1 ) {
			LOG_ADD( "Tx command FAILED" );
			break;
		}

		mipi_dsi_buf_init(&s6e8ab0_rx_buf);
		readed_size = mipi_dsi_cmds_rx(pMFD, &s6e8ab0_tx_buf, &s6e8ab0_rx_buf, &s6e8ab0_read_reg_cmd, one_read_size);
		for( i=0; i < readed_size; i++, show_cnt++) 
		{
			LOG_ADD( "%02x ", s6e8ab0_rx_buf.data[i] );
			if( destBuffer != NULL && show_cnt < srcLength ) {
				destBuffer[show_cnt] = s6e8ab0_rx_buf.data[i];
			} // end if
		} // end for
		LOG_ADD( "." );
		read_pos += readed_size;
		if( read_pos > srcLength ) break;
	} // end for
//	MIPI_OUTP(MIPI_DSI_BASE + 0x38, 0x10000000); // hs
	if( isUseMutex ) mutex_unlock(&(s6e8ab0_lcd.lock));
	if( j == loop_limit ) LOG_ADD( "Overrun" );

	LOG_FINISH();
	
}

#ifdef LCD_FACTORY_TEST
static lcd_read_mtp( char* pDest, const int isUseMutex )
{
	int i, j;
 	char pBuffer[256];
 	char temp_lcd_mtp_data[MTP_DATA_SIZE+16] = {0,};

 	j = 0;
 	read_reg( MTP_REGISTER, 8, temp_lcd_mtp_data, FALSE );
 	for( i = 2; i<8; i++ ) pDest[j++] = temp_lcd_mtp_data[i];
 		read_reg( MTP_REGISTER, 16, temp_lcd_mtp_data, FALSE );
 	for( i = 0; i<16; i++ ) pDest[j++] = temp_lcd_mtp_data[i];
 		read_reg( MTP_REGISTER, 24, temp_lcd_mtp_data, FALSE );
 	for( i = 8; i<10; i++ ) pDest[j++] = temp_lcd_mtp_data[i];
 	{
  		sprintf( pBuffer, "read_reg MTP :" );
  		for( i = 0; i < MTP_DATA_SIZE; i++ ) sprintf( pBuffer+strlen(pBuffer), " %02x", pDest[i] );
  			DPRINT( "%s\n", pBuffer );
 	}
}
#endif

static int make_gamma_table(void)
{
	int ret = 0;
	int i;
	u32 gamma;

	for(i = 0 ; i < MAX_GAMMA_VALUE ; i++)
	{
#ifdef MAPPING_TBL_AUTO_BRIGHTNESS
	    gamma = candela_table[i];
#else
		gamma = s6e8ab0_lcd.lcd_brightness_table[i].lux;
#endif
		if( gamma > SmartDimming_CANDELA_UPPER_LIMIT ) gamma = SmartDimming_CANDELA_UPPER_LIMIT;

		memcpy(GAMMA_SmartDimming_COND_SET_table[i], GAMMA_SmartDimming_COND_SET, sizeof(GAMMA_SmartDimming_COND_SET));
		calc_gamma_table(&(s6e8ab0_lcd.smart), gamma, &GAMMA_SmartDimming_COND_SET_table[i][SmartDimming_GammaUpdate_Pos]);
	}

	return ret;
}

static int lcd_on_seq(struct msm_fb_data_type *mfd)
{
	int	txCnt;
	int	txAmount;
	int	ret = 0;
	volatile int	isFailed_TX = FALSE;
	unsigned int	acl_level;

	int i;
	char pBuffer[256];
	
	// in result of test, gamma update can update before gammaTableCommand of LCD-ON-sequence.
	// if update gamma after gammaTableCommand cannot be update to lcd
	// if update gamma after displayOnCommand cannot be update lcd.
	// so gammaTableCommand is moved to ahead of displayOnCommand.


	if( s6e8ab0_lcd.lcd_state.initialized )	{
		DPRINT("%s : Already ON -> return 1\n", __func__ );
		ret = 1;
	}
	else {
		mutex_lock(&(s6e8ab0_lcd.lock));		
		DPRINT("%s +\n", __func__);

		txAmount = ARRAY_SIZE(s6e8ab0_display_on_before_read_id);
		txCnt = mipi_dsi_cmds_tx(mfd, &s6e8ab0_tx_buf, s6e8ab0_display_on_before_read_id,
				txAmount );    
		if( txAmount != txCnt )	
		{
			isFailed_TX = TRUE;
			goto failed_tx_lcd_on_seq;
		}

#ifdef LCD_FACTORY_TEST
		read_reg( 0xD1, LCD_ID_BUFFER_SIZE, s6e8ab0_lcd.lcd_id_buffer, FALSE );

		lcd_read_mtp( lcd_mtp_data, FALSE );
		s6e8ab0_lcd.isSmartDimming_loaded = TRUE;

		update_id( s6e8ab0_lcd.lcd_id_buffer, &s6e8ab0_lcd );

		update_LCD_SEQ_by_id( &s6e8ab0_lcd );

#else
		if( !s6e8ab0_lcd.isSmartDimming_loaded )
		{
			read_reg( MTP_REGISTER, MTP_DATA_SIZE, lcd_mtp_data, FALSE );
#if 0 // kyNam_110915_ do not check MTP-Fail-check			
			for( i=0, j=0; i < MTP_DATA_SIZE; i++ ) if(lcd_mtp_data[i]==0) j++;
			if( j > MTP_DATA_SIZE/2 )
			{
				DPRINT( "MTP Load FAIL\n" );
				s6e8ab0_lcd.isSmartDimming_loaded = FALSE;
				isFAIL_mtp_load = TRUE;

#ifdef SmartDimming_useSampleValue
				for( i = 0; i < MTP_DATA_SIZE; i++ )	 lcd_mtp_data[i] = lcd_mtp_data_default[i];
				s6e8ab0_lcd.isSmartDimming_loaded = TRUE;
#endif 				
			} 
			else 
#endif 			
			{
				LOG_START();
				LOG_ADD( "mtp :" );
				for( i = 0; i < MTP_DATA_SIZE; i++ )	LOG_ADD( " %02x", lcd_mtp_data[i] );
				LOG_FINISH();

				s6e8ab0_lcd.isSmartDimming_loaded = TRUE;
			}
		}

		if( s6e8ab0_lcd.factory_id_line == lcd_id_unknown )
		{
			read_reg( 0xD1, LCD_ID_BUFFER_SIZE, s6e8ab0_lcd.lcd_id_buffer, FALSE );
			update_id( s6e8ab0_lcd.lcd_id_buffer, &s6e8ab0_lcd );

			printk(KERN_WARNING " It goes to ID unknown!\n");
			update_LCD_SEQ_by_id( &s6e8ab0_lcd );
			make_gamma_table();
		}
#endif

		txAmount = ARRAY_SIZE(s6e8ab0_display_on_before_gamma_cmds);
		txCnt = mipi_dsi_cmds_tx(mfd, &s6e8ab0_tx_buf, s6e8ab0_display_on_before_gamma_cmds,
				txAmount );    

		if( txAmount != txCnt )	
		{
			isFailed_TX = TRUE;
			goto failed_tx_lcd_on_seq;
		}

		// update stored-gamma value to gammaTable

		LOG_START();
		if( s6e8ab0_lcd.isSmartDimming )
		{
			lcd_gamma_smartDimming_apply( mfd, s6e8ab0_lcd.stored_gamma );
		} 
		else
		{
			lcd_gamma_apply( mfd, s6e8ab0_lcd.stored_gamma );
		}
		s6e8ab0_lcd.lcd_gamma = s6e8ab0_lcd.stored_gamma;
		
		mipi_dsi_cmds_tx(mfd, &s6e8ab0_tx_buf, s6e8ab0_display_on_after_gamma_cmds,
				ARRAY_SIZE(s6e8ab0_display_on_after_gamma_cmds));    

		s6e8ab0_lcd.lcd_state.initialized = TRUE;
		s6e8ab0_lcd.lcd_state.display_on =  state_lcd_on = TRUE;
		s6e8ab0_lcd.lcd_state.powered_up = TRUE;


		// set elvss
		lcd_set_elvss(mfd, &s6e8ab0_lcd);
		s6e8ab0_lcd.lcd_elvss = s6e8ab0_lcd.stored_elvss;

		VREGOUT_SET[1] = s6e8ab0_lcd.lcd_id_buffer[0];
		VREGOUT_SET[2] = s6e8ab0_lcd.lcd_id_buffer[1];		
		VREGOUT_SET[3] = s6e8ab0_lcd.lcd_id_buffer[2];
		mipi_dsi_cmds_tx(mfd, &s6e8ab0_tx_buf, s6e8ab0_wxga_display_on,
				ARRAY_SIZE(s6e8ab0_wxga_display_on));  

		// set acl 
		if( s6e8ab0_lcd.enabled_acl )
		{
			s6e8ab0_lcd.stored_acl = s6e8ab0_lcd.lcd_gamma; //bl level
			lcd_set_acl(mfd, &s6e8ab0_lcd);
		}
		LOG_FINISH();
		DPRINT("%s - : gamma=%d, ACl=%d, Elvss=%d\n", __func__, s6e8ab0_lcd.lcd_gamma, s6e8ab0_lcd.lcd_acl, s6e8ab0_lcd.lcd_elvss );

failed_tx_lcd_on_seq:
		if( isFailed_TX ) {
			s6e8ab0_lcd.factory_id_line = lcd_id_unknown;
			DPRINT("%s : tx FAILED\n", __func__ );
		}

#ifdef CONFIG_CMC624_P8LTE
		if(p8lte_has_cmc624())
		{
			sec_esd_enable(); // esd driver
			cmc_esd_enable(); // esd driver
		}
#endif		

	mutex_unlock(&(s6e8ab0_lcd.lock));
	}

	return ret;
}

static int lcd_on(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

#ifdef MAPPING_TBL_AUTO_BRIGHTNESS
	DPRINT("[%s] + MAPPING_TBL_AUTO_BRIGHTNESS\n", __func__);
#else
	DPRINT("[%s] +\n", __func__);
#endif

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;
	pMFD = mfd;

	lcd_on_seq( mfd );

	if (s6e8ab0_lcd.goal_brightness != s6e8ab0_lcd.stored_gamma)
		lcd_set_brightness(mfd, s6e8ab0_lcd.goal_brightness);
	
//	readCmd(mfd, DTYPE_GEN_READ, 0x05, 0x00, 8, 1 );
//	readCmd(mfd, DTYPE_GEN_READ, 0x0A, 0x00, 8, 1 );
//	readCmd(mfd, DTYPE_GEN_READ, 0x0D, 0x00, 8, 1 );
//	readCmd(mfd, DTYPE_GEN_READ, 0x0E, 0x00, 8, 1 );
	DPRINT("[%s] -\n", __func__);
	return 0;
}


static int lcd_off_seq(struct msm_fb_data_type *mfd)
{
	if( !s6e8ab0_lcd.lcd_state.initialized )
	{
		DPRINT("%s : Already OFF -> return 1\n", __func__ );
		return 1;
	}

	DPRINT("%s\n", __func__);

#ifdef CONFIG_CMC624_P8LTE
	if(p8lte_has_cmc624())
	{
		 sec_esd_disable();
		 cmc_esd_disable();
	}
#endif
	mutex_lock(&(s6e8ab0_lcd.lock));
//	set_lcd_esd_ignore( TRUE );
	mipi_dsi_cmds_tx(mfd, &s6e8ab0_tx_buf, s6e8ab0_display_off_cmds,
			ARRAY_SIZE(s6e8ab0_display_off_cmds));
	s6e8ab0_lcd.lcd_state.display_on = state_lcd_on = FALSE;
	s6e8ab0_lcd.lcd_state.initialized = FALSE;
	s6e8ab0_lcd.lcd_state.powered_up = FALSE;
//	set_lcd_esd_ignore( FALSE );
	mutex_unlock(&(s6e8ab0_lcd.lock));

	//gpio_set_value(GPIO_S6E8AB0_RESET, 0);
	//msleep(120);

	return 0;
}


static int lcd_off(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

#ifdef CONFIG_HAS_EARLYSUSPEND
	struct fb_info *info;
	unsigned short *bits;
	if(!lcd_esd_refresh)	
	{	
	DPRINT("%s : Draw Black screen.\n", __func__);	
	info = registered_fb[0];
	bits = (unsigned short *)(info->screen_base);
#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER	
	memset(bits, 0x00, 1280*800*4*3); /*info->var.xres*info->var.yres*/
#else
	memset(bits, 0x00, 1280*800*4*2);
#endif
	}
#if 0 // for recovery ICS (andre.b.kim)
	DPRINT("%s : to be early_suspend. return.\n", __func__);
	return 0;
#endif
#endif 

    DPRINT("%s +\n", __func__);

	mfd = platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;
	pMFD = mfd;

	lcd_off_seq(mfd);
	DPRINT("%s -\n", __func__);
	return 0;
}

static int __devinit lcd_shutdown(struct platform_device *pdev)
{
	struct msm_fb_data_type *mfd;

	DPRINT("%s +\n", __func__);

	mfd = pMFD;//platform_get_drvdata(pdev);
	if (!mfd)
		return -ENODEV;
	if (mfd->key != MFD_KEY)
		return -EINVAL;
//	pMFD = mfd;

	lcd_off_seq(mfd);
	gpio_set_value(GPIO_S6E8AB0_RESET, 0);
//	mipi_S6E8AB0_panel_power(FALSE);

	DPRINT("%s -\n", __func__);
	return 0;
}

static void lcd_gamma_apply( struct msm_fb_data_type *mfd, int srcGamma)
{
	LOG_ADD( " GAMMA(%d=%dcd)", srcGamma, s6e8ab0_lcd.lcd_brightness_table[srcGamma].lux );
	mipi_dsi_cmds_tx(mfd, &s6e8ab0_tx_buf, s6e8ab0_lcd.lcd_brightness_table[srcGamma].cmd,	1);
	mipi_dsi_cmds_tx(mfd, &s6e8ab0_tx_buf, &s6e8ab0_gamma_update_cmd, 1);
}



static void lcd_gamma_smartDimming_apply( struct msm_fb_data_type *mfd, int srcGamma)
{
	int gamma_lux;
	int i;

	DPRINT("[%s] +\n", __func__);

#if 1
	if(srcGamma >= MAX_GAMMA_VALUE)	srcGamma = MAX_GAMMA_VALUE-1;
	if(srcGamma < 0) srcGamma = 0;
	DSI_CMD_SmartDimming_GAMMA.payload = GAMMA_SmartDimming_COND_SET_table[srcGamma];
#else

#ifdef MAPPING_TBL_AUTO_BRIGHTNESS
    gamma_lux = candela_table[srcGamma];
#else
	gamma_lux = s6e8ab0_lcd.lcd_brightness_table[srcGamma].lux;
#endif

	if( gamma_lux > SmartDimming_CANDELA_UPPER_LIMIT ) gamma_lux = SmartDimming_CANDELA_UPPER_LIMIT;

	for( i = SmartDimming_GammaUpdate_Pos; i < sizeof(GAMMA_SmartDimming_COND_SET); i++ ) 	GAMMA_SmartDimming_COND_SET[i] = 0;
	calc_gamma_table(&(s6e8ab0_lcd.smart), gamma_lux, GAMMA_SmartDimming_COND_SET+SmartDimming_GammaUpdate_Pos);

#ifdef SmartDimming_16bitBugFix
	if( gamma_lux > SmartDimming_CANDELA_UPPER_LIMIT /2 )
	{
		if( (int)GAMMA_SmartDimming_COND_SET[21] < 0x70 && GAMMA_SmartDimming_COND_SET[20] == 0 ) GAMMA_SmartDimming_COND_SET[20] =1;
		if( (int)GAMMA_SmartDimming_COND_SET[23] < 0x70 && GAMMA_SmartDimming_COND_SET[22] == 0 ) GAMMA_SmartDimming_COND_SET[22] =1;
		if( (int)GAMMA_SmartDimming_COND_SET[25] < 0x70 && GAMMA_SmartDimming_COND_SET[24] == 0 ) GAMMA_SmartDimming_COND_SET[24] =1;
	}
#endif 	
#endif

#if 0 // don't need log 
	char pBuffer[256];
	pBuffer[0] = 0;
	for( i =0; i < sizeof(GAMMA_SmartDimming_COND_SET); i++ )
	{
		sprintf( pBuffer+ strlen(pBuffer), " %02x", GAMMA_SmartDimming_COND_SET[i] );
	}
	DPRINT( "SD: %03d %s\n", gamma_lux, pBuffer );
#endif 
	mipi_dsi_cmds_tx(mfd, &s6e8ab0_tx_buf, &DSI_CMD_SmartDimming_GAMMA,	1);
	mipi_dsi_cmds_tx(mfd, &s6e8ab0_tx_buf, &s6e8ab0_gamma_update_cmd, 1);


	LOG_ADD( " SDIMMING(%d=%dcd)", srcGamma, gamma_lux );
}


static void lcd_gamma_ctl(struct msm_fb_data_type *mfd, struct lcd_setting *lcd)
{
	int gamma_level;

	gamma_level = s6e8ab0_lcd.stored_gamma;
	if(s6e8ab0_lcd.lcd_gamma != gamma_level)
	{

		if( s6e8ab0_lcd.isSmartDimming )
		{
			mutex_lock(&(s6e8ab0_lcd.lock));
			lcd_gamma_smartDimming_apply( mfd, gamma_level );
			mutex_unlock(&(s6e8ab0_lcd.lock));
		} 
		else {
			mutex_lock(&(s6e8ab0_lcd.lock));
			lcd_gamma_apply( mfd, gamma_level );
			mutex_unlock(&(s6e8ab0_lcd.lock));
		}

		s6e8ab0_lcd.lcd_gamma = gamma_level;
		/*
#ifdef MAPPING_TBL_AUTO_BRIGHTNESS
        LOG_ADD(" gamma_ctl(%d->%d(lcd),%dcd)", gamma_level, s6e8ab0_lcd.lcd_gamma, 
        candela_table[s6e8ab0_lcd.lcd_gamma] );
#else
		LOG_ADD(" gamma_ctl(%d->%d(lcd),%s,%dcd)", gamma_level, s6e8ab0_lcd.lcd_gamma, 
			s6e8ab0_lcd.lcd_brightness_table[s6e8ab0_lcd.lcd_gamma].strID,
			s6e8ab0_lcd.lcd_brightness_table[s6e8ab0_lcd.lcd_gamma].lux );
		
#endif*/
	}
	else
	{
		s6e8ab0_lcd.lcd_gamma = gamma_level;
		//DPRINT("lcd_gamma_ctl %d -> %d(lcd)\n", gamma_level, s6e8ab0_lcd.lcd_gamma );
	}

	return;

}

static void lcd_set_brightness(struct msm_fb_data_type *mfd, int gamma_level)
{
	//TODO: lock
       //spin_lock_irqsave(&bl_ctrl_lock, irqflags);

   s6e8ab0_lcd.stored_gamma = gamma_level;
   s6e8ab0_lcd.stored_elvss= gamma_level;
   s6e8ab0_lcd.stored_acl = gamma_level;
   s6e8ab0_lcd.goal_brightness = gamma_level;
   
	lcd_gamma_ctl(mfd, &s6e8ab0_lcd);
	lcd_set_elvss(mfd, &s6e8ab0_lcd);
	if(s6e8ab0_lcd.enabled_acl && 
		((!s6e8ab0_lcd.lcd_acl &&  s6e8ab0_lcd.stored_acl > 1) ||
		(s6e8ab0_lcd.lcd_acl &&  s6e8ab0_lcd.stored_acl < 2)) )lcd_set_acl(mfd, &s6e8ab0_lcd);

#if !defined(CONFIG_HAS_EARLYSUSPEND)
	if (s6e8ab0_lcd.lcd_gamma <= LCD_OFF_GAMMA_VALUE) {
		lcd_off_seq( mfd );	// brightness = 0 -> LCD Off 
	}
#endif 

	//TODO: unlock
	//spin_unlock_irqrestore(&bl_ctrl_lock, irqflags);

}


#define DIM_BL 20
#define MIN_BL 30
#define MAX_BL 255

static int get_gamma_value_from_bl(int bl_value )// same as Seine, Celox 
{
	int gamma_value =0;
	int gamma_val_x10 =0;

	if(bl_value >= MIN_BL){
#ifdef MAPPING_TBL_AUTO_BRIGHTNESS	

		if (unlikely(!s6e8ab0_lcd.auto_brightness && bl_value > 250))
			bl_value = 250;
  
        	switch (bl_value) {
        	case 0 ... 29:
        		gamma_value = 0; // 30cd
        		break;
        	case 30 ... 254:
                gamma_value = (bl_value - candela_table[0]) / 10;
       		break;
        	case 255:
                gamma_value = CANDELA_TABLE_SIZE - 1;
    		break;
				
        	default:
              DPRINT(" >>> bl_value:%d , do not gamma_update\n ",bl_value);
              break;
        	}
      
	        DPRINT(" >>> bl_value:%d,gamma_value: %d\n ",bl_value,gamma_value);
#else

		gamma_val_x10 = 10 *(MAX_GAMMA_VALUE-1)*bl_value/(MAX_BL-MIN_BL) + (10 - 10*(MAX_GAMMA_VALUE-1)*(MIN_BL)/(MAX_BL-MIN_BL));
		gamma_value=(gamma_val_x10 +5)/10;
		
#endif			
	}else{
		gamma_value =0;
	}

	return gamma_value;
}

static void set_backlight(struct msm_fb_data_type *mfd)
{	
	int bl_level = mfd->bl_level;
	int gamma_level;
	static int pre_bl_level = 0;

	DPRINT("[%s] (%d)+\n", __func__, bl_level);

	// brightness tuning
//	bl_level = 201; // 240cd
//	bl_level = 211; // 250cd	
///////////MAX
	savLastBrightness = bl_level;
	if(FlagMaxBrightLimit)
	{
		if(bl_level > BrightLimitValue)
		{
			DPRINT("MaxBrightLimit -> 200/180cd %d\n", BrightLimitValue);
			return;
		}
	}
//////////	
	gamma_level = get_gamma_value_from_bl(bl_level);

	if ((s6e8ab0_lcd.lcd_state.initialized) 
		&& (s6e8ab0_lcd.lcd_state.powered_up) 
		&& (s6e8ab0_lcd.lcd_state.display_on))
	{
		if(s6e8ab0_lcd.lcd_gamma != gamma_level)
		{
			LOG_START();
			lcd_set_brightness(mfd, gamma_level);
			LOG_FINISH();
		}
		else
		{
		//	LOG_START();
		//	lcd_set_brightness(mfd, gamma_level);
		//	LOG_CLEAR();
		}
	}
	else
	{
	       s6e8ab0_lcd.stored_gamma = gamma_level;
	       s6e8ab0_lcd.stored_elvss = gamma_level;
	       s6e8ab0_lcd.stored_acl = gamma_level;
	       s6e8ab0_lcd.goal_brightness  = gamma_level;
       	DPRINT("brightness(Ignore_OFF) %d(bl)->gamma=%d stored=%d\n",bl_level,gamma_level,s6e8ab0_lcd.stored_gamma);
	}

	pre_bl_level = bl_level;
	DPRINT("[%s] -\n", __func__);

}

///////////MAX
void set_lcd_MaxLimit()
{
	int tmpValue;
	FlagMaxBrightLimit = 1;
	BrightLimitValue = BRIGHTLIMITVALUE;
	if(savLastBrightness > BrightLimitValue)
	{
		tmpValue = pMFD->bl_level;
		pMFD->bl_level = 	BrightLimitValue-5;
		set_backlight(pMFD);
		pMFD->bl_level = tmpValue;
	}
	DPRINT("MaxBrightLimit -> 200cd\n");
}
void set_lcd_MaxLimit2()
{
	int tmpValue;
	FlagMaxBrightLimit = 1;
	BrightLimitValue = BRIGHTLIMITVALUE2;
	if(savLastBrightness > BrightLimitValue)
	{
		tmpValue = pMFD->bl_level; 
		pMFD->bl_level = 	BrightLimitValue-5;
		set_backlight(pMFD);
		pMFD->bl_level = tmpValue;
	}
	DPRINT("MaxBrightLimit -> 180cd\n");
}

void unset_lcd_MaxLimit()
{
	FlagMaxBrightLimit = 0;
	set_backlight(pMFD);
	DPRINT("MaxBrightLimit -> expired\n");
}
//////////

u8 get_offset_brightness(u32 brightness)
{
	u8 offset = 0;

	switch (brightness) {
	case 0 ... 7:
		if(s6e8ab0_lcd.factory_id_elvssType == lcd_id_elvss_each )offset = ELVSS_OFFSET_MIN; // 100cd ~
		else if((s6e8ab0_lcd.factory_id_elvssType == lcd_id_elvss_normal ))offset = ELVSS_OFFSET_DYN_MIN;
		break;
	case 8 ... 12:
		offset = ELVSS_OFFSET_1; //150 ~ 100cd
		break;
	case 13 ... 17:
		offset = ELVSS_OFFSET_2; //200 ~ 150cd
		break;
	case 18 ...  23:
		offset = ELVSS_OFFSET_MAX; //250 ~ 200cd
		break;
	default:
		offset = ELVSS_OFFSET_MAX;
		break;
	}
	return offset;
}

static  int lcd_apply_elvss(struct msm_fb_data_type *mfd, int srcElvss, struct lcd_setting *lcd)
{
	int i = 0;
	static char elvss_cmd[3] = {0,};
	static char elvss_cmd_dynamic[6] = {0,};
	
	static struct dsi_cmd_desc s6e8aa0_elvss_cmd = 
	{ DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(elvss_cmd), elvss_cmd};
	static struct dsi_cmd_desc s6e8aa0_elvss_table_dynamic[] = 
	{{DTYPE_GEN_LWRITE, 1, 0, 0, 0,   sizeof(ELVSS_DYNAMIC), ELVSS_DYNAMIC}, 
	{ DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(elvss_cmd_dynamic), elvss_cmd_dynamic}};

	if (!srcElvss) {
		printk("[ERROR:LCD]:%s:get_elvss_value() failed\n", __func__);
		return -1;
	}
	DPRINT("[%s] +\n", __func__);

	if( lcd->factory_id_elvssType == lcd_id_elvss_each )
	{
		elvss_cmd[0] = 0xb1;
		elvss_cmd[1] = 0x84;
		elvss_cmd[2] = srcElvss;

		mipi_dsi_cmds_tx(mfd, &s6e8ab0_tx_buf, &(s6e8aa0_elvss_cmd),	1);
	//	printk("huggac : apply elvss : %x\n", srcElvss);
	}

	if( lcd->factory_id_elvssType == lcd_id_elvss_normal )
	{
		elvss_cmd_dynamic[i] = 0xb2; // addresss
		for(i = 1; i < 6 ; i++)elvss_cmd_dynamic[i] = srcElvss; //data

		mipi_dsi_cmds_tx(mfd, &s6e8ab0_tx_buf, &(s6e8aa0_elvss_table_dynamic),	1);			
	}
	
}

u8 old_elvss_ref = 0;
static  int lcd_set_elvss(struct msm_fb_data_type *mfd, struct lcd_setting *lcd)
{

	u8 ref = 0;
	u8 elvss_limit_min, elvss_limit_max;

	if( lcd->factory_id_elvssType == lcd_id_elvss_each )// id[2] != 0x44
	{
		ref = (lcd->eachElvss_value + 0x80);// + 0x80
		if (lcd->eachElvss_limit == 0x00) 
			{
				elvss_limit_max = DYNAMIC_ELVSS_MAX_VALUE_32;
				elvss_limit_min = DYNAMIC_ELVSS_MIN_VALUE;
			}
		else if (lcd->eachElvss_limit == 0x01)
			{
				elvss_limit_max = DYNAMIC_ELVSS_MAX_VALUE_24;
				elvss_limit_min = DYNAMIC_ELVSS_MIN_VALUE;
			}
		else {
			printk("[ERROR:ELVSS]:%s undefined elvss limit value :%x\n", __func__, lcd->eachElvss_limit);
			return 0;
			}
		ref += get_offset_brightness(s6e8ab0_lcd.stored_elvss); //0x80 + 0x0c~0x04
		
		if(old_elvss_ref == ref)return 0; 
		else old_elvss_ref = ref;

		printk("%s : elvss bl-level : %d, bl-ref : %x\n", __func__, s6e8ab0_lcd.stored_elvss,ref);

		if (ref < elvss_limit_min)	   ref = elvss_limit_min;
		else if (ref > elvss_limit_max)ref = elvss_limit_max;		

	}

	else if( lcd->factory_id_elvssType == lcd_id_elvss_normal ) // id[2] == 0x44
	{
		ref = get_offset_brightness(s6e8ab0_lcd.stored_elvss);
	}

//	mutex_lock(&(s6e8ab0_lcd.lock));
	lcd_apply_elvss( mfd, ref, &s6e8ab0_lcd );
//	mutex_unlock(&(s6e8ab0_lcd.lock));	

	return 0;
}


const unsigned char *ACL_CUTOFF_TABLE[2] = {
	SEQ_ACL_OFF,
	SEQ_ACL_CUTOFF_40,
};

static void lcd_set_acl(struct msm_fb_data_type *mfd, struct lcd_setting *lcd)
{
	static struct dsi_cmd_desc s6e8aa0_seq_acl_off = 
	{ DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(SEQ_ACL_OFF), SEQ_ACL_OFF};
	static struct dsi_cmd_desc s6e8aa0_seq_acl_on = 
	{ DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(SEQ_ACL_ON), SEQ_ACL_ON};
	static struct dsi_cmd_desc s6e8aa0_acl_cutoff_table = 
	{ DTYPE_GEN_LWRITE, 1, 0, 0, 0, sizeof(SEQ_ACL_CUTOFF_40), SEQ_ACL_CUTOFF_40};

/*
	unsigned int 			enabled_acl; // enable
	unsigned int 			stored_acl; // bl level
	unsigned int 			lcd_acl; //current 40%/off status
*/
	if (lcd->enabled_acl) {
		switch (lcd->stored_acl) //bl level
			{
			case 0 ... 1: // 30cd ~ 40cd 
				if (lcd->lcd_acl != 0) {  // 40%
					mipi_dsi_cmds_tx(mfd, &s6e8ab0_tx_buf, &(s6e8aa0_seq_acl_off),	1); 
					lcd->lcd_acl = 0;					
					}
					break;
					
			default: // more than 40cd  
				if (lcd->lcd_acl != 40) {  // not 40% 
					mipi_dsi_cmds_tx(mfd, &s6e8ab0_tx_buf, &(s6e8aa0_seq_acl_on),	1);
					mipi_dsi_cmds_tx(mfd, &s6e8ab0_tx_buf, &(s6e8aa0_acl_cutoff_table),	1);				
					lcd->lcd_acl = 40;					
					}
					break;
			}
	} 
	
	else {
		mipi_dsi_cmds_tx(mfd, &s6e8ab0_tx_buf, &(s6e8aa0_seq_acl_off),	1);
		lcd->lcd_acl = 0;
	}
	printk("%s acl_enable:%d , bl-level:%d, cur_acl:%d\n",__func__, lcd->enabled_acl, lcd->stored_acl, lcd->lcd_acl);	
}


/////////////////[
struct class *sysfs_lcd_class;
struct device *sysfs_panel_dev;

static ssize_t power_reduce_show(struct device *dev, struct 
device_attribute *attr, char *buf)
{
//	struct ld9040 *lcd = dev_get_drvdata(dev);
	char temp[3];

	sprintf(temp, "%d\n", s6e8ab0_lcd.enabled_acl);
	strcpy(buf, temp);

	return strlen(buf);
}

static ssize_t power_reduce_store(struct device *dev, struct 
device_attribute *attr, const char *buf, size_t size)
{
	int value;
	int rc;
	struct msm_fb_data_type *mfd;
	mfd = dev_get_drvdata(dev);

	rc = strict_strtoul(buf, (unsigned int) 0, (unsigned long *)&value);
	if (rc < 0) return rc;
	else {
		if (s6e8ab0_lcd.enabled_acl != value) {
			s6e8ab0_lcd.enabled_acl = value;
			//			if (lcd->ldi_enable)
			{         
				LOG_START();
				LOG_ADD("acl_set_store(changed) : %d -> ", value);	
				lcd_set_acl(pMFD, &s6e8ab0_lcd);    // Arimy to make func
				LOG_FINISH();
			}
		}
		return size;
	}
}

static DEVICE_ATTR(power_reduce, 0664,
		power_reduce_show, power_reduce_store);

static ssize_t lcd_type_show(struct device *dev, struct 
device_attribute *attr, char *buf)
{

	char temp[15];
	sprintf(temp, "SMD_AMS529HA01\n");
	strcat(buf, temp);
	return strlen(buf);
}

static DEVICE_ATTR(lcd_type, 0664,
		lcd_type_show, NULL);
#if 0
static ssize_t octa_lcdtype_show(struct device *dev, struct 
device_attribute *attr, char *buf)
{
	char pBuffer[64] = {0,};
	char cntBuffer;

	cntBuffer = 0;
	cntBuffer += sprintf(pBuffer +cntBuffer, "WXGA");

	switch(s6e8ab0_lcd.factory_id_line )
	{
	case lcd_id_a1_line: 
		cntBuffer += sprintf(pBuffer +cntBuffer, " A1");
	break;
	
	case lcd_id_a2_line: 
		cntBuffer += sprintf(pBuffer +cntBuffer, " A2");
	break;

	case lcd_id_unknown: 
	default:
		cntBuffer += sprintf(pBuffer +cntBuffer, " A?");
	break;
	} // end switch

	switch(s6e8ab0_lcd.factory_id_material )
	{
	case lcd_id_material_M2:
			cntBuffer += sprintf( pBuffer +cntBuffer, " M2" );
		break;
	case lcd_id_material_M3_cond_3:
			cntBuffer += sprintf( pBuffer +cntBuffer, " M3C3" );
		break;
	case lcd_id_material_M3_cond_1:
			cntBuffer += sprintf( pBuffer +cntBuffer, " M3C1" );
		break;
	case lcd_id_material_M3_cond_12:
			cntBuffer += sprintf( pBuffer +cntBuffer, " M3C12" );
		break;
	case lcd_id_material_M3_cond_13:
			cntBuffer += sprintf( pBuffer +cntBuffer, " M3C13" );
		break;
	case lcd_id_material_M3_cond_14:
			cntBuffer += sprintf( pBuffer +cntBuffer, " M3C14" );
		break;
	case lcd_id_material_SM2_cond_15:
			cntBuffer += sprintf( pBuffer +cntBuffer, " SM2C15" );
		break;
	case lcd_id_material_Future:
			cntBuffer += sprintf( pBuffer +cntBuffer, " MF" );
		break;
	case lcd_id_unknown:
	default :
			cntBuffer += sprintf( pBuffer +cntBuffer, " M?" );
		break;				
	} // end switch
/*
	switch(s6e8ab0_lcd.factory_id_IC )
	{
	case lcd_id_IC_S6E8AB0: 
		cntBuffer += sprintf(pBuffer +cntBuffer, " S6E8AB0");
	break;
	
	case lcd_id_unknown: 
	default:
		cntBuffer += sprintf(pBuffer +cntBuffer, " IC?");
	break;
	} // end switch
*/
		cntBuffer += sprintf(pBuffer +cntBuffer, " 0x%x", s6e8ab0_lcd.IdPannel);
	switch(s6e8ab0_lcd.factory_id_elvssType)
	{
	case lcd_id_elvss_normal:
		cntBuffer += sprintf(pBuffer +cntBuffer, " elvss");
	break;
	
	case lcd_id_elvss_each:
		cntBuffer += sprintf(pBuffer +cntBuffer, " IElvss(%x)", s6e8ab0_lcd.eachElvss_value);
	break;

	case lcd_id_unknown: 
	default:
		cntBuffer += sprintf(pBuffer +cntBuffer, " ?elvss");
	break;
	} // end switch

#ifdef SmartDimming_useSampleValue
	if( isFAIL_mtp_load )
	{
		if(s6e8ab0_lcd.isSmartDimming)
			cntBuffer += sprintf(pBuffer +cntBuffer, " SMARTDIM");
	} else 				
#endif 				
	if(s6e8ab0_lcd.isSmartDimming)
		cntBuffer += sprintf(pBuffer +cntBuffer, " SmartDim");


	strcpy( buf, pBuffer );
	
	return strlen(buf);
}

static DEVICE_ATTR(octa_lcdtype, 0664,
		octa_lcdtype_show, NULL);
#endif
static ssize_t octa_mtp_show(struct device *dev, struct 
device_attribute *attr, char *buf)
{
	char pBuffer[160] = {0,};
	char cntBuffer;
	int i;

	cntBuffer = 0;

	cntBuffer += sprintf(pBuffer +cntBuffer, "mtp : ");
	for( i = 0; i < MTP_DATA_SIZE; i++ )	
	{ 
		cntBuffer += sprintf(pBuffer +cntBuffer, " %02x", lcd_mtp_data[i] );
		if( (i+1) % (IV_MAX+1) == 0 ) cntBuffer += sprintf(pBuffer +cntBuffer, "." );  // because, one of mtp is 2byte 
	}

	strcpy( buf, pBuffer );
	
	return strlen(buf);
}

static DEVICE_ATTR(octa_mtp, 0664,
		octa_mtp_show, NULL);

static ssize_t octa_adjust_mtp_show(struct device *dev, struct 
device_attribute *attr, char *buf)
{
	char pBuffer[160] = {0,};
	char cntBuffer;
	int i,j;
	int	is_over255;

	cntBuffer = 0;
	is_over255 = FALSE;

	cntBuffer += sprintf(pBuffer +cntBuffer, "adjust_mtp : ");
	for( i = 0; i < CI_MAX; i ++ )
	{
		for( j = 0; j < IV_MAX; j++ )
		{
			if( s6e8ab0_lcd.smart.adjust_mtp[i][j] > 255 ) 			
			{
				cntBuffer += sprintf(pBuffer +cntBuffer, " %04X", s6e8ab0_lcd.smart.adjust_mtp[i][j] );
				is_over255 = TRUE;
			}
			else cntBuffer += sprintf(pBuffer +cntBuffer, " %02x", s6e8ab0_lcd.smart.adjust_mtp[i][j] );
		}
		cntBuffer += sprintf(pBuffer +cntBuffer, "." );
	}
	if( is_over255 ) cntBuffer += sprintf(pBuffer +cntBuffer, " OVER255" );

	strcpy( buf, pBuffer );
	
	return strlen(buf);
}

static DEVICE_ATTR(octa_adjust_mtp, 0664,
		octa_adjust_mtp_show, NULL);

static ssize_t lcd_sysfs_show_gamma_mode(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
//	struct lcd_setting *plcd = dev_get_drvdata(dev);
	char temp[10];

	switch (s6e8ab0_lcd.gamma_mode) {
	case 0:
		sprintf(temp, "2.2 mode\n");
		strcat(buf, temp);
		break;
	case 1:
		sprintf(temp, "1.9 mode\n");
		strcat(buf, temp);
		break;
	default:
		dev_info(dev, "gamma mode could be 0:2.2, 1:1.9 or 2:1.7)n");
		break;
	}

	return strlen(buf);
}

static ssize_t lcd_sysfs_store_gamma_mode(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	struct lcd_setting *plcd = dev_get_drvdata(dev);
	int rc;

	dev_info(dev, "lcd_sysfs_store_gamma_mode\n");

	rc = strict_strtoul(buf, 0, (unsigned long *)&plcd->gamma_mode);
	if (rc < 0)
		return rc;
	
//	if (lcd->ldi_enable)
//		ld9040_gamma_ctl(lcd);  Arimy to make func

	return len;
}

static DEVICE_ATTR(gamma_mode, 0664,
		lcd_sysfs_show_gamma_mode, lcd_sysfs_store_gamma_mode);

static ssize_t lcd_sysfs_show_gamma_table(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
//	struct lcd_setting *plcd = dev_get_drvdata(dev);
	char temp[3];

	sprintf(temp, "%d\n", s6e8ab0_lcd.gamma_table_count);
	strcpy(buf, temp);

	return strlen(buf);
}

static DEVICE_ATTR(gamma_table, 0664,
		lcd_sysfs_show_gamma_table, NULL);


static int lcd_power(struct msm_fb_data_type *mfd, struct lcd_setting *plcd, int power)
{
	int ret = 0;
	
    if(power == FB_BLANK_UNBLANK)
    {
		if(s6e8ab0_lcd.lcd_state.display_on == TRUE)
			return ret;

    		// LCD ON
    		DPRINT("%s : LCD_ON\n", __func__);
    		lcd_on_seq(mfd);
    }
    else if(power == FB_BLANK_POWERDOWN)
    {
        if (s6e8ab0_lcd.lcd_state.powered_up && s6e8ab0_lcd.lcd_state.display_on) {
			
		// LCD OFF
    		DPRINT("%s : LCD_OFF\n", __func__);
		lcd_off_seq(mfd);
	    }
    }

	return ret;
}


static ssize_t lcd_sysfs_store_lcd_power(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	int rc;
	int lcd_enable;
//	struct lcd_setting *plcd = dev_get_drvdata(dev);
	struct msm_fb_data_type *mfd;
	mfd = dev_get_drvdata(dev);
	
	dev_info(dev, "lcd_sysfs_store_lcd_power\n");

	rc = strict_strtoul(buf, 0, (unsigned long *)&lcd_enable);
	if (rc < 0)
		return rc;

	if(lcd_enable) {
		lcd_power(mfd, &s6e8ab0_lcd, FB_BLANK_UNBLANK);
	}
	else {
		lcd_power(mfd, &s6e8ab0_lcd, FB_BLANK_POWERDOWN);
	}

	return len;
}

static DEVICE_ATTR(lcd_power, 0664,
		NULL, lcd_sysfs_store_lcd_power);



#ifdef MAPPING_TBL_AUTO_BRIGHTNESS
static ssize_t lcd_sysfs_store_auto_brightness(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t len)
{
	int value;
	int rc;

	int last_gamma;
	int new_gamma;
	int check_gamma;
		
	dev_info(dev, "lcd_sysfs_store_auto_brightness\n");

	rc = strict_strtoul(buf, (unsigned int)0, (unsigned long *)&value);
	if (rc < 0)
		return rc;
	else {
		if (s6e8ab0_lcd.auto_brightness != value) {
			check_gamma = (pMFD == NULL)? FALSE:TRUE;
			dev_info(dev, "%s - %d, %d, check_gamma: %d \n", __func__, s6e8ab0_lcd.auto_brightness, value, check_gamma);

			if (check_gamma)
				last_gamma = get_gamma_value_from_bl(pMFD->bl_level);
			
			mutex_lock(&(s6e8ab0_lcd.lock));
			s6e8ab0_lcd.auto_brightness = value;
			mutex_unlock(&(s6e8ab0_lcd.lock));

			if(check_gamma) {
				new_gamma = get_gamma_value_from_bl(pMFD->bl_level);
				if(new_gamma != last_gamma)
					set_backlight(pMFD);
			}	

		}
	}
	return len;
}

static DEVICE_ATTR(auto_brightness, 0664,
		NULL, lcd_sysfs_store_auto_brightness);

#endif

/////////////////]



#ifdef CONFIG_HAS_EARLYSUSPEND
static void early_suspend(struct early_suspend *h) 
{
	DPRINT("%s +\n", __func__);
	lcd_off_seq(pMFD);
	DPRINT("%s -\n", __func__);

 return;
}

static void late_resume(struct early_suspend *h) 
{
	DPRINT("%s\n", __func__);
	lcd_on_seq(pMFD);
//	set_lcd_esd_ignore( FALSE );
 return;
}
#endif



#ifdef CONFIG_CMC624_P8LTE
static struct class *amoled_class;
struct device *amoled_dev;

static ssize_t lcdtype_show(struct device *dev, struct 
device_attribute *attr, char *buf)
{
	char temp[15];
	sprintf(temp, "SMD_AMS767KC01\n");
	strcat(buf, temp);
	return strlen(buf);
}

static DEVICE_ATTR(lcdtype, 0664,
		lcdtype_show, NULL);
#endif

static int __devinit lcd_probe(struct platform_device *pdev)
{
	int ret = 0;
	int default_gamma_value;

	if (pdev->id == 0) {
		pdata = pdev->dev.platform_data;
		DPRINT("%s\n", __func__);

#ifdef CONFIG_CMC624_P8LTE
if(p8lte_has_cmc624())
{
		p8lte_cmc624_init();
}		
#endif
		return 0;
	}

#ifdef CONFIG_CMC624_P8LTE
	if(p8lte_has_cmc624())
	{
			p8lte_cmc624_init();
	}		
#endif

	DPRINT("%s+ \n", __func__);

	pMFD = platform_get_drvdata(pdev);
	
	// get default-gamma address from gamma table
	for( default_gamma_value = 0; default_gamma_value <= MAX_GAMMA_VALUE; default_gamma_value++)
		if( lcd_Gamma22_Meterial13_table[default_gamma_value].lux >= DEFAULT_CANDELA_VALUE ) break;

#ifdef MAPPING_TBL_AUTO_BRIGHTNESS
    for( default_gamma_value = 0; default_gamma_value < MAX_GAMMA_VALUE; default_gamma_value++)
    	if( candela_table[default_gamma_value]>= DEFAULT_CANDELA_VALUE ) break;
#endif

	
	s6e8ab0_lcd.stored_gamma = default_gamma_value;
	s6e8ab0_lcd.stored_elvss = default_gamma_value;
	s6e8ab0_lcd.lcd_gamma = -1;
	s6e8ab0_lcd.enabled_acl= 1;
	s6e8ab0_lcd.stored_acl= default_gamma_value;
	s6e8ab0_lcd.goal_brightness = default_gamma_value;
	s6e8ab0_lcd.lcd_acl = -1;
	s6e8ab0_lcd.lcd_state.display_on = state_lcd_on = false;
	s6e8ab0_lcd.lcd_state.initialized= false;
	s6e8ab0_lcd.lcd_state.powered_up= false;
	s6e8ab0_lcd.factory_id_line	= lcd_id_unknown;
	s6e8ab0_lcd.factory_id_material	= lcd_id_unknown;
	s6e8ab0_lcd.factory_id_material	= lcd_id_unknown;
	s6e8ab0_lcd.factory_id_boardDCDC	= lcd_id_unknown;
	s6e8ab0_lcd.factory_id_lcdDCDC	= lcd_id_unknown;
	s6e8ab0_lcd.lcd_brightness_table = lcd_Gamma22_Meterial13_table;	
	s6e8ab0_lcd.lcd_acl_table = lcd_acl_table;
	s6e8ab0_lcd.lcd_elvss_table = lcd_elvss_table;
	s6e8ab0_lcd.eachElvss_value = 0;
	s6e8ab0_lcd.eachElvss_vector = 0;
	s6e8ab0_lcd.IdPannel = 0;
	s6e8ab0_lcd.isSmartDimming = FALSE;
	s6e8ab0_lcd.isSmartDimming_loaded = FALSE;	
#ifdef MAPPING_TBL_AUTO_BRIGHTNESS
	s6e8ab0_lcd.auto_brightness = 0;
#endif
	mutex_init( &(s6e8ab0_lcd.lock) );

#ifdef CONFIG_HAS_EARLYSUSPEND
	 s6e8ab0_lcd.early_suspend.suspend = early_suspend;
	 s6e8ab0_lcd.early_suspend.resume = late_resume;
	 s6e8ab0_lcd.early_suspend.level = LCD_OFF_GAMMA_VALUE;
	 register_early_suspend(&s6e8ab0_lcd.early_suspend);
#endif

	DPRINT("msm_fb_add_device +\n");
	msm_fb_add_device(pdev);
	DPRINT("msm_fb_add_device -\n");

/////////////[ sysfs
    sysfs_lcd_class = class_create(THIS_MODULE, "lcd");
	if (IS_ERR(sysfs_lcd_class))
		pr_err("Failed to create class(sysfs_lcd_class)!\n");

	sysfs_panel_dev = device_create(sysfs_lcd_class,
		NULL, 0, NULL, "panel");
	if (IS_ERR(sysfs_panel_dev))
		pr_err("Failed to create device(sysfs_panel_dev)!\n");

	ret = device_create_file(sysfs_panel_dev, &dev_attr_power_reduce);
	if (ret < 0)
		dev_err(&(pdev->dev), "failed to add sysfs entries\n");

	ret = device_create_file(sysfs_panel_dev, &dev_attr_lcd_type);  
	if (ret < 0)
		dev_err(&(pdev->dev), "failed to add sysfs entries\n");

//	ret = device_create_file(sysfs_panel_dev, &dev_attr_octa_lcdtype);  
//	if (ret < 0)
//		DPRINT("octa_lcdtype failed to add sysfs entries\n");
//		dev_err(&(pdev->dev), "failed to add sysfs entries\n");

	ret = device_create_file(sysfs_panel_dev, &dev_attr_octa_mtp);  
	if (ret < 0)
		DPRINT("octa_mtp failed to add sysfs entries\n");

	ret = device_create_file(sysfs_panel_dev, &dev_attr_octa_adjust_mtp);  
	if (ret < 0)
		DPRINT("octa_adjust_mtp failed to add sysfs entries\n");

	ret = device_create_file(sysfs_panel_dev, &dev_attr_lcd_power);  
	if (ret < 0)
		DPRINT("lcd_power failed to add sysfs entries\n");
//		dev_err(&(pdev->dev), "failed to add sysfs entries\n");

	// mdnie sysfs create
	//init_mdnie_class();
#ifdef MAPPING_TBL_AUTO_BRIGHTNESS
#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
    if(1){
      struct backlight_device *pbd = NULL;
      pbd = backlight_device_register("panel", NULL, NULL,NULL,NULL);
      if (IS_ERR(pbd)) {
        DPRINT("Could not register 'panel' backlight device\n");
      }
      else
      {
        ret = device_create_file(&pbd->dev, &dev_attr_auto_brightness);
        if (ret < 0)
          DPRINT("auto_brightness failed to add sysfs entries\n");
      }
    }
#endif
#endif
////////////]	
	DPRINT("%s- \n", __func__);

	return 0;
}

static struct platform_driver this_driver = {
	.probe  = lcd_probe,
	.driver = {
		.name   = "mipi_lcd_wxga",
	},
	.shutdown = lcd_shutdown
};

static struct msm_fb_panel_data s6e8ab0_panel_data = {
	.on		= lcd_on,
	.off		= lcd_off,
	.set_backlight = set_backlight,
};

static int ch_used[3];

int mipi_s6e8ab0_wxga_device_register(struct msm_panel_info *pinfo,
					u32 channel, u32 panel)
{
	struct platform_device *pdev = NULL;
	int ret;

	if ((channel >= 3) || ch_used[channel])
		return -ENODEV;

	ch_used[channel] = TRUE;

	pdev = platform_device_alloc("mipi_lcd_wxga", (panel << 8)|channel);
	if (!pdev)
		return -ENOMEM;

	s6e8ab0_panel_data.panel_info = *pinfo;

	ret = platform_device_add_data(pdev, &s6e8ab0_panel_data,
		sizeof(s6e8ab0_panel_data));
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_add_data failed!\n", __func__);
		goto err_device_put;
	}

	ret = platform_device_add(pdev);
	if (ret) {
		printk(KERN_ERR
		  "%s: platform_device_register failed!\n", __func__);
		goto err_device_put;
	}

#ifdef CONFIG_CMC624_P8LTE
	/*  Prepare sysfs node for acl on/off  */
	amoled_class = class_create(THIS_MODULE, "amoled");
	if(IS_ERR(amoled_class)) {
		printk("Failed to create class(amoled_class)!!\n");
		ret = -1;
	}

	/*  P8LTE has not cmc623, fake device for TouchWIZ Platform */
	amoled_dev = device_create(amoled_class, NULL, 0, NULL, "dev");
	if (IS_ERR(amoled_dev))
		printk("[LCD] Failed to create device(amoled_dev)!\n");
	if (device_create_file(amoled_dev, &dev_attr_lcdtype) < 0)
		printk("[LCD] Failed to create device file(%s)!\n", dev_attr_lcdtype.attr.name);
#endif

	return 0;

err_device_put:
	platform_device_put(pdev);
	return ret;
}

static int __init lcd_init(void)
{
	DPRINT("%s \n", __func__);
	mipi_dsi_buf_alloc(&s6e8ab0_tx_buf, DSI_BUF_SIZE);
	mipi_dsi_buf_alloc(&s6e8ab0_rx_buf, DSI_BUF_SIZE);

	return platform_driver_register(&this_driver);
}

module_init(lcd_init);
