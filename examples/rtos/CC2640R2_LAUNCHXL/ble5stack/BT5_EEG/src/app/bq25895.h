/**
 * @file    bq25895.h
 * @author  gjmsilly
 * @brief   bq25895 Firmware for CC2640R2F
 * @version 1.0
 * @date    2020-07-27
 *
 * @copyright (c) 2020 gjmsilly
 *
 */
 
#ifndef __BQ25895_H
#define __BQ25895_H

#include <stdint.h>


/*---------- P32 Register Maps --------*/
// TI 缂栬瘧鍣� 灏忕瀵归綈

/*********************************************/
/** 	Structure definition for REG00 		**/
/*********************************************/
typedef union
{
   uint8_t value; 						
   struct
   {
       
       uint8_t iinlim             :6;  	// input current limit
       uint8_t en_ilim            :1;	// enable ilim pin
       uint8_t en_hiz             :1;	// enable hiz mode
	   
   } control_bit;
} BQ25895REG00;

/*********************************************/
/** 	Structure definition for REG01 		**/
/*********************************************/
typedef union
{
   uint8_t value; 							
   struct
   {
       
       uint8_t vindpm_os             :5;  	// input voltage limit offset
       uint8_t bcold          	     :1;	// boost mode cold temperature monitor threshold
       uint8_t bhot             	 :2;	// boost mode hot temperature monitor threshold
	   
   } control_bit;
} BQ25895REG01;

/*********************************************/
/** 	Structure definition for REG02 		**/
/*********************************************/
typedef union
{
   uint8_t value; 							
   struct
   {
       
       uint8_t auto_dpdm_en          :1;  	// automatic d+/d- detection enable
       uint8_t force_dpdm          	 :1;	// force d+/d- detection
       uint8_t maxc_en             	 :1;	// maxcharge adapter enable
	   uint8_t hvdcp_en          	 :1;  	// high voltage dcp enable
       uint8_t ico_en           	 :1;	// input current optimizer (ico) enable
       uint8_t boost_freq            :1;	// boost mode frequency selection
	   uint8_t conv_rate        	 :1;  	// adc conversion rate selection
       uint8_t conv_start          	 :1;	// adc conversion start control
	   
   } control_bit;
} BQ25895REG02;

/*********************************************/
/** 	Structure definition for REG03 		**/
/*********************************************/
typedef union
{
   uint8_t value; 							
   struct
   {
       
       uint8_t rsv0            		 :1;  	
       uint8_t sys_min          	 :3;	// minimum system voltage limit
       uint8_t chg_config            :1;	// charge enable configuration
       uint8_t otg_config            :1;	// boost (otg) mode configuration
       uint8_t wd_rst             	 :1;	// i2c watchdog timer reset
       uint8_t bat_loaden            :1;	// battery load (ibatload) enable
	   
   } control_bit;
} BQ25895REG03;

/*********************************************/
/** 	Structure definition for REG04 		**/
/*********************************************/
typedef union
{
   uint8_t value; 							
   struct
   {
       
       uint8_t ichg             	:7;   // fast charge current limit	
       uint8_t en_pumpx             :1;   // current pulse control enable	
	   
   } control_bit;
} BQ25895REG04;

/*********************************************/
/** 	Structure definition for REG05 		**/
/*********************************************/
typedef union
{
   uint8_t value; 							
   struct
   {
       
       uint8_t iterm             	:4;   // termination current limit
       uint8_t iprechg              :4;   // precharge current limit	
	   
   } control_bit;
} BQ25895REG05;

/*********************************************/
/** 	Structure definition for REG06 		**/
/*********************************************/
typedef union
{
   uint8_t value; 							
   struct
   {
       
       uint8_t vrechg             	:1;   // battery recharge threshold offset
       uint8_t batlowv              :1;   // battery precharge to fast charge threshold
	   uint8_t vreg					:6;   // charge voltage limit
	   
   } control_bit;
} BQ25895REG06;

/*********************************************/
/** 	Structure definition for REG07 		**/
/*********************************************/
typedef union
{
   uint8_t value; 							
   struct
   {
       
       uint8_t rsv0         	    :1;   
       uint8_t chg_timer            :2;   // fast charge timer setting
	   uint8_t en_timer				:1;   // charging safety timer enable
	   uint8_t watchdog             :2;   // i2c watchdog timer setting
	   uint8_t stat_dis				:1;   // stat pin disable
	   uint8_t en_term				:1;   // charging termination enable
	   
   } control_bit;
} BQ25895REG07;

/*********************************************/
/** 	Structure definition for REG08 		**/
/*********************************************/
typedef union
{
   uint8_t value; 							
   struct
   {
       
       uint8_t treg         	   	:2;   // thermal regulation threshold
       uint8_t vclamp             	:3;   // ir compensation voltage clamp
	   uint8_t bat_comp				:3;   // ir compensation resistor setting
	   
   } control_bit;
} BQ25895REG08;

/*********************************************/
/** 	Structure definition for REG09 		**/
/*********************************************/
typedef union
{
   uint8_t value; 							
   struct
   {
       
       uint8_t pumpx_dn         	:1;   // current pulse control voltage down enable
       uint8_t pumpx_up             :1;   // current pulse control voltage up enable
	   uint8_t batfet_rst_en		:1;   // batfet full system reset enable
       uint8_t batfet_dly         	:1;   // batfet turn off delay control
       uint8_t rsv4             	:1;   
	   uint8_t batfet_dis			:1;   // force batfet off to enable ship mode
       uint8_t tmr2x_en             :1;   // safety timer setting during dpm or thermal regulation
	   uint8_t force_ico			:1;   // force start input current optimizer (ico)
	   
   } control_bit;
} BQ25895REG09;

/*********************************************/
/** 	Structure definition for REG0A 		**/
/*********************************************/
typedef union
{
   uint8_t value; 							
   struct
   {
       
       uint8_t rsv03         	   	:4;   
       uint8_t boostv             	:4;   // Boost Mode Voltage Regulation
	   
   } control_bit;
} BQ25895REG0A;

/*********************************************/
/** 	Structure definition for REG0B 		**/
/*********************************************/
typedef union
{
   uint8_t value; 							
   struct
   {
       
       uint8_t vsys_stat         	:1;   // vsys regulation status
       uint8_t sdp_stat       	    :1;   // usb input status
	   uint8_t pg_stat				:1;   // power good status
	   uint8_t chrg_stat            :2;   // charging status
	   uint8_t vbus_stat			:3;   // vbus status register
	   
   } control_bit;
} BQ25895REG0B;

/*********************************************/
/** 	Structure definition for REG0E 		**/
/*********************************************/
typedef union
{
   uint8_t value; 							
   struct
   {
       
       uint8_t batv         		:7;   // ADC conversion of Battery Voltage
       uint8_t therm_stat       	:1;   // Thermal Regulation Status
	   
   } control_bit;
} BQ25895REG0E;

/*********************************************/
/** 	Structure definition for REG0F 		**/
/*********************************************/
typedef union
{
   uint8_t value; 							
   struct
   {
       
       uint8_t sysv         		:7;   // ADC conversion of System Voltage
       uint8_t rsv8       	   	    :1;  
	   
   } control_bit;
} BQ25895REG0F;

/*********************************************/
/** 	Structure definition for REG11 		**/
/*********************************************/
typedef union
{
   uint8_t value; 							
   struct
   {
       
       uint8_t vbusv         		:7;   // ADC conversion of VBUS voltage
       uint8_t vbus_gd       	   	:1;   // VBUS Good Status
	   
   } control_bit;
} BQ25895REG11;

/*********************************************/
/** 	Structure definition for REG12 		**/
/*********************************************/
typedef union
{
   uint8_t value; 							
   struct
   {
       
       uint8_t ichgr         		:7;   // ADC conversion of Charge Current
       uint8_t rsv8       	  	 	:1; 
	   
   } control_bit;
} BQ25895REG12;

/*********************************************/
/** 	Structure definition for REG14 		**/
/*********************************************/
typedef union 
{
   uint8_t value; 							
   struct
   {
       
       uint8_t dev_rev        		:2;   // Device Revision: 01
       uint8_t ts_profile     	    :1;   // Temperature Profile
       uint8_t pn 	        		:3;   // Device Configuration: 111
       uint8_t ico_optimized   	    :1;   // Input Current Optimizer (ICO) Status
       uint8_t reg_rst       		:1;   // Register Reset
	   
   } control_bit;
} BQ25895REG14;

/*---------- P48 Register Maps END--------*/

/*********************************************/
/* Define BQ25895 address	                 */
/*********************************************/
#define BQ25895_Addr				0x6A

/*********************************************/
/* Define names for the register addresses   */
/*********************************************/
#define BQ25895_REG00				(0x0000)
#define BQ25895_REG01				(0x0001)
#define BQ25895_REG02				(0x0002)
#define BQ25895_REG03				(0x0003)
#define BQ25895_REG04				(0x0004)
#define BQ25895_REG05				(0x0005)
#define BQ25895_REG06				(0x0006)
#define BQ25895_REG07				(0x0007)
#define BQ25895_REG08				(0x0008)
#define BQ25895_REG09				(0x0009)
#define BQ25895_REG0A				(0x000A)
#define BQ25895_REG0B				(0x000B)
#define BQ25895_REG0E				(0x000E)
#define BQ25895_REG0F				(0x000F)
#define BQ25895_REG11				(0x0011)
#define BQ25895_REG12				(0x0012)
#define BQ25895_REG14				(0x0014)

void BQ25895_init(void);
uint8_t BQ25895_Getdata( uint8_t slaveAddr, uint8_t regAddr );
void BQ25895_SetParam ( uint8_t slaveAddr , uint8_t regAddr, uint8_t data);

#endif /* __BQ25895_H */
