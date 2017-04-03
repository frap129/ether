#ifndef __FIH_RAMTABLE_H
#define __FIH_RAMTABLE_H

#define FIH_RAM_BASE					0x0F100000
#define FIH_RAM_SIZE					0x00700000
#define FIH_RAM_SIZE_MB					7
 /* -------------------------------------------------------- */
/* modem rf_nv */
#define FIH_MODEM_RF_NV_BASE			FIH_RAM_BASE
#define FIH_MODEM_RF_NV_SIZE			0x00200000
#define NV_RF_ADDR						FIH_MODEM_RF_NV_BASE
#define NV_RF_SIZE						FIH_MODEM_RF_NV_SIZE

/* modem cust_nv */
#define FIH_MODEM_CUST_NV_BASE			(FIH_MODEM_RF_NV_BASE + FIH_MODEM_RF_NV_SIZE)
#define FIH_MODEM_CUST_NV_SIZE			0x00200000
#define NV_CUST_ADDR					FIH_MODEM_CUST_NV_BASE
#define NV_CUST_SIZE					FIH_MODEM_CUST_NV_SIZE

/* modem log */
#define FIH_MODEM_LOG_BASE				(FIH_MODEM_CUST_NV_BASE + FIH_MODEM_CUST_NV_SIZE)
#define FIH_MODEM_LOG_SIZE				0x00100000

/* -------------------------------------------------------- 5MB */
/* last_alog_main */
#define FIH_LAST_ALOG_MAIN_BASE			(FIH_MODEM_LOG_BASE + FIH_MODEM_LOG_SIZE)
#define FIH_LAST_ALOG_MAIN_SIZE			0x00040000

/* last_alog_events */
#define FIH_LAST_ALOG_EVENTS_BASE		(FIH_LAST_ALOG_MAIN_BASE + FIH_LAST_ALOG_MAIN_SIZE)
#define FIH_LAST_ALOG_EVENTS_SIZE		0x00040000

/* last_alog_radio */
#define	FIH_LAST_ALOG_RADIO_BASE		(FIH_LAST_ALOG_EVENTS_BASE + FIH_LAST_ALOG_EVENTS_SIZE)
#define FIH_LAST_ALOG_RADIO_SIZE		0x00040000

/* last_alog_system */
#define FIH_LAST_ALOG_SYSTEM_BASE		(FIH_LAST_ALOG_RADIO_BASE + FIH_LAST_ALOG_RADIO_SIZE)
#define FIH_LAST_ALOG_SYSTEM_SIZE		0x00040000

/* last_kmsg */
#define FIH_LAST_KMSG_BASE				(FIH_LAST_ALOG_SYSTEM_BASE + FIH_LAST_ALOG_SYSTEM_SIZE)
#define FIH_LAST_KMSG_SIZE				0x00040000

/* last_blog */
#define FIH_LAST_BLOG_BASE				(FIH_LAST_KMSG_BASE + FIH_LAST_KMSG_SIZE)
#define FIH_LAST_BLOG_SIZE				0x00020000
#define FIH_DEBUG_LAST_BLOG_ADDR		FIH_LAST_BLOG_BASE

/* blog */
#define FIH_BLOG_BASE					(FIH_LAST_BLOG_BASE + FIH_LAST_BLOG_SIZE)
#define FIH_BLOG_SIZE					0x00020000
#define FIH_DEBUG_BLOG_ADDR				FIH_BLOG_BASE
#define FIH_DEBUG_BLOG_SIZE				FIH_BLOG_SIZE
#define FIH_DEBUG_BLOG_LIMT				(FIH_BLOG_BASE + FIH_BLOG_SIZE)
 /* -------------------------------------------------------- 6.5MB */
/* hwid:hwcfg */
#define FIH_HWID_HWCFG_BASE				(FIH_BLOG_BASE + FIH_BLOG_SIZE)
#define FIH_HWID_HWCFG_SIZE				0x00000040
#define FIH_HWID_ADDR					FIH_HWID_HWCFG_BASE
#define FIH_HWID_SIZE					FIH_HWID_HWCFG_SIZE

/* secboot:devinfo */
#define FIH_SECBOOT_DEVINFO_BASE		(FIH_HWID_HWCFG_BASE + FIH_HWID_HWCFG_SIZE)
#define FIH_SECBOOT_DEVINFO_SIZE		0x00000040

/* secboot:unlock */
#define FIH_SECBOOT_UNLOCK_BASE			(FIH_SECBOOT_DEVINFO_BASE + FIH_SECBOOT_DEVINFO_SIZE)
#define FIH_SECBOOT_UNLOCK_SIZE			0x00000100

/* sutinfo */
#define FIH_SUTINFO_BASE				(FIH_SECBOOT_UNLOCK_BASE + FIH_SECBOOT_UNLOCK_SIZE)
#define FIH_SUTINFO_SIZE				0x00000080
#define FIH_SUT_ADDR					FIH_SUTINFO_BASE
#define FIH_SUT_SIZE					FIH_SUTINFO_SIZE

/* no use 1 */
#define FIH_NO_USE_1_BASE				(FIH_SUTINFO_BASE + FIH_SUTINFO_SIZE)
#define FIH_NO_USE_1_SIZE				0x00000010

/* bset */
#define FIH_BSET_BASE					(FIH_NO_USE_1_BASE + FIH_NO_USE_1_SIZE)
#define FIH_BSET_SIZE					0x00000010
#define FIH_BSET_MEM_ADDR				FIH_BSET_BASE
#define FIH_BSET_MEM_SIZE				FIH_BSET_SIZE

/* bat-id adc */
#define FIH_BAT_ID_ADC_BASE				(FIH_BSET_BASE + FIH_BSET_SIZE)
#define FIH_BAT_ID_ADC_SIZE				0x00000010

/* no use 2 */
#define FIH_NO_USE_2_BASE				(FIH_BAT_ID_ADC_BASE + FIH_BAT_ID_ADC_SIZE)
#define FIH_NO_USE_2_SIZE				0x00000010

/* apr */
#define FIH_APR_BASE					(FIH_NO_USE_2_BASE + FIH_NO_USE_2_SIZE)
#define FIH_APR_SIZE					0x00000020
#define FIH_APR_MEM_ADDR				FIH_APR_BASE
#define FIH_APR_MEM_SIZE				FIH_APR_SIZE

/* no use 3 */
#define FIH_NO_USE_3_BASE				(FIH_APR_BASE + FIH_APR_SIZE)
#define FIH_NO_USE_3_SIZE				0x000001A0

/* no use 4 */
#define FIH_NO_USE_4_BASE				(FIH_NO_USE_3_BASE + FIH_NO_USE_3_SIZE)
#define FIH_NO_USE_4_SIZE				0x00000C00

/* e2p */
#define FIH_E2P_BASE					(FIH_NO_USE_4_BASE + FIH_NO_USE_4_SIZE)
#define FIH_E2P_SIZE					0x00001000
#define FIH_E2P_ST_ADDR					FIH_E2P_BASE
#define FIH_E2P_ST_SIZE					FIH_E2P_SIZE

/* cda */
#define FIH_CDA_BASE					(FIH_E2P_BASE + FIH_E2P_SIZE)
#define FIH_CDA_SIZE					0x00001000
#define FIH_CDA_ST_ADDR					FIH_CDA_BASE
#define FIH_CDA_ST_SIZE					FIH_CDA_SIZE

/* no use 5 */
#define FIH_NO_USE_5_BASE				(FIH_CDA_BASE + FIH_CDA_SIZE)
#define FIH_NO_USE_5_SIZE				0x00005000

/* fver */
#define FIH_FVER_BASE					(FIH_NO_USE_5_BASE + FIH_NO_USE_5_SIZE)
#define FIH_FVER_SIZE					0x00008000
#define FIH_FVER_ADDR					FIH_FVER_BASE

/* sensordata */
#define FIH_SENSORDATA_BASE				(FIH_FVER_BASE + FIH_FVER_SIZE)
#define FIH_SENSORDATA_SIZE				0x00004000
#define FIH_SENSOR_MEM_ADDR				FIH_SENSORDATA_BASE
#define FIH_SENSOR_MEM_SIZE				FIH_SENSORDATA_SIZE

/* no use 6 */
#define FIH_NO_USE_6_BASE				(FIH_SENSORDATA_BASE + FIH_SENSORDATA_SIZE)
#define FIH_NO_USE_6_SIZE				0x0006C000
 /* -------------------------------------------------------- 7MB */

/* FIH,Jimi,2014/12/03, modify for ramtable*/
/**************************************************************
 * START       | SIZE        | TARGET
 * --------------------------------------------------------
 * 0x0F10_0000 | 0x0020_0000 | modem rf_nv (2MB)
 * 0x0F30_0000 | 0x0020_0000 | modem cust_nv (2MB)
 * 0x0F50_0000 | 0x0010_0000 | modem log (1MB)
  * -------------------------------------------------------- 5MB
 * 0x0F60_0000 | 0x0004_0000 | last_alog_main (256KB)
 * 0x0F64_0000 | 0x0004_0000 | last_alog_events (256KB)
 * 0x0F68_0000 | 0x0004_0000 | last_alog_radio (256KB)
 * 0x0F6C_0000 | 0x0004_0000 | last_alog_system (256KB)
 * 0x0F70_0000 | 0x0004_0000 | last_kmsg (256KB)
 * 0x0F74_0000 | 0x0002_0000 | last_blog (128KB)
 * 0x0F76_0000 | 0x0002_0000 | blog (128KB)
 * -------------------------------------------------------- 6.5MB
 * 0x0F78_0000 | 0x0000_0040 | hwid:hwcfg (64B)
 * 0x0F78_0040 | 0x0000_0040 | secboot:devinfo (64B)
 * 0x0F78_0080 | 0x0000_0100 | secboot:unlock (256B)
 * 0x0F78_0180 | 0x0000_0080 | sutinfo (128B)
 * 0x0F78_0200 | 0x0000_0010 | no use (16B)
 * 0x0F78_0210 | 0x0000_0010 | bset (16B)
 * 0x0F78_0220 | 0x0000_0010 | bat-id adc (16B)
 * 0x0F78_0230 | 0x0000_0010 | no use (16B)
 * 0x0F78_0240 | 0x0000_0020 | apr (32B)
 * 0x0F78_0260 | 0x0000_01A0 | no use (416B)
 * 0x0F78_0400 | 0x0000_0C00 | no use (3KB)
 * 0x0F78_1000 | 0x0000_1000 | e2p (4KB)
 * 0x0F78_2000 | 0x0000_1000 | cda (4KB)
 * 0x0F78_3000 | 0x0000_5000 | no use 5 (20KB)
 * 0x0F78_8000 | 0x0000_8000 | fver (32KB)
 * 0x0F79_0000 | 0x0000_4000 | sensordata (16KB)
 * 0x0F79_4000 | 0x0006_C000 | no use 6 (432KB)
 * 0x0F80_0000 |             | end
 * -------------------------------------------------------- 7MB
 * 0x0F80_0000 | 0x0010_0000 | no use (1MB)
 * 0x0F90_0000 | 0x0010_0000 | lk (1MB)
 * --------------------------------------------------------
 * 0x0F80_0000 | 0xXXXX_XXXX | HLOS (MB)
 */

#endif /* __FIH_RAMTABLE_H */
