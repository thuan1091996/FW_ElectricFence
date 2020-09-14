/*
 * L80.h
 *
 *  Created on: Sep 13, 2019
 *      Author: DFMPC-01201117
 */

#ifndef SRC_TEMPLATES_HARDWAREABSTRACTLAYER_L80_H_
#define SRC_TEMPLATES_HARDWAREABSTRACTLAYER_L80_H_

#include <stdint.h>
#include <string.h>
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <stdio.h>


/*****************************************************************************************************************/
/*													CONSTANTS DEFINITION										 */
/*****************************************************************************************************************/

#define cPI 							3.14159265358979323846
#define cSTART_CHAR		 				(uint8_t)'$'
#define cEND_LINE		 				(uint8_t)'\n'
#define cCARRIAGE_RETURNS 				(uint8_t)'\r'
#define cSEPERATOR						(uint8_t)','

// Different commands to set the update rate from once a second (1 Hz) to 10 times a second (10Hz)
#define PMTK_SET_NMEA_UPDATE_1HZ		"$PMTK220,1000*1F"
#define PMTK_SET_NMEA_UPDATE_5HZ		"$PMTK220,200*2C"
#define PMTK_SET_NMEA_UPDATE_10HZ		"$PMTK220,100*2F"


#define PMTK_SET_NMEA_OUTPUT_RMCONLY	"$PMTK314,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"		// Turn on only the second sentence (GPRMC)
#define PMTK_SET_NMEA_OUTPUT_GGAONLY	"$PMTK314,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*29\r\n"		// Turn on only the second sentence (GPGGA)
#define PMTK_SET_NMEA_OUTPUT_ALLDATA	"$PMTK314,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n"		// Turn on ALL THE DATA


#define PMTK_CMD_COLD_START				"$PMTK103*30\r\n"		// Turn GPS to "cold start"
#define PMTK_CMD_WARM_START				"$PMTK102*31\r\n"		// Turn GPS to "warm start"
#define PMTK_CMD_HOT_START				"$PMTK101*32\r\n"		// Turn GPS to "hot start"

#define PMTK_SET_FLP_MODE				"$PMTK262,1*29\r\n"		// Tracking mode

/*****************************************************************************************************************/
/*													MACROS DEFINITION											 */
/*****************************************************************************************************************/
#define HAL_CORE_SENDER(Data, Len)		GPS_HAL_T.CORE_SERVICES_T.DataSending(Data, Len)
#define mGPSData						GPS_HAL_T.GPSData_T

/*****************************************************************************************************************/
/*													   DATA-STRUCTS												 */
/*****************************************************************************************************************/
typedef struct
{
	double LAT;
	double LON;
	double ALT;
	double NumOfSat;
}tGPSInfo;

typedef struct
{
	uint8_t Hour;
	uint8_t Minute;
	double 	Second;
}tUTCTime;

typedef struct
{
	uint8_t Hour;
	double 	Minute;
}tLatitude;

typedef enum
{
	NORTH=0,
	SOUTH,
	EAST,
	WEST
}tDirection;

typedef struct
{
	uint16_t Hour;
	double   Minute;
}tLongitude;

typedef struct
{
	tUTCTime    GPS_Time;
	tLatitude   GPS_LATITUDE;
	tDirection  GPS_NS;
	tLongitude 	GPS_LONGITUDE;
	tDirection  GPS_EW;
	uint8_t 	GPS_SATELLITES;
	double 		GPS_ALTITUDE;

}tGPGGASentences;

typedef enum
{
	CONNECT_OK = 0,		//0
	CONNECT_FAIL,		//1
	CONNECTING			//2
}tGPSConnectStatus;

enum eGPRMC
{
	GPRMC_MID = 0,		//0
	GPRMC_UTC,			//1
	GPRMC_STS,			//2
	GPRMC_LAT,			//3
	GPRMC_LAT_NS,		//4
	GPRMC_LON,			//5
	GPRMC_LON_EW,		//6
	GPRMC_SOG,			//7
	GPRMC_COG,			//8
	GPRMC_DAT,			//9
	GPRMC_MAV,			//10
	GPRMC_MAV_EW,		//11
	GPRMC_MOD			//12
};

enum eGPGGA
{
	GPGGA_MID = 0,         		//0
	GPGGA_UTC,         			//1
	GPGGA_LAT,         			//2
	GPGGA_LAT_NS,      			//3
	GPGGA_LON,         			//4
	GPGGA_LON_EW,      			//5
	GPGGA_STS,		  			//6
	GPGGA_NOSV,        			//7
	GPGGA_HDOP,        			//8
	GPGGA_ATT,         			//9
	GPGGA_ATT_M,       			//10
	GPGGA_GEOID,       			//11
	GPGGA_GEOID_M,     			//12
	GPGGA_DGPSA,				//13
	GPGGA_DGPS_STATION_ID		//14
};



#endif /* SRC_TEMPLATES_HARDWAREABSTRACTLAYER_L80_H_ */
