/*
 * L80.c
 *
 *  Created on: Sep 13, 2019
 *      Author: DFMPC-01201117
 */

#include "L80.h"

#include "../HAL.h"

/*****************************************************************************************************************/
/*													GLOBAL VARIABLES											 */
/*****************************************************************************************************************/
extern tGPS_HAL GPS_HAL_T;

static tGPIODef GPS_GPIO_PWR = {
		.pin	= PD_10,
		.mode	= 4,									//PIN_PUSH_PULL,
		.pull	= PIN_PULL_UP,
};
/*****************************************************************************************************************/
/*												HARDWARE ABSTRACT LAYER									 		 */
/*****************************************************************************************************************/
static void GPSInit();

static void GPSDataHandler(uint8_t* Data_U8P, uint8_t Length_U8);
static void GPSGetData();
static void GPSSleeping(void* Def);
static void GPSWakeup(void* Def);

void __attribute__((weak)) HAL_GPS_Interaction(tGPS_HAL* HAL)
{
	HAL->GPSType_E = L80;
	HAL->PERIPHERAL_SERVICES_T.Initialize	= &GPSInit;
	HAL->PERIPHERAL_SERVICES_T.DataHandling	= &GPSDataHandler;
	HAL->PERIPHERAL_SERVICES_T.GetGPSData	= &GPSGetData;
	HAL->PERIPHERAL_SERVICES_T.Sleeping		= &GPSSleeping;
	HAL->PERIPHERAL_SERVICES_T.Wakeup		= &GPSWakeup;
}

/*****************************************************************************************************************/
/*													PRIVATE VARIABLES											 */
/*****************************************************************************************************************/
uint8_t				RxBuffer_U8A[255];

tGPSInfo			Temp_GPS;

tGPGGASentences 	Message;
uint8_t 			decode_done		= 0;
uint8_t				getdata_success = 0;
tGPSConnectStatus	ConnectStatus	= CONNECT_FAIL;
uint32_t			step			= 0;
tGPSInfo			Last_GPS;

double				K, P, P_Last, P_temp;
const double		Q				= 0.0122;
const double 		R				= 0.025;
uint8_t				InfoInit		= 0;

/*****************************************************************************************************************/
/*													PRIVATE FUNCTION											 */
/*****************************************************************************************************************/
static void L80Setting(void);
static uint8_t GetField(uint8_t Start_U8, uint8_t* Output_U8P);
static tGPSConnectStatus GPGGA_Decoder(void);
static double GPSConvLat2Float(void);
static double GPSConvLon2Float(void);

/*****************************************************************************************************************/
/*													FUNCTION IMPLEMENT									 	 	 */
/*****************************************************************************************************************/
static void GPSInit(void)
{
	GPS_HAL_T.Enable_B = 1;
	GPS_HAL_T.DeviceStatus_E = Activated;

	// init UART0
	LEUART_GPS_Initialize();
	Delay_ms(1);

	decode_done = 0;
	getdata_success = 0;

	GPS_HAL_T.GPS_GPIO_PWR->Initialize(&GPS_GPIO_PWR, SET);	// Init GPIO_PWR and disable
	Delay_s(1);

	L80Setting();
	Delay_s(3);

	GPS_HAL_T.GPS_GPIO_PWR->Write(&GPS_GPIO_PWR, RESET);	// Init GPIO_PWR and disable

/*	//TODO: used for STM32 to interrupt each byte
 	HAL_UART_Receive_IT(&huart3,&Rx_byte,1);
	Delay(3);
 */
}

/**
 * 	Setting the transmitter of L80
 *	Just use the GPGGA information
 *	receive in Cold Start
 */
static void L80Setting(void)
{
	// Set GPS receiver turn on only the second sentence (GPGGA)
	HAL_CORE_SENDER((uint8_t*)PMTK_SET_NMEA_OUTPUT_GGAONLY, strlen( PMTK_SET_NMEA_OUTPUT_GGAONLY));
	Delay_s(1);

//	// Tracking mode
//	HAL_CORE_SENDER((uint8_t*)PMTK_SET_FLP_MODE, strlen(PMTK_SET_FLP_MODE));
//
//	// Set GPS receiver in mode Cold start
//	HAL_CORE_SENDER((uint8_t*)PMTK_CMD_COLD_START, strlen(PMTK_CMD_COLD_START));
//	Delay_s(1);
}

/* Data Receiving Interrupt */
static void GPSDataHandler(uint8_t* Data_U8P, uint8_t Length_U8)		// A byte per interrupt event
{
	uint32_t StatePWR;
	GPS_HAL_T.GPS_GPIO_PWR->Read(&GPS_GPIO_PWR, &StatePWR);

	if (!StatePWR)	// for test
		return;

	static uint8_t Count = 0;
	uint8_t Rx_byte = *Data_U8P;

	if ((Rx_byte == cSTART_CHAR) || (Count > 125))
		Count = 0;

	RxBuffer_U8A[Count++] = Rx_byte;

	if ((Rx_byte == cEND_LINE) || (Rx_byte == cCARRIAGE_RETURNS))
	{
		ConnectStatus = GPGGA_Decoder();
	}
}

static void GPSMessageHandler()
{
	uint8_t state;
	switch (state)
	{
		case 0/*CONNECTING*/:

			break;
		case 1/*SAMPLING*/:

			break;
		case 2/*Decode*/:

			break;
		default:
			break;
	}
}

static void GPSGetData()
{
	const uint16_t TimeoutMeasurementLimit = GPS_HAL_T.GPSConfiguration_TP->TimeoutMeansurement_U16 / 5;
	const uint16_t TimeoutConnectingLimit  = GPS_HAL_T.GPSConfiguration_TP->TimeoutConnecting_U16 / 5;

	uint8_t RetryMesurement  = GPS_HAL_T.GPSConfiguration_TP->RetryMesurement_U8;
	uint8_t RetryCondition = 0;

	const uint8_t NBMesure    = MAX(GPS_HAL_T.GPSConfiguration_TP->Sampling_U8, 1);
	const uint8_t NBSatelites = MAX(GPS_HAL_T.GPSConfiguration_TP->NumberSatelite_U8, 2);
	const uint8_t FilterType  = MAX(GPS_HAL_T.GPSConfiguration_TP->FilterType_U8, 1);

	uint8_t CounterGetData = 0;
	uint8_t GpsStage = 0;
	uint16_t TimeoutMeasurement = 1;
	uint16_t TotalMeasureTime = 0;
	uint16_t TimeoutGpsKalman = 0;
	uint16_t TimeoutGpsGetSatelite = 0;
	uint8_t NBSatelitesMaximum = 0;

	float stoLatitude[NBMesure];
	float stoLongitude[NBMesure];
	float stoAltitude[NBMesure];
//	uint8_t stoSatelite[NBMesure];

	GPS_HAL_T.GPS_GPIO_PWR->Write(&GPS_GPIO_PWR, SET);
//	Delay_s(1);
//	L80Setting();
//	Delay_s(1);

	do
	{
		memset(stoLatitude, 0, sizeof(stoLatitude));
		memset(stoLongitude, 0, sizeof(stoLongitude));
		memset(stoAltitude, 0, sizeof(stoAltitude));

		CounterGetData = 0;
		GpsStage = 0;
//		TimeoutMeasurement = 1;
		TimeoutGpsKalman = 0;
		TimeoutGpsGetSatelite = 0;

		NBSatelitesMaximum = 0;

		while (ConnectStatus != CONNECT_OK)
		{
			if (TimeoutMeasurement >= TimeoutConnectingLimit)
			{
				GpsStage = 1;
				break;
			}

			TimeoutMeasurement++;
			Delay_s(5);
		}

		if (ConnectStatus == CONNECT_OK)
		{
			while ((GpsStage == 0) && (CounterGetData < NBMesure))
			{
				TimeoutGpsKalman = 1;
				NBSatelitesMaximum < Message.GPS_SATELLITES ? NBSatelitesMaximum = Message.GPS_SATELLITES : 0;

				while (Message.GPS_SATELLITES < NBSatelites)
				{
					if (TimeoutGpsKalman >= TimeoutMeasurementLimit)
					{
						GpsStage = 2;
						break;
					}

					TimeoutGpsKalman++;
					Delay_s(5);
				}

				TimeoutGpsGetSatelite += TimeoutGpsKalman;

				if (GpsStage == 2)
					break;

				stoLatitude[CounterGetData]  = GPSConvLat2Float();
				stoLongitude[CounterGetData] = GPSConvLon2Float();
				stoAltitude[CounterGetData]  = Message.GPS_ALTITUDE;
				//stoSatelite[CounterGetData] = CounterSatelite;

				CounterGetData++;
				Delay_s(5);
			}
		}

		RetryCondition = (NBMesure > CounterGetData);

		TimeoutMeasurement += TimeoutGpsGetSatelite;
		TotalMeasureTime += TimeoutMeasurement;

	} while(RetryMesurement-- && RetryCondition && ENA_RETRY);

	memcpy(&NBMesure,&CounterGetData, sizeof(CounterGetData));

	uint8_t debugcode = 0;

#if GPS_DEBUG_INFO
	mGPSData->DebugInfo_T.NumSatellites_U8 = (((debugcode << 4) | 0x0F) & (NBSatelitesMaximum & 0x0F));
	mGPSData->DebugInfo_T.TimeMeasument_U16 = TotalMeasureTime * 5;
	mGPSData->DebugInfo_T.NumMeasument_U8 = CounterGetData;
#endif

	GPS_HAL_T.GPS_GPIO_PWR->Write(&GPS_GPIO_PWR, RESET);	// Disable GPS


	if ((GpsStage == 1 && ConnectStatus != CONNECT_OK) && CounterGetData == 0)
	{
		// Export results
		mGPSData->Altitude_S32  = 0;
		mGPSData->Longitude_S32 = 0;
		mGPSData->Latitude_S32  = 0;

		return;
	}
	else if (GpsStage == 2 )
	{
		// Export results
		mGPSData->Latitude_S32  = 1000000 * GPSConvLat2Float();
		mGPSData->Longitude_S32 = 1000000 * GPSConvLon2Float();
		mGPSData->Altitude_S32  = Message.GPS_ALTITUDE;

		return;
	}

	if (NBMesure == 1) // no use Kalman filter = use one measure
	{
		// Export results
		mGPSData->Latitude_S32  = 1000000 * GPSConvLat2Float();
		mGPSData->Longitude_S32 = 1000000 * GPSConvLon2Float();
		mGPSData->Altitude_S32  = Message.GPS_ALTITUDE;

		return;
	}

	/* Moving average for NBMesure */
	float avgLatitude  = 0.0;
	float avgLongitude = 0.0;
	float avgAltitude  = 0.0;

	float sumLatitude  = 0.0;
	float sumLongitude = 0.0;
	float sumAltitude  = 0.0;

	for (uint8_t i = 0; i < NBMesure; i++)
	{
		sumLatitude  += stoLatitude[i];
		sumLongitude += stoLongitude[i];
		sumAltitude  += stoAltitude[i];
	}

	avgLatitude  = ((float)sumLatitude)  / NBMesure; 			// Average value Latitude
	avgLongitude = ((float)sumLongitude) / NBMesure; 			// Average value Longitude
	avgAltitude  = ((float)sumAltitude)  / NBMesure; 			// Average value Altitude

	// 3: for average data
	if (FilterType == 3)
	{
		// Export results
		mGPSData->Latitude_S32  = 1000000 * avgLatitude; 					// Average
		mGPSData->Longitude_S32 = 1000000 * avgLongitude;
		mGPSData->Altitude_S32  = avgAltitude;

		return;
	}

	// The noise in the system
	const float Q = 0.022;
	const float R = 0.617;

	float P;
	float K = 0;
	float P_temp;
	float P_last = 0;

//	float Lat_Moyen = 0.0;     						/*!< vi do */
//	float Lng_Moyen = 0.0;     						/*!< kinh do */
//	float Att_Moyen = 0.0;							/*!< do cao */

	float Lat_Old = 0.0;     						/*!< vi do */
	float Lng_Old = 0.0;     						/*!< kinh do */
	float Att_Old = 0.0;							/*!< do cao */
	float Lat_Pre = 0.0;     						/*!< vi do */
	float Lng_Pre = 0.0;     						/*!< kinh do */
	float Att_Pre = 0.0;							/*!< do cao */

	uint8_t stoSel[NBMesure];

	if (FilterType == 2)	// 2: for Kalman and average filter
	{
		float stoDisPre[NBMesure];

		memset(stoSel, 0, NBMesure);

		for (uint8_t i = 0; i < NBMesure; i++)
		{
			float lat1 = stoLatitude[i];
			float lng1 = stoLongitude[i];
			float distance = 0.0;

			float latitude1  = lat1 * cPI / 180;
			float longitude1 = lng1 * cPI / 180;
			float latitude2  = avgLatitude * cPI / 180;
			float longitude2 = avgLongitude * cPI / 180;

			float x = sin((latitude2 - latitude1) / 2);
			float y = sin((longitude2 - longitude1) / 2);
			float value = (x * x) + (cos(latitude1) * cos(latitude2) * y * y);

			distance =  6371.0 * 2 * 1000 * atan2(sqrt(value), sqrt(1 - value));
			stoDisPre[i] = distance;
		}

		float value_cib = 1000.0;
		uint8_t m = 0;

		for (uint8_t i = 0; i < NBMesure / 2; ++i)
		{
			value_cib = 1000.0;

			for (uint8_t j = 0; j < NBMesure; j++)
			{
				if ((stoDisPre[j] < value_cib) && (stoSel[j] != 1))
				{
					value_cib = stoDisPre[j];
					m = j;
				}
			}

			stoSel[m] = 1;
		}
	}
	else if (FilterType == 1)	// 1: for Kalman filter
	{
		memset(stoSel, 1, NBMesure);
	}

//	Lat_Moyen = avg_Lat; 		// Average
//	Lng_Moyen = avg_Lng;
//	Att_Moyen = avg_Att;

	Lat_Old = stoLatitude[0];
	Lng_Old = stoLongitude[0];
	Att_Old = stoAltitude[0];

	for (uint8_t i = 1; i < NBMesure; i++)
	{
		if (stoSel[i] == 1)
		{
			Lat_Pre = Lat_Old;
			Lng_Pre = Lng_Old;
			Att_Pre = Att_Old;
			P_temp  = P_last + Q;

			K = P_temp * (1.0/(P_temp + R));			// calculate the Kalman gain

			// correct
			Lat_Old = Lat_Pre + K * (stoLatitude[i]  - Lat_Pre);
			Lng_Old = Lng_Pre + K * (stoLongitude[i] - Lng_Pre);
			Att_Old = Att_Pre + K * (stoAltitude[i]  - Att_Pre);

			P = (1 - K) * P_temp;
			P_last = P;
		}
	}

	// Export results
	mGPSData->Latitude_S32  = 1000000 * Lat_Old;
	mGPSData->Longitude_S32 = 1000000 * Lng_Old;
	mGPSData->Altitude_S32  = Att_Old;
}

static void GPSSleeping(void* Def)
{
	GPS_HAL_T.GPS_GPIO_PWR->Write(&GPS_GPIO_PWR, RESET);
}

static void GPSWakeup(void* Def)
{
//	GPS_HAL_T.GPS_GPIO_PWR->Write(&GPS_GPIO_PWR, SET);
}

static uint8_t GetField(uint8_t Start_U8, uint8_t* Output_U8P)
{
	while (((RxBuffer_U8A[Start_U8]) != cSEPERATOR) && ((RxBuffer_U8A[Start_U8]) != cCARRIAGE_RETURNS))
	{
		*(Output_U8P++) = RxBuffer_U8A[Start_U8++];
	}

	return Start_U8;
}

/**
 * Decode the GPGGA Sentences to get the location information in degree
 *	Field Position			Mean
 *	1						UTCTime
 *	2						LATITUDE
 *	3						N/S
 *	4						LONGITUDE
 *	5						E/W
 *	7						NUMBER OF THE SATELLITES IS USING
 *	9						ALTITUDE
 */
static tGPSConnectStatus GPGGA_Decoder(void)
{
	char Buffer_U8A[10];
	uint8_t Start_U8 = 0;

	enum eGPGGA Field_U8 = GPGGA_MID;
	tGPSConnectStatus RET = CONNECTING;

	while (Field_U8 <= GPGGA_ATT)
	{
		char Temp_U8A[10];

		memset(Buffer_U8A, 0, 10);

		Start_U8 = GetField(Start_U8, (uint8_t*)Buffer_U8A) + 1;	// +1: igrone ","

		switch (Field_U8)
		{
		case GPGGA_MID:
			if (Buffer_U8A[1] != 'G')		// ASCII 71: G
			{
				RET = CONNECTING;
				goto RETURN;
			}

			break;

			/**
			 * format : hhmmss.sss
			 */
		case GPGGA_UTC:						//UTCTime
			if (Buffer_U8A[0] == 0)
			{
				decode_done = 1;
				RET = CONNECT_FAIL;
				goto RETURN;
			}

			strncpy( Temp_U8A, Buffer_U8A, 2);
			Temp_U8A[2] = 0;
			Message.GPS_Time.Hour = atoi(Temp_U8A);

			strncpy(Temp_U8A, &Buffer_U8A[2], 2);
			Temp_U8A[2] = 0;
			Message.GPS_Time.Minute = atoi(Temp_U8A);

			strncpy(Temp_U8A, &Buffer_U8A[4], 6);
			Temp_U8A[6] = 0;
			Message.GPS_Time.Second = atof(Temp_U8A);

			break;

			/**
			 * format: ddmm.mmmm
			 */
		case GPGGA_LAT:	//LATITUDE
			if (Buffer_U8A[0] == 0)
			{
				decode_done = 1;
				RET = CONNECT_FAIL;
				goto RETURN;
			}

			strncpy(Temp_U8A, Buffer_U8A, 2);
			Temp_U8A[2] = 0;
			Message.GPS_LATITUDE.Hour = atof(Temp_U8A);

			strncpy(Temp_U8A, &Buffer_U8A[2], 7);
			Temp_U8A[7] = 0;
			Message.GPS_LATITUDE.Minute = atof(Temp_U8A);

			break;

			/**
			 * format: N/S
			 * 'N'=North
			 * 'S'=South
			 */
		case GPGGA_LAT_NS:	// N/S
			if (Buffer_U8A[0] == 0)
			{
				decode_done = 1;
				RET = CONNECT_FAIL;
				goto RETURN;
			}

			if (Buffer_U8A[0] == 78)				//# 'N'
				Message.GPS_NS = NORTH;
			else
				Message.GPS_NS = SOUTH;

			break;

			/**
			 * format: dddmm.mmmm
			 */
		case GPGGA_LON:	//LONGTITUDE
			if (Buffer_U8A[0] == 0)
			{
				decode_done = 1;
				RET = CONNECT_FAIL;
				goto RETURN;
			}

			strncpy(Temp_U8A, Buffer_U8A, 3);
			Temp_U8A[3] = 0;
			Message.GPS_LONGITUDE.Hour = atof(Temp_U8A);

			strncpy(Temp_U8A, &Buffer_U8A[3], 7);
			Temp_U8A[7] = 0;
			Message.GPS_LONGITUDE.Minute = atof(Temp_U8A);

			break;

			/**
			 * format: E/W
			 * 'E'=East
			 * 'W'=West
			 */
		case GPGGA_LON_EW:	// E/W
			if (Buffer_U8A[0] == 0)
			{
				decode_done = 1;
				RET = CONNECT_FAIL;
				goto RETURN;
			}

			if (Buffer_U8A[0] == 69)
				Message.GPS_EW = EAST;
			else
				Message.GPS_EW = WEST;

			break;
			/**
			 * fix status
			 * 0=Invalid
			 * 1=GPS Fix
			 * 2=DGPS Fix
			 */
		case GPGGA_STS:
			if (Buffer_U8A[0] == 48)
			{
				decode_done = 1;
				RET = CONNECT_FAIL;
				goto RETURN;
			}
			break;
			/**
			 * Number of satellites being used (0~12)
			 */
		case GPGGA_NOSV:
			if (Buffer_U8A[0] == 0)
			{
				decode_done = 1;
				RET = CONNECT_FAIL;
				goto RETURN;
			}

			Message.GPS_SATELLITES = atof(Buffer_U8A);
			break;
		case 8:
			{
				/*MOI VAT SINH RA TREN DOI DEU CO LY DO CUA NO!*/
				/*CO NHUNG THU KO BIET NO DE LAM GI, NHUNG THIEU NO THI CHAY KO DC*/
			}
			break;
			/**
			 * Altitude in meters
			 */
		case GPGGA_ATT:	//ALTITUDE
			if (Buffer_U8A[0] == 0)
			{
				decode_done = 1;
				RET = CONNECT_FAIL;
				goto RETURN;
			}

			Message.GPS_ALTITUDE = atof(Buffer_U8A);
			decode_done = 1;

//			mGPSData->Latitude_S32 = GPSConvLat2Float();
//			mGPSData->Longitude_S32 = GPSConvLon2Float();
//			mGPSData->Altitude_S32 = Message.GPS_ALTITUDE;

			RET = CONNECT_OK;
			goto RETURN;
			break;

		default:
			RET = CONNECT_FAIL;
			goto RETURN;
			break;
		}

		Field_U8++;
	}

	RETURN:
		return RET;
}

static double GPSConvLon2Float(void)
{
	double temp;
	temp = Message.GPS_LONGITUDE.Hour + Message.GPS_LONGITUDE.Minute / 60;

	if (Message.GPS_EW == WEST)
		return -temp;
	else
		return temp;
}

static double GPSConvLat2Float(void)
{
	double temp;
	temp = Message.GPS_LATITUDE.Hour + Message.GPS_LATITUDE.Minute / 60;

	if (Message.GPS_NS == SOUTH)
		return -temp;
	else
		return temp;
}

