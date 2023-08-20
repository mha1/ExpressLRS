#if defined(TARGET_RX)

#include "SerialHoTT_TLM.h"
#include "common.h"
#include "telemetry.h"

extern Telemetry telemetry;


// Variables / constants for HoTT telemetry //
FIFO_GENERIC<AP_MAX_BUF_LEN> hottInputBuffer;
FIFO_GENERIC<AP_MAX_BUF_LEN> hottOutputBuffer;

#define SENSOR_ID_GPS_B 	0x8A  	// our telemetry ID fake GPS module
#define SENSOR_ID_GPS_T		0xA0  	// sensor ID for text mode adressing
#define START_FRAME_B   	0x7C  		// HoTT start of frame marker
#define END_FRAME			0x7D  		// HoTT end of frame marker

typedef struct {
	uint8_t  StartByte 		= START_FRAME_B;      		//  0 0x7C
	uint8_t  Packet_ID 		= SENSOR_ID_GPS_B;      	//  1 0x8A HOTT_GPS_PACKET_ID
	uint8_t  WarnBeep 		= 0;       					//  2 warn beep (0 = no beep, 0x00..0x1A warn beeps)
	uint8_t  Packet_ID_text = SENSOR_ID_GPS_T; 			//  3 0xA0 Sensor ID text mode
	uint8_t  alarmInverse 	= 0;						//  4 0 Inverse status
	uint8_t  GPSInverse		= 0;						//  5 1 = no GPS signal
	uint8_t  Direction		= 0;						//  6 1 = 2 degrees; 0 = N, 90 = E, 180 = S, 270 = W
	uint16_t Speed			= 0;						//  7 1 = 1 km/h
	uint8_t  Lat_NS			= 0;						//  9 example: N48D39'0988'', 0 = N 
	uint16_t Lat_DegMin		= 0;						// 10 48D39' = 4839 = 0x12e7
	uint16_t Lat_Sec    	= 0;						// 12 0988'' = 988 = 0x03DC
	uint8_t  Lon_NS			= 0;						// 14 example: E09D25'9360'', 0 = E 
	uint16_t Lon_DegMin		= 0;						// 15 09D25' = 0925 = 0x039D
	uint16_t Lon_Sec    	= 0;						// 17 9360'' = 9360 = 0x2490
	uint16_t Distance		= 0;						// 19 1 = 1m
	uint16_t Altitude 		= 500;						// 21 500 = 0m
	uint16_t m_per_sec 	    = 30000;					// 23 30000 = 0.00m/s (1 = 0.01m/s)
	uint8_t  m_per_3sec 	= 120;					    // 25 120 = 0m/3s (1 = 1m/3s)
	uint8_t  Satellites 	= 0;					    // 26 n visible satellites
	uint8_t  FixChar 		= 2;					    // 27 GPS fix character. Display if DGPS, 2D oder 3D
	uint8_t  HomeDir 		= 0;					    // 28 GPS home direction 1 = 2 degreed
	int8_t   Roll 			= 0;					    // 29 signed roll angle 1 = 2 degrees
	int8_t   Pitch			= 0;					    // 30 signed pitch angle 1 = 2 degrees
	int8_t   Yaw			= 0;					    // 31 signed yaw angle 1 = 2 degrees
	uint8_t  TimeHours		= 0;						// 32 GPS time hours
	uint8_t  TimeMinutes	= 0;						// 33 GPS time minutes	
	uint8_t  TimeSeconds	= 0;						// 34 GPS time seconds
	uint8_t  TimeHundreds	= 0;						// 35 GPS time 1/100 seconds
	uint16_t MSLAltitude 	= 0;						// 36 1 = 1m
	uint8_t  Vibrations		= 0;						// 38 vibration level in %
	uint8_t  ASCII1			= '-';						// 39 free ASCII character 1
	uint8_t  ASCII2			= ' ';						// 40 free ASCII character 2
	uint8_t  ASCII3			= '-';						// 41 free ASCII character 3
	uint8_t  Version 		= 0;        				// 42 version number
	uint8_t  EndByte 		= END_FRAME;        		// 43 0x7D
	uint8_t  CRC 			= 0x00;            			// 44 CRC
} PACKED GPSPacket_t; 

static uint8_t _size = 0;
static uint8_t buf[128];
static GPSPacket_t gps;


void SerialHoTT_TLM::handleUARTin()
{  
    static uint32_t lastTLMsent = 0;
    uint32_t now = millis();

    while(hottTLMport.available() && _size < 64)
        buf[_size++] = (uint8_t)hottTLMport.read();

    if(_size > 63) 
        _size = 0;

    if(_size == 45) {
        memcpy(&gps, &buf[0], 45);
        _size = 0;

        CRSF_MK_FRAME_T(crsf_sensor_baro_vario_t) crsfBaro = {0};

        crsfBaro.p.altitude = htobe16((gps.Altitude - 500)*10 + 10000);
        crsfBaro.p.verticalspd = htobe16(gps.m_per_sec - 30000);

        CRSF::SetHeaderAndCrc((uint8_t *)&crsfBaro, CRSF_FRAMETYPE_BARO_ALTITUDE, CRSF_FRAME_SIZE(sizeof(crsf_sensor_baro_vario_t)), CRSF_ADDRESS_CRSF_TRANSMITTER);
        telemetry.AppendTelemetryPackage((uint8_t *)&crsfBaro);
    }

/*
    if(now < lastTLMsent + 100)
        return;

    lastTLMsent = now; 

    CRSF_MK_FRAME_T(crsf_sensor_baro_vario_t) crsfBaro = {0};

    crsfBaro.p.altitude = htobe16((gps.Altitude - 500)*10 + 10000);
    crsfBaro.p.verticalspd = htobe16(gps.m_per_sec - 30000);

    CRSF::SetHeaderAndCrc((uint8_t *)&crsfBaro, CRSF_FRAMETYPE_BARO_ALTITUDE, CRSF_FRAME_SIZE(sizeof(crsf_sensor_baro_vario_t)), CRSF_ADDRESS_CRSF_TRANSMITTER);
    telemetry.AppendTelemetryPackage((uint8_t *)&crsfBaro);
*/
}

void SerialHoTT_TLM::handleUARTout()
{
    static uint32_t lastPoll = 0;
    uint32_t now = millis();

    if(now < lastPoll + 70)
        return;

    lastPoll = now;    

    uint8_t buf[2] = { 0x80, 0x8a};
    _size = 0;

    hottTLMport.enableTx(true);
    hottTLMport.write(buf, 2);
    hottTLMport.enableTx(false);

/*
    uint8_t size = hottInputBuffer.size();

    if(hottInputBuffer.size()) {
        uint8_t buf[size];
        hottInputBuffer.popBytes(buf, size);
        _outputPort->write(buf, size);
    }
*/
/*
    auto size = hottOutputBuffer.size();
    uint8_t buf[size];
    hottOutputBuffer.popBytes(buf, size);
    _outputPort->write(buf, size);
*/
}
#endif
