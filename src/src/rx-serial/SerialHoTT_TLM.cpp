#if defined(TARGET_RX) && (defined(PLATFORM_ESP8266) || defined(PLATFORM_ESP32))

#include "SerialHoTT_TLM.h"
#include "FIFO.h"
#include "telemetry.h"


#define HOTT_POLL_RATE 150      // default HoTT bus poll rate [ms]
#define HOTT_LEAD_OUT 10        // minimum gap between end of payload to next poll

#define DISCOVERY_TIMEOUT 30000 // 30s device discovery time

#define VARIO_MIN_CRSFRATE 1000 // CRSF telemetry packets will be sent if
#define GPS_MIN_CRSFRATE 5000   // min rate timers in [ms] have expired
#define BATT_MIN_CRSFRATE 5000  // or packet value has changed. Fastest to
                                // be expected update rate will by about 150ms due
                                // to HoTT bus speed if only a HoTT Vario is connected and
                                // values change every HoTT bus poll cycle.

#define SUMD_HEADER_SIZE		3														// 3 Bytes header
#define SUMD_DATA_SIZE_16CH		(16*2)													// 2 Bytes per channel
#define SUMD_CRC_SIZE			2														// 16 bit CRC
#define SUMD_FRAME_16CH_LEN		(SUMD_HEADER_SIZE+SUMD_DATA_SIZE_16CH+SUMD_CRC_SIZE)

const auto SUMD_CALLBACK_INTERVAL_MS = 10;

typedef struct crsf_sensor_gps_s
{
    int32_t latitude;     // degree / 10,000,000 big endian
    int32_t longitude;    // degree / 10,000,000 big endian
    uint16_t groundspeed; // km/h / 10 big endian
    uint16_t heading;     // GPS heading, degree/100 big endian
    uint16_t altitude;    // meters, +1000m big endian
    uint8_t satellites;   // satellites
} PACKED crsf_sensor_gps_t;

extern Telemetry telemetry;
extern bool hottTlmSerialSUMD;

uint32_t SerialHoTT_TLM::sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData)
{
    if (!frameAvailable || !hottTlmSerialSUMD) {
        return DURATION_IMMEDIATELY;
    }

    uint8_t outBuffer[SUMD_FRAME_16CH_LEN];

    outBuffer[0] = 0xA8;	//Graupner
    outBuffer[1] = 0x01;	//SUMD
    outBuffer[2] = 0x10;	//16CH

    uint16_t us = (CRSF_to_US(channelData[0]) << 3);
    outBuffer[3] = us >> 8;
    outBuffer[4] = us & 0x00ff;
    us = (CRSF_to_US(channelData[1]) << 3);
    outBuffer[5] = us >> 8;
    outBuffer[6] = us & 0x00ff;
    us = (CRSF_to_US(channelData[2]) << 3);
    outBuffer[7] = us >> 8;
    outBuffer[8] = us & 0x00ff;
    us = (CRSF_to_US(channelData[3]) << 3);
    outBuffer[9] = us >> 8;
    outBuffer[10] = us & 0x00ff;
    us = (CRSF_to_US(channelData[7]) << 3); //channel 8 mapped to 5 to move arm channel away from the aileron function
    outBuffer[11] = us >> 8;
    outBuffer[12] = us & 0x00ff;
    us = (CRSF_to_US(channelData[5]) << 3);
    outBuffer[13] = us >> 8;
    outBuffer[14] = us & 0x00ff;
    us = (CRSF_to_US(channelData[6]) << 3);
    outBuffer[15] = us >> 8;
    outBuffer[16] = us & 0x00ff;
    us = (CRSF_to_US(channelData[4]) << 3); //channel 5 mapped to 8
    outBuffer[17] = us >> 8;
    outBuffer[18] = us & 0x00ff;
    us = (CRSF_to_US(channelData[8]) << 3);
    outBuffer[19] = us >> 8;
    outBuffer[20] = us & 0x00ff;
    us = (CRSF_to_US(channelData[9]) << 3);
    outBuffer[21] = us >> 8;
    outBuffer[22] = us & 0x00ff;
    us = (CRSF_to_US(channelData[10]) << 3);
    outBuffer[23] = us >> 8;
    outBuffer[24] = us & 0x00ff;
    us = (CRSF_to_US(channelData[11]) << 3);
    outBuffer[25] = us >> 8;
    outBuffer[26] = us & 0x00ff;
    us = (CRSF_to_US(channelData[12]) << 3);
    outBuffer[27] = us >> 8;
    outBuffer[28] = us & 0x00ff;
    us = (CRSF_to_US(channelData[13]) << 3);
    outBuffer[29] = us >> 8;
    outBuffer[30] = us & 0x00ff;
    us = (CRSF_to_US(channelData[14]) << 3);
    outBuffer[31] = us >> 8;
    outBuffer[32] = us & 0x00ff;
    us = (CRSF_to_US(channelData[15]) << 3);
    outBuffer[33] = us >> 8;
    outBuffer[34] = us & 0x00ff;

    uint16_t crc = crc2Byte.calc(outBuffer, (SUMD_HEADER_SIZE + SUMD_DATA_SIZE_16CH), 0);
    outBuffer[35] = (uint8_t)(crc >> 8);
    outBuffer[36] = (uint8_t)(crc & 0x00ff);

    Serial1.write(outBuffer, sizeof(outBuffer));
    //_outputPort->write(outBuffer, sizeof(outBuffer));

    return SUMD_CALLBACK_INTERVAL_MS;
}

int SerialHoTT_TLM::getMaxSerialReadSize()
{
    return HOTT_MAX_BUF_LEN - hottInputBuffer.size();
}

void SerialHoTT_TLM::processBytes(uint8_t *bytes, u_int16_t size)
{
    hottInputBuffer.pushBytes(bytes, size);

    uint8_t bufferSize = hottInputBuffer.size();

    if (bufferSize == sizeof(hottBusFrame))
    {
        // frame complete, prepare to poll next device after lead out time elapsed
        lastPoll = millis() - HOTT_POLL_RATE + HOTT_LEAD_OUT;

        // fetch received serial data
        hottInputBuffer.popBytes((uint8_t *)&hottBusFrame, bufferSize);

        // process received frame if CRC is ok
        if (hottBusFrame.payload[STARTBYTE_INDEX] == START_FRAME_B &&
            hottBusFrame.payload[ENDBYTE_INDEX] == END_FRAME &&
            hottBusFrame.payload[CRC_INDEX] == calcFrameCRC((uint8_t *)&hottBusFrame.payload))
        {
            processFrame();
        }
    }
}

void SerialHoTT_TLM::sendQueuedData(uint32_t maxBytesToSend)
{
    uint32_t now = millis();

    // device discovery timer
    if (discoveryMode && (now - discoveryTimerStart >= DISCOVERY_TIMEOUT))
    {
        discoveryMode = false;
    }

    // device polling scheduler
    if (now - lastPoll >= HOTT_POLL_RATE)
    {
        lastPoll = now;

        // start up in device discovery mode, after timeout regular operation
        pollNextDevice();
    }

    // CRSF packet scheduler
    scheduleCRSFtelemetry(now);
}

void SerialHoTT_TLM::pollNextDevice()
{
    // clear serial in buffer
    hottInputBuffer.flush();

    // work out next device to be polled all in discovery
    // mode, only detected ones in non-discovery mode)
    for (uint i = 0; i < LAST_DEVICE; i++)
    {
        if (nextDevice == LAST_DEVICE)
        {
            nextDevice = FIRST_DEVICE;
        }

        if (device[nextDevice].present || discoveryMode)
        {
            pollDevice(device[nextDevice++].deviceID);

            break;
        }

        nextDevice++;
    }
}

void SerialHoTT_TLM::pollDevice(uint8_t id)
{
    // send data request to device
    _outputPort->write(START_OF_CMD_B);
    _outputPort->write(id);
}

void SerialHoTT_TLM::processFrame()
{
    void *frameData = (void *)&hottBusFrame.payload;

    // store received frame
    switch (hottBusFrame.payload[DEVICE_INDEX])
    {
    case SENSOR_ID_GPS_B:
        device[GPS].present = true;
        memcpy((void *)&gps, frameData, sizeof(gps));
        break;

    case SENSOR_ID_EAM_B:
        device[EAM].present = true;
        memcpy((void *)&eam, frameData, sizeof(eam));
        break;

    case SENSOR_ID_GAM_B:
        device[GAM].present = true;
        memcpy((void *)&gam, frameData, sizeof(gam));
        break;

    case SENSOR_ID_VARIO_B:
        device[VARIO].present = true;
        memcpy((void *)&vario, frameData, sizeof(vario));
        break;

    case SENSOR_ID_ESC_B:
        device[ESC].present = true;
        memcpy((void *)&esc, frameData, sizeof(esc));
        break;
    }
}

uint8_t SerialHoTT_TLM::calcFrameCRC(uint8_t *buf)
{
    uint16_t sum = 0;

    for (uint8_t i = 0; i < FRAME_SIZE - 1; i++)
    {
        sum += buf[i];
    }

    return sum = sum & 0xff;
}

void SerialHoTT_TLM::scheduleCRSFtelemetry(uint32_t now)
{
    // HoTT combined GPS/Vario -> send GPS and vario packet
    if (device[GPS].present)
    {
        sendCRSFgps(now);
        sendCRSFvario(now);
    }
    else
        // HoTT stand alone Vario and no GPS/Vario -> just send vario packet
        if (device[VARIO].present)
        {
            sendCRSFvario(now);
        }

    // HoTT GAM, EAM, ESC -> send batter packet
    if (device[GAM].present || device[EAM].present || device[ESC].present)
    {
        sendCRSFbattery(now);

        // HoTT GAM and EAM but no GPS/Vario or Vario -> send vario packet too
        if ((!device[GPS].present && !device[VARIO].present) && (device[GAM].present || device[EAM].present))
        {
            sendCRSFvario(now);
        }
    }
}

void SerialHoTT_TLM::sendCRSFvario(uint32_t now)
{
    // indicate external sensor is present
    telemetry.SetCrsfBaroSensorDetected();

    // prepare CRSF telemetry packet
    CRSF_MK_FRAME_T(crsf_sensor_baro_vario_t)
    crsfBaro = {0};
    crsfBaro.p.altitude = htobe16(getHoTTaltitude() * 10 + 5000); // Hott 500 = 0m, ELRS 10000 = 0.0m
    crsfBaro.p.verticalspd = htobe16(getHoTTvv() - 30000);
    CRSF::SetHeaderAndCrc((uint8_t *)&crsfBaro, CRSF_FRAMETYPE_BARO_ALTITUDE, CRSF_FRAME_SIZE(sizeof(crsf_sensor_baro_vario_t)), CRSF_ADDRESS_CRSF_TRANSMITTER);

    // send packet only if min rate timer expired or values have changed
    if ((now - lastVarioSent >= VARIO_MIN_CRSFRATE) || (lastVarioCRC != crsfBaro.crc))
    {
        lastVarioSent = now;

        telemetry.AppendTelemetryPackage((uint8_t *)&crsfBaro);
    }

    lastVarioCRC = crsfBaro.crc;
}

void SerialHoTT_TLM::sendCRSFgps(uint32_t now)
{
    // prepare CRSF telemetry packet
    CRSF_MK_FRAME_T(crsf_sensor_gps_t)
    crsfGPS = {0};
    crsfGPS.p.latitude = htobe32(getHoTTlatitude());
    crsfGPS.p.longitude = htobe32(getHoTTlongitude());
    crsfGPS.p.groundspeed = htobe16(getHoTTgroundspeed() * 10); // Hott 1 = 1 km/h, ELRS 1 = 0.1km/h
    crsfGPS.p.heading = htobe16(getHoTTheading() * 100);
    crsfGPS.p.altitude = htobe16(getHoTTMSLaltitude() + 1000); // HoTT 1 = 1m, CRSF: 0m = 1000
    crsfGPS.p.satellites = getHoTTsatellites();
    CRSF::SetHeaderAndCrc((uint8_t *)&crsfGPS, CRSF_FRAMETYPE_GPS, CRSF_FRAME_SIZE(sizeof(crsf_sensor_gps_t)), CRSF_ADDRESS_CRSF_TRANSMITTER);

    // send packet only if min rate timer expired or values have changed
    if ((now - lastGPSSent >= GPS_MIN_CRSFRATE) || (lastGPSCRC != crsfGPS.crc))
    {
        lastGPSSent = now;

        telemetry.AppendTelemetryPackage((uint8_t *)&crsfGPS);
    }

    lastGPSCRC = crsfGPS.crc;
}

void SerialHoTT_TLM::sendCRSFbattery(uint32_t now)
{
    // indicate external sensor is present
    telemetry.SetCrsfBatterySensorDetected();

    // prepare CRSF telemetry packet
    CRSF_MK_FRAME_T(crsf_sensor_battery_t)
    crsfBatt = {0};
    crsfBatt.p.voltage = htobe16(getHoTTvoltage());
    crsfBatt.p.current = htobe16(getHoTTcurrent());
    crsfBatt.p.capacity = htobe24(getHoTTcapacity() * 10); // HoTT: 1 = 10mAh, CRSF: 1 ? 1 = 1mAh
    crsfBatt.p.remaining = getHoTTremaining();
    CRSF::SetHeaderAndCrc((uint8_t *)&crsfBatt, CRSF_FRAMETYPE_BATTERY_SENSOR, CRSF_FRAME_SIZE(sizeof(crsf_sensor_battery_t)), CRSF_ADDRESS_CRSF_TRANSMITTER);

    // send packet only if min rate timer expired or values have changed
    if ((now - lastBatterySent >= BATT_MIN_CRSFRATE) || (lastBatteryCRC != crsfBatt.crc))
    {
        lastBatterySent = now;

        telemetry.AppendTelemetryPackage((uint8_t *)&crsfBatt);
    }

    lastBatteryCRC = crsfBatt.crc;
}

// HoTT telemetry data getters
uint16_t SerialHoTT_TLM::getHoTTvoltage()
{
    if (device[EAM].present)
    {
        return eam.mainVoltage;
    }
    else if (device[GAM].present)
    {
        return (gam.inputVoltage);
    }
    else if (device[ESC].present)
    {
        return esc.inputVoltage;
    }

    return 0;
}

uint16_t SerialHoTT_TLM::getHoTTcurrent()
{
    if (device[EAM].present)
    {
        return eam.current;
    }
    else if (device[GAM].present)
    {
        return gam.current;
    }
    else if (device[ESC].present)
    {
        return esc.current;
    }

    return 0;
}

uint32_t SerialHoTT_TLM::getHoTTcapacity()
{
    if (device[EAM].present)
    {
        return eam.capacity;
    }
    else if (device[GAM].present)
    {
        return (gam.capacity);
    }
    else if (device[ESC].present)
    {
        return esc.capacity;
    }

    return 0;
}

int16_t SerialHoTT_TLM::getHoTTaltitude()
{
    if (device[GPS].present)
    {
        return gps.altitude;
    }
    else if (device[VARIO].present)
    {
        return vario.altitude;
    }
    else if (device[EAM].present)
    {
        return eam.altitude;
    }
    else if (device[GAM].present)
    {
        return gam.altitude;
    }

    return 0;
}

int16_t SerialHoTT_TLM::getHoTTvv()
{
    if (device[GPS].present)
    {
        return (gps.mPerSec);
    }
    else if (device[VARIO].present)
    {
        return (vario.mPerSec);
    }
    else if (device[EAM].present)
    {
        return (eam.mPerSec);
    }
    else if (device[GAM].present)
    {
        return (gam.mPerSec);
    }

    return 0;
}

uint8_t SerialHoTT_TLM::getHoTTremaining()
{
    if (device[GAM].present)
    {
        return gam.fuelScale;
    }

    return 0;
}

int32_t SerialHoTT_TLM::getHoTTlatitude()
{
    if (!device[GPS].present)
    {
        return 0;
    }

    uint8_t deg = gps.latDegMin / DegMinScale;

    int32_t Lat = deg * DegScale +
                  ((gps.latDegMin - (deg * DegMinScale)) * MinScale) / MinDivide +
                  (gps.latSec * SecScale) / MinDivide;

    if (gps.latNS != 0)
    {
        Lat = -Lat;
    }

    return (Lat);
}

int32_t SerialHoTT_TLM::getHoTTlongitude()
{
    if (!device[GPS].present)
    {
        return 0;
    }

    uint8_t deg = gps.lonDegMin / DegMinScale;

    int32_t Lon = deg * DegScale +
                  ((gps.lonDegMin - (deg * DegMinScale)) * MinScale) / MinDivide +
                  (gps.lonSec * SecScale) / MinDivide;

    if (gps.lonEW != 0)
        Lon = -Lon;

    return Lon;
}

uint16_t SerialHoTT_TLM::getHoTTgroundspeed()
{
    if (device[GPS].present)
    {
        return gps.speed;
    }

    return 0;
}

uint16_t SerialHoTT_TLM::getHoTTheading()
{
    if (!device[GPS].present)
    {
        return 0;
    }

    uint16_t heading = gps.direction * 2;

    if (heading > 180)
    {
        heading -= 360;
    }

    return (heading);
}

uint8_t SerialHoTT_TLM::getHoTTsatellites()
{
    if (device[GPS].present)
    {
        return gps.satellites;
    }

    return 0;
}

uint16_t SerialHoTT_TLM::getHoTTMSLaltitude()
{
    if (device[GPS].present)
    {
        return gps.mslAltitude;
    }

    return 0;
}

uint32_t SerialHoTT_TLM::htobe24(uint32_t val)
{
#if (__BYTE_ORDER__ == __ORDER_BIG_ENDIAN__)
    return val;
#else
    uint8_t *ptrByte = (uint8_t *)&val;

    uint8_t swp = ptrByte[0];
    ptrByte[0] = ptrByte[2];
    ptrByte[2] = swp;

    return val;
#endif
}

#endif
