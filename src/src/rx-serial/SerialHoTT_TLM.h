#pragma once

#include "SerialIO.h"
#include "device.h"

//#define HOTT_TLM_ESP32_NO_DIODE

class SerialHoTT_TLM : public SerialIO {
public:
    explicit SerialHoTT_TLM(Stream &out, Stream &in) : SerialIO(&out, &in) {}

    virtual ~SerialHoTT_TLM() {}

    uint32_t sendRCFrameToFC(bool frameAvailable, uint32_t *channelData) override { return DURATION_IMMEDIATELY; }; 
    int getMaxSerialReadSize() override;
    void handleUARTout() override;

    void setLinkQualityStats(uint16_t lq, uint16_t rssi) override { /* not supported */ };
    void sendMSPFrameToFC(uint8_t* data) override { /* not supported */ };
    void sendLinkStatisticsToFC() override { /* not supported */ };

private:
    void processBytes(uint8_t *bytes, u_int16_t size) override;
    
    void processByte(uint8_t byte) override { /* not supported */ };

    void pollNextDevice();
    void pollDevice(uint8_t id);
    void processFrame();
    uint8_t calcFrameCRC(uint8_t *buf);
    
    void scheduleCRSFtelemetry(uint32_t now);
    void sendCRSFvario(uint32_t now);
    void sendCRSFgps(uint32_t now);
    void sendCRSFbattery(uint32_t now);

    uint16_t getHoTTvoltage();
    uint16_t getHoTTcurrent();
    uint32_t getHoTTcapacity();
    int16_t  getHoTTaltitude();
    int16_t  getHoTTvv();
    uint8_t  getHoTTremaining();
    int32_t  getHoTTlatitude();
    int32_t  getHoTTlongitude();
    uint16_t getHoTTgroundspeed();
    uint16_t getHoTTheading();
    uint8_t  getHoTTsatellites();
    uint16_t getHoTTMSLaltitude();

    uint32_t htobe24(uint32_t val);
};
