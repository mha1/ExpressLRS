#pragma once

#include "SerialIO.h"
#include "crc.h"

#define SUMDBAUD    115200
#define SUMDCONFIG  SERIAL_8N1

class SUMD {
public:
    SUMD() { crc2Byte.init(16, 0x1021); }
    ~SUMD() {}

    void prepareSUMD(uint32_t *channelData, uint8_t* outBuffer); 

private:
    Crc2Byte crc2Byte;
};

class SerialSUMD : SUMD, public SerialIO {
public:
    explicit SerialSUMD(Stream &out, Stream &in) : SUMD(), SerialIO(&out, &in) {}
    virtual ~SerialSUMD() {}

    void queueLinkStatisticsPacket() override {}
    void queueMSPFrameTransmission(uint8_t* data) override {}
    uint32_t sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData) override;

private:
    void processBytes(uint8_t *bytes, uint16_t size) override {};
};
