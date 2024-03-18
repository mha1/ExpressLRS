#pragma once

#include "SerialIO.h"
#include "crc.h"

#define SUMDBAUD    115200
#define SUMDCONFIG  SERIAL_8N1

#define SUMD_HEADER_SIZE		3														// 3 Bytes header
#define SUMD_DATA_SIZE_16CH		(16*2)													// 2 Bytes per channel
#define SUMD_CRC_SIZE			2														// 16 bit CRC
#define SUMD_FRAME_16CH_LEN		(SUMD_HEADER_SIZE+SUMD_DATA_SIZE_16CH+SUMD_CRC_SIZE)

const auto SUMD_CALLBACK_INTERVAL_MS = 10;

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
