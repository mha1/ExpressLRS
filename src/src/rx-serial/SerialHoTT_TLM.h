#include "SerialIO.h"
#include "device.h"
#include "FIFO_GENERIC.h"

#define HOTT_MAX_BUF_LEN    64

extern FIFO_GENERIC<HOTT_MAX_BUF_LEN> hottInputBuffer;

class SerialHoTT_TLM : public SerialIO {
public:
    explicit SerialHoTT_TLM(Stream &out, Stream &in) : SerialIO(&out, &in) {}

    virtual ~SerialHoTT_TLM() {}

    void setLinkQualityStats(uint16_t lq, uint16_t rssi) override {};
    uint32_t sendRCFrameToFC(bool frameAvailable, uint32_t *channelData) override { return DURATION_IMMEDIATELY; }; 
    void sendMSPFrameToFC(uint8_t* data) override {};
    void sendLinkStatisticsToFC() override {};

    int getMaxSerialReadSize() override;
    void handleUARTout() override;

private:
    void processBytes(uint8_t *bytes, u_int16_t size) override;
    void processByte(uint8_t byte) override {};

    void pollNextDevice();
    void processFrame();
    uint8_t calcFrameCRC(uint8_t *buf);
    
    void sendCRSFtelemetry();

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

    uint32_t htobe24(uint32_t val);

    //EspSoftwareSerial::UART hottTLMport;
};
