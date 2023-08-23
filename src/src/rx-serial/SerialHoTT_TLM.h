#include "SerialIO.h"
#include "device.h"
#include "SoftwareSerial.h"

class SerialHoTT_TLM : public SerialIO {
public:
    explicit SerialHoTT_TLM(Stream &out, Stream &in) : SerialIO(&out, &in) {}

    virtual ~SerialHoTT_TLM() {
        hottTLMport.end();
    }

    void setLinkQualityStats(uint16_t lq, uint16_t rssi) override {};
    uint32_t sendRCFrameToFC(bool frameAvailable, uint32_t *channelData) override { return DURATION_IMMEDIATELY; }; 
    void sendMSPFrameToFC(uint8_t* data) override {};
    void sendLinkStatisticsToFC() override {};
    void handleUARTin() override {};

    void handleUARTout() override;

private:
    void processBytes(uint8_t *bytes, u_int16_t size) override {};
    void processByte(uint8_t byte) override {};

    void poll(uint8_t id);
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

    EspSoftwareSerial::UART hottTLMport;
};
