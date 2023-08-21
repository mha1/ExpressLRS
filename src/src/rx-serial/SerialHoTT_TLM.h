#include "SerialIO.h"
#include "FIFO_GENERIC.h"
#include "telemetry_protocol.h"
#include <SoftwareSerial.h>
#include "device.h"

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
    void AppendTLMpacket(uint8_t *telemetryPacket);
    uint8_t calcFrameCRC(uint8_t *buf);

    EspSoftwareSerial::UART hottTLMport;
};
