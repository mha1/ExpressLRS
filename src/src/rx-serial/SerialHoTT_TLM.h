#include "SerialIO.h"
#include "FIFO_GENERIC.h"
#include "telemetry_protocol.h"
#include "ESPSoftwareSerial.h"
#include "device.h"

// Variables / constants for HoTT telemetry //
extern FIFO_GENERIC<AP_MAX_BUF_LEN> hottInputBuffer;
extern FIFO_GENERIC<AP_MAX_BUF_LEN> hottOutputBuffer;

class SerialHoTT_TLM : public SerialIO {
public:
    explicit SerialHoTT_TLM(Stream &out, Stream &in) : SerialIO(&out, &in) {
        Serial.end();

        hottTLMport.begin(19200, SWSERIAL_8E1, GPIO_PIN_RCSIGNAL_TX, GPIO_PIN_RCSIGNAL_TX, false);
        hottTLMport.enableTx(false);
    }

    virtual ~SerialHoTT_TLM() {
        hottTLMport.end();
    }

    void setLinkQualityStats(uint16_t lq, uint16_t rssi) override {};
    uint32_t sendRCFrameToFC(bool frameAvailable, uint32_t *channelData) override { return DURATION_IMMEDIATELY; }; 
    void sendMSPFrameToFC(uint8_t* data) override {};
    void sendLinkStatisticsToFC() override {};

    void handleUARTin() override;
    void handleUARTout() override;

private:
    void processBytes(uint8_t *bytes, u_int16_t size) override {};
    void processByte(uint8_t byte) override {};
    
    EspSoftwareSerial::UART hottTLMport;
};
