#if !defined(_RADIOLIB_LORAWAN_RELAY_H) && !RADIOLIB_EXCLUDE_LORAWAN
#define _RADIOLIB_LORAWAN_RELAY_H

#include "LoRaWAN.h"

// TS011 relay-to-NS MAC command IDs (used on fPort = RADIOLIB_LORAWAN_FPORT_TS011)
#define RADIOLIB_LORAWAN_TS011_CID_FORWARD_UPLINK_REQ             (0x00)
#define RADIOLIB_LORAWAN_TS011_CID_FORWARD_DOWNLINK_ANS           (0x01)
#define RADIOLIB_LORAWAN_TS011_CID_FORWARD_JOIN_REQ               (0x02)
#define RADIOLIB_LORAWAN_TS011_CID_FORWARD_JOIN_ANS               (0x03)

// ForwardUplinkReq / ForwardJoinReq metadata byte layout
// [SNR_margin(4bit) | WOR_ch_idx(2bit) | hop_count(2bit)]
#define RADIOLIB_LORAWAN_TS011_FWD_META_SNR_SHIFT                 (4)
#define RADIOLIB_LORAWAN_TS011_FWD_META_CH_SHIFT                  (2)
#define RADIOLIB_LORAWAN_TS011_FWD_META_HOP_MASK                  (0x03)

// Maximum number of end devices the relay can serve simultaneously
#define RADIOLIB_LORAWAN_RELAY_MAX_DEVICES                        (16)

// Default relay parameters
#define RADIOLIB_LORAWAN_RELAY_DEFAULT_TOFFSET_MS                 (2500)
#define RADIOLIB_LORAWAN_RELAY_DEFAULT_RELAY_DR                   (3)
#define RADIOLIB_LORAWAN_RELAY_WOR_ACK_LATENCY_MS                 (150)
#define RADIOLIB_LORAWAN_RELAY_UPLINK_WINDOW_MS                   (1000)
#define RADIOLIB_LORAWAN_RELAY_RXR_TX_ADVANCE_MS                  (20)

/*!
  \struct RelayedDevice_t
  \brief Per-end-device entry held by the relay.
*/
struct RelayedDevice_t {
  uint32_t devAddr;
  uint8_t worSIntKey[RADIOLIB_AES128_KEY_SIZE];
  uint8_t worSEncKey[RADIOLIB_AES128_KEY_SIZE];
  bool valid;
};

/*!
  \class LoRaWANRelay
  \brief TS011 relay node. Extends LoRaWANNode so the relay can use its own
  LoRaWAN session (beginOTAA / activateOTAA) for relay-to-NS communication,
  while relayLoop() handles the relay-to-end-device protocol side.
*/
class LoRaWANRelay : public LoRaWANNode {
  public:

    /*!
      \brief Constructor.
      \param phy Pointer to the PhysicalLayer radio (shared for both relay-ED and relay-NS links).
      \param band Pointer to the LoRaWAN band definition.
      \param subBand Sub-band index for fixed-channel plans (US915, AU915, CN470).
    */
    LoRaWANRelay(PhysicalLayer* phy, const LoRaWANBand_t* band, uint8_t subBand = 0);

    /*!
      \brief Register an end device that may use this relay.
      The relay must know the end device's WOR session keys to verify WOR frames
      and encrypt WOR-ACK replies. Call this after the end device has joined.
      \param devAddr End device DevAddr.
      \param worSIntKey 16-byte WOR_S_INT_KEY derived during the device's join.
      \param worSEncKey 16-byte WOR_S_ENC_KEY derived during the device's join.
      \returns \ref status_codes
    */
    int16_t addRelayedDevice(uint32_t devAddr, const uint8_t* worSIntKey, const uint8_t* worSEncKey);

    /*!
      \brief Remove a registered end device from the relay table.
      \param devAddr End device DevAddr to remove.
    */
    void removeRelayedDevice(uint32_t devAddr);

    /*!
      \brief Run one relay cycle (blocking).
      Scans WOR channels with CAD; if a WOR or WOR-J frame is detected, handles the
      full relay sequence: WOR-ACK → uplink receive → forward to NS → downlink forward to ED.
      \returns RADIOLIB_ERR_NONE if no activity or relay completed, negative on error.
    */
    int16_t relayLoop();

    /*!
      \brief Set the data rate the relay uses for WOR-ACK and RXR transmissions to end devices.
      \param dr Data rate index.
    */
    void setRelayDataRate(uint8_t dr)   { this->relayDr = dr; }

    /*!
      \brief Set the TOffset value advertised in WOR-ACK (ms from ED uplink end to RXR window open).
      Must be large enough for the relay to forward the uplink to NS and receive the response.
      \param ms Offset in milliseconds.
    */
    void setTOffset(uint16_t ms)        { this->tOffsetMs = ms; }

    /*!
      \brief Set the CADPeriodicity value advertised in WOR-ACK.
      \param p Periodicity index (0 = 1s, per TS011 table).
    */
    void setCADPeriodicity(uint8_t p)   { this->cadPeriodicity = p; }

    /*!
      \brief Set the XTAL accuracy advertised in WOR-ACK (in ppm).
      \param ppm Crystal accuracy.
    */
    void setXTALAccuracy(uint16_t ppm)  { this->xtalAccuracy = ppm; }

#if !RADIOLIB_GODMODE
  private:
#endif

    RelayedDevice_t relayedDevices[RADIOLIB_LORAWAN_RELAY_MAX_DEVICES] = {};

    // relay parameters advertised in WOR-ACK
    uint8_t relayDr = RADIOLIB_LORAWAN_RELAY_DEFAULT_RELAY_DR;
    uint16_t tOffsetMs = RADIOLIB_LORAWAN_RELAY_DEFAULT_TOFFSET_MS;
    uint8_t cadPeriodicity = 0;
    uint16_t xtalAccuracy = 0;

    // look up a registered end device by DevAddr
    RelayedDevice_t* findRelayedDevice(uint32_t devAddr);

    // configure radio for a given channel and direction (wraps setPhyProperties)
    int16_t configureRadioForChannel(const LoRaWANChannel_t* ch, uint8_t dir);

    // handle a regular WOR frame (relay-assisted Class A uplink)
    int16_t handleWorUplink(uint8_t* worBuf, uint8_t chIdx, RadioLibTime_t tWorRx);

    // handle a WOR-J frame (relay-assisted OTAA join)
    int16_t handleWorJoin(const uint8_t* worBuf, uint8_t chIdx, RadioLibTime_t tWorRx);

    // build and transmit a WOR-ACK frame on the specified txAck channel
    int16_t sendWorAckFrame(RelayedDevice_t* dev, uint16_t fCntWor, uint8_t ackChIdx);

    // receive a LoRa frame from an end device on the given freq/DR with timeout
    int16_t receiveEdFrame(uint32_t freq, uint8_t dr, uint8_t* buf, size_t* len, RadioLibTime_t timeoutMs);

    // forward a relayed uplink to the NS via the relay's own LoRaWAN session
    // returns downlink payload in dlBuf / dlLen if the NS responds
    int16_t forwardUplinkToNS(uint32_t devAddr, const uint8_t* frame, size_t frameLen,
                               int8_t snrMargin, uint8_t worChIdx,
                               uint8_t* dlBuf, size_t* dlLen);

    // forward a relayed Join-Request to the NS
    int16_t forwardJoinToNS(const uint8_t* joinFrame, size_t joinFrameLen,
                             uint8_t worChIdx, uint8_t* dlBuf, size_t* dlLen);

    // transmit a forwarded downlink frame to an end device at the scheduled tTx
    int16_t sendForwardedFrame(const uint8_t* frame, size_t frameLen,
                                uint32_t freq, uint8_t dr, RadioLibTime_t tTx);
};

#endif
