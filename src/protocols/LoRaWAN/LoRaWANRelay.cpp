#include "LoRaWANRelay.h"
#include <string.h>

#if !RADIOLIB_EXCLUDE_LORAWAN

LoRaWANRelay::LoRaWANRelay(PhysicalLayer* phy, const LoRaWANBand_t* band, uint8_t subBand)
  : LoRaWANNode(phy, band, subBand) {
  memset(this->relayedDevices, 0, sizeof(this->relayedDevices));
}

int16_t LoRaWANRelay::addRelayedDevice(uint32_t devAddr, const uint8_t* worSIntKey, const uint8_t* worSEncKey) {
  // update if already registered
  for(int i = 0; i < RADIOLIB_LORAWAN_RELAY_MAX_DEVICES; i++) {
    if(this->relayedDevices[i].valid && this->relayedDevices[i].devAddr == devAddr) {
      memcpy(this->relayedDevices[i].worSIntKey, worSIntKey, RADIOLIB_AES128_KEY_SIZE);
      memcpy(this->relayedDevices[i].worSEncKey, worSEncKey, RADIOLIB_AES128_KEY_SIZE);
      return(RADIOLIB_ERR_NONE);
    }
  }
  // find free slot
  for(int i = 0; i < RADIOLIB_LORAWAN_RELAY_MAX_DEVICES; i++) {
    if(!this->relayedDevices[i].valid) {
      this->relayedDevices[i].devAddr = devAddr;
      memcpy(this->relayedDevices[i].worSIntKey, worSIntKey, RADIOLIB_AES128_KEY_SIZE);
      memcpy(this->relayedDevices[i].worSEncKey, worSEncKey, RADIOLIB_AES128_KEY_SIZE);
      this->relayedDevices[i].valid = true;
      return(RADIOLIB_ERR_NONE);
    }
  }
  return(RADIOLIB_ERR_NO_CHANNEL_AVAILABLE);
}

void LoRaWANRelay::removeRelayedDevice(uint32_t devAddr) {
  for(int i = 0; i < RADIOLIB_LORAWAN_RELAY_MAX_DEVICES; i++) {
    if(this->relayedDevices[i].valid && this->relayedDevices[i].devAddr == devAddr) {
      memset(&this->relayedDevices[i], 0, sizeof(RelayedDevice_t));
      return;
    }
  }
}

RelayedDevice_t* LoRaWANRelay::findRelayedDevice(uint32_t devAddr) {
  for(int i = 0; i < RADIOLIB_LORAWAN_RELAY_MAX_DEVICES; i++) {
    if(this->relayedDevices[i].valid && this->relayedDevices[i].devAddr == devAddr) {
      return(&this->relayedDevices[i]);
    }
  }
  return(NULL);
}

int16_t LoRaWANRelay::configureRadioForChannel(const LoRaWANChannel_t* ch, uint8_t dir) {
  return(this->setPhyProperties(ch, dir, this->txPowerMax - 2*this->txPowerSteps));
}

int16_t LoRaWANRelay::relayLoop() {
  Module* mod = this->getModule();

  // scan both WOR channels for a preamble (CAD)
  int8_t detectedChIdx = -1;
  for(int i = 0; i < 2; i++) {
    const LoRaWANChannel_t* worCh = &this->band->txWoR[i];
    if(worCh->freq == 0) {
      continue;
    }

    int16_t state = this->configureRadioForChannel(worCh, RADIOLIB_LORAWAN_UPLINK);
    if(state != RADIOLIB_ERR_NONE) {
      continue;
    }

    state = this->phyLayer->scanChannel();
    if(state == RADIOLIB_LORA_DETECTED) {
      detectedChIdx = i;
      break;
    }
  }

  if(detectedChIdx < 0) {
    return(RADIOLIB_ERR_NONE);
  }

  // activity on WOR channel: receive WOR or WOR-J frame
  const LoRaWANChannel_t* worCh = &this->band->txWoR[detectedChIdx];
  // allocate for the larger of the two frame types
  uint8_t worBuf[RADIOLIB_LORAWAN_WOR_JOIN_FRAME_LEN];
  size_t worLen = 0;

  int16_t state = this->receiveEdFrame(worCh->freq, worCh->dr, worBuf, &worLen, 1200);
  if(state != RADIOLIB_ERR_NONE || worLen == 0) {
    return(RADIOLIB_ERR_NONE);
  }

  RadioLibTime_t tWorRx = mod->hal->millis();

  // MHDR must be proprietary
  if((worBuf[0] & RADIOLIB_LORAWAN_MHDR_MTYPE_MASK) != RADIOLIB_LORAWAN_MHDR_MTYPE_PROPRIETARY) {
    return(RADIOLIB_ERR_NONE);
  }

  if(worLen == RADIOLIB_LORAWAN_WOR_FRAME_LEN) {
    return(this->handleWorUplink(worBuf, (uint8_t)detectedChIdx, tWorRx));
  } else if(worLen == RADIOLIB_LORAWAN_WOR_JOIN_FRAME_LEN) {
    return(this->handleWorJoin(worBuf, (uint8_t)detectedChIdx, tWorRx));
  }

  return(RADIOLIB_ERR_NONE);
}

int16_t LoRaWANRelay::relayLoopOnDetected(uint8_t chIdx) {
  Module* mod = this->getModule();
  if(chIdx >= 2 || this->band->txWoR[chIdx].freq == 0) {
    return(RADIOLIB_ERR_NONE);
  }
  const LoRaWANChannel_t* worCh = &this->band->txWoR[chIdx];
  uint8_t worBuf[RADIOLIB_LORAWAN_WOR_JOIN_FRAME_LEN];
  size_t worLen = 0;

  // WOR preamble = 256 symbols at SF9/125kHz ≈ 1048ms, payload ≈ 132ms → frame ≈ 1180ms.
  // CAD may fire at the very start of the preamble, so receive() must cover the full frame.
  int16_t state = this->receiveEdFrame(worCh->freq, worCh->dr, worBuf, &worLen, 1200);
  if(state != RADIOLIB_ERR_NONE || worLen == 0) {
    return(RADIOLIB_ERR_NONE);
  }
  RadioLibTime_t tWorRx = mod->hal->millis();

  if((worBuf[0] & RADIOLIB_LORAWAN_MHDR_MTYPE_MASK) != RADIOLIB_LORAWAN_MHDR_MTYPE_PROPRIETARY) {
    return(RADIOLIB_ERR_NONE);
  }
  if(worLen == RADIOLIB_LORAWAN_WOR_FRAME_LEN) {
    return(this->handleWorUplink(worBuf, chIdx, tWorRx));
  } else if(worLen == RADIOLIB_LORAWAN_WOR_JOIN_FRAME_LEN) {
    return(this->handleWorJoin(worBuf, chIdx, tWorRx));
  }
  return(RADIOLIB_ERR_NONE);
}

int16_t LoRaWANRelay::handleWorUplink(uint8_t* worBuf, uint8_t chIdx, RadioLibTime_t tWorRx) {
  (void)tWorRx;

  // extract WOR fields
  uint32_t devAddr = LoRaWANNode::ntoh<uint32_t>(&worBuf[RADIOLIB_LORAWAN_WOR_DEV_ADDR_POS]);
  uint16_t fCntWor = LoRaWANNode::ntoh<uint16_t>(&worBuf[RADIOLIB_LORAWAN_WOR_FCNT_POS]);
  uint8_t  edDr    = (worBuf[RADIOLIB_LORAWAN_WOR_DR_PERIOD_POS] >> 4) & 0x0F;
  uint32_t edFreq  = LoRaWANNode::ntoh<uint32_t>(&worBuf[RADIOLIB_LORAWAN_WOR_FREQ_POS], 3);

  RADIOLIB_DEBUG_PROTOCOL_PRINTLN("WOR received: DevAddr=%08lx FCnt=%u DR=%u Freq=%lu",
    (unsigned long)devAddr, fCntWor, edDr, (unsigned long)edFreq);

  // look up end device
  RelayedDevice_t* dev = this->findRelayedDevice(devAddr);
  if(dev == NULL) {
    RADIOLIB_DEBUG_PROTOCOL_PRINTLN("WOR: unknown DevAddr %08lx, ignoring", (unsigned long)devAddr);
    return(RADIOLIB_ERR_NONE);
  }

  // verify MIC
  if(!this->verifyMIC(worBuf, RADIOLIB_LORAWAN_WOR_FRAME_LEN, dev->worSIntKey)) {
    RADIOLIB_DEBUG_PROTOCOL_PRINTLN("WOR MIC invalid");
    return(RADIOLIB_ERR_NONE);
  }

  // select txAck channel mirroring the WOR channel index
  uint8_t ackChIdx = (chIdx < 2 && this->band->txAck[chIdx].freq != 0) ? chIdx : 0;

  // send WOR-ACK to end device
  int16_t state = this->sendWorAckFrame(dev, fCntWor, ackChIdx);
  if(state != RADIOLIB_ERR_NONE) {
    return(RADIOLIB_ERR_NONE);
  }

  // receive the actual uplink from end device on signaled freq/DR
  uint8_t ulBuf[255];
  size_t ulLen = 0;
  state = this->receiveEdFrame(edFreq, edDr, ulBuf, &ulLen, RADIOLIB_LORAWAN_RELAY_UPLINK_WINDOW_MS);
  if(state != RADIOLIB_ERR_NONE || ulLen == 0) {
    RADIOLIB_DEBUG_PROTOCOL_PRINTLN("WOR: no uplink received after WOR-ACK");
    return(RADIOLIB_ERR_NONE);
  }

  RadioLibTime_t tUplinkEnd = this->getModule()->hal->millis();
  int8_t snr = (int8_t)this->phyLayer->getSNR();

  RADIOLIB_DEBUG_PROTOCOL_PRINTLN("WOR: uplink received (%d bytes, SNR=%d)", (int)ulLen, (int)snr);

  // forward to NS, receive optional downlink
  uint8_t dlBuf[255];
  size_t dlLen = sizeof(dlBuf);
  state = this->forwardUplinkToNS(devAddr, ulBuf, ulLen, snr, chIdx, dlBuf, &dlLen);
  if(state < RADIOLIB_ERR_NONE) {
    return(RADIOLIB_ERR_NONE);
  }

  // if NS returned a forwarded downlink, send it to end device in RXR window
  if(dlLen > 0) {
    // tTx: end device opens RXR at tUplinkEnd + tOffsetMs; relay transmits slightly before
    RadioLibTime_t tTx = tUplinkEnd + this->tOffsetMs - RADIOLIB_LORAWAN_RELAY_RXR_TX_ADVANCE_MS;
    (void)this->sendForwardedFrame(dlBuf, dlLen, edFreq, this->relayDr, tTx);
  }

  return(RADIOLIB_ERR_NONE);
}

int16_t LoRaWANRelay::handleWorJoin(const uint8_t* worBuf, uint8_t chIdx, RadioLibTime_t tWorRx) {
  (void)tWorRx;

  // extract WOR-J fields
  uint8_t  edDr    = (worBuf[RADIOLIB_LORAWAN_WOR_JOIN_DR_PERIOD_POS] >> 4) & 0x0F;
  uint32_t edFreq  = LoRaWANNode::ntoh<uint32_t>(&worBuf[RADIOLIB_LORAWAN_WOR_JOIN_FREQ_POS], 3);
  uint64_t joinEui = LoRaWANNode::ntoh<uint64_t>(&worBuf[RADIOLIB_LORAWAN_WOR_JOIN_EUI_POS]);
  uint16_t devNonce = LoRaWANNode::ntoh<uint16_t>(&worBuf[RADIOLIB_LORAWAN_WOR_JOIN_DEV_NONCE_POS]);

  RADIOLIB_DEBUG_PROTOCOL_PRINTLN("WOR-J received: JoinEUI=%016llx DevNonce=%u DR=%u Freq=%lu",
    (unsigned long long)joinEui, devNonce, edDr, (unsigned long)edFreq);

  // WOR-J: relay does not have NwkKey for MIC verification; proceed without key check
  // send WOR-ACK on txAck channel (no payload encryption - relay uses zero keys)
  uint8_t ackChIdx = (chIdx < 2 && this->band->txAck[chIdx].freq != 0) ? chIdx : 0;

  // build unencrypted WOR-ACK (relay side: no WOR_S_ENC_KEY for join)
  uint8_t ackFrame[RADIOLIB_LORAWAN_WOR_ACK_FRAME_LEN];
  ackFrame[0] = RADIOLIB_LORAWAN_MHDR_MTYPE_PROPRIETARY | RADIOLIB_LORAWAN_MHDR_MAJOR_R1;

  // plaintext payload (not encrypted for join case since relay lacks NwkKey)
  uint8_t payload[RADIOLIB_LORAWAN_WOR_ACK_PAYLOAD_LEN] = { 0 };
  payload[RADIOLIB_LORAWAN_WOR_ACK_RELAY_DR_POS]   = this->relayDr;
  payload[RADIOLIB_LORAWAN_WOR_ACK_XTAL_ACC_POS]   = (uint8_t)(this->xtalAccuracy & 0xFF);
  payload[RADIOLIB_LORAWAN_WOR_ACK_XTAL_ACC_POS+1] = (uint8_t)(this->xtalAccuracy >> 8);
  payload[RADIOLIB_LORAWAN_WOR_ACK_CAD_PERIOD_POS] = this->cadPeriodicity;
  LoRaWANNode::hton<uint16_t>(&payload[RADIOLIB_LORAWAN_WOR_ACK_TOFFSET_POS], this->tOffsetMs);

  memcpy(&ackFrame[1], payload, RADIOLIB_LORAWAN_WOR_ACK_PAYLOAD_LEN);

  // MIC = 0 for join WOR-ACK (relay has no key; ED will accept with nwkKey attempt)
  memset(&ackFrame[1 + RADIOLIB_LORAWAN_WOR_ACK_PAYLOAD_LEN], 0, 4);

  const LoRaWANChannel_t* ackCh = &this->band->txAck[ackChIdx];
  if(ackCh->freq != 0) {
    int16_t state = this->configureRadioForChannel(ackCh, RADIOLIB_LORAWAN_DOWNLINK);
    if(state == RADIOLIB_ERR_NONE) {
      this->phyLayer->transmit(ackFrame, RADIOLIB_LORAWAN_WOR_ACK_FRAME_LEN);
    }
  }

  // receive the Join-Request from end device
  uint8_t joinBuf[255];
  size_t joinLen = 0;
  int16_t state = this->receiveEdFrame(edFreq, edDr, joinBuf, &joinLen, RADIOLIB_LORAWAN_RELAY_UPLINK_WINDOW_MS);
  if(state != RADIOLIB_ERR_NONE || joinLen == 0) {
    RADIOLIB_DEBUG_PROTOCOL_PRINTLN("WOR-J: no Join-Request received");
    return(RADIOLIB_ERR_NONE);
  }

  RadioLibTime_t tJoinEnd = this->getModule()->hal->millis();
  RADIOLIB_DEBUG_PROTOCOL_PRINTLN("WOR-J: Join-Request received (%d bytes)", (int)joinLen);

  // forward Join-Request to NS
  uint8_t dlBuf[255];
  size_t dlLen = sizeof(dlBuf);
  state = this->forwardJoinToNS(joinBuf, joinLen, chIdx, dlBuf, &dlLen);
  if(state < RADIOLIB_ERR_NONE) {
    return(RADIOLIB_ERR_NONE);
  }

  // forward Join-Accept to end device in RXR window
  if(dlLen > 0) {
    RadioLibTime_t tTx = tJoinEnd + this->tOffsetMs - RADIOLIB_LORAWAN_RELAY_RXR_TX_ADVANCE_MS;
    (void)this->sendForwardedFrame(dlBuf, dlLen, edFreq, this->relayDr, tTx);
  }

  return(RADIOLIB_ERR_NONE);
}

int16_t LoRaWANRelay::sendWorAckFrame(RelayedDevice_t* dev, uint16_t fCntWor, uint8_t ackChIdx) {
  const LoRaWANChannel_t* ackCh = &this->band->txAck[ackChIdx];
  if(ackCh->freq == 0) {
    return(RADIOLIB_ERR_NONE);
  }

  // build plaintext WOR-ACK payload (7 bytes)
  uint8_t payload[RADIOLIB_LORAWAN_WOR_ACK_PAYLOAD_LEN] = { 0 };
  payload[RADIOLIB_LORAWAN_WOR_ACK_CAD_TO_RX_POS]  = 0;   // CadToRx: relay-internal, 0 = immediate
  payload[RADIOLIB_LORAWAN_WOR_ACK_RELAY_DR_POS]   = this->relayDr;
  payload[RADIOLIB_LORAWAN_WOR_ACK_XTAL_ACC_POS]   = (uint8_t)(this->xtalAccuracy & 0xFF);
  payload[RADIOLIB_LORAWAN_WOR_ACK_XTAL_ACC_POS+1] = (uint8_t)(this->xtalAccuracy >> 8);
  payload[RADIOLIB_LORAWAN_WOR_ACK_CAD_PERIOD_POS] = this->cadPeriodicity;
  LoRaWANNode::hton<uint16_t>(&payload[RADIOLIB_LORAWAN_WOR_ACK_TOFFSET_POS], this->tOffsetMs);

  // encrypt payload with WOR_S_ENC_KEY
  // CTR: addr=devAddr, fcnt=fCntWor (lower 16 bits of FCnt_WOR), dir=downlink, port=0, counter=false
  uint8_t ackFrame[RADIOLIB_LORAWAN_WOR_ACK_FRAME_LEN];
  ackFrame[0] = RADIOLIB_LORAWAN_MHDR_MTYPE_PROPRIETARY | RADIOLIB_LORAWAN_MHDR_MAJOR_R1;

  this->processAES(payload, RADIOLIB_LORAWAN_WOR_ACK_PAYLOAD_LEN,
                   dev->worSEncKey, &ackFrame[1],
                   dev->devAddr, (uint32_t)fCntWor,
                   RADIOLIB_LORAWAN_DOWNLINK, 0, false);

  // MIC over MHDR + encrypted payload (8 bytes) using WOR_S_INT_KEY
  uint32_t mic = this->generateMIC(ackFrame, RADIOLIB_LORAWAN_WOR_ACK_FRAME_LEN - sizeof(uint32_t),
                                    dev->worSIntKey);
  LoRaWANNode::hton<uint32_t>(&ackFrame[RADIOLIB_LORAWAN_WOR_ACK_FRAME_LEN - sizeof(uint32_t)], mic);

  // configure radio and transmit
  int16_t state = this->configureRadioForChannel(ackCh, RADIOLIB_LORAWAN_DOWNLINK);
  RADIOLIB_ASSERT(state);

  state = this->phyLayer->transmit(ackFrame, RADIOLIB_LORAWAN_WOR_ACK_FRAME_LEN);
  RADIOLIB_DEBUG_PROTOCOL_PRINTLN("WOR-ACK sent: state=%d", state);

  return(state);
}

int16_t LoRaWANRelay::receiveEdFrame(uint32_t freq, uint8_t dr, uint8_t* buf, size_t* len, RadioLibTime_t timeoutMs) {
  LoRaWANChannel_t ch;
  ch.idx   = 0;
  ch.freq  = freq;
  ch.drMin = dr;
  ch.drMax = dr;
  ch.dr    = dr;

  int16_t state = this->configureRadioForChannel(&ch, RADIOLIB_LORAWAN_UPLINK);
  if(state != RADIOLIB_ERR_NONE) {
    return(state);
  }

  // blocking receive with timeout
  uint8_t tmpBuf[255];
  state = this->phyLayer->receive(tmpBuf, 0, timeoutMs);
  if(state != RADIOLIB_ERR_NONE) {
    *len = 0;
    return(state);
  }

  size_t pktLen = this->phyLayer->getPacketLength(true);
  if(pktLen == 0 || pktLen > 255) {
    *len = 0;
    return(RADIOLIB_ERR_NONE);
  }

  state = this->phyLayer->readData(buf, pktLen);
  if(state == RADIOLIB_ERR_LORA_HEADER_DAMAGED) {
    state = RADIOLIB_ERR_NONE;
  }
  *len = (state == RADIOLIB_ERR_NONE) ? pktLen : 0;

  return(state);
}

int16_t LoRaWANRelay::forwardUplinkToNS(uint32_t devAddr, const uint8_t* frame, size_t frameLen,
                                          int8_t snrMargin, uint8_t worChIdx,
                                          uint8_t* dlBuf, size_t* dlLen) {
  // ForwardUplinkReq: CID(1) + Metadata(1) + PHYPayload[frameLen]
  size_t fwdLen = 2 + frameLen;
  uint8_t* fwdBuf = new uint8_t[fwdLen];
  if(fwdBuf == NULL) {
    return(RADIOLIB_ERR_NONE);
  }

  fwdBuf[0] = RADIOLIB_LORAWAN_TS011_CID_FORWARD_UPLINK_REQ;
  // Metadata: [SNR_margin(4bit, signed) | WOR_ch_idx(2bit) | hop_count(2bit=0)]
  uint8_t snrField = (uint8_t)((snrMargin & 0x0F) << RADIOLIB_LORAWAN_TS011_FWD_META_SNR_SHIFT);
  fwdBuf[1] = snrField | ((worChIdx & 0x03) << RADIOLIB_LORAWAN_TS011_FWD_META_CH_SHIFT);
  memcpy(&fwdBuf[2], frame, frameLen);

  RADIOLIB_DEBUG_PROTOCOL_PRINTLN("ForwardUplinkReq: DevAddr=%08lx len=%d", (unsigned long)devAddr, (int)fwdLen);

  int16_t state = this->sendReceive(fwdBuf, fwdLen, RADIOLIB_LORAWAN_FPORT_TS011,
                                     dlBuf, dlLen);
  delete[] fwdBuf;

  if(state < RADIOLIB_ERR_NONE) {
    *dlLen = 0;
    return(state);
  }

  // parse ForwardDownlinkAns if downlink received
  if(*dlLen > 1 && dlBuf[0] == RADIOLIB_LORAWAN_TS011_CID_FORWARD_DOWNLINK_ANS) {
    // shift out the CID byte
    *dlLen -= 1;
    memmove(dlBuf, &dlBuf[1], *dlLen);
    RADIOLIB_DEBUG_PROTOCOL_PRINTLN("ForwardDownlinkAns: %d bytes for ED", (int)*dlLen);
  } else {
    // no downlink or unrecognized CID
    *dlLen = 0;
  }

  return(RADIOLIB_ERR_NONE);
}

int16_t LoRaWANRelay::forwardJoinToNS(const uint8_t* joinFrame, size_t joinFrameLen,
                                        uint8_t worChIdx, uint8_t* dlBuf, size_t* dlLen) {
  // ForwardJoinReq: CID(1) + Metadata(1) + JoinRequest[joinFrameLen]
  size_t fwdLen = 2 + joinFrameLen;
  uint8_t* fwdBuf = new uint8_t[fwdLen];
  if(fwdBuf == NULL) {
    return(RADIOLIB_ERR_NONE);
  }

  fwdBuf[0] = RADIOLIB_LORAWAN_TS011_CID_FORWARD_JOIN_REQ;
  fwdBuf[1] = (worChIdx & 0x03) << RADIOLIB_LORAWAN_TS011_FWD_META_CH_SHIFT;
  memcpy(&fwdBuf[2], joinFrame, joinFrameLen);

  RADIOLIB_DEBUG_PROTOCOL_PRINTLN("ForwardJoinReq: len=%d", (int)fwdLen);

  int16_t state = this->sendReceive(fwdBuf, fwdLen, RADIOLIB_LORAWAN_FPORT_TS011,
                                     dlBuf, dlLen);
  delete[] fwdBuf;

  if(state < RADIOLIB_ERR_NONE) {
    *dlLen = 0;
    return(state);
  }

  // parse ForwardJoinAns
  if(*dlLen > 1 && dlBuf[0] == RADIOLIB_LORAWAN_TS011_CID_FORWARD_JOIN_ANS) {
    *dlLen -= 1;
    memmove(dlBuf, &dlBuf[1], *dlLen);
    RADIOLIB_DEBUG_PROTOCOL_PRINTLN("ForwardJoinAns: %d bytes Join-Accept for ED", (int)*dlLen);
  } else {
    *dlLen = 0;
  }

  return(RADIOLIB_ERR_NONE);
}

int16_t LoRaWANRelay::sendForwardedFrame(const uint8_t* frame, size_t frameLen,
                                           uint32_t freq, uint8_t dr, RadioLibTime_t tTx) {
  Module* mod = this->getModule();

  // wait until the scheduled TX time
  RadioLibTime_t tNow = mod->hal->millis();
  if(tTx > tNow) {
    mod->hal->delay(tTx - tNow);
  }

  LoRaWANChannel_t rxrCh;
  rxrCh.idx   = 0;
  rxrCh.freq  = freq;
  rxrCh.drMin = dr;
  rxrCh.drMax = dr;
  rxrCh.dr    = dr;

  int16_t state = this->configureRadioForChannel(&rxrCh, RADIOLIB_LORAWAN_DOWNLINK);
  if(state != RADIOLIB_ERR_NONE) {
    return(state);
  }

  state = this->phyLayer->transmit(frame, frameLen);
  RADIOLIB_DEBUG_PROTOCOL_PRINTLN("Forwarded downlink sent to ED: state=%d", state);

  return(state);
}

#endif // !RADIOLIB_EXCLUDE_LORAWAN
