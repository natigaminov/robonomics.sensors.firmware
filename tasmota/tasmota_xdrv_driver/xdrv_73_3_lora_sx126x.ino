/*
  xdrv_73_3_lora_sx126x.ino - LoRa support for Tasmota

  SPDX-FileCopyrightText: 2024 Theo Arends

  SPDX-License-Identifier: GPL-3.0-only                                                                                 
*/

#ifdef USE_SPI
#ifdef USE_SPI_LORA
#ifdef USE_LORA_SX126X
/*********************************************************************************************\
 * Latest Semtech SX1261/62 Long Range (LoRa)
 * - LilyGo T3S3 LoRa32 868MHz ESP32S3 (uses SX1262)
 * - LilyGo TTGO T-Weigh ESP32 LoRa 868MHz HX711 (uses SX1262)
 * - Heltec (CubeCell) (uses SX1262)
 * - Waveshare SX1262 Lora Node (HF) and (LF)
 * 
 * Used GPIO's:
 * - SPI_CLK
 * - SPI_MISO 
 * - SPI_MOSI
 * - LoRa_CS
 * - LoRa_Rst
 * - Lora_Busy
 * - Lora_DI1
\*********************************************************************************************/

//#define USE_LORA_SX126X_DEBUG

#include <RadioLib.h>
SX1262 LoRaRadioSX1262 = nullptr;   // Select LoRa support

bool LoraSx126xBusy(void) {
  // This is not consistently implemented in the used library
  uint32_t timeout;
  SetNextTimeInterval(timeout, 100);
  while ((1 == digitalRead(Pin(GPIO_LORA_BUSY))) && !TimeReached(timeout)) {
    delay(0);
  }
  return TimeReached(timeout);
}

/*********************************************************************************************/

void IRAM_ATTR LoraSx126xOnInterrupt(void);
void LoraSx126xOnInterrupt(void) {
  // This is called after EVERY type of enabled interrupt so chk for valid receivedFlag in LoraAvailableSx126x()
  if (!Lora->send_flag && !Lora->received_flag && !Lora->receive_time) {
    Lora->receive_time = millis();
  }
  Lora->received_flag = true;              // we got a packet, set the flag
}

bool LoraSx126xAvailable(void) {
  if (Lora->send_flag) {
    Lora->received_flag = false;           // Reset receive flag as it was caused by send interrupt

    uint32_t time = millis();
    int state = LoRaRadioSX1262.startReceive();  // Put module back to listen mode
    Lora->send_flag = false;
    if (state != RADIOLIB_ERR_NONE) {
      AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("S6X: Rcvd (%d) restarted (%d)"), time, state);
    }
  }
  else if (Lora->received_flag) {
    uint32_t irq_stat = LoRaRadioSX1262.getIrqStatus();

#ifdef USE_LORA_SX126X_DEBUG
    if (irq_stat != 0) {
      AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("S6X: Flag (%d) irq %04X"), millis(), irq_stat);
    }
#endif  // USE_LORA_SX126X_DEBUG

    if (0 == (irq_stat & RADIOLIB_SX126X_IRQ_RX_DONE)) {
      Lora->received_flag = false;         // Reset receive flag
    }
  }
  return (Lora->received_flag);            // Check if the receive flag is set
}

int LoraSx126xReceive(char* data) {
  Lora->received_flag = false;             // Reset flag
  int packet_size = LoRaRadioSX1262.getPacketLength();
  int state = LoRaRadioSX1262.readData((uint8_t*)data, TAS_LORA_MAX_PACKET_LENGTH -1);
  // LoRaWan downlink frames are sent without CRC, which will raise error on SX126x. We can ignore that error
  if (RADIOLIB_ERR_CRC_MISMATCH == state) {
    state = RADIOLIB_ERR_NONE;
    AddLog(LOG_LEVEL_DEBUG, PSTR("S6X: Ignoring CRC error"));
  }
  if (RADIOLIB_ERR_NONE == state) { 
    Lora->rssi = LoRaRadioSX1262.getRSSI();
    Lora->snr = LoRaRadioSX1262.getSNR();
  } else {
    packet_size = 0;                       // Some other error occurred
    AddLog(LOG_LEVEL_DEBUG, PSTR("S6X: Receive error %d"), state);
  }
  return packet_size;
}

bool LoraSx126xSend(uint8_t* data, uint32_t len, bool invert) {
  int state1 = RADIOLIB_ERR_NONE;
  int state2 = RADIOLIB_ERR_NONE;
  if (invert) {
    LoRaRadioSX1262.standby();
    state1 = LoRaRadioSX1262.invertIQ(true);
    LoraSx126xBusy();
  }
  int state = LoRaRadioSX1262.transmit(data, len);
  Lora->send_flag = true;                  // Use this flag as LoRaRadioSX1262.transmit enable send interrupt
  if (invert) {
    LoraSx126xBusy();
    state2 = LoRaRadioSX1262.invertIQ(false);
    LoRaRadioSX1262.standby();
  }
  if (state != RADIOLIB_ERR_NONE || state1 != RADIOLIB_ERR_NONE || state2 != RADIOLIB_ERR_NONE) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("S6X: Send error %d %d %d"), state1, state, state2);
  }
  return (RADIOLIB_ERR_NONE == state);
}

bool LoraSx126xConfig(void) {
  LoRaRadioSX1262.setCodingRate(Lora->settings.coding_rate);
  LoRaRadioSX1262.setSyncWord(Lora->settings.sync_word);
  LoRaRadioSX1262.setPreambleLength(Lora->settings.preamble_length);
  LoRaRadioSX1262.setCurrentLimit(Lora->settings.current_limit);
  LoRaRadioSX1262.setCRC(Lora->settings.crc_bytes);
  LoRaRadioSX1262.setSpreadingFactor(Lora->settings.spreading_factor);
  LoRaRadioSX1262.setBandwidth(Lora->settings.bandwidth);
  LoRaRadioSX1262.setFrequency(Lora->settings.frequency);
  LoRaRadioSX1262.setOutputPower(Lora->settings.output_power);
  if (Lora->settings.implicit_header) { 
    LoRaRadioSX1262.implicitHeader(Lora->settings.implicit_header);
  } else { 
    LoRaRadioSX1262.explicitHeader();
  }
  LoRaRadioSX1262.invertIQ(false);
  return true;
}

bool LoraSx126xInit(void) {
  LoRaRadioSX1262 = new Module(Pin(GPIO_LORA_CS), Pin(GPIO_LORA_DI1), Pin(GPIO_LORA_RST), Pin(GPIO_LORA_BUSY));
  if (RADIOLIB_ERR_NONE == LoRaRadioSX1262.begin(Lora->settings.frequency)) {
    LoraSx126xConfig();
    LoRaRadioSX1262.setDio1Action(LoraSx126xOnInterrupt);
    if (RADIOLIB_ERR_NONE == LoRaRadioSX1262.startReceive()) {
      return true;
    }
  }
  return false;
}

#endif  // USE_LORA_SX126X
#endif  // USE_SPI_LORA
#endif  // USE_SPI
