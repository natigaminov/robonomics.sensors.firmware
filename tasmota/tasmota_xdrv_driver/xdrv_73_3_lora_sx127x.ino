/*
  xdrv_73_3_lora_sx127x.ino - LoRa support for Tasmota

  SPDX-FileCopyrightText: 2024 Theo Arends

  SPDX-License-Identifier: GPL-3.0-only
*/

#ifdef USE_SPI
#ifdef USE_SPI_LORA
#ifdef USE_LORA_SX127X
/*********************************************************************************************\
 * Legacy Semtech SX1276/77/78/79 Long Range (LoRa)
 * - LilyGo TTGO T3 LoRa32 868MHz ESP32 (uses SX1276)
 * - LilyGo TTGO T-Higrow 868MHz (uses SX1276)
 * - DFRobot FireBeetle Covers LoRa Radio 868MHz (uses SX1278)
 * - M5Stack LoRa868 (uses AI-01 which uses SX1276)
 *
 * Used GPIO's:
 * - SPI_CLK
 * - SPI_MISO
 * - SPI_MOSI
 * - LoRa_CS
 * - LoRa_DIO0
 * - LoRa_Rst
 * - LoRa_DIO1
\*********************************************************************************************/

//#define USE_LORA_SX127X_DEBUG

#include <RadioLib.h>
SX1276 LoRaRadioSX1276 = nullptr;                // Select LoRa support
LoRaWANNode node(&LoRaRadioSX1276, &EU868);      // For LoRaWAN connect

bool LoraSx127xBusy(void) {
  // This is not consistently implemented in the used library
  uint32_t timeout;
  SetNextTimeInterval(timeout, 100);
  while (!TimeReached(timeout)) {
    delay(0);
  }
  return TimeReached(timeout);
}

/*********************************************************************************************/

void IRAM_ATTR LoraSx127xOnInterrupt(void);
void LoraSx127xOnInterrupt(void) {
  // This is called after EVERY type of enabled interrupt so chk for valid receivedFlag in LoraAvailableSx127x()
  if (!Lora->send_flag && !Lora->received_flag && !Lora->receive_time) {
    Lora->receive_time = millis();
  }
  Lora->received_flag = true;              // we got a packet, set the flag
}

bool LoraSx127xAvailable(void) {
  if (Lora->send_flag) {
    Lora->received_flag = false;           // Reset receive flag as it was caused by send interrupt

    uint32_t time = millis();
    int state = LoRaRadioSX1276.startReceive();  // Put module back to listen mode

    Lora->send_flag = false;
    if (state != RADIOLIB_ERR_NONE) {
      AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("S7X: Rcvd (%d) restarted (%d)"), time, state);
    }
  }
  else if (Lora->received_flag) {
    uint32_t irq_stat = LoRaRadioSX1276.getIRQFlags();

#ifdef USE_LORA_SX127X_DEBUG
    if (irq_stat != 0) {
      AddLog(LOG_LEVEL_DEBUG_MORE, PSTR("S7X: Flag (%d) irq %04X"), millis(), irq_stat);
    }
#endif  // USE_LORA_SX127X_DEBUG

    if (0 == (irq_stat & RADIOLIB_SX127X_CLEAR_IRQ_FLAG_RX_DONE)) {
      Lora->received_flag = false;         // Reset receive flag
    }
  }
  return (Lora->received_flag);            // Check if the receive flag is set
}

int LoraSx127xReceive(char* data) {
  Lora->received_flag = false;             // Reset flag
  int packet_size = LoRaRadioSX1276.getPacketLength();
  int state = LoRaRadioSX1276.readData((uint8_t*)data, TAS_LORA_MAX_PACKET_LENGTH -1);

  // LoRaWan downlink frames are sent without CRC, which will raise error on SX127x. We can ignore that error
  if (RADIOLIB_ERR_CRC_MISMATCH == state) {
    state = RADIOLIB_ERR_NONE;
    AddLog(LOG_LEVEL_DEBUG, PSTR("S7X: Ignoring CRC error"));
  }
  if (RADIOLIB_ERR_NONE == state) {
    Lora->rssi = LoRaRadioSX1276.getRSSI();
    Lora->snr = LoRaRadioSX1276.getSNR();
  } else {
    packet_size = 0;                       // Some other error occurred
    AddLog(LOG_LEVEL_DEBUG, PSTR("S7X: Receive error %d"), state);
  }
  return packet_size;
}

bool LoraSx127xSend(uint8_t* data, uint32_t len, bool invert) {
  int state1 = RADIOLIB_ERR_NONE;
  int state2 = RADIOLIB_ERR_NONE;

  if (invert) {
    LoRaRadioSX1276.standby();
    state1 = LoRaRadioSX1276.invertIQ(false);
    LoraSx127xBusy();
  }

  int state = LoRaRadioSX1276.transmit(data, len);	
  Lora->send_flag = true;                  // Use this flag as LoRaRadioSX1276.transmit enable send interrupt
	if (invert) {
    LoraSx127xBusy();
    state2 = LoRaRadioSX1276.invertIQ(false);
    LoRaRadioSX1276.standby();
  }
  if (state != RADIOLIB_ERR_NONE || state1 != RADIOLIB_ERR_NONE || state2 != RADIOLIB_ERR_NONE) {
    AddLog(LOG_LEVEL_DEBUG, PSTR("S7X: Send error %d %d %d %d"), state1, state, state2);
  }
  return (RADIOLIB_ERR_NONE == state);
}

bool LoraSx127xConfig(void) {
  LoRaRadioSX1276.setCodingRate(Lora->settings.coding_rate);
  LoRaRadioSX1276.setSyncWord(Lora->settings.sync_word);
  LoRaRadioSX1276.setPreambleLength(Lora->settings.preamble_length);
  LoRaRadioSX1276.setCurrentLimit(Lora->settings.current_limit);
	if (Lora->settings.crc_bytes) {
    LoRaRadioSX1276.setCRC(true);
  } else { 
		LoRaRadioSX1276.setCRC(false);
  }
  LoRaRadioSX1276.setCRC(true);
  LoRaRadioSX1276.setSpreadingFactor(Lora->settings.spreading_factor);
  LoRaRadioSX1276.setBandwidth(Lora->settings.bandwidth);
  LoRaRadioSX1276.setFrequency(Lora->settings.frequency);
  LoRaRadioSX1276.setOutputPower(Lora->settings.output_power);
  // if (Lora->settings.implicit_header) {
  //   !LoRaRadioSX1276.implicitHeader(Lora->settings.implicit_header);
  // } else {
  //   !LoRaRadioSX1276.explicitHeader();
  // }
  !LoRaRadioSX1276.invertIQ(false);
  return true;
}

bool LoraSx127xInit(void) {
  LoRaRadioSX1276 = new Module(Pin(GPIO_LORA_CS), Pin(GPIO_LORA_DI0), Pin(GPIO_LORA_RST), Pin(GPIO_LORA_DI1));
	int16_t state = LoRaRadioSX1276.begin(Lora->settings.frequency);
  if (RADIOLIB_ERR_NONE == state) {
    LoraSx127xConfig();
    LoRaRadioSX1276.setDio0Action(LoraSx127xOnInterrupt, RISING);
    if (RADIOLIB_ERR_NONE == LoRaRadioSX1276.startReceive()) {
      return true;
    }
  }
  return false;
}

#endif  // USE_LORA_SX127X
#endif  // USE_SPI_LORA
#endif  // USE_SPI
