#pragma once
#include <Arduino.h>
namespace app_config {
constexpr char kFirmwareName[] = "xiao-telemetry-bridge";
constexpr char kFirmwareVersion[] = "4.0.0-xiao-communication";
// GNSS TX -> D7 (RX), GNSS RX <- D6 (TX).
constexpr int kGnssRxPin = D7, kGnssTxPin = D6;
// Communication D0 (TX) -> control D6 (RX).
// Communication D1 (RX) <- control D7 (TX).
constexpr int kControlUartRxPin = D1, kControlUartTxPin = D0;
// External microSD SPI.  The communication node writes compact binary records
// at a deliberately low, fixed cadence so storage never competes with control.
constexpr int kSdSckPin = D8, kSdMisoPin = D9, kSdMosiPin = D10;
constexpr int kSdCsPin = 21;
constexpr uint32_t kSdSpiHz = 10000000UL;
// Low-rate, power-loss-conscious logging: each completed record is flushed so
// an abrupt power cut can affect at most the record currently being written.
constexpr uint32_t kSdLogIntervalMs = 2000UL,kSdLogFlushEveryRecords = 1UL;
// The map page stores one coarse point every two minutes.  The current
// position is sent separately at a low rate, so no recent high-rate trail is kept.
constexpr uint32_t kTrackCoarseIntervalMs = 120000UL;
constexpr uint16_t kTrackHistoryCapacity = 144;
constexpr float kTrackLocalRadiusM = 50.0f;
constexpr uint32_t kControlUartBaud = 921600UL;
constexpr uint32_t kGnssBaud = 115200UL;
constexpr uint16_t kGnssUartRxBufferBytes = 2048, kGnssReadBudgetBytes = 512;
constexpr uint16_t kGnssInputLineChars = 127, kGnssMaxSentenceChars = 110;
constexpr uint32_t kGnssSentenceTimeoutMs = 500UL, kGnssNoDataTimeoutMs = 1500UL;
constexpr uint32_t kGnssNavIntervalMs = 100UL;
constexpr char kApSsid[] = "BOAT-CONTROL", kApPassword[] = "12345678";
constexpr uint16_t kHttpPort = 80;
}
