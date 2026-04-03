/**
 * @file nrf_radio.h
 * @brief NRF UART Radio driver for telemetry
 */

#ifndef NRF_RADIO_H
#define NRF_RADIO_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

// Packet types
typedef enum {
    PKT_TELEMETRY = 0x01,   // Regular telemetry data
    PKT_STATUS    = 0x02,   // Status/event message
    PKT_GPS       = 0x03,   // GPS position
    PKT_COMMAND   = 0x04,   // Command (RX)
    PKT_ACK       = 0x05    // Acknowledgment
} PacketType;

// Telemetry packet structure (fixed size for reliability)
#pragma pack(push, 1)
typedef struct {
    uint8_t  header;        // 0xAA
    uint8_t  type;          // PacketType
    uint16_t seq;           // Sequence number
    uint32_t timestamp;     // ms since boot
    uint8_t  state;         // Flight state
    int16_t  altitude;      // dm (decimeters) - altitude * 10
    int16_t  velocity;      // cm/s - velocity * 100
    int16_t  accel;         // mg (milli-g) - accel * 1000
    int16_t  temperature;   // 0.1°C
    uint8_t  gps_sats;      // GPS satellites
    uint8_t  flags;         // Bit flags: deployed, armed, etc.
    uint8_t  checksum;      // XOR checksum
} TelemetryPacket;

typedef struct {
    uint8_t  header;
    uint8_t  type;
    uint16_t seq;
    uint32_t timestamp;
    int32_t  latitude;      // degrees * 1e6
    int32_t  longitude;     // degrees * 1e6
    int16_t  gps_alt;       // meters
    uint8_t  gps_sats;
    uint8_t  hdop;          // hdop * 10
    uint8_t  checksum;
} GPSPacket;

typedef struct {
    uint8_t  header;
    uint8_t  type;
    uint16_t seq;
    uint32_t timestamp;
    char     message[16];
    uint8_t  checksum;
} StatusPacket;
#pragma pack(pop)

// Flags byte bits
#define FLAG_ARMED      (1 << 0)
#define FLAG_DEPLOYED   (1 << 1)
#define FLAG_GPS_FIX    (1 << 2)
#define FLAG_LOW_BATT   (1 << 3)
#define FLAG_ERROR      (1 << 7)

bool nrf_init(void);
void nrf_send_telemetry(const TelemetryPacket *pkt);
void nrf_send_gps(const GPSPacket *pkt);
void nrf_send_status(const char *message);
void nrf_send_raw(const uint8_t *data, size_t len);
bool nrf_receive(uint8_t *buffer, size_t max_len, size_t *received);
void nrf_process(void);
uint16_t nrf_get_tx_count(void);

#endif // NRF_RADIO_H
