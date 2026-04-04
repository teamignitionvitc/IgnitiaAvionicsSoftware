/**
 * @file logger.c
 * @brief Buffered logger with double buffering for non-blocking writes.
 */

#include "logger.h"
#include "state_machine.h"
#include "config.h"

#include <stdio.h>
#include <stdint.h>
#include <string.h>

#define LOGGER_BUFFER_CAPACITY         64u
#define LOGGER_SAMPLE_INTERVAL_MS      100u
#define LOGGER_FLUSH_INTERVAL_MS       500u

typedef struct {
    uint32_t timestamp_ms;
    float altitude_m;
    float velocity_mps;
    float accel_g;
    FlightState state;
} LogEntry;

// IMPROVEMENT (Issue 4): Double buffering for non-blocking writes
// Two buffers: one for active writes, one for flushing
static LogEntry buffer_a[LOGGER_BUFFER_CAPACITY];
static LogEntry buffer_b[LOGGER_BUFFER_CAPACITY];

static LogEntry *active_buffer = buffer_a;    // Buffer currently being written to
static LogEntry *flush_buffer = buffer_b;     // Buffer being flushed (or ready to flush)

static uint16_t active_count = 0;             // Number of entries in active buffer
static uint16_t flush_count = 0;              // Number of entries in flush buffer
static bool flush_pending = false;            // True if flush buffer has data to write

static uint32_t last_sample_time = 0;
static uint32_t last_flush_time = 0;

static void push_entry(const LogEntry *entry) {
    if (active_count < LOGGER_BUFFER_CAPACITY) {
        active_buffer[active_count] = *entry;
        active_count++;
    }
    // If buffer is full, oldest entries are dropped (could also trigger immediate flush)
}

static void flush_entries(void) {
    // If there's already a flush pending, skip (prevents blocking)
    if (flush_pending && flush_count > 0) {
        return;  // Previous flush not complete yet
    }
    
    // Swap buffers: active becomes flush, flush becomes active
    LogEntry *temp = active_buffer;
    active_buffer = flush_buffer;
    flush_buffer = temp;
    
    flush_count = active_count;
    active_count = 0;
    flush_pending = true;
    
    // Perform the actual flush (in real implementation, this would be async)
#if DEBUG_ENABLE
    for (uint16_t i = 0; i < flush_count; i++) {
        const LogEntry *e = &flush_buffer[i];
        printf("$L,%lu,%.2f,%.2f,%.3f,%s\r\n",
               e->timestamp_ms,
               e->altitude_m,
               e->velocity_mps,
               e->accel_g,
               state_name(e->state));
    }
#endif

    flush_count = 0;
    flush_pending = false;
}

void logger_init(void) {
    active_buffer = buffer_a;
    flush_buffer = buffer_b;
    active_count = 0;
    flush_count = 0;
    flush_pending = false;
    last_sample_time = 0;
    last_flush_time = 0;
    
    memset(buffer_a, 0, sizeof(buffer_a));
    memset(buffer_b, 0, sizeof(buffer_b));
}

void logger_write(const SensorData *data) {
    if (!data) {
        return;
    }

    if ((data->timestamp_ms - last_sample_time) < LOGGER_SAMPLE_INTERVAL_MS) {
        return;
    }

    last_sample_time = data->timestamp_ms;

    LogEntry entry = {
        .timestamp_ms = data->timestamp_ms,
        .altitude_m = data->filtered_altitude_m,
        .velocity_mps = data->fused_velocity_mps,
        .accel_g = data->accel_mag_g,
        .state = get_state()
    };

    push_entry(&entry);

    if ((data->timestamp_ms - last_flush_time) >= LOGGER_FLUSH_INTERVAL_MS) {
        flush_entries();
        last_flush_time = data->timestamp_ms;
    }
}

void logger_reset(void) {
    logger_init();
}
