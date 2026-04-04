/**
 * @file event_system.c
 * @brief Simple event system implementation
 */

#include "event_system.h"
#include <string.h>

#define MAX_SUBSCRIBERS_PER_EVENT 4

typedef struct {
    EventHandler handlers[MAX_SUBSCRIBERS_PER_EVENT];
    uint8_t count;
} EventSubscribers;

static EventSubscribers subscribers[EVENT_MAX];

void event_init(void) {
    memset(subscribers, 0, sizeof(subscribers));
}

bool event_subscribe(FlightEvent event, EventHandler handler) {
    if (event >= EVENT_MAX || !handler) {
        return false;
    }
    
    EventSubscribers *subs = &subscribers[event];
    
    // Check if already subscribed
    for (uint8_t i = 0; i < subs->count; i++) {
        if (subs->handlers[i] == handler) {
            return true;  // Already subscribed
        }
    }
    
    // Add new subscriber
    if (subs->count < MAX_SUBSCRIBERS_PER_EVENT) {
        subs->handlers[subs->count] = handler;
        subs->count++;
        return true;
    }
    
    return false;  // No slots available
}

void event_emit(FlightEvent event, const SensorData *data) {
    if (event >= EVENT_MAX) {
        return;
    }
    
    EventSubscribers *subs = &subscribers[event];
    
    // Call all registered handlers
    for (uint8_t i = 0; i < subs->count; i++) {
        if (subs->handlers[i]) {
            subs->handlers[i](event, data);
        }
    }
}

const char* event_name(FlightEvent event) {
    switch (event) {
        case EVENT_NONE: return "NONE";
        case EVENT_SYSTEM_INIT: return "SYSTEM_INIT";
        case EVENT_ARMED: return "ARMED";
        case EVENT_DISARMED: return "DISARMED";
        case EVENT_FREEFALL_DETECTED: return "FREEFALL_DETECTED";
        case EVENT_APOGEE_DETECTED: return "APOGEE_DETECTED";
        case EVENT_DEPLOYMENT_REQUESTED: return "DEPLOYMENT_REQUESTED";
        case EVENT_DEPLOYMENT_SUCCESS: return "DEPLOYMENT_SUCCESS";
        case EVENT_DEPLOYMENT_FAILED: return "DEPLOYMENT_FAILED";
        case EVENT_LANDED: return "LANDED";
        case EVENT_SENSOR_FAULT: return "SENSOR_FAULT";
        case EVENT_SENSOR_RECOVERED: return "SENSOR_RECOVERED";
        default: return "UNKNOWN";
    }
}
