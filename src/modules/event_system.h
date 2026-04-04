/**
 * @file event_system.h
 * @brief Simple event system for decoupled flight control
 */

#ifndef EVENT_SYSTEM_H
#define EVENT_SYSTEM_H

#include <stdint.h>
#include <stdbool.h>
#include "flight_data.h"

/**
 * @brief Flight event types
 */
typedef enum {
    EVENT_NONE = 0,
    EVENT_SYSTEM_INIT,
    EVENT_ARMED,
    EVENT_DISARMED,
    EVENT_FREEFALL_DETECTED,
    EVENT_APOGEE_DETECTED,
    EVENT_DEPLOYMENT_REQUESTED,
    EVENT_DEPLOYMENT_SUCCESS,
    EVENT_DEPLOYMENT_FAILED,
    EVENT_LANDED,
    EVENT_SENSOR_FAULT,
    EVENT_SENSOR_RECOVERED,
    EVENT_MAX
} FlightEvent;

/**
 * @brief Event handler callback function type
 * @param event The event that occurred
 * @param data Optional sensor data context
 */
typedef void (*EventHandler)(FlightEvent event, const SensorData *data);

/**
 * @brief Initialize event system
 */
void event_init(void);

/**
 * @brief Subscribe to an event
 * @param event Event type to subscribe to
 * @param handler Callback function to invoke when event occurs
 * @return true if subscription successful, false if no slots available
 */
bool event_subscribe(FlightEvent event, EventHandler handler);

/**
 * @brief Emit an event to all subscribers
 * @param event Event type to emit
 * @param data Optional sensor data context (can be NULL)
 */
void event_emit(FlightEvent event, const SensorData *data);

/**
 * @brief Get event name as string
 * @param event Event type
 * @return Event name string
 */
const char* event_name(FlightEvent event);

#endif // EVENT_SYSTEM_H
