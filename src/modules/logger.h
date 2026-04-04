/**
 * @file logger.h
 * @brief Buffered flight logger.
 */

#ifndef LOGGER_H
#define LOGGER_H

#include "flight_data.h"

void logger_init(void);
void logger_write(const SensorData *data);
void logger_reset(void);

#endif // LOGGER_H
