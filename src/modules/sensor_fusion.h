/**
 * @file sensor_fusion.h
 * @brief Sensor filtering and velocity fusion helpers.
 */

#ifndef SENSOR_FUSION_H
#define SENSOR_FUSION_H

#include "flight_data.h"

void fusion_init(void);
void filter_data(SensorData *data);
void fuse_sensors(SensorData *data);
void fusion_reset(void);

#endif // SENSOR_FUSION_H
