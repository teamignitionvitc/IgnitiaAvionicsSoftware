/**
 * @file sd_logger.h
 * @brief SD card CSV data logger
 */

#ifndef SD_LOGGER_H
#define SD_LOGGER_H

#include <stdbool.h>

bool sd_logger_init(void);
void sd_logger_write_line(const char *csv_line);
void sd_logger_flush(void);
bool sd_logger_is_ready(void);

#endif // SD_LOGGER_H
