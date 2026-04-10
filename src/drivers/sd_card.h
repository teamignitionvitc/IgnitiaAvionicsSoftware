/**
 * @file sd_card.h
 * @brief SPI SD card driver for RP2040
 */

#ifndef SD_CARD_H
#define SD_CARD_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

bool sd_init(void);
bool sd_write_block(uint32_t block_addr, const uint8_t *data);
bool sd_read_block(uint32_t block_addr, uint8_t *data);
bool sd_is_initialized(void);

#endif // SD_CARD_H
