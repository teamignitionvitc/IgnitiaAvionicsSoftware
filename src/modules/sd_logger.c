/**
 * @file sd_logger.c
 * @brief SD card CSV logger with MBR + FAT16 filesystem and flight state checkpoint
 *
 * Layout:
 *   Sector 0:              MBR with partition table
 *   Sector PART_START:     FAT16 boot sector (BPB)
 *   Sector PART_START+1..  FAT, root directory, data area
 *   Sector 33000:          Flight state checkpoint (raw, outside partition)
 */

#include "sd_logger.h"
#include "sd_card.h"
#include "config.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

#define SD_BLOCK_SIZE       512
#define PART_START          1
#define SECTORS_PER_CLUSTER 4
#define RESERVED_SECTORS    1
#define NUM_FATS            1
#define ROOT_DIR_ENTRIES    512
#define ROOT_DIR_SECTORS    32
#define FAT_SECTORS         32
#define PART_TOTAL_SECTORS  32000

#define FAT_START           (PART_START + RESERVED_SECTORS)
#define ROOT_DIR_START      (FAT_START + FAT_SECTORS)
#define DATA_START          (ROOT_DIR_START + ROOT_DIR_SECTORS)

#define FAT16_EOC           0xFFFF

static uint8_t block_buf[SD_BLOCK_SIZE];

// Logger state
static bool logger_ready = false;
static uint32_t current_cluster = 2;
static uint8_t sector_in_cluster = 0;
static uint16_t buf_pos = 0;
static uint32_t file_size = 0;

// ---------------------------------------------------------------------------
// Write MBR with one FAT16 partition
// ---------------------------------------------------------------------------
static void write_mbr(void) {
    memset(block_buf, 0, SD_BLOCK_SIZE);
    uint8_t *pe = block_buf + 446;
    pe[0] = 0x00;
    pe[1] = 0x00; pe[2] = 0x02; pe[3] = 0x00;
    pe[4] = 0x04;
    pe[5] = 0x01; pe[6] = 0x01; pe[7] = 0x01;
    pe[8]  = (PART_START) & 0xFF;
    pe[9]  = (PART_START >> 8) & 0xFF;
    pe[10] = (PART_START >> 16) & 0xFF;
    pe[11] = (PART_START >> 24) & 0xFF;
    pe[12] = (PART_TOTAL_SECTORS) & 0xFF;
    pe[13] = (PART_TOTAL_SECTORS >> 8) & 0xFF;
    pe[14] = (PART_TOTAL_SECTORS >> 16) & 0xFF;
    pe[15] = (PART_TOTAL_SECTORS >> 24) & 0xFF;
    block_buf[510] = 0x55;
    block_buf[511] = 0xAA;
    sd_write_block(0, block_buf);
}

// ---------------------------------------------------------------------------
// Write FAT16 Boot Sector (BPB)
// ---------------------------------------------------------------------------
static void write_boot_sector(void) {
    memset(block_buf, 0, SD_BLOCK_SIZE);
    block_buf[0] = 0xEB; block_buf[1] = 0x3C; block_buf[2] = 0x90;
    memcpy(block_buf + 3, "MSDOS5.0", 8);
    block_buf[11] = 0x00; block_buf[12] = 0x02;
    block_buf[13] = SECTORS_PER_CLUSTER;
    block_buf[14] = RESERVED_SECTORS & 0xFF; block_buf[15] = 0;
    block_buf[16] = NUM_FATS;
    block_buf[17] = ROOT_DIR_ENTRIES & 0xFF;
    block_buf[18] = (ROOT_DIR_ENTRIES >> 8) & 0xFF;
    block_buf[19] = PART_TOTAL_SECTORS & 0xFF;
    block_buf[20] = (PART_TOTAL_SECTORS >> 8) & 0xFF;
    block_buf[21] = 0xF8;
    block_buf[22] = FAT_SECTORS & 0xFF;
    block_buf[23] = (FAT_SECTORS >> 8) & 0xFF;
    block_buf[24] = 0x3F; block_buf[25] = 0x00;
    block_buf[26] = 0xFF; block_buf[27] = 0x00;
    block_buf[28] = PART_START & 0xFF;
    block_buf[29] = (PART_START >> 8) & 0xFF;
    block_buf[30] = 0; block_buf[31] = 0;
    block_buf[32] = 0; block_buf[33] = 0;
    block_buf[34] = 0; block_buf[35] = 0;
    block_buf[36] = 0x80;
    block_buf[37] = 0x00;
    block_buf[38] = 0x29;
    block_buf[39] = 0xCA; block_buf[40] = 0x4E;
    block_buf[41] = 0x5A; block_buf[42] = 0x54;
    memcpy(block_buf + 43, "IGNITIA LOG", 11);
    memcpy(block_buf + 54, "FAT16   ", 8);
    block_buf[510] = 0x55;
    block_buf[511] = 0xAA;
    sd_write_block(PART_START, block_buf);
}

// ---------------------------------------------------------------------------
// Initialize FAT
// ---------------------------------------------------------------------------
static void write_fat(void) {
    for (uint32_t s = 0; s < FAT_SECTORS; s++) {
        memset(block_buf, 0, SD_BLOCK_SIZE);
        if (s == 0) {
            block_buf[0] = 0xF8; block_buf[1] = 0xFF;
            block_buf[2] = 0xFF; block_buf[3] = 0xFF;
            block_buf[4] = 0xFF; block_buf[5] = 0xFF;
        }
        sd_write_block(FAT_START + s, block_buf);
    }
}

// ---------------------------------------------------------------------------
// Create root directory with LOG.CSV
// ---------------------------------------------------------------------------
static void write_root_dir(void) {
    memset(block_buf, 0, SD_BLOCK_SIZE);
    for (uint32_t i = 0; i < ROOT_DIR_SECTORS; i++) {
        sd_write_block(ROOT_DIR_START + i, block_buf);
    }
    memset(block_buf, 0, SD_BLOCK_SIZE);
    memcpy(block_buf + 0, "LOG     ", 8);
    memcpy(block_buf + 8, "CSV", 3);
    block_buf[11] = 0x20;
    block_buf[26] = 2;
    block_buf[27] = 0;
    sd_write_block(ROOT_DIR_START, block_buf);
}

// ---------------------------------------------------------------------------
// Update file size in root directory
// ---------------------------------------------------------------------------
static void update_file_size(void) {
    sd_read_block(ROOT_DIR_START, block_buf);
    block_buf[28] = (file_size) & 0xFF;
    block_buf[29] = (file_size >> 8) & 0xFF;
    block_buf[30] = (file_size >> 16) & 0xFF;
    block_buf[31] = (file_size >> 24) & 0xFF;
    sd_write_block(ROOT_DIR_START, block_buf);
}

// ---------------------------------------------------------------------------
// Extend FAT chain
// ---------------------------------------------------------------------------
static void fat_extend_chain(uint32_t prev_cluster, uint32_t new_cluster) {
    uint32_t prev_sec = FAT_START + (prev_cluster * 2) / SD_BLOCK_SIZE;
    uint32_t prev_off = (prev_cluster * 2) % SD_BLOCK_SIZE;
    sd_read_block(prev_sec, block_buf);
    block_buf[prev_off] = new_cluster & 0xFF;
    block_buf[prev_off + 1] = (new_cluster >> 8) & 0xFF;
    sd_write_block(prev_sec, block_buf);

    uint32_t new_sec = FAT_START + (new_cluster * 2) / SD_BLOCK_SIZE;
    uint32_t new_off = (new_cluster * 2) % SD_BLOCK_SIZE;
    sd_read_block(new_sec, block_buf);
    block_buf[new_off] = 0xFF;
    block_buf[new_off + 1] = 0xFF;
    sd_write_block(new_sec, block_buf);
}

static uint32_t cluster_to_sector(uint32_t cluster) {
    return DATA_START + (cluster - 2) * SECTORS_PER_CLUSTER;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
bool sd_logger_init(void) {
    if (!sd_init()) {
        printf("[SD_LOG] SD card init failed\r\n");
        return false;
    }

    printf("[SD_LOG] Creating FAT16 filesystem with MBR...\r\n");
    write_mbr();
    write_boot_sector();
    write_fat();
    write_root_dir();

    // CSV header — matches Core 1 telemetry format
    const char *header = "time_ms,state,alt,temp,press_hPa,baro_alt,"
                         "ax,ay,az,gx,gy,gz,"
                         "lat,lon,gps_alt,sats,speed,fix,"
                         "vel,max_alt,uv,deployed,gp12\r\n";

    current_cluster = 2;
    sector_in_cluster = 0;
    buf_pos = 0;
    file_size = 0;

    memset(block_buf, 0, SD_BLOCK_SIZE);
    size_t hdr_len = strlen(header);
    memcpy(block_buf, header, hdr_len);
    buf_pos = (uint16_t)hdr_len;
    file_size = (uint32_t)hdr_len;

    logger_ready = true;
    printf("[SD_LOG] Ready — logging to LOG.CSV\r\n");
    return true;
}

void sd_logger_write_line(const char *csv_line) {
    if (!logger_ready || !csv_line) return;
    size_t len = strlen(csv_line);
    if (len == 0) return;

    if (buf_pos + len >= SD_BLOCK_SIZE) {
        sd_logger_flush();
    }

    size_t copy_len = len;
    if (copy_len > (size_t)(SD_BLOCK_SIZE - buf_pos)) {
        copy_len = SD_BLOCK_SIZE - buf_pos;
    }
    memcpy(block_buf + buf_pos, csv_line, copy_len);
    buf_pos += (uint16_t)copy_len;
    file_size += (uint32_t)copy_len;
}

void sd_logger_flush(void) {
    if (!logger_ready || buf_pos == 0) return;

    if (buf_pos < SD_BLOCK_SIZE) {
        memset(block_buf + buf_pos, 0, SD_BLOCK_SIZE - buf_pos);
    }

    uint32_t sector = cluster_to_sector(current_cluster) + sector_in_cluster;

    if (!sd_write_block(sector, block_buf)) {
        printf("[SD_LOG] Write failed sector %lu\r\n", (unsigned long)sector);
        return;
    }

    memset(block_buf, 0, SD_BLOCK_SIZE);
    buf_pos = 0;

    sector_in_cluster++;
    if (sector_in_cluster >= SECTORS_PER_CLUSTER) {
        uint32_t prev = current_cluster;
        current_cluster++;
        sector_in_cluster = 0;
        fat_extend_chain(prev, current_cluster);
    }

    update_file_size();
}

bool sd_logger_is_ready(void) {
    return logger_ready;
}
