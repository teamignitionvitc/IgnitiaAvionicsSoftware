/**
 * @file sd_card.c
 * @brief SPI SD card driver for RP2040
 *
 * Initializes an SD card in SPI mode and provides raw block read/write.
 * Uses SPI1 on pins defined in config.h.
 */

#include "sd_card.h"
#include "config.h"
#include "hardware/spi.h"
#include "hardware/gpio.h"
#include "pico/stdlib.h"
#include <stdio.h>
#include <string.h>

static bool sd_initialized = false;
static bool sd_sdhc = false;  // true if SDHC/SDXC (block addressing)

// ---------------------------------------------------------------------------
// Low-level SPI helpers
// ---------------------------------------------------------------------------
static void cs_select(void) {
    gpio_put(SD_PIN_CS, 0);
}

static void cs_deselect(void) {
    gpio_put(SD_PIN_CS, 1);
    // Clock out 8 bits so card releases DO
    uint8_t dummy = 0xFF;
    spi_write_blocking(SD_SPI_PORT, &dummy, 1);
}

static uint8_t spi_transfer(uint8_t tx) {
    uint8_t rx;
    spi_write_read_blocking(SD_SPI_PORT, &tx, &rx, 1);
    return rx;
}

static void sd_send_cmd(uint8_t cmd, uint32_t arg, uint8_t crc) {
    spi_transfer(0x40 | cmd);
    spi_transfer((uint8_t)(arg >> 24));
    spi_transfer((uint8_t)(arg >> 16));
    spi_transfer((uint8_t)(arg >> 8));
    spi_transfer((uint8_t)(arg));
    spi_transfer(crc);
}

static uint8_t sd_read_response(void) {
    uint8_t r;
    for (int i = 0; i < 10; i++) {
        r = spi_transfer(0xFF);
        if (r != 0xFF) return r;
    }
    return 0xFF;
}

static bool sd_wait_ready(uint32_t timeout_ms) {
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start) < timeout_ms) {
        if (spi_transfer(0xFF) == 0xFF) return true;
    }
    return false;
}

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------
bool sd_init(void) {
    // Init SPI at slow speed (400 kHz) for card initialization
    spi_init(SD_SPI_PORT, 400000);
    gpio_set_function(SD_PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(SD_PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(SD_PIN_MISO, GPIO_FUNC_SPI);

    // CS as GPIO output, default high (deselected)
    gpio_init(SD_PIN_CS);
    gpio_set_dir(SD_PIN_CS, GPIO_OUT);
    gpio_put(SD_PIN_CS, 1);

    // Send 80+ clock pulses with CS high to put card in native mode
    sleep_ms(10);
    for (int i = 0; i < 10; i++) {
        spi_transfer(0xFF);
    }

    // CMD0: GO_IDLE_STATE (reset)
    cs_select();
    sd_send_cmd(0, 0, 0x95);  // CMD0 CRC
    uint8_t r1 = sd_read_response();
    cs_deselect();

    if (r1 != 0x01) {
        printf("[SD] CMD0 failed (r1=0x%02X)\r\n", r1);
        return false;
    }

    // CMD8: SEND_IF_COND (check voltage range, detect SD v2)
    cs_select();
    sd_send_cmd(8, 0x000001AA, 0x87);  // CMD8 CRC
    r1 = sd_read_response();
    bool sd_v2 = false;
    if (r1 == 0x01) {
        // Read 4 bytes of R7 response
        uint8_t r7[4];
        for (int i = 0; i < 4; i++) r7[i] = spi_transfer(0xFF);
        if (r7[2] == 0x01 && r7[3] == 0xAA) {
            sd_v2 = true;
        }
    }
    cs_deselect();

    // ACMD41: SD_SEND_OP_COND (initialize card)
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start) < 2000) {
        // CMD55 + ACMD41
        cs_select();
        sd_send_cmd(55, 0, 0x65);
        r1 = sd_read_response();
        cs_deselect();

        cs_select();
        sd_send_cmd(41, sd_v2 ? 0x40000000 : 0, 0x77);
        r1 = sd_read_response();
        cs_deselect();

        if (r1 == 0x00) break;  // Card ready
        sleep_ms(10);
    }

    if (r1 != 0x00) {
        printf("[SD] ACMD41 timeout (r1=0x%02X)\r\n", r1);
        return false;
    }

    // CMD58: READ_OCR (check CCS bit for SDHC)
    sd_sdhc = false;
    if (sd_v2) {
        cs_select();
        sd_send_cmd(58, 0, 0xFD);
        r1 = sd_read_response();
        if (r1 == 0x00) {
            uint8_t ocr[4];
            for (int i = 0; i < 4; i++) ocr[i] = spi_transfer(0xFF);
            if (ocr[0] & 0x40) sd_sdhc = true;  // CCS bit set = SDHC
        }
        cs_deselect();
    }

    // Switch to fast SPI speed
    spi_set_baudrate(SD_SPI_PORT, SD_SPI_FAST_BAUD);

    sd_initialized = true;
    printf("[SD] Init OK (%s)\r\n", sd_sdhc ? "SDHC" : "SD");
    return true;
}

bool sd_write_block(uint32_t block_addr, const uint8_t *data) {
    if (!sd_initialized) return false;

    // SDHC uses block addressing, SD uses byte addressing
    uint32_t addr = sd_sdhc ? block_addr : (block_addr * 512);

    cs_select();
    if (!sd_wait_ready(500)) {
        cs_deselect();
        return false;
    }

    // CMD24: WRITE_SINGLE_BLOCK
    sd_send_cmd(24, addr, 0xFF);
    uint8_t r1 = sd_read_response();
    if (r1 != 0x00) {
        cs_deselect();
        return false;
    }

    // Send data token
    spi_transfer(0xFE);

    // Send 512 bytes of data
    spi_write_blocking(SD_SPI_PORT, data, 512);

    // Send dummy CRC
    spi_transfer(0xFF);
    spi_transfer(0xFF);

    // Check data response
    uint8_t resp = spi_transfer(0xFF);
    if ((resp & 0x1F) != 0x05) {
        cs_deselect();
        return false;
    }

    // Wait for write to complete
    sd_wait_ready(500);
    cs_deselect();
    return true;
}

bool sd_read_block(uint32_t block_addr, uint8_t *data) {
    if (!sd_initialized) return false;

    uint32_t addr = sd_sdhc ? block_addr : (block_addr * 512);

    cs_select();
    sd_send_cmd(17, addr, 0xFF);
    uint8_t r1 = sd_read_response();
    if (r1 != 0x00) {
        cs_deselect();
        return false;
    }

    // Wait for data token (0xFE)
    uint32_t start = to_ms_since_boot(get_absolute_time());
    while ((to_ms_since_boot(get_absolute_time()) - start) < 500) {
        uint8_t token = spi_transfer(0xFF);
        if (token == 0xFE) {
            // Read 512 bytes
            spi_read_blocking(SD_SPI_PORT, 0xFF, data, 512);
            // Read and discard CRC
            spi_transfer(0xFF);
            spi_transfer(0xFF);
            cs_deselect();
            return true;
        }
    }

    cs_deselect();
    return false;
}

bool sd_is_initialized(void) {
    return sd_initialized;
}
