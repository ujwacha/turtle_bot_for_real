/**
 * @file crc8.h
 * @brief Header file for CRC-8 calculation functions.
 */

#ifndef CRC_H
#define CRC_H

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdint.h>

/**
 * @def WIDTH
 * @brief The width (in bits) of the CRC-8 hash.
 */
#define WIDTH 8

/**
 * @def CRC8_TABLE_SIZE
 * @brief The size of the CRC-8 lookup table.
 */
#define CRC8_TABLE_SIZE 256

/**
 * @def TOP_BIT
 * @brief The top bit of an 8-bit value.
 */
#define TOP_BIT (1 << (WIDTH - 1))

/**
 * @brief Initializes the CRC-8 lookup table.
 *
 * This function initializes the CRC-8 lookup table based on the given polynomial.
 *
 * @param table Pointer to the CRC-8 lookup table.
 * @param poly The polynomial used for CRC-8 calculation.
 */
void init_CRC_Table(uint8_t *table, const uint8_t poly);

/**
 * @brief Calculates the CRC-8 hash for the given buffer.
 *
 * This function calculates the CRC-8 hash for the specified buffer using the precomputed lookup table.
 *
 * @param buf Pointer to the input buffer.
 * @param len Length of the input buffer.
 * @param table Pointer to the precomputed CRC-8 lookup table.
 * @return The CRC-8 hash value.
 */
uint8_t get_CRC_Hash(const uint8_t *buf, const uint16_t len, const uint8_t *table);

#ifdef __cplusplus
}
#endif

#endif // CRC_
