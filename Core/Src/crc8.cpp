/**
 *************************************************************************************************
 * @file crc8.c
 * @brief Implementation file for CRC-8 calculation functions.
 * 
 * For more information on CRC calculation in C, see \
 * \link https://barrgroup.com/embedded-systems/how-to/crc-calculation-c-code \endlink.
 **************************************************************************************************
 */

#include <stdint.h>
#include "crc8.hpp"

/**
 * @brief Initializes the CRC-8 lookup table.
 *
 * This function initializes the CRC-8 lookup table based on the given polynomial.
 *
 * @param table Pointer to the CRC-8 lookup table.
 * @param poly The polynomial used for CRC-8 calculation.
 */
void init_CRC_Table(uint8_t *table, const uint8_t poly)
{
    uint8_t remainder;
    for (int dividend = 0; dividend < 256; ++dividend)
    {
        remainder = dividend << (WIDTH - 8);

        for (uint8_t b = 8; b > 0; --b)
        {
            if (remainder & TOP_BIT)
            {
                remainder = (remainder << 1) ^ poly;
            }
            else
            {
                remainder = (remainder << 1);
            }
        }
        table[dividend] = remainder;
    }
}

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
uint8_t get_CRC_Hash(const uint8_t *buf, const uint16_t len, const uint8_t *table)
{
    uint8_t data;
    uint8_t remainder = 0;

    for (uint16_t index = 0; index < len; ++index)
    {
        data = buf[index] ^ (remainder >> (WIDTH - 8));
        remainder = table[data] ^ (remainder << 8);
    }

    return remainder;
}
