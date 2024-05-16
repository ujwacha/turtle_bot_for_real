/***********************************************************************************************
 * @file crc8.cpp
 * @brief Implementation file for the crc8 class
 * @author Robotics Team, IOE Pulchowk Campus
 * @date 2023
 **********************************************************************************************/

#include "crc8.hpp"

/**
 * @brief Constructor for CRC8 class.
 * 
 * This constructor initializes the CRC8 object with the specified polynomial.
 * It pre-computes the CRC hash table used for CRC calculation.
 * 
 * @param poly The polynomial value for CRC calculation.
 */
CRC8::CRC8(uint8_t poly)
{
    uint8_t remainder;
    int dividend = 0;

    for (; dividend < 256; ++dividend)
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
        table_[dividend] = remainder;
    }
}

/**
 * @brief Calculate CRC-8 checksum for the given data.
 * 
 * This method calculates the CRC-8 checksum for the provided data buffer.
 * 
 * @param buf Pointer to the data buffer.
 * @param len Length of the data buffer.
 * @return CRC-8 checksum value.
 */
uint8_t CRC8::get_hash(uint8_t *buf, uint16_t len)
{
    uint8_t data;
    uint8_t remainder = 0;

    for (uint16_t index = 0; index < len; ++index)
    {
        data = buf[index] ^ (remainder >> (WIDTH - 8));
        remainder = table_[data] ^ (remainder << 8);
    }

    return remainder;
}