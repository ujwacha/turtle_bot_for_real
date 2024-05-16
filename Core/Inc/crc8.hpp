/**
 ******************************************************************************
 * @file    crc8.hpp
 * @brief   Header file 8-bit crc calculation
 * @author  Robotics Team, IOE Pulchowk Campus
 ******************************************************************************
 */

#ifndef CRC8_HPP_
#define CRC8_HPP_

#include <stdint.h>

#define WIDTH (8)                       /**< CRC width */
#define CRC_HASH_TABLE_SIZE (256)       /**< Size of CRC hash table */
#define TOP_BIT (1 << 7)                /**< Top bit position */

/**
 * @brief Class for CRC-8 checksum calculation.
 */
class CRC8
{
public:
    /**
     * @brief Constructor for CRC8 class.
     * @param polynomial Polynomial value for CRC calculation.
     */
    CRC8(uint8_t polynomial);

    /**
     * @brief Destructor for CRC8 class.
     */
    ~CRC8() {}

    /**
     * @brief Calculate CRC-8 checksum for the given data.
     * @param buf Pointer to the data buffer.
     * @param len Length of the data buffer.
     * @return CRC-8 checksum value.
     */
    uint8_t get_hash(uint8_t *buf, uint16_t len);

private:
    uint8_t table_[CRC_HASH_TABLE_SIZE]; /**< CRC hash table. */
};

#endif // CRC_HPP_