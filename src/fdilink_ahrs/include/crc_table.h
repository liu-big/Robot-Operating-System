/**
 * @file crc_table.h
 * @author FDILink
 * @brief CRC校验相关的函数声明。
 * @version 1.0
 * @date 2024-07-29
 * 
 * @copyright Copyright (c) 2024
 * 
 */
#ifndef CRC_TABLE_H
#define CRC_TABLE_H

#include <stdint.h>

/**
 * @brief 计算8位CRC校验和。
 * @param p 指向数据缓冲区的指针。
 * @param counter 数据长度。
 * @return uint8_t 计算出的CRC8校验和。
 */
uint8_t CRC8_Table(uint8_t* p, uint8_t counter);

/**
 * @brief 计算16位CRC校验和。
 * @param p 指向数据缓冲区的指针。
 * @param counter 数据长度。
 * @return uint16_t 计算出的CRC16校验和。
 */
uint16_t CRC16_Table(uint8_t *p, uint8_t counter);

/**
 * @brief 计算32位CRC校验和。
 * @param p 指向数据缓冲区的指针。
 * @param counter 数据长度。
 * @return uint32_t 计算出的CRC32校验和。
 */
uint32_t CRC32_Table(uint8_t *p, uint8_t counter);

#endif // CRC_TABLE_H
