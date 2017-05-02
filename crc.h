/*
 * crc.h
 *
 *  Created on: 22 ���. 2017 �.
 *      Author: Andrew
 */

#ifndef CRC_H_
#define CRC_H_

const unsigned char crc8LUT[256] = {
    0x00, 0x18, 0x30, 0x28, 0x60, 0x78, 0x50, 0x48,
    0xC0, 0xD8, 0xF0, 0xE8, 0xA0, 0xB8, 0x90, 0x88,
    0x98, 0x80, 0xA8, 0xB0, 0xF8, 0xE0, 0xC8, 0xD0,
    0x58, 0x40, 0x68, 0x70, 0x38, 0x20, 0x08, 0x10,
    0x28, 0x30, 0x18, 0x00, 0x48, 0x50, 0x78, 0x60,
    0xE8, 0xF0, 0xD8, 0xC0, 0x88, 0x90, 0xB8, 0xA0,
    0xB0, 0xA8, 0x80, 0x98, 0xD0, 0xC8, 0xE0, 0xF8,
    0x70, 0x68, 0x40, 0x58, 0x10, 0x08, 0x20, 0x38,
    0x50, 0x48, 0x60, 0x78, 0x30, 0x28, 0x00, 0x18,
    0x90, 0x88, 0xA0, 0xB8, 0xF0, 0xE8, 0xC0, 0xD8,
    0xC8, 0xD0, 0xF8, 0xE0, 0xA8, 0xB0, 0x98, 0x80,
    0x08, 0x10, 0x38, 0x20, 0x68, 0x70, 0x58, 0x40,
    0x78, 0x60, 0x48, 0x50, 0x18, 0x00, 0x28, 0x30,
    0xB8, 0xA0, 0x88, 0x90, 0xD8, 0xC0, 0xE8, 0xF0,
    0xE0, 0xF8, 0xD0, 0xC8, 0x80, 0x98, 0xB0, 0xA8,
    0x20, 0x38, 0x10, 0x08, 0x40, 0x58, 0x70, 0x68,
    0xA0, 0xB8, 0x90, 0x88, 0xC0, 0xD8, 0xF0, 0xE8,
    0x60, 0x78, 0x50, 0x48, 0x00, 0x18, 0x30, 0x28,
    0x38, 0x20, 0x08, 0x10, 0x58, 0x40, 0x68, 0x70,
    0xF8, 0xE0, 0xC8, 0xD0, 0x98, 0x80, 0xA8, 0xB0,
    0x88, 0x90, 0xB8, 0xA0, 0xE8, 0xF0, 0xD8, 0xC0,
    0x48, 0x50, 0x78, 0x60, 0x28, 0x30, 0x18, 0x00,
    0x10, 0x08, 0x20, 0x38, 0x70, 0x68, 0x40, 0x58,
    0xD0, 0xC8, 0xE0, 0xF8, 0xB0, 0xA8, 0x80, 0x98,
    0xF0, 0xE8, 0xC0, 0xD8, 0x90, 0x88, 0xA0, 0xB8,
    0x30, 0x28, 0x00, 0x18, 0x50, 0x48, 0x60, 0x78,
    0x68, 0x70, 0x58, 0x40, 0x08, 0x10, 0x38, 0x20,
    0xA8, 0xB0, 0x98, 0x80, 0xC8, 0xD0, 0xF8, 0xE0,
    0xD8, 0xC0, 0xE8, 0xF0, 0xB8, 0xA0, 0x88, 0x90,
    0x18, 0x00, 0x28, 0x30, 0x78, 0x60, 0x48, 0x50,
    0x40, 0x58, 0x70, 0x68, 0x20, 0x38, 0x10, 0x08,
    0x80, 0x98, 0xB0, 0xA8, 0xE0, 0xF8, 0xD0, 0xC8
}; // crc8LUT

#endif /* CRC_H_ */
