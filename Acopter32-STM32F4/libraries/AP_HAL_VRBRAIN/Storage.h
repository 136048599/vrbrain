/* -*- Mode: C++; indent-tabs-mode: nil; c-basic-offset: 2 -*- */
/*
 * Storage.h --- AP_HAL_VRBRAIN storage driver.
 *
 * Copyright (C) 2013, Virtualrobotix.com Roberto Navoni , Emile 
 * All Rights Reserved.
 *
 * This software is released under the "BSD3" license.  Read the file
 * "LICENSE" for more information.
 *
 * Written by Roberto Navoni  <info@virtualrobotix.com>, 11 January 2013
 */

#ifndef __AP_HAL_VRBRAIN_STORAGE_H__
#define __AP_HAL_VRBRAIN_STORAGE_H__

#include <AP_HAL_VRBRAIN.h>
#include <i2c.h>
#include <hal.h>

#define MC24C64		//Defines the EEPROM MC24C64
#define EEPROM_ADDRESS	0xA0

#define EEPROM_PAGE_SIZE	(uint32_t)0x10000
#define EEPROM_START_ADDRESS	0x00

class VRBRAIN::VRBRAINStorage : public AP_HAL::Storage
{
public:
  VRBRAINStorage(i2c_dev *dev, AP_HAL::Semaphore* semaphore):
  _dev(dev),
  _sem(semaphore)
  {};
  void init(void* machtnichts);
  void     read_block(void *dst, uint16_t src, size_t n);

  void write_block(uint16_t dst, const void* src, size_t n);
private:
  i2c_dev *_dev;
  AP_HAL::Semaphore*  _sem;
};

#endif // __AP_HAL_VRBRAIN_STORAGE_H__
