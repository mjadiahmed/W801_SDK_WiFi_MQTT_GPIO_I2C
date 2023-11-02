#ifndef I2C_UTILS_H
#define I2C_UTILS_H

#include "wm_include.h"
#include "wm_i2c.h"
#include <string.h>
#include "wm_gpio_afsel.h"

// Function to read one byte from the specified address of the eeprom
u8 i2c_bus_ReadOneByte(u16 addr);

// Function to read multiple bytes from the specified address of the eeprom
void i2c_bus_ReadLenByte(u16 addr, u8 *buf, u16 len);

// Function to write one byte to the specified address of the eeprom
void i2c_bus_WriteOneByte(u16 addr, u8 data);

// Function to check if the eeprom is normal or not
u8 i2c_bus_Check(void);

// Function to read multiple bytes from the specified address of the eeprom
void i2c_bus_Read(u16 addr, u8 *buf, u16 len);

// Function to write multiple bytes from the specified address of the eeprom
void i2c_bus_Write(u16 addr, u8 *buf, u16 len);

// Function to perform the I2C demo
int i2c_sensor_demo(char *buf);





#endif
