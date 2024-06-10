#ifndef LIS3DH_H
#define LIS3DH_H

#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>

// Register address map
#define WHO_AM_I 0x0F
#define CTRL_REG1 0x20
#define CTRL_REG4 0x23
#define OUT_X_L 0x28
#define OUT_X_H 0x29
#define OUT_Y_L 0x2A
#define OUT_Y_H 0x2B
#define OUT_Z_L 0x2C
#define OUT_Z_H 0x2D

// SPI Configuration
#define SPI1_NODE DT_NODELABEL(spi1)
#define MY_GPIO1 DT_NODELABEL(gpio1)
#define GPIO_CS_PIN 7

// Function declarations
int lis3dh_init(const struct device *spi_dev, const struct device *gpio_dev);
int lis3dh_read_reg(uint8_t reg);
void lis3dh_write_reg(uint8_t reg, uint8_t value);
void lis3dh_read_chip_id(void);
int lis3dh_convert_twos_complement(int16_t value);
void lis3dh_get_acceleration(int *x, int *y, int *z);
void lis3dh_initialize_sensor(void);

#endif // LIS3DH_H
