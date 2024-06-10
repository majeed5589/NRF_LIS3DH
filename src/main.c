#include <zephyr/kernel.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include "lis3dh.h"

#define SLEEP_TIME_MS 1000

void main(void) {
    const struct device *spi1_dev = DEVICE_DT_GET(SPI1_NODE);
    const struct device *gpio1_dev = DEVICE_DT_GET(MY_GPIO1);

    if (lis3dh_init(spi1_dev, gpio1_dev) < 0) {
        printk("Failed to initialize LIS3DH sensor\n");
        return;
    }

    while (1) {
        int acc_x, acc_y, acc_z;
        lis3dh_get_acceleration(&acc_x, &acc_y, &acc_z);

        printk("X= %6dmg Y= %6dmg Z= %6dmg \r\n", acc_x, acc_y, acc_z);
        k_msleep(SLEEP_TIME_MS);
    }
}
