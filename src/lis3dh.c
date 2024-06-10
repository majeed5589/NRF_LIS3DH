#include "lis3dh.h"

// Static buffers for SPI transactions
static uint8_t spi_tx_buf[2];
static uint8_t spi_rx_buf[2];

// SPI and GPIO device pointers
static const struct device *spi1_dev;
static const struct device *gpio1_dev;

static struct spi_config spi_cfg = {
    .frequency = 125000U,
    .operation = SPI_WORD_SET(8), // Ensure SPI mode 0
    .slave = 0,
};

static uint8_t tx_buffer[2];
static uint8_t rx_buffer[2];

int lis3dh_init(const struct device *spi_dev, const struct device *gpio_dev) {
    spi1_dev = spi_dev;
    gpio1_dev = gpio_dev;

    if (!device_is_ready(spi1_dev) || !device_is_ready(gpio1_dev)) {
        printk("SPI or GPIO device not ready\n");
        return -1;
    }

    gpio_pin_configure(gpio1_dev, GPIO_CS_PIN, GPIO_OUTPUT);
    gpio_pin_set(gpio1_dev, GPIO_CS_PIN, 1);

    lis3dh_read_chip_id();
    lis3dh_initialize_sensor();

    return 0;
}

int lis3dh_convert_twos_complement(int16_t value) {
    if (value & 0x8000) {
        value = ~value + 1;
        value = -value;
    }
    return value;
}

int lis3dh_read_reg(uint8_t reg) {
    spi_tx_buf[0] = reg | 0x80; // Set MSB for read operation
    spi_tx_buf[1] = 0x00; // Dummy byte

    struct spi_buf tx_buf = {.buf = spi_tx_buf, .len = 2};
    struct spi_buf rx_buf = {.buf = spi_rx_buf, .len = 2};
    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};
    struct spi_buf_set rx = {.buffers = &rx_buf, .count = 1};

    gpio_pin_set(gpio1_dev, GPIO_CS_PIN, 0);
    int error = spi_transceive(spi1_dev, &spi_cfg, &tx, &rx);
    gpio_pin_set(gpio1_dev, GPIO_CS_PIN, 1);

    if (error != 0) {
        printk("SPI transceive error: %i\n", error);
        return error;
    }

    return spi_rx_buf[1];
}

void lis3dh_write_reg(uint8_t reg, uint8_t value) {
    spi_tx_buf[0] = reg & 0x7F; // Clear MSB for write operation
    spi_tx_buf[1] = value;

    struct spi_buf tx_buf = {.buf = spi_tx_buf, .len = 2};
    struct spi_buf_set tx = {.buffers = &tx_buf, .count = 1};

    gpio_pin_set(gpio1_dev, GPIO_CS_PIN, 0);
    int error = spi_write(spi1_dev, &spi_cfg, &tx);
    gpio_pin_set(gpio1_dev, GPIO_CS_PIN, 1);

    if (error < 0) {
        printk("Write Register Failed: %d\n", error);
    }

    printk("TX buffer: 0x%x 0x%x\n", spi_tx_buf[0], spi_tx_buf[1]);
}

void lis3dh_read_chip_id(void) {
    tx_buffer[0] = WHO_AM_I | 0x80; // Set MSB for read operation
    tx_buffer[1] = 0x00; // Dummy byte to ensure transaction completes

    struct spi_buf tx_spi_bufs[] = {
        {.buf = tx_buffer, .len = sizeof(tx_buffer)},
    };

    struct spi_buf_set spi_tx_buffer_set = {
        .buffers = tx_spi_bufs,
        .count = 1,
    };

    struct spi_buf rx_spi_bufs[] = {
        {.buf = rx_buffer, .len = sizeof(rx_buffer)},
    };

    struct spi_buf_set spi_rx_buffer_set = {
        .buffers = rx_spi_bufs,
        .count = 1,
    };

    gpio_pin_set(gpio1_dev, GPIO_CS_PIN, 0);
    int err = spi_transceive(spi1_dev, &spi_cfg, &spi_tx_buffer_set, &spi_rx_buffer_set);
    gpio_pin_set(gpio1_dev, GPIO_CS_PIN, 1);

    if (err < 0) {
        printk("Read Registers Failed: %d\n", err);
    } else {
        printk("WHO AM I REGISTER VALUE IS: 0x%x\n", rx_buffer[1]);
    }

    printk("TX buffer: 0x%x 0x%x\n", tx_buffer[0], tx_buffer[1]);
    printk("RX buffer: 0x%x 0x%x\n", rx_buffer[0], rx_buffer[1]);
}

void lis3dh_initialize_sensor(void) {
    // Set data rate to 10 Hz, enable X, Y, Z axes
    lis3dh_write_reg(CTRL_REG1, 0x27);

    // Set full scale to 2g, high resolution mode
    lis3dh_write_reg(CTRL_REG4, 0x08);
}

void lis3dh_get_acceleration(int *x, int *y, int *z) {
    *x = ((lis3dh_read_reg(OUT_X_H) << 8) | lis3dh_read_reg(OUT_X_L));
    *y = ((lis3dh_read_reg(OUT_Y_H) << 8) | lis3dh_read_reg(OUT_Y_L));
    *z = ((lis3dh_read_reg(OUT_Z_H) << 8) | lis3dh_read_reg(OUT_Z_L));

    /* Transform X value from two's complement to 16-bit int */
    *x = lis3dh_convert_twos_complement(*x);
    /* Convert X absolute value to mg value */
    *x = *x * 0.06;

    /* Transform Y value from two's complement to 16-bit int */
    *y = lis3dh_convert_twos_complement(*y);
    /* Convert Y absolute value to mg value */
    *y = *y * 0.06;

    /* Transform Z value from two's complement to 16-bit int */
    *z = lis3dh_convert_twos_complement(*z);
    /* Convert Z absolute value to mg value */
    *z = *z * 0.06;
}
