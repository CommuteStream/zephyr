#include <spi.h>
#include <init.h>
#include <sensor.h>

#include "icm20601.h"

enum icm20601_gyro_scale {
    five_hundred_dps,
    one_thousand_dps,
    two_thousand_dps,
    four_thousand_dps
};


enum icm20601_accel_scale {
    four_g,
    eight_g,
    sixteen_g,
    thirty_two_g,
};



void icm20601_init(struct icm20601 *device, SPI_HandleTypeDef *spi, GPIO_TypeDef *gpio, uint16_t pin, uint32_t timeout) {
    device->spi = spi;
    device->gpio = gpio;
    device->pin = pin;
    device->timeout = timeout;
}

static int16_t icm_int16_conv(uint8_t h, uint8_t l) {
    return (h << 8 & 0xFF) | (l & 0xFF);
}

icm20601_result icm20601_get_data(struct icm20601 *device, struct icm20601_data *data) {
    int result = 0;
    uint8_t trans[15] = {
        ACCEL_XOUT_H | ICM_READ_BIT,
        ACCEL_XOUT_L | ICM_READ_BIT,
        ACCEL_YOUT_H | ICM_READ_BIT,
        ACCEL_YOUT_L | ICM_READ_BIT,
        ACCEL_ZOUT_H | ICM_READ_BIT,
        ACCEL_ZOUT_L | ICM_READ_BIT,
        TEMP_OUT_H | ICM_READ_BIT,
        TEMP_OUT_L | ICM_READ_BIT,
        GYRO_XOUT_H | ICM_READ_BIT,
        GYRO_XOUT_L | ICM_READ_BIT,
        GYRO_YOUT_H | ICM_READ_BIT,
        GYRO_YOUT_L | ICM_READ_BIT,
        GYRO_ZOUT_H | ICM_READ_BIT,
        GYRO_ZOUT_L | ICM_READ_BIT,
        0,
    };
    uint8_t recv[15];
    trans[0] = ACCEL_XOUT_H | ICM_READ_BIT;
    result = HAL_SPI_TransmitReceive(device->spi, trans, recv, 15, device->timeout);
    data->xa = icm_int16_conv(recv[1], recv[2]);
    data->xa = icm_int16_conv(recv[3], recv[4]);
    data->xa = icm_int16_conv(recv[5], recv[6]);
    data->temp = icm_int16_conv(recv[7], recv[8]);
    data->xg = icm_int16_conv(recv[9], recv[10]);
    data->yg = icm_int16_conv(recv[11], recv[12]);
    data->zg = icm_int16_conv(recv[13], recv[14]);
    return result;
}

icm20601_result icm20601_get_x_accel(struct icm20601 *device, int16_t *x) {
    return icm20601_read_i16(device, ACCEL_XOUT_H, ACCEL_XOUT_L, x);
}

icm20601_result icm20601_get_y_accel(struct icm20601 *device, int16_t *y) {
    return icm20601_read_i16(device, ACCEL_YOUT_H, ACCEL_YOUT_L, y);
}

icm20601_result icm20601_get_z_accel(struct icm20601 *device, int16_t *z) {
    return icm20601_read_i16(device, ACCEL_ZOUT_H, ACCEL_ZOUT_L, z);
}

icm20601_result icm20601_get_x_gyro(struct icm20601 *device, int16_t *x) {
    return icm20601_read_i16(device, GYRO_XOUT_H, GYRO_XOUT_L, x);
}

icm20601_result icm20601_get_y_gyro(struct icm20601 *device, int16_t *y) {
    return icm20601_read_i16(device, GYRO_YOUT_H, GYRO_YOUT_L, y);
}

icm20601_result icm20601_get_z_gyro(struct icm20601 *device, int16_t *z) {
    return icm20601_read_i16(device, GYRO_ZOUT_H, GYRO_ZOUT_L, z);
}

icm20601_result icm20601_get_temp(struct icm20601 *device, int16_t *temp) {
    return icm20601_read_i16(device, TEMP_OUT_H, TEMP_OUT_L, temp);
}

icm20601_result icm20601_get_x_gyro_offset(struct icm20601 *device, int16_t *x) {
    return icm20601_read_i16(device, XG_OFFS_USRH, XG_OFFS_USRL, x);
}

icm20601_result icm20601_get_y_gyro_offset(struct icm20601 *device, int16_t *y) {
    return icm20601_read_i16(device, YG_OFFS_USRH, YG_OFFS_USRL, y);
}

icm20601_result icm20601_get_z_gyro_offset(struct icm20601 *device, int16_t *z) {
    return icm20601_read_i16(device, ZG_OFFS_USRH, ZG_OFFS_USRL, z);
}

icm20601_result icm20601_set_x_gyro_offset(struct icm20601 *device, int16_t x) {
    return icm20601_write_i16(device, XG_OFFS_USRH, XG_OFFS_USRL, x);
}

icm20601_result icm20601_set_y_gyro_offset(struct icm20601 *device, int16_t y) {
    return icm20601_write_i16(device, YG_OFFS_USRH, YG_OFFS_USRL, y);
}

icm20601_result icm20601_set_z_gyro_offset(struct icm20601 *device, int16_t z) {
    return icm20601_write_i16(device, ZG_OFFS_USRH, ZG_OFFS_USRL, z);
}

icm20601_result icm20601_get_x_accel_offset(struct icm20601 *device, int16_t *x) {
    return icm20601_read_i16(device, XA_OFFSET_H, XA_OFFSET_L, x);
}

icm20601_result icm20601_get_y_accel_offset(struct icm20601 *device, int16_t *y) {
    return icm20601_read_i16(device, YA_OFFSET_H, YA_OFFSET_L, y);
}

icm20601_result icm20601_get_z_accel_offset(struct icm20601 *device, int16_t *z) {
    return icm20601_read_i16(device, ZA_OFFSET_H, ZA_OFFSET_L, z);
}

icm20601_result icm20601_set_x_accel_offset(struct icm20601 *device, int16_t x) {
    return icm20601_write_i16(device, XA_OFFSET_H, XA_OFFSET_L, x);
}

icm20601_result icm20601_set_y_accel_offset(struct icm20601 *device, int16_t y) {
    return icm20601_write_i16(device, YA_OFFSET_H, YA_OFFSET_L, y);
}

icm20601_result icm20601_set_z_accel_offset(struct icm20601 *device, int16_t z) {
    return icm20601_write_i16(device, ZA_OFFSET_H, ZA_OFFSET_L, z);
}

/**
 * Start a SPI transaction
 * TODO make this indirect using function pointer or compile time config
 */
void icm20601_start(struct icm20601 *device) {
    HAL_GPIO_WritePin(device->gpio, device->pin, 0);
}

/**
 * Stop a SPI transaction
 * TODO make this indirect using function pointer or compile time config
 */
void icm20601_stop(struct icm20601 *device) {
    return HAL_GPIO_WritePin(device->gpio, device->pin, 1);
}


/**
 * Write a byte to a register
 */
static icm20601_result icm20601_write_u8(struct icm20601 *device, uint8_t addr, uint8_t val) {
uint8_t buf[2] = {addr, val};
    int result = ICM_OK;
    result |= HAL_SPI_Transmit(device->spi, buf, 2, device->timeout);
    return result;
}

static icm20601_result icm20601_read_u8(struct icm20601 *device, uint8_t addr, uint8_t *val) {
    uint8_t write_buf[2] = {addr | ICM_READ_BIT, 0};
    uint8_t read_buf[2] = { 0, 0 };
    int result = ICM_OK;
    result |= HAL_SPI_TransmitReceive(device->spi, write_buf, read_buf, 2, device->timeout);
    *val = read_buf[1];
    return result;
}

static icm20601_result icm20601_read_i16(struct icm20601 *device, uint8_t addrh, uint8_t addrl, int16_t *val) {
    int result = ICM_OK;
    uint8_t trans[3] = {addrh | ICM_READ_BIT, addrl | ICM_READ_BIT, 0};
    uint8_t recv[3] = {0, 0, 0};
    result |= HAL_SPI_TransmitReceive(device->spi, trans, recv, 3, device->timeout);
    *val = recv[1] << 8 | recv[2];
    return result;
}

static icm20601_result icm20601_write_i16(struct icm20601 *device, uint8_t addrh, uint8_t addrl, int16_t val) {
    int result = ICM_OK;
    uint8_t high = val >> 8 & 0xFF;
    uint8_t low = val & 0xFF;
    result |= icm20601_write_u8(device, addrh, high);
    result |= icm20601_write_u8(device, addrl, low);
    return result;
}

static struct icm20601_data icm20601_driver;

DEVICE_AND_API_INIT(icm20601, CONFIG_ICM20601_NAME, icm20601_init,
        &icm20601_driver, NULL, POST_KERNEL,
        CONFIG_SENSOR_INIT_PRIORITY, &icm20601_driver_api);
