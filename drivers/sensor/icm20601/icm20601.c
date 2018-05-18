#include <spi.h>
#include <init.h>
#include <sensor.h>
#include "icm20601.h"

#if defined(CONFIG_ICM20601_SPI_GPIO_CS)
static struct spi_cs_control icm20601_cs_ctrl;
#endif

#define SPI_CS NULL

static struct spi_config icm20601_spi_conf = {
	.frequency = CONFIG_ICM20601_SPI_BUS_FREQ,
	.operation = (SPI_OP_MODE_MASTER | SPI_MODE_CPOL |
		      SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE),
	.slave     = CONFIG_ICM20601_SPI_SELECT_SLAVE,
	.cs        = SPI_CS,
};



static int icm20601_raw_read(struct icm20601_data *data, u8_t reg_addr, u8_t *value, u8_t len) {
	struct spi_config *spi_cfg = &icm20601_spi_conf;
	u8_t buffer_tx[2] = { reg_addr | ICM20601_SPI_READ, 0 };
	const struct spi_buf tx_buf = {
			.buf = buffer_tx,
			.len = 2,
	};
	const struct spi_buf_set tx = {
		.buffers = &tx_buf,
		.count = 1
	};
	const struct spi_buf rx_buf[2] = {
		{
			.buf = NULL,
			.len = 1,
		},
		{
			.buf = value,
			.len = len,
		}
	};
	const struct spi_buf_set rx = {
		.buffers = rx_buf,
		.count = 2
	};


	if (len > 64) {
		return -EIO;
	}

	if (spi_transceive(data->spi, spi_cfg, &tx, &rx)) {
		return -EIO;
	}

	return 0;
}

static int icm20601_raw_write(struct icm20601_data *data, u8_t reg_addr,
		u8_t *value, u8_t len)
{
	struct spi_config *spi_cfg = &icm20601_spi_conf;
	u8_t buffer_tx[1] = { reg_addr & ~ICM20601_SPI_READ };
	const struct spi_buf tx_buf[2] = {
		{
			.buf = buffer_tx,
			.len = 1,
		},
		{
			.buf = value,
			.len = len,
		}
	};
	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = 2
	};

	if (len > 64) {
		return -EIO;
	}

	if (spi_write(data->spi, spi_cfg, &tx)) {
		return -EIO;
	}

	return 0;
}

static inline int icm20601_write_reg8(struct device *dev, u8_t addr, u8_t val) {
	struct icm20601_data *data = dev->driver_data;
	return icm20601_raw_write(data, addr, &val, 1);
}

static inline int icm20601_read_reg8(struct device *dev, u8_t addr, u8_t *val) {
	struct icm20601_data *data = dev->driver_data;
	return icm20601_raw_read(data, addr, val, 1);
}

static inline int icm20601_update_reg8(struct device *dev, u8_t addr, u8_t mask,
		u8_t val) {
	struct icm20601_data *data = dev->driver_data;

	u8_t tmp_val;
	
	icm20601_raw_read(data, addr, &tmp_val, 1);
	tmp_val = (tmp_val & ~mask) | (val & mask);

	return icm20601_raw_write(data, addr, &tmp_val, 1);
}

static inline int icm20601_reset(struct device *dev) {
	return icm20601_write_reg8(dev, ICM20601_REG_PWR_MGMT_1, 
			(1 << ICM20601_SHIFT_PWR_MGMT_1_RESET) & ICM20601_MASK_PWR_MGMT_1_RESET);
}

static inline int icm20601_power_on(struct device *dev) {
	return icm20601_write_reg8(dev, ICM20601_REG_PWR_MGMT_1, 
			(1 << ICM20601_SHIFT_PWR_MGMT_1_CLK_SEL) & ICM20601_MASK_PWR_MGMT_1_CLK_SEL);
}

static inline int icm20601_get_who_am_i(struct device *dev, u8_t *whoami) {
	return icm20601_read_reg8(dev, ICM20601_REG_WHO_AM_I, whoami);
}

static inline int icm20601_set_sample_rate_div(struct device *dev,
		u8_t rate_div) {
	return icm20601_write_reg8(dev, ICM20601_REG_PWR_MGMT_1, rate_div);
}

static inline int icm20601_set_accel_standby_avg(struct device *dev, u8_t avg) {
	u8_t dec2;
	switch(avg) {
		case 4:
			dec2 = 0;
			break;
		case 8:
			dec2 = 1;
			break;
		case 16:
			dec2 = 2;
			break;
		case 32:
			dec2 = 3;
			break;
		default:
			SYS_LOG_DBG("invalid number of samples to average, must be 4, 8, 16, or 32");
			return -EINVAL;
	}
	return icm20601_update_reg8(dev, ICM20601_REG_ACCEL_CONFIG_2,
			ICM20601_MASK_ACCEL_CONFIG_2_DEC2_CFG,
			dec2 << ICM20601_SHIFT_ACCEL_CONFIG_2_DEC2_CFG);
}

static inline int icm20601_set_accel_lpf(struct device *dev,
		bool lpf_enable, u8_t lpf_mode) {
	u8_t update = 0;
	if(lpf_mode > 7) {
		SYS_LOG_DBG("invalid accelerometer low pass filter mode, must be 0 to 7");
		return -EINVAL;
	}
	if(lpf_enable) {
		update = 1 << ICM20601_SHIFT_ACCEL_CONFIG_2_ACCEL_FCHOICE_B;
	}
	update |= lpf_mode << ICM20601_SHIFT_ACCEL_CONFIG_2_A_DLPF_CFG;
	u8_t mask = ICM20601_MASK_ACCEL_CONFIG_2_A_DLPF_CFG
		| ICM20601_MASK_ACCEL_CONFIG_2_ACCEL_FCHOICE_B;
	return icm20601_update_reg8(dev, ICM20601_REG_ACCEL_CONFIG_2, mask, update);
}

static int icm20601_init_chip(struct device *dev) {
	int err = 0;
	u8_t chip_id;
	
	if(icm20601_reset(dev) < 0) {
		SYS_LOG_DBG("failed to reset device");
		return -EIO;
	}

	if(icm20601_power_on(dev) < 0) {
		SYS_LOG_DBG("failed to power on device");
		return -EIO;
	}

	if(icm20601_get_who_am_i(dev, &chip_id) < 0) {
		SYS_LOG_DBG("failed reading chip id");
		return -EIO;
	}

	if(chip_id != ICM20601_VAL_WHO_AM_I) {
		SYS_LOG_DBG("invalid chip id 0x%x", chip_id);
		return -EIO;
	}

	if(icm20601_set_sample_rate_div(dev, ICM20601_DEFAULT_SAMPLE_RATE_DIV) < 0) {
		SYS_LOG_DB("failed to set sample rate divider");
		return -EIO;
	}

	if(icm20601_set_accel_standby_avg(dev, ICM20601_DEFAULT_ACCEL_STANDBY_AVG) < 0) {
		SYS_LOG_DB("failed to set accelerometer standby sample averaging");
		return -EIO;
	}

	if(icm20601_set_accel_lpf(dev, ICM20601_DEFAULT_ACCEL_LPF_ENABLE, ICM20601_DEFAULT_ACCEL_LPF_CFG) < 0) {
		SYS_LOG_DB("failed to set accelerometer low-pass filter settings");
		return -EIO;
	}
	
	if(icm20601_set_accel_fs_raw(dev, ICM20601_DEFAULT_ACCEL_FULLSCALE) < 0) {
		SYS_LOG_DBG("failed to set accelerometer full-scale");
		return -EIO;
	}

	if(icm20601_set_gyro_standby_avg(dev, ICM20601_DEFAULT_GYRO_STANDBY_AVG) < 0) {
		SYS_LOG_DB("failed to set accelerometer standby sample averaging");
		return -EIO;
	}

	if(icm20601_set_gyro_lpf(dev, ICM20601_DEFAULT_GYRO_LPF_ENABLE, ICM20601_DEFAULT_GYRO_LPF_CFG) < 0) {
		SYS_LOG_DB("failed to set accelerometer low-pass filter settings");
		return -EIO;
	}

	if(icm20601_set_gyro_fs_raw(dev, ICM20601_DEFAULT_GYRO_FULLSCALE) < 0) {
		SYS_LOG_DBG("failed to set gyroscope full-scale");
		return -EIO;
	}
	
	return 0;
}

/*
int icm20601_get_x_accel(struct icm20601 *device, int16_t *x) {
    return icm20601_read_i16(device, ACCEL_XOUT_H, ACCEL_XOUT_L, x);
}

int icm20601_get_y_accel(struct icm20601 *device, int16_t *y) {
    return icm20601_read_i16(device, ACCEL_YOUT_H, ACCEL_YOUT_L, y);
}

int icm20601_get_z_accel(struct icm20601 *device, int16_t *z) {
    return icm20601_read_i16(device, ACCEL_ZOUT_H, ACCEL_ZOUT_L, z);
}

int icm20601_get_x_gyro(struct icm20601 *device, int16_t *x) {
    return icm20601_read_i16(device, GYRO_XOUT_H, GYRO_XOUT_L, x);
}

int icm20601_get_y_gyro(struct icm20601 *device, int16_t *y) {
    return icm20601_read_i16(device, GYRO_YOUT_H, GYRO_YOUT_L, y);
}

int icm20601_get_z_gyro(struct icm20601 *device, int16_t *z) {
    return icm20601_read_i16(device, GYRO_ZOUT_H, GYRO_ZOUT_L, z);
}

int icm20601_get_temp(struct icm20601 *device, int16_t *temp) {
    return icm20601_read_i16(device, TEMP_OUT_H, TEMP_OUT_L, temp);
}

int icm20601_get_x_gyro_offset(struct icm20601 *device, int16_t *x) {
    return icm20601_read_i16(device, XG_OFFS_USRH, XG_OFFS_USRL, x);
}

int icm20601_get_y_gyro_offset(struct icm20601 *device, int16_t *y) {
    return icm20601_read_i16(device, YG_OFFS_USRH, YG_OFFS_USRL, y);
}

int icm20601_get_z_gyro_offset(struct icm20601 *device, int16_t *z) {
    return icm20601_read_i16(device, ZG_OFFS_USRH, ZG_OFFS_USRL, z);
}

int icm20601_set_x_gyro_offset(struct icm20601 *device, int16_t x) {
    return icm20601_write_i16(device, XG_OFFS_USRH, XG_OFFS_USRL, x);
}

int icm20601_set_y_gyro_offset(struct icm20601 *device, int16_t y) {
    return icm20601_write_i16(device, YG_OFFS_USRH, YG_OFFS_USRL, y);
}

int icm20601_set_z_gyro_offset(struct icm20601 *device, int16_t z) {
    return icm20601_write_i16(device, ZG_OFFS_USRH, ZG_OFFS_USRL, z);
}

int icm20601_get_x_accel_offset(struct icm20601 *device, int16_t *x) {
    return icm20601_read_i16(device, XA_OFFSET_H, XA_OFFSET_L, x);
}

int icm20601_get_y_accel_offset(struct icm20601 *device, int16_t *y) {
    return icm20601_read_i16(device, YA_OFFSET_H, YA_OFFSET_L, y);
}

int icm20601_get_z_accel_offset(struct icm20601 *device, int16_t *z) {
    return icm20601_read_i16(device, ZA_OFFSET_H, ZA_OFFSET_L, z);
}

int icm20601_set_x_accel_offset(struct icm20601 *device, int16_t x) {
    return icm20601_write_i16(device, XA_OFFSET_H, XA_OFFSET_L, x);
}

int icm20601_set_y_accel_offset(struct icm20601 *device, int16_t y) {
    return icm20601_write_i16(device, YA_OFFSET_H, YA_OFFSET_L, y);
}

int icm20601_set_z_accel_offset(struct icm20601 *device, int16_t z) {
    return icm20601_write_i16(device, ZA_OFFSET_H, ZA_OFFSET_L, z);
}
*/

int icm20601_init(struct device *dev) {
	const struct icm20601_config * const config = dev->config->config_info;
	struct icm20601_data *data = dev->driver_data;

	data->spi = device_get_binding(config->dev_name);
	if (!data->spi) {
		SYS_LOG_DBG("master not found: %s",
			    config->dev_name);
		return -EINVAL;
	}

#if defined(CONFIG_ICM20601_SPI_GPIO_CS)
	/* handle SPI CS thru GPIO if it is the case */
	if (IS_ENABLED(CONFIG_ICM20601_SPI_GPIO_CS)) {
		icm20601_cs_ctrl.gpio_dev = device_get_binding(
			CONFIG_ICM20601_SPI_GPIO_CS_DRV_NAME);
		if (!icm20601_cs_ctrl.gpio_dev) {
			SYS_LOG_ERR("Unable to get GPIO SPI CS device");
			return -ENODEV;
		}

		icm20601_cs_ctrl.gpio_pin = CONFIG_ICM20601_SPI_GPIO_CS_PIN;
		icm20601_cs_ctrl.delay = 0;

		icm20601_spi_conf.cs = &icm20601_cs_ctrl;

		SYS_LOG_DBG("SPI GPIO CS configured on %s:%u",
			    CONFIG_ICM20601_SPI_GPIO_CS_DRV_NAME,
			    CONFIG_ICM20601_SPI_GPIO_CS_PIN);
	}
#endif

	if (icm20601_init_chip(dev) < 0) {
		SYS_LOG_DBG("failed to initialize chip");
		return -EIO;
	}

	return 0;
}


static struct icm20601_data icm20601_driver;

DEVICE_AND_API_INIT(icm20601, CONFIG_ICM20601_NAME, icm20601_init,
        &icm20601_driver, NULL, POST_KERNEL,
        CONFIG_SENSOR_INIT_PRIORITY, &icm20601_driver_api);
