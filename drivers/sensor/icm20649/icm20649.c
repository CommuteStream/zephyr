#include <spi.h>
#include <init.h>
#include <sensor.h>
#include "icm20649.h"

#define SENSOR_PI_DOUBLE			(SENSOR_PI / 1000000.0)
#define SENSOR_DEG2RAD_DOUBLE			(SENSOR_PI_DOUBLE / 180)
#define SENSOR_G_DOUBLE				(SENSOR_G / 1000000.0)

#if defined(CONFIG_ICM20649_SPI_GPIO_CS)
static struct spi_cs_control icm20649_cs_ctrl;
#endif

#define SPI_CS NULL

static struct spi_config icm20649_spi_conf = {
	.frequency = CONFIG_ICM20649_SPI_BUS_FREQ,
	.operation = (SPI_OP_MODE_MASTER | SPI_MODE_CPOL |
		      SPI_MODE_CPHA | SPI_WORD_SET(8) | SPI_LINES_SINGLE),
	.slave     = CONFIG_ICM20649_SPI_SELECT_SLAVE,
	.cs        = SPI_CS,
};



static int icm20649_raw_read(struct icm20649_data *data, u8_t reg_addr, u8_t *value, u8_t len) {
	struct spi_config *spi_cfg = &icm20649_spi_conf;
	u8_t buffer_tx[2] = { reg_addr | ICM20649_SPI_READ, 0 };
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

static int icm20649_raw_write(struct icm20649_data *data, u8_t reg_addr,
		u8_t *value, u8_t len)
{
	struct spi_config *spi_cfg = &icm20649_spi_conf;
	u8_t buffer_tx[1] = { reg_addr & ~ICM20649_SPI_READ };
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

static inline int icm20649_write_reg8(struct device *dev, u8_t addr, u8_t val) {
	struct icm20649_data *data = dev->driver_data;
	return icm20649_raw_write(data, addr, &val, 1);
}

static inline int icm20649_read_reg8(struct device *dev, u8_t addr, u8_t *val) {
	struct icm20649_data *data = dev->driver_data;
	return icm20649_raw_read(data, addr, val, 1);
}

static inline int icm20649_read_s16(struct device *dev, u8_t addr_l, u8_t addr_h, s16_t *val) {
	u8_t l, h = 0;
	int err = icm20649_read_reg8(dev, addr_l, &l);
	err |= icm20649_read_reg8(dev, addr_h, &h);
	*val = (s16_t)((u16_t)(l) |
				((u16_t)(h) << 8));
	return err;
}


static inline int icm20649_update_reg8(struct device *dev, u8_t addr, u8_t mask,
		u8_t val) {
	struct icm20649_data *data = dev->driver_data;

	u8_t tmp_val;
	
	icm20649_raw_read(data, addr, &tmp_val, 1);
	tmp_val = (tmp_val & ~mask) | (val & mask);

	return icm20649_raw_write(data, addr, &tmp_val, 1);
}

static inline int icm20649_reset(struct device *dev) {
	return icm20649_write_reg8(dev, ICM20649_REG_PWR_MGMT_1, 
			(1 << ICM20649_SHIFT_PWR_MGMT_1_DEVICE_RESET) & ICM20649_MASK_PWR_MGMT_1_DEVICE_RESET);
}

static inline int icm20649_auto_clock(struct device *dev) {
	return icm20649_write_reg8(dev, ICM20649_REG_PWR_MGMT_1, 
			(1 << ICM20649_SHIFT_PWR_MGMT_1_CLKSEL) & ICM20649_MASK_PWR_MGMT_1_CLKSEL);
}

static inline int icm20649_wakeup(struct device *dev) {
	return icm20649_write_reg8(dev, ICM20649_REG_PWR_MGMT_1, 
			(0 << ICM20649_SHIFT_PWR_MGMT_1_SLEEP) & ICM20649_MASK_PWR_MGMT_1_SLEEP);
}

static inline int icm20649_sleep(struct device *dev) {
	return icm20649_write_reg8(dev, ICM20649_REG_PWR_MGMT_1, 
			(1 << ICM20649_SHIFT_PWR_MGMT_1_SLEEP) & ICM20649_MASK_PWR_MGMT_1_SLEEP);
}

static inline int icm20649_get_who_am_i(struct device *dev, u8_t *whoami) {
	return icm20649_read_reg8(dev, ICM20649_REG_WHO_AM_I, whoami);
}

/*
static inline int icm20649_set_accel_sample_rate_div(struct device *dev,
		u8_t rate_div) {
	return icm20649_write_reg8(dev, ICM20649_REG_PWR_MGMT_1, rate_div);
}
*/

static inline int icm20649_set_accel_decimator(struct device *dev, u8_t avg) {
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
	return icm20649_update_reg8(dev, ICM20649_REG_ACCEL_CONFIG_2,
			ICM20649_MASK_ACCEL_CONFIG_2_DEC3_CFG,
			dec2 << ICM20649_SHIFT_ACCEL_CONFIG_2_DEC3_CFG);
}

static inline int icm20649_set_accel_filter_choice(struct device *dev,
		u8_t fchoice)
{
	return icm20649_update_reg8(dev, ICM20649_REG_ACCEL_CONFIG,
			ICM20649_MASK_ACCEL_CONFIG_ACCEL_FCHOICE,
		fchoice << ICM20649_SHIFT_ACCEL_CONFIG_ACCEL_FCHOICE);
}

static inline int icm20649_set_accel_lpf(struct device *dev, u8_t lpf_mode) {
	if(lpf_mode > 7) {
		SYS_LOG_DBG("invalid accelerometer low pass filter mode, must be 0 to 7");
		return -EINVAL;
	}
	return icm20649_update_reg8(dev, ICM20649_REG_ACCEL_CONFIG,
			ICM20649_MASK_ACCEL_CONFIG_ACCEL_DLPFCFG,
			lpf_mode << ICM20649_SHIFT_ACCEL_CONFIG_ACCEL_DLPFCFG);
}

static int icm20649_set_accel_fs_sel(struct device *dev, u8_t fs) {
	u8_t update = 0;
	if(fs > 3) {
		SYS_LOG_DBG("invalid accelerometer full-scale must be 0 to 3");
		return -EINVAL;
	}
	update |= fs << ICM20649_SHIFT_ACCEL_CONFIG_ACCEL_FS_SEL;
	return icm20649_update_reg8(dev, ICM20649_REG_ACCEL_CONFIG,
			ICM20649_MASK_ACCEL_CONFIG_ACCEL_FS_SEL, update);
}

static inline int icm20649_set_gyro_filter_choice(struct device *dev, u8_t fchoice) {
	if(fchoice > 3) {
		SYS_LOG_DBG("invalid gyro filter choice, must be 0 to 3");
		return -EINVAL;
	}
	return icm20649_update_reg8(dev, ICM20649_REG_GYRO_CONFIG_1,
			ICM20649_MASK_GYRO_CONFIG_1_GYRO_FCHOICE,
			fchoice << ICM20649_SHIFT_GYRO_CONFIG_1_GYRO_FCHOICE);
}

static inline int icm20649_set_gyro_lpf(struct device *dev, u8_t lpf_mode) {
	if(lpf_mode > 7) {
		SYS_LOG_DBG("invalid gyro low pass filter mode, must be 0 to 7");
		return -EINVAL;
	}
	return icm20649_update_reg8(dev, ICM20649_REG_GYRO_CONFIG_1,
			ICM20649_MASK_GYRO_CONFIG_1_GYRO_DLPFCFG,
			lpf_mode << ICM20649_SHIFT_GYRO_CONFIG_1_GYRO_DLPFCFG);
}

static int icm20649_set_gyro_fs_sel(struct device *dev, u8_t fs) {
	u8_t update = 0;
	if(fs > 3) {
		SYS_LOG_DBG("invalid gyro full-scale must be 0 to 3");
		return -EINVAL;
	}
	update |= fs << ICM20649_SHIFT_GYRO_CONFIG_1_GYRO_FS_SEL;
	return icm20649_update_reg8(dev, ICM20649_REG_GYRO_CONFIG_1,
			ICM20649_MASK_GYRO_CONFIG_1_GYRO_FS_SEL, update);
}

static int icm20649_set_odr_align(struct device *dev, u8_t align) {
	return icm20649_update_reg8(dev, ICM20649_REG_ODR_ALIGN_EN,
			ICM20649_MASK_ODR_ALIGN_EN,
			align << ICM20649_SHIFT_ODR_ALIGN_EN);
}

static int icm20649_attr_set(struct device *dev, enum sensor_channel chan,
			   enum sensor_attribute attr,
			   const struct sensor_value *val)
{
	SYS_LOG_WRN("attr_set() not supported on this channel.");
	return -ENOTSUP;
}

static int icm20649_sample_fetch_accel(struct device *dev)
{
	struct icm20649_data *data = dev->driver_data;

	if (icm20649_read_s16(dev, ICM20649_REG_ACCEL_XOUT_L, ICM20649_REG_ACCEL_XOUT_H, &data->accel_sample_x) < 0) {
		SYS_LOG_DBG("failed to fetch accel x sample");
		return -EIO;
	}
	if (icm20649_read_s16(dev, ICM20649_REG_ACCEL_YOUT_L, ICM20649_REG_ACCEL_YOUT_H, &data->accel_sample_y) < 0) {
		SYS_LOG_DBG("failed to fetch accel y sample");
		return -EIO;
	}
	if (icm20649_read_s16(dev, ICM20649_REG_ACCEL_ZOUT_L, ICM20649_REG_ACCEL_ZOUT_H, &data->accel_sample_z) < 0) {
		SYS_LOG_DBG("failed to fetch accel z sample");
		return -EIO;
	}
	return 0;
}

static int icm20649_sample_fetch_gyro(struct device *dev)
{
	struct icm20649_data *data = dev->driver_data;

	if (icm20649_read_s16(dev, ICM20649_REG_GYRO_XOUT_L, ICM20649_REG_GYRO_XOUT_H, &data->gyro_sample_x) < 0) {
		SYS_LOG_DBG("failed to fetch gyro x sample");
		return -EIO;
	}
	if (icm20649_read_s16(dev, ICM20649_REG_GYRO_YOUT_L, ICM20649_REG_GYRO_YOUT_H, &data->gyro_sample_y) < 0) {
		SYS_LOG_DBG("failed to fetch gyro y sample");
		return -EIO;
	}
	if (icm20649_read_s16(dev, ICM20649_REG_GYRO_ZOUT_L, ICM20649_REG_GYRO_ZOUT_H, &data->gyro_sample_z) < 0) {
		SYS_LOG_DBG("failed to fetch gyro z sample");
		return -EIO;
	}
	return 0;
}


static int icm20649_sample_fetch_temp(struct device *dev)
{
	struct icm20649_data *data = dev->driver_data;

	if (icm20649_read_s16(dev, ICM20649_REG_TEMP_OUT_L, ICM20649_REG_TEMP_OUT_H, &data->temp_sample) < 0) {
		SYS_LOG_DBG("failed to fetch temperature");
		return -EIO;
	}
	return 0;
}


static int icm20649_sample_fetch(struct device *dev, enum sensor_channel chan)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_XYZ:
		icm20649_sample_fetch_accel(dev);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm20649_sample_fetch_gyro(dev);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		icm20649_sample_fetch_temp(dev);
		break;
	case SENSOR_CHAN_ALL:
		icm20649_sample_fetch_accel(dev);
		icm20649_sample_fetch_gyro(dev);
		icm20649_sample_fetch_temp(dev);
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}

static inline void icm20649_gyro_convert(struct sensor_value *val, s16_t raw_val)
{
	/* Degrees per second, without conversion */
	val->val1 = raw_val;
}

static inline int icm20649_gyro_channel_get(enum sensor_channel chan,
					   struct sensor_value *val,
					   struct icm20649_data *data)
{
	switch (chan) {
	case SENSOR_CHAN_GYRO_X:
		icm20649_gyro_convert(val, data->gyro_sample_x);
		break;
	case SENSOR_CHAN_GYRO_Y:
		icm20649_gyro_convert(val, data->gyro_sample_y);
		break;
	case SENSOR_CHAN_GYRO_Z:
		icm20649_gyro_convert(val, data->gyro_sample_z);
		break;
	case SENSOR_CHAN_GYRO_XYZ:
		icm20649_gyro_convert(&val[0], data->gyro_sample_x);
		icm20649_gyro_convert(&val[1], data->gyro_sample_y);
		icm20649_gyro_convert(&val[2], data->gyro_sample_z);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}


static void icm20649_gyro_channel_get_temp(struct sensor_value *val,
					  struct icm20649_data *data)
{
	/* val = temp_sample / 256 + 25 */
	val->val1 = data->temp_sample / 256 + 25;
	val->val2 = (data->temp_sample % 256) * (1000000 / 256);
}

static inline void icm20649_accel_convert(struct sensor_value *val, s16_t raw_val)
{
	/* exposed as gravity in G's*/
	val->val1 = raw_val;

}


static inline int icm20649_accel_channel_get(enum sensor_channel chan,
					    struct sensor_value *val,
					    struct icm20649_data *data)
{
	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
		icm20649_accel_convert(val, data->accel_sample_x);
		break;
	case SENSOR_CHAN_ACCEL_Y:
		icm20649_accel_convert(val, data->accel_sample_y);
		break;
	case SENSOR_CHAN_ACCEL_Z:
		icm20649_accel_convert(val, data->accel_sample_z);
		break;
	case SENSOR_CHAN_ACCEL_XYZ:
		icm20649_accel_convert(&val[0], data->accel_sample_x);
		icm20649_accel_convert(&val[1], data->accel_sample_y);
		icm20649_accel_convert(&val[2], data->accel_sample_z);
		break;
	default:
		return -ENOTSUP;
	}
	return 0;
}


static int icm20649_channel_get(struct device *dev,
			       enum sensor_channel chan,
			       struct sensor_value *val)
{
	struct icm20649_data *data = dev->driver_data;

	switch (chan) {
	case SENSOR_CHAN_ACCEL_X:
	case SENSOR_CHAN_ACCEL_Y:
	case SENSOR_CHAN_ACCEL_Z:
	case SENSOR_CHAN_ACCEL_XYZ:
		icm20649_accel_channel_get(chan, val, data);
		break;
	case SENSOR_CHAN_GYRO_X:
	case SENSOR_CHAN_GYRO_Y:
	case SENSOR_CHAN_GYRO_Z:
	case SENSOR_CHAN_GYRO_XYZ:
		icm20649_gyro_channel_get(chan, val, data);
		break;
	case SENSOR_CHAN_DIE_TEMP:
		icm20649_gyro_channel_get_temp(val, data);
		break;
	default:
		return -ENOTSUP;
	}

	return 0;
}


static const struct sensor_driver_api icm20649_api_funcs = {
	.attr_set = icm20649_attr_set,
#if CONFIG_ICM20649_TRIGGER
	.trigger_set = icm20649_trigger_set,
#endif
	.sample_fetch = icm20649_sample_fetch,
	.channel_get = icm20649_channel_get,
};

static int icm20649_init_chip(struct device *dev) {
	u8_t chip_id;

	if(icm20649_get_who_am_i(dev, &chip_id) < 0) {
		SYS_LOG_DBG("failed reading chip id");
		return -EIO;
	}
	
	if(icm20649_reset(dev) < 0) {
		SYS_LOG_DBG("failed to reset device");
		return -EIO;
	}

	if(icm20649_wakeup(dev) < 0) {
		SYS_LOG_DBG("failed to power on device");
		return -EIO;
	}

	k_sleep(100);

	if(chip_id != ICM20649_VAL_WHO_AM_I) {
		SYS_LOG_DBG("invalid chip id 0x%x", chip_id);
		return -EIO;
	}

	/*
	if(icm20649_set_accel_sample_rate_div(dev, ICM20649_DEFAULT_ACCEL_SAMPLE_RATE_DIV) < 0) {
		SYS_LOG_DBG("failed to set accelerometer sample rate divider");
		return -EIO;
	}

	if(icm20649_set_accel_decimator(dev, ICM20649_DEFAULT_ACCEL_DECIMATOR) < 0) {
		SYS_LOG_DBG("failed to set accelerometer sample averaging decimator");
		return -EIO;
	}

	if(icm20649_set_accel_filter_choice(dev, ICM20649_DEFAULT_ACCEL_LPF_ENABLE) < 0) {
		SYS_LOG_DBG("failed to set accelerometer low-pass filter settings");
		return -EIO;
	}

	if(icm20649_set_accel_lpf(dev, ICM20649_DEFAULT_ACCEL_LPF_CFG) < 0) {
		SYS_LOG_DBG("failed to set accelerometer low-pass filter settings");
		return -EIO;
	}
	*/
	
	if(icm20649_set_accel_fs_sel(dev, ICM20649_DEFAULT_ACCEL_FULLSCALE) < 0) {
		SYS_LOG_DBG("failed to set accelerometer full-scale");
		return -EIO;
	}

	/*
	if(icm20649_set_gyro_sample_rate_div(dev, ICM20649_DEFAULT_GYRO_SAMPLE_RATE_DIV) < 0) {
		SYS_LOG_DBG("failed to set gyro sample rate divider");
		return -EIO;
	}
	
	if(icm20649_set_gyro_filter_choice(dev, ICM20649_DEFAULT_GYRO_LPF_CHOICE) < 0) {
		SYS_LOG_DBG("failed to set gyroscope low-pass filter settings");
		return -EIO;
	}

	if(icm20649_set_gyro_lpf(dev, ICM20649_DEFAULT_GYRO_LPF_CFG) < 0) {
		SYS_LOG_DBG("failed to set gyroscope low-pass filter settings");
		return -EIO;
	}
	*/

	if(icm20649_set_gyro_fs_sel(dev, ICM20649_DEFAULT_GYRO_FULLSCALE) < 0) {
		SYS_LOG_DBG("failed to set gyroscope full-scale");
		return -EIO;
	}

	if(icm20649_set_odr_align(dev, 1) < 0) {
		SYS_LOG_DBG("failed to set odr align");
		return -EIO;
	}

	#ifdef ICM20649_TRIGGER
	if(icm20649_fifo_int_enable(dev) < 0) {
		SYS_LOG_DBG("failed to enable FIFO interrupts");
		return -EIO;
	}
	if(icm20649_fifo_enable_all(dev) < 0) {
		SYS_LOG_DBG("failed to enable FIFO");
		return -EIO;
	}
	if(icm20649_fifo_reset(dev) < 0) {
		SYS_LOG_DBG("failed to reset FIFO");
		return -EIO;
	}
	#endif

	SYS_LOG_DBG("Successfully initialized ICM20649");
	return 0;
}

/*
int icm20649_get_x_accel(struct icm20649 *device, int16_t *x) {
    return icm20649_read_i16(device, ACCEL_XOUT_H, ACCEL_XOUT_L, x);
}

int icm20649_get_y_accel(struct icm20649 *device, int16_t *y) {
    return icm20649_read_i16(device, ACCEL_YOUT_H, ACCEL_YOUT_L, y);
}

int icm20649_get_z_accel(struct icm20649 *device, int16_t *z) {
    return icm20649_read_i16(device, ACCEL_ZOUT_H, ACCEL_ZOUT_L, z);
}

int icm20649_get_x_gyro(struct icm20649 *device, int16_t *x) {
    return icm20649_read_i16(device, GYRO_XOUT_H, GYRO_XOUT_L, x);
}

int icm20649_get_y_gyro(struct icm20649 *device, int16_t *y) {
    return icm20649_read_i16(device, GYRO_YOUT_H, GYRO_YOUT_L, y);
}

int icm20649_get_z_gyro(struct icm20649 *device, int16_t *z) {
    return icm20649_read_i16(device, GYRO_ZOUT_H, GYRO_ZOUT_L, z);
}

int icm20649_get_temp(struct icm20649 *device, int16_t *temp) {
    return icm20649_read_i16(device, TEMP_OUT_H, TEMP_OUT_L, temp);
}

int icm20649_get_x_gyro_offset(struct icm20649 *device, int16_t *x) {
    return icm20649_read_i16(device, XG_OFFS_USRH, XG_OFFS_USRL, x);
}

int icm20649_get_y_gyro_offset(struct icm20649 *device, int16_t *y) {
    return icm20649_read_i16(device, YG_OFFS_USRH, YG_OFFS_USRL, y);
}

int icm20649_get_z_gyro_offset(struct icm20649 *device, int16_t *z) {
    return icm20649_read_i16(device, ZG_OFFS_USRH, ZG_OFFS_USRL, z);
}

int icm20649_set_x_gyro_offset(struct icm20649 *device, int16_t x) {
    return icm20649_write_i16(device, XG_OFFS_USRH, XG_OFFS_USRL, x);
}

int icm20649_set_y_gyro_offset(struct icm20649 *device, int16_t y) {
    return icm20649_write_i16(device, YG_OFFS_USRH, YG_OFFS_USRL, y);
}

int icm20649_set_z_gyro_offset(struct icm20649 *device, int16_t z) {
    return icm20649_write_i16(device, ZG_OFFS_USRH, ZG_OFFS_USRL, z);
}

int icm20649_get_x_accel_offset(struct icm20649 *device, int16_t *x) {
    return icm20649_read_i16(device, XA_OFFSET_H, XA_OFFSET_L, x);
}

int icm20649_get_y_accel_offset(struct icm20649 *device, int16_t *y) {
    return icm20649_read_i16(device, YA_OFFSET_H, YA_OFFSET_L, y);
}

int icm20649_get_z_accel_offset(struct icm20649 *device, int16_t *z) {
    return icm20649_read_i16(device, ZA_OFFSET_H, ZA_OFFSET_L, z);
}

int icm20649_set_x_accel_offset(struct icm20649 *device, int16_t x) {
    return icm20649_write_i16(device, XA_OFFSET_H, XA_OFFSET_L, x);
}

int icm20649_set_y_accel_offset(struct icm20649 *device, int16_t y) {
    return icm20649_write_i16(device, YA_OFFSET_H, YA_OFFSET_L, y);
}

int icm20649_set_z_accel_offset(struct icm20649 *device, int16_t z) {
    return icm20649_write_i16(device, ZA_OFFSET_H, ZA_OFFSET_L, z);
}
*/

static struct icm20649_config icm20649_config = {
	.dev_name = CONFIG_ICM20649_SPI_MASTER_DEV_NAME,
};


int icm20649_init(struct device *dev) {
	const struct icm20649_config * const config = &icm20649_config;
	struct icm20649_data *data = dev->driver_data;

	data->spi = device_get_binding(config->dev_name);
	if (!data->spi) {
		SYS_LOG_DBG("master not found: %s",
			    config->dev_name);
		return -EINVAL;
	}

#if defined(CONFIG_ICM20649_SPI_GPIO_CS)
	/* handle SPI CS thru GPIO if it is the case */
	if (IS_ENABLED(CONFIG_ICM20649_SPI_GPIO_CS)) {
		icm20649_cs_ctrl.gpio_dev = device_get_binding(
			CONFIG_ICM20649_SPI_GPIO_CS_DRV_NAME);
		if (!icm20649_cs_ctrl.gpio_dev) {
			SYS_LOG_ERR("Unable to get GPIO SPI CS device");
			return -ENODEV;
		}

		icm20649_cs_ctrl.gpio_pin = CONFIG_ICM20649_SPI_GPIO_CS_PIN;
		icm20649_cs_ctrl.delay = 0;

		icm20649_spi_conf.cs = &icm20649_cs_ctrl;

		SYS_LOG_DBG("SPI GPIO CS configured on %s:%u",
			    CONFIG_ICM20649_SPI_GPIO_CS_DRV_NAME,
			    CONFIG_ICM20649_SPI_GPIO_CS_PIN);
	}
#endif

	if (icm20649_init_chip(dev) < 0) {
		SYS_LOG_DBG("failed to initialize chip");
		return -EIO;
	}

	return 0;
}


static struct icm20649_data icm20649_driver;

DEVICE_AND_API_INIT(icm20649, CONFIG_ICM20649_NAME, icm20649_init,
        &icm20649_driver, NULL, POST_KERNEL,
        CONFIG_SENSOR_INIT_PRIORITY, &icm20649_api_funcs);
