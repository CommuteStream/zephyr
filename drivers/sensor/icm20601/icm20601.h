/* TDK ICM 20601 3 axis gyro and accelerometer */
/*
 * Copyright (c) 2018 LeanUp LLC
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef __SENSOR_ICM20601_H__
#define __SENSOR_ICM20601_H__

#include <zephyr/types.h>
#include <device.h>

#define ICM20601_REG_SELF_TEST_X_GYRO    0x00
#define ICM20601_REG_SELF_TEST_Y_GYRO    0x01
#define ICM20601_REG_SELF_TEST_Z_GYRO    0x02
#define ICM20601_REG_SELF_TEST_X_ACCEL   0x0D
#define ICM20601_REG_SELF_TEST_Y_ACCEL   0x0E
#define ICM20601_REG_SELF_TEST_Z_ACCEL   0x0F
#define ICM20601_REG_XG_OFFS_USRH        0x13
#define ICM20601_REG_XG_OFFS_USRL        0x14
#define ICM20601_REG_YG_OFFS_USRH        0x15
#define ICM20601_REG_YG_OFFS_USRL        0x16
#define ICM20601_REG_ZG_OFFS_USRH        0x17
#define ICM20601_REG_ZG_OFFS_USRL        0x18
#define ICM20601_REG_SMPLRT_DIV          0x19

#define ICM20601_REG_CONFIG                0x1A
#define ICM20601_MASK_CONFIG_FIFO_MODE     BIT(6)
#define ICM20601_SHIFT_CONFIG_FIFO_MODE    6
#define ICM20601_MASK_CONFIG_EXT_SYNC_SET  (BIT(5) | BIT(4) | BIT(3))
#define ICM20601_SHIFT_CONFIG_EXT_SYNC_SET 3
#define ICM20601_MASK_CONFIG_DLPF_CFG      (BIT(2) | BIT(1) | BIT(0))
#define ICM20601_SHIFT_CONFIG_DLPF_CFG     0

#define ICM20601_REG_GYRO_CONFIG             0x1B
#define ICM20601_MASK_GYRO_CONFIG_XG_ST      BIT(7)
#define ICM20601_SHIFT_GYRO_CONFIG_XG_ST     7
#define ICM20601_MASK_GYRO_CONFIG_YG_ST      BIT(6)
#define ICM20601_SHIFT_GYRO_CONFIG_YG_ST     6
#define ICM20601_MASK_GYRO_CONFIG_ZG_ST      BIT(5)
#define ICM20601_SHIFT_GYRO_CONFIG_ZG_ST     5
#define ICM20601_MASK_GYRO_CONFIG_G_SCALE    (BIT(4) | BIT(3))
#define ICM20601_SHIFT_GYRO_CONFIG_G_SCALE   3
#define ICM20601_MASK_GYRO_CONFIG_FCHOICE_B  (BIT(1) | BIT(0))
#define ICM20601_SHIFT_GYRO_CONFIG_FCHOICE_B 0

#define ICM20601_REG_ACCEL_CONFIG          0x1C
#define ICM20601_MASK_ACCEL_CONFIG_XA_ST   BIT(7)
#define ICM20601_SHIFT_ACCEL_CONFIG_XA_ST  7
#define ICM20601_MASK_ACCEL_CONFIG_YA_ST   BIT(6)
#define ICM20601_SHIFT_ACCEL_CONFIG_YA_ST  6
#define ICM20601_MASK_ACCEL_CONFIG_ZA_ST   BIT(5)
#define ICM20601_SHIFT_ACCEL_CONFIG_ZA_ST  5
#define ICM20601_MASK_ACCEL_CONFIG_A_SCALE (BIT(4) | BIT(3))
#define ICM20601_SHIFT_ACCEL_CONFIG_A_SCALE 3

#define ICM20601_REG_ACCEL_CONFIG_2                   0x1D
#define ICM20601_MASK_ACCEL_CONFIG_2_DEC2_CFG         (BIT(5) | BIT(4))
#define ICM20601_SHIFT_ACCEL_CONFIG_2_DEC2_CFG        4
#define ICM20601_MASK_ACCEL_CONFIG_2_ACCEL_FCHOICE_B  BIT(3)
#define ICM20601_SHIFT_ACCEL_CONFIG_2_ACCEL_FCHOICE_B 3
#define ICM20601_MASK_ACCEL_CONFIG_2_A_DLPF_CFG       (BIT(2) | BIT(1) | BIT(0))
#define ICM20601_SHIFT_ACCEL_CONFIG_2_A_DLPF_CFG      0

#define ICM20601_REG_LP_MODE_CFG         0x1E
#define ICM20601_REG_ACCEL_WOM_THR       0x1F
#define ICM20601_REG_FIFO_EN             0x23
#define ICM20601_REG_FSYNC_INT           0x36
#define ICM20601_REG_INT_PIN_CFG         0x37

#define ICM20601_REG_INT_ENABLE                  0x38
#define ICM20601_MASK_INT_ENABLE_WOM_INT         (BIT(7) | BIT(6) | BIT(5))
#define ICM20601_SHIFT_INT_ENABLE_WOM_INT        5
#define ICM20601_MASK_INT_ENABLE_FIFO_OFLOW_INT  BIT(4)
#define ICM20601_SHIFT_INT_ENABLE_FIFO_OFLOW_INT 4
#define ICM20601_MASK_INT_ENABLE_GDRIVE_INT      BIT(2)
#define ICM20601_SHIFT_INT_ENABLE_GDRIVE_INT     2
#define ICM20601_MASK_INT_ENABLE_DATA_RDY_INT    BIT(0)
#define ICM20601_SHIFT_INT_ENABLE_DATA_RDY_INT   0

#define ICM20601_SPI_READ BIT(7)
#define ICM20601_REG_INT_STATUS          0x3A
#define ICM20601_REG_ACCEL_XOUT_H        0x3B
#define ICM20601_REG_ACCEL_XOUT_L        0x3C
#define ICM20601_REG_ACCEL_YOUT_H        0x3D
#define ICM20601_REG_ACCEL_YOUT_L        0x3E
#define ICM20601_REG_ACCEL_ZOUT_H        0x3F
#define ICM20601_REG_ACCEL_ZOUT_L        0x40
#define ICM20601_REG_TEMP_OUT_H          0x41
#define ICM20601_REG_TEMP_OUT_L          0x42
#define ICM20601_REG_GYRO_XOUT_H         0x43
#define ICM20601_REG_GYRO_XOUT_L         0x44
#define ICM20601_REG_GYRO_YOUT_H         0x45
#define ICM20601_REG_GYRO_YOUT_L         0x46
#define ICM20601_REG_GYRO_ZOUT_H         0x47
#define ICM20601_REG_GYRO_ZOUT_L         0x48
#define ICM20601_REG_SIGNAL_PATH_RESET   0x68
#define ICM20601_REG_ACCEL_INTEL_CTRL    0x69
#define ICM20601_REG_USER_CTRL           0x6A

#define ICM20601_REG_PWR_MGMT_1                   0x6B
#define ICM20601_MASK_PWR_MGMT_1_RESET            BIT(7)
#define ICM20601_SHIFT_PWR_MGMT_1_RESET           7
#define ICM20601_MASK_PWR_MGMT_1_SLEEP            BIT(6)
#define ICM20601_SHIFT_PWR_MGMT_1_SLEEP           6
#define ICM20601_MASK_PWR_MGMT_1_ACCEL_CYCLE      BIT(5)
#define ICM20601_SHIFT_PWR_MGMT_1_ACCEL_CYCLE     5
#define ICM20601_MASK_PWR_MGMT_1_GYRO_STANDBY     BIT(4)
#define ICM20601_SHIFT_PWR_MGMT_1_GYRO_STANDBY    4
#define ICM20601_MASK_PWR_MGMT_1_TEMP_DISABLE     BIT(3)
#define ICM20601_SHIFT_PWR_MGMT_1_TEMP_DISABLE    3
#define ICM20601_MASK_PWR_MGMT_1_CLK_SEL          (BIT(2) | BIT(1) | BIT(0))
#define ICM20601_SHIFT_PWR_MGMT_1_CLK_SEL         0

#define ICM20601_REG_PWR_MGMT_2                              0x6C
#define ICM20601_MASK_PWR_MGMT_2_FIFO_LP_ENABLE              BIT(7)
#define ICM20601_SHIFT_PWR_MGMT_2_FIFO_LP_ENABLE             7
#define ICM20601_MASK_PWR_MGMT_2_STANDBY_XA                  BIT(5)
#define ICM20601_SHIFT_PWR_MGMT_2_STANDBY_XA                 5
#define ICM20601_MASK_PWR_MGMT_2_STANDBY_YA                  BIT(4)
#define ICM20601_SHIFT_PWR_MGMT_2_STANDBY_YA                 4
#define ICM20601_MASK_PWR_MGMT_2_STANDBY_ZA                  BIT(3)
#define ICM20601_SHIFT_PWR_MGMT_2_STANDBY_ZA                 3
#define ICM20601_MASK_PWR_MGMT_2_STANDBY_XG                  BIT(2)
#define ICM20601_SHIFT_PWR_MGMT_2_STANDBY_XG                 2
#define ICM20601_MASK_PWR_MGMT_2_STANDBY_YG                  BIT(1)
#define ICM20601_SHIFT_PWR_MGMT_2_STANDBY_YG                 1
#define ICM20601_MASK_PWR_MGMT_2_STANDBY_ZG                  BIT(0)
#define ICM20601_SHIFT_PWR_MGMT_2_STANDBY_ZG                 0

#define ICM20601_REG_FIFO_COUNTH         0x72
#define ICM20601_REG_FIFO_COUNTL         0x73
#define ICM20601_REG_FIFO_R_W            0x74
#define ICM20601_REG_WHO_AM_I            0x75
#define ICM20601_VAL_WHO_AM_I            0xAC
#define ICM20601_REG_XA_OFFSET_H         0x77
#define ICM20601_REG_XA_OFFSET_L         0x78
#define ICM20601_REG_YA_OFFSET_H         0x7A
#define ICM20601_REG_YA_OFFSET_L         0x7B
#define ICM20601_REG_ZA_OFFSET_H         0x7D
#define ICM20601_REG_ZA_OFFSET_L         0x7E

#if CONFIG_ICM20601_SAMPLE_RATE_DIV == 0
	#define ICM20601_DEFAULT_SAMPLE_RATE_DIV = 1
#else
	#define ICM20601_DEFAULT_SAMPLE_RATE_DIV = CONFIG_ICM20601_SAMPLE_RATE_DIV
#endif

#define ICM20601_DEFAULT_ACCEL_STANDBY_AVG 4
#define ICM20601_DEFAULT_ACCEL_LPF_ENABLE true
#define ICM20601_DEFAULT_ACCEL_LPF_CFG 0

#if CONFIG_ICM20601_ACCEL_FS == 0
	#define ICM20601_ACCEL_FS_RUNTIME 1
	#define ICM20601_DEFAULT_ACCEL_FULLSCALE		3
#elif CONFIG_ICM20601_ACCEL_FS == 4
	#define ICM20601_DEFAULT_ACCEL_FULLSCALE		0
#elif CONFIG_ICM20601_ACCEL_FS == 8
	#define ICM20601_DEFAULT_ACCEL_FULLSCALE		1
#elif CONFIG_ICM20601_ACCEL_FS == 16 
	#define ICM20601_DEFAULT_ACCEL_FULLSCALE		2
#elif CONFIG_ICM20601_ACCEL_FS == 32
	#define ICM20601_DEFAULT_ACCEL_FULLSCALE		3
#endif

#define ICM20601_DEFAULT_GYRO_STANDBY_AVG 4
#define ICM20601_DEFAULT_GYRO_LPF_ENABLE true
#define ICM20601_DEFAULT_GYRO_LPF_CFG 0

#if CONFIG_ICM20601_GYRO_FS == 0
	#define ICM20601_GYRO_FS_RUNTIME 1
	#define ICM20601_DEFAULT_GYRO_FULLSCALE		3
#elif CONFIG_ICM20601_GYRO_FS == 500
	#define ICM20601_DEFAULT_GYRO_FULLSCALE		0
#elif CONFIG_ICM20601_GYRO_FS == 1000
	#define ICM20601_DEFAULT_GYRO_FULLSCALE		1
#elif CONFIG_ICM20601_GYRO_FS == 2000
	#define ICM20601_DEFAULT_GYRO_FULLSCALE		2
#elif CONFIG_ICM20601_GYRO_FS == 4000
	#define ICM20601_DEFAULT_GYRO_FULLSCALE		3
#endif

struct icm20601_data {
    struct device *spi;
    s16_t accel_sample_x;
    s16_t accel_sample_y;
    s16_t acell_sample_z;

    s16_t temp_sample;

    s16_t gyro_sample_x;
    s16_t gyro_sample_y;
    s16_t gyro_sample_z;

#ifdef CONFIG_ICM20601_TRIGGER
	struct device *gpio;
	struct gpio_callback gpio_cb;

	struct sensor_trigger data_ready_trigger;
	sensor_trigger_handler_t data_ready_handler;

#if defined(CONFIG_ICM20601_TRIGGER_OWN_THREAD)
	K_THREAD_STACK_MEMBER(thread_stack, CONFIG_ICM20601_THREAD_STACK_SIZE);
	struct k_thread thread;
	struct k_sem gpio_sem;
#elif defined(CONFIG_ICM20601_TRIGGER_GLOBAL_THREAD)
	struct k_work work;
	struct device *dev;
#endif

#endif /* CONFIG_ICM20601_TRIGGER */
}

#define SYS_LOG_DOMAIN "ICM20601"
#define SYS_LOG_LEVEL CONFIG_SYS_LOG_SENSOR_LEVEL
#include <logging/sys_log.h>
#endif /* __SENSOR_ICM20601_H__ */
