// ---------------------------------------------------------------------------------------------------------------------
// includes
// ---------------------------------------------------------------------------------------------------------------------

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <inttypes.h>
#include <stddef.h>
#include "main.h"
#include "peripheral_config.h"
#include "app_entry.h"
#include "otp.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "app_examples_conf.h"

#include "hw_conf.h"

#include "imu9_axes_impl.h"
// ------------------------------------------------------------------------------------------------
// private constant macros
// ------------------------------------------------------------------------------------------------

#define OUT_XYZ_SIZE    6

#define LIM3DS_I2C_ADDR (0b00011110)
#define SENSOR_HUB_SLAVE_WRITE (0x00)
#define SENSOR_HUB_SLAVE_READ (0x01)

// ------------------------------------------------------------------------------------------------
// private function macros
// ------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------------
// private typedefs, structures, unions and enums
// ------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------------
// private variables
// ------------------------------------------------------------------------------------------------

static float magnetic_mG[3];
static axis3bit16_t data_raw_magnetic;

static float acceleration_mg[3];
static float angular_rate_mdps[3];
static axis3bit16_t data_raw_magnetic;
static axis1bit32_t data_raw_pressure;
static axis1bit16_t data_raw_temperature;
static axis3bit16_t data_raw_acceleration;
static axis3bit16_t data_raw_angular_rate;

static lis2mdl_ctx_t *mag_ctx_ptr;
static lsm6dsl_ctx_t *dev_ctx_ptr;

static char tx_buffer[1000];
// ------------------------------------------------------------------------------------------------
// public variables
// ------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------------
// private function prototypes
// ------------------------------------------------------------------------------------------------

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len);

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);

static int32_t lsm6dsl_read_lis2mdl_cx(void* ctx, uint8_t reg, uint8_t* data,
        uint16_t len);
static int32_t lsm6dsl_write_lis2mdl_cx(void* ctx, uint8_t reg, uint8_t* data,
        uint16_t len);

// ------------------------------------------------------------------------------------------------
// private functions
// ------------------------------------------------------------------------------------------------

static int32_t platform_write(void *handle, uint8_t reg, uint8_t *bufp,
                              uint16_t len)
{
  if (handle == &hi2c1)
  {
    if(HAL_I2C_Mem_Write(handle, LSM6DSL_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, bufp, len, 1000) != HAL_OK)
    {
      APP_DBG_MSG("I2C ERROR!");
    }
  }
  return 0;
}

static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len)
{
  if (handle == &hi2c1)
  {
    if(HAL_I2C_Mem_Read(handle, LSM6DSL_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000) != HAL_OK)
    {
      APP_DBG_MSG("I2C ERROR!");
    }
  }
  return 0;
}

/*
 * Read data byte from internal register of a slave device connected
 * to master I2C interface
 */
static int32_t lsm6dsl_read_lis2mdl_cx(void* ctx, uint8_t reg, uint8_t* data,
        uint16_t len)
{

// ******** 15. Turn off the accelerometer

    uint8_t ctrl = 0x00;

    platform_write(&IMU_I2C, LSM6DSL_CTRL1_XL, &ctrl, 1);


  // ******** 16. Enable access to embedded functions registers (bank A)

    ctrl = 0x80;

    platform_write(&IMU_I2C, LSM6DSL_FUNC_CFG_ACCESS, &ctrl, 1);


  // ******** 17. Write slave address

    ctrl = (LIM3DS_I2C_ADDR << 1) | SENSOR_HUB_SLAVE_READ;

    platform_write(&IMU_I2C, LSM6DSL_SLV0_ADD, &ctrl, 1);


  // ******** 18. Write register

    ctrl = reg;

    platform_write(&IMU_I2C, LSM6DSL_SLV0_SUBADD, &ctrl, 1);


  // ******** 19. Write slave configuration

    ctrl = len;  // no decimation (00), one sensor (00), src mode read disabled (0), number of read operations (001)

    platform_write(&IMU_I2C, LSM6DSL_SLAVE0_CONFIG, &ctrl, 1);


  // ******** 20. Disable access to embedded functions registers (bank A)

    ctrl = 0x00;

    platform_write(&IMU_I2C, LSM6DSL_FUNC_CFG_ACCESS, &ctrl, 1);


  // ******** 21. Enable embedded functions

    ctrl = 0x04;

    platform_write(&IMU_I2C, LSM6DSL_CTRL10_C , &ctrl, 1);

  // ******** 22. Sensor hub trigger signal is XL Data-Ready + Enable internal pull-up on SDx/SCx lines + Enable auxiliary I2C master

    ctrl = 0x09;

    platform_write(&IMU_I2C, LSM6DSL_MASTER_CONFIG, &ctrl, 1);


  // ******** 23. Turn on the accelerometer

    ctrl = 0x80;

    platform_write(&IMU_I2C, LSM6DSL_CTRL1_XL, &ctrl, 1);

    // lsm6dsl_xl_data_rate_set(&dev_ctx, LSM6DSL_XL_ODR_104Hz);

    // ******** 11. 12. Wait for the sensor hub communication to be concluded (loop)
    do
    {
     
        platform_read(&IMU_I2C, LSM6DSL_FUNC_SRC1, &ctrl, 1);


    } while ((ctrl & 0x01) == 0);

    // read data ?

    platform_read(&IMU_I2C, LSM6DSL_SENSORHUB1_REG, data, len);

    // ******** 25. Disable embedded functions

    ctrl = 0x00;

    platform_write(&IMU_I2C, LSM6DSL_CTRL10_C , &ctrl, 1);
    // platform_write(&IMU_I2C, LSM6DSL_CTRL1_XL, &ctrl, 1);


    // ******** 26. Turn Off HUB SENSOR  Master I2C

    ctrl = 0x00;

    platform_write(&IMU_I2C, LSM6DSL_MASTER_CONFIG, &ctrl, 1);

    return 0;
}


// * Write data byte to internal register of a slave device connected
// * to master I2C interface
//
static int32_t lsm6dsl_write_lis2mdl_cx(void* ctx, uint8_t reg, uint8_t* data,
        uint16_t len)
{

// ******** 15. Turn off the accelerometer

    uint8_t ctrl = 0x00;

    platform_write(&IMU_I2C, LSM6DSL_CTRL1_XL, &ctrl, 1);


  // ******** 16. Enable access to embedded functions registers (bank A)

    ctrl = 0x80;

    platform_write(&IMU_I2C, LSM6DSL_FUNC_CFG_ACCESS, &ctrl, 1);


  // ******** 17. Write slave address

    ctrl = (LIM3DS_I2C_ADDR << 1) | SENSOR_HUB_SLAVE_WRITE;

    platform_write(&IMU_I2C, LSM6DSL_SLV0_ADD, &ctrl, 1);


  // ******** 18. Write register

    ctrl = reg;

    platform_write(&IMU_I2C, LSM6DSL_SLV0_SUBADD, &ctrl, 1);


  // ******** 19.1 Write data to register

    platform_write(&IMU_I2C, LSM6DSL_DATAWRITE_SRC_MODE_SUB_SLV0, data, 1);


  // ******** 19.2 Write slave configuration

    ctrl = 0x10;  // no decimation (00), one sensor (01), src mode read disabled (0), number of read operations (000)

    platform_write(&IMU_I2C, LSM6DSL_SLAVE0_CONFIG, &ctrl, 1);

  // ******** 19.3 Enable Write Once

    ctrl = 0x20; 

    platform_write(&IMU_I2C, LSM6DSL_SLAVE1_CONFIG, &ctrl, 1);


  // ******** 20. Disable access to embedded functions registers (bank A)

    ctrl = 0x00;

    platform_write(&IMU_I2C, LSM6DSL_FUNC_CFG_ACCESS, &ctrl, 1);


  // ******** 21. Enable embedded functions

    ctrl = 0x04;

    platform_write(&IMU_I2C, LSM6DSL_CTRL10_C , &ctrl, 1);

  // ******** 22. Sensor hub trigger signal is XL Data-Ready + Enable internal pull-up on SDx/SCx lines + Enable auxiliary I2C master

    ctrl = 0x09;

    platform_write(&IMU_I2C, LSM6DSL_MASTER_CONFIG, &ctrl, 1);


  // ******** 23.1 Turn on the accelerometer

    ctrl = 0x80;

    platform_write(&IMU_I2C, LSM6DSL_CTRL1_XL, &ctrl, 1);

    /*
    *  23.2 Wait Sensor Hub operation flag set
    */
    lsm6dsl_acceleration_raw_get(dev_ctx_ptr, data_raw_acceleration.u8bit);
    uint8_t drdy = 0;
    do
    {
        lsm6dsl_xl_flag_data_ready_get(dev_ctx_ptr, &drdy);
    } while (!drdy);


    // ******** 24. Wait for the sensor hub communication to be concluded (loop)
    do
    {

        platform_read(&IMU_I2C, LSM6DSL_FUNC_SRC1, &ctrl, 1);


    } while ((ctrl & 0x01) == 0);
  
  // ******** 25. Disable embedded functions

    ctrl = 0x00;

    platform_write(&IMU_I2C, LSM6DSL_CTRL10_C , &ctrl, 1);
    // platform_write(&IMU_I2C, LSM6DSL_CTRL1_XL, &ctrl, 1);



  // ******** 26. Turn Off HUB SENSOR  Master I2C

    ctrl = 0x00;

    platform_write(&IMU_I2C, LSM6DSL_MASTER_CONFIG, &ctrl, 1);

  return 0;
}

// ------------------------------------------------------------------------------------------------
// interrupt handlers
// ------------------------------------------------------------------------------------------------

// ------------------------------------------------------------------------------------------------
// public functions
// ------------------------------------------------------------------------------------------------
void init_imu9_axes(lsm6dsl_ctx_t* dev_ctx, lis2mdl_ctx_t* mag_ctx)
{
	printf(" init_imu9_axes ! \r\n");
	dev_ctx_ptr = dev_ctx;
	mag_ctx_ptr = mag_ctx;
	static uint8_t whoamI, rst;

	lsm6dsl_sh_cfg_read_t lis2mdl_conf = {
			.slv_add = LIM3DS_I2C_ADDR,
			.slv_subadd = LIS2MDL_OUTX_L_REG,
			.slv_len = OUT_XYZ_SIZE,
	};

	dev_ctx->write_reg = platform_write;
	dev_ctx->read_reg = platform_read;
	dev_ctx->handle = &IMU_I2C;

	/*
	 * Configure low level function to access to external device
	 */
	mag_ctx->read_reg = lsm6dsl_read_lis2mdl_cx;
	mag_ctx->write_reg = lsm6dsl_write_lis2mdl_cx;
	mag_ctx->handle = &IMU_I2C;

  /*
//   * Initialize platform specific hardware
//   */
//  platform_init();
	APP_DBG_MSG(" LSM6DSL init ! ");
  /*
   * Check device ID
   */
  lsm6dsl_device_id_get(dev_ctx, &whoamI);
  if (whoamI != LSM6DSL_ID)
  {
	  APP_DBG_MSG(" LSM6DSL not find ! ");
	  ERROR_HANDLER();
  }

  APP_DBG_MSG(" LSM6DSL find ! ");

  /*
   * Restore default configuration
   */
  lsm6dsl_reset_set(dev_ctx, PROPERTY_ENABLE);
  do {
	  osDelay(1);
	  lsm6dsl_reset_get(dev_ctx, &rst);
  } while (rst);

  APP_DBG_MSG(" LSM6DSL reset ! ");
  /*
   * Some hardware require to enable pull up on master I2C interface
   */
   //lsm6dsl_sh_pin_mode_set(dev_ctx, LSM6DSL_INTERNAL_PULL_UP);

  /*
   * Check if LIS2MDL connected to Sensor Hub
   */
  lis2mdl_device_id_get(mag_ctx, &whoamI);
  if (whoamI != LIS2MDL_ID)
  {
	  APP_DBG_MSG(" LIS2MDL not find ! ");
	  ERROR_HANDLER();
  }

  APP_DBG_MSG(" LIS2MDL find ! ");

  /*
   * Set XL full scale and Gyro full scale
   */
  lsm6dsl_xl_full_scale_set(dev_ctx, LSM6DSL_2g);
  lsm6dsl_gy_full_scale_set(dev_ctx, LSM6DSL_2000dps);

  /*
   * Configure LIS2MDL on the I2C master line
   */

  /*
   *  Restore default configuration
   */
  lis2mdl_reset_set(mag_ctx, PROPERTY_ENABLE);
  do {
	  osDelay(1);
	  lis2mdl_reset_get(mag_ctx, &rst);
  } while (rst);

  APP_DBG_MSG(" LIS2MDL reset ! ");
  /*
   * Configure LIS2MDL on the I2C master line
   */
  lis2mdl_operating_mode_set(mag_ctx, LIS2MDL_CONTINUOUS_MODE);
  lis2mdl_offset_temp_comp_set(mag_ctx, PROPERTY_ENABLE);
  lis2mdl_block_data_update_set(mag_ctx, PROPERTY_ENABLE);
  lis2mdl_data_rate_set(mag_ctx, LIS2MDL_ODR_50Hz);


  /*
   * Prepare sensor hub to read data from external Slave0
   */
  lsm6dsl_sh_slv0_cfg_read(dev_ctx, &lis2mdl_conf);

  /*
   * Configure Sensor Hub to read 1 slave
   */
  lsm6dsl_sh_num_of_dev_connected_set(dev_ctx, LSM6DSL_SLV_0);

  /*
   * Enable master and XL trigger
   */
  lsm6dsl_func_en_set(dev_ctx, PROPERTY_ENABLE);
  lsm6dsl_sh_master_set(dev_ctx, PROPERTY_ENABLE);

  /*
   * Set XL and Gyro Output Data Rate
   */

  lsm6dsl_xl_data_rate_set(dev_ctx, LSM6DSL_XL_ODR_52Hz);
  lsm6dsl_gy_data_rate_set(dev_ctx, LSM6DSL_GY_ODR_26Hz);
  APP_DBG_MSG(" IMU Init ");

}

int32_t imu9_get_X_axes(LSM6DSL_Axes_t *acc)
{
	uint8_t status;
	axis3bit16_t data_raw;
	float sensitivity = 0.0f;
	int32_t ret;

	/* Wait for data ready */
	do
	{
		if (lsm6dsl_xl_flag_data_ready_get(dev_ctx_ptr, &status) != LSM6DSL_OK)
		{
			return LSM6DSL_ERROR;
		}
	}
	while (status == 0U);

	/* Read accelero data */



	/* Read raw data values. */
	if (lsm6dsl_acceleration_raw_get(dev_ctx_ptr, data_raw.u8bit) != LSM6DSL_OK)
	{
		return LSM6DSL_ERROR;
	}

	/* Get LSM6DSL actual sensitivity. */

	lsm6dsl_fs_xl_t full_scale;

	/* Read actual full scale selection from sensor. */
	if (lsm6dsl_xl_full_scale_get(dev_ctx_ptr, &full_scale) != LSM6DSL_OK)
	{
		return LSM6DSL_ERROR;
	}

	/* Store the Sensitivity based on actual full scale. */
	switch (full_scale)
	{
	case LSM6DSL_2g:
		sensitivity = LSM6DSL_ACC_SENSITIVITY_FS_2G;
		break;

	case LSM6DSL_4g:
		sensitivity = LSM6DSL_ACC_SENSITIVITY_FS_4G;
		break;

	case LSM6DSL_8g:
		sensitivity = LSM6DSL_ACC_SENSITIVITY_FS_8G;
		break;

	case LSM6DSL_16g:
		sensitivity = LSM6DSL_ACC_SENSITIVITY_FS_16G;
		break;

	default:
		ret = LSM6DSL_ERROR;
		break;
	}

	/* Calculate the data. */
	acc->x = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
	acc->y = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
	acc->z = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

	return LSM6DSL_OK;
}

int32_t imu9_get_G_axes(LSM6DSL_Axes_t *angularRate)
{
	uint8_t status;
	axis3bit16_t data_raw;
	float sensitivity = 0.0f;
	int32_t ret;

	/* Wait for data ready */
	do
	{
		if (lsm6dsl_gy_flag_data_ready_get(dev_ctx_ptr, &status) != LSM6DSL_OK)
		{
			return LSM6DSL_ERROR;
		}
	}
	while (status == 0U);

	/* Read accelero data */



	/* Read raw data values. */
	if (lsm6dsl_angular_rate_raw_get(dev_ctx_ptr, data_raw.u8bit) != LSM6DSL_OK)
	{
		return LSM6DSL_ERROR;
	}

	/* Get LSM6DSL actual sensitivity. */

	lsm6dsl_fs_xl_t full_scale;

	/* Read actual full scale selection from sensor. */
	if (lsm6dsl_gy_full_scale_get(dev_ctx_ptr, &full_scale) != LSM6DSL_OK)
	{
		return LSM6DSL_ERROR;
	}

	  /* Store the sensitivity based on actual full scale. */
	  switch (full_scale)
	  {
	    case LSM6DSL_125dps:
	    	sensitivity = LSM6DSL_GYRO_SENSITIVITY_FS_125DPS;
	      break;

	    case LSM6DSL_250dps:
	    	sensitivity = LSM6DSL_GYRO_SENSITIVITY_FS_250DPS;
	      break;

	    case LSM6DSL_500dps:
	    	sensitivity = LSM6DSL_GYRO_SENSITIVITY_FS_500DPS;
	      break;

	    case LSM6DSL_1000dps:
	    	sensitivity = LSM6DSL_GYRO_SENSITIVITY_FS_1000DPS;
	      break;

	    case LSM6DSL_2000dps:
	    	sensitivity = LSM6DSL_GYRO_SENSITIVITY_FS_2000DPS;
	      break;

	    default:
	      ret = LSM6DSL_ERROR;
	      break;
	  }

	/* Calculate the data. */
	  angularRate->x = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
	  angularRate->y = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
	  angularRate->z = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

	return LSM6DSL_OK;
}
int32_t imu9_get_M_axes(LIS2MDL_Axes_t *mag)
{
	uint8_t status;
	axis3bit16_t data_raw;
	float sensitivity = LIS2MDL_MAG_SENSITIVITY_FS_50GAUSS;
	int32_t ret;

	/* Wait for data ready */
	do
	{
		if (lis2mdl_mag_data_ready_get(mag_ctx_ptr, &status) != LIS2MDL_OK)
		{
			return LIS2MDL_ERROR;
		}
	}
	while (status == 0U);

	/* Read accelero data */



	/* Read raw data values. */
	if (lis2mdl_magnetic_raw_get(mag_ctx_ptr, data_raw.u8bit) != LIS2MDL_OK)
	{
		return LIS2MDL_ERROR;
	}

	/* Calculate the data. */
	  mag->x = (int32_t)((float)((float)data_raw.i16bit[0] * sensitivity));
	  mag->y = (int32_t)((float)((float)data_raw.i16bit[1] * sensitivity));
	  mag->z = (int32_t)((float)((float)data_raw.i16bit[2] * sensitivity));

	return LSM6DSL_OK;
}
void lunch_test_task_imu9_axes(void)
{

  while(1)
  {
    osDelay(100);

    uint8_t drdy;
    lsm6dsl_emb_sh_read_t sh_reg;

    /*
     * Read output only if new value is available
     */
    lsm6dsl_xl_flag_data_ready_get(dev_ctx_ptr, &drdy);
    if (drdy)
    {
      /*
       * Read acceleration field data
       */
      memset(data_raw_acceleration.u8bit, 0x0, 3 * sizeof(int16_t));
      lsm6dsl_acceleration_raw_get(dev_ctx_ptr, data_raw_acceleration.u8bit);
      // acceleration_mg[0] =
      //   lsm6dsl_from_fs2g_to_mg(data_raw_acceleration.i16bit[0]);
      // acceleration_mg[1] =
      //   lsm6dsl_from_fs2g_to_mg(data_raw_acceleration.i16bit[1]);
      // acceleration_mg[2] =
      //   lsm6dsl_from_fs2g_to_mg(data_raw_acceleration.i16bit[2]);

      // APP_DBG_MSG("Acceleration [mg]: %4.2f %4.2f %4.2f",
      //         acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);

      APP_DBG_MSG("Acceleration [mg]:%d %d %d",
              data_raw_acceleration.i16bit[0], data_raw_acceleration.i16bit[1], data_raw_acceleration.i16bit[2]);
    }

    lis2mdl_mag_data_ready_get(mag_ctx_ptr, &drdy);
    if (drdy)
    {
      /*
       * Read magnetic data from sensor hub register: XL trigger a new read to
       * mag sensor
       */
      // lsm6dsl_sh_read_data_raw_get(&dev_ctx, &sh_reg);
      // memcpy((uint8_t *)&data_raw_magnetic,
      //        (uint8_t *)&sh_reg.sh_byte_1,
      //        OUT_XYZ_SIZE);

      memset(data_raw_magnetic.u8bit, 0x0, 3 * sizeof(int16_t));
      lis2mdl_magnetic_raw_get(mag_ctx_ptr, data_raw_magnetic.u8bit);
      // magnetic_mG[0] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[0]);
      // magnetic_mG[1] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[1]);
      // magnetic_mG[2] = lis2mdl_from_lsb_to_mgauss(data_raw_magnetic.i16bit[2]);

      // APP_DBG_MSG("Mag [mG]: %4.2f %4.2f %4.2f",
      //         magnetic_mG[0], magnetic_mG[1], magnetic_mG[2]);


      APP_DBG_MSG("Magnetic field [mG]:%d %d %d",
              data_raw_magnetic.i16bit[0], data_raw_magnetic.i16bit[1], data_raw_magnetic.i16bit[2]);

    }

    lsm6dsl_gy_flag_data_ready_get(dev_ctx_ptr, &drdy);
    if (drdy)
    {
      /*
       * Read angular rate field data
       */
      memset(data_raw_angular_rate.u8bit, 0x0, 3 * sizeof(int16_t));
      lsm6dsl_angular_rate_raw_get(dev_ctx_ptr, data_raw_angular_rate.u8bit);
      // angular_rate_mdps[0] =
      //   lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[0]);
      // angular_rate_mdps[1] =
      //   lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[1]);
      // angular_rate_mdps[2] =
      //   lsm6dsl_from_fs2000dps_to_mdps(data_raw_angular_rate.i16bit[2]);

      // APP_DBG_MSG("Angular rate [mdps]: %4.2f %4.2f %4.2f",
      //             angular_rate_mdps[0],
      //             angular_rate_mdps[1],
      //             angular_rate_mdps[2]);



      APP_DBG_MSG("Angular rate [mdps]:%d %d %d",
              data_raw_angular_rate.i16bit[0], data_raw_angular_rate.i16bit[1], data_raw_angular_rate.i16bit[2]);


    }
  }

}
