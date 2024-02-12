#include "lis3dh_driver.h"

#include <stdio.h>
#include "stm32l4xx_hal.h"
#include "lis3dh_reg.h"
#include "cmsis_os.h"


extern I2C_HandleTypeDef hi2c1;
#define SENSOR_BUS hi2c1

static int16_t data_raw_acceleration[3];
static float acceleration_mg[3];
static uint8_t whoamI;
static uint8_t tx_buffer[1000];

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);

/* Main Example --------------------------------------------------------------*/
void lis3dh_read_fifo(void)
{
    /* Initialize mems driver interface */
    stmdev_ctx_t dev_ctx;
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = &SENSOR_BUS;
    /* Initialize platform specific hardware */
    platform_init();
    /* Check device ID */
    lis3dh_device_id_get(&dev_ctx, &whoamI);

    if (whoamI != LIS3DH_ID) {
        while (1) {
            /* manage here device not found */
            platform_delay(1);
        }
    }

    /*  Enable Block Data Update */
    lis3dh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    /* Set Output Data Rate to 25 hz */
    lis3dh_data_rate_set(&dev_ctx, LIS3DH_ODR_25Hz);
    /* Set full scale to 2 g */
    lis3dh_full_scale_set(&dev_ctx, LIS3DH_2g);
    /* Set operating mode to high resolution */
    lis3dh_operating_mode_set(&dev_ctx, LIS3DH_HR_12bit);
    /* Set FIFO watermark to 25 samples */
    lis3dh_fifo_watermark_set(&dev_ctx, 25);
    /* Set FIFO mode to Stream mode: Accumulate samples and
     * override old data */
    lis3dh_fifo_mode_set(&dev_ctx, LIS3DH_DYNAMIC_STREAM_MODE);
    /* Enable FIFO */
    lis3dh_fifo_set(&dev_ctx, PROPERTY_ENABLE);

    while (1) {
        uint8_t flags;
        uint8_t num = 0;
        /* Check if FIFO level over threshold */
        lis3dh_fifo_fth_flag_get(&dev_ctx, &flags);

        if (flags) {
            /* Read number of sample in FIFO */
            lis3dh_fifo_data_level_get(&dev_ctx, &num);

            while (num-- > 0) {
                /* Read XL samples */
                lis3dh_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
                acceleration_mg[0] =
                        lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[0]);
                acceleration_mg[1] =
                        lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[1]);
                acceleration_mg[2] =
                        lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[2]);
                sprintf((char *)tx_buffer,
                        "Acceleration [mg]:%4.2f\t%4.2f\t%4.2f\r\n",
                        acceleration_mg[0], acceleration_mg[1], acceleration_mg[2]);
            }
        }
    }
}

/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
/* Write multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Write(handle, LIS3DH_I2C_ADD_L, reg,
                    I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
}

/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
/* Read multiple command */
  reg |= 0x80;
  HAL_I2C_Mem_Read(handle, LIS3DH_I2C_ADD_L, reg,
                   I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms) {
    osDelay(ms);
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void) {

}