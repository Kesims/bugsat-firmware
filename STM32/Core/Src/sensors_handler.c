#include "sensors_handler.h"
#include "BMP390.h"
#include "lis3dh_reg.h"
#include "indicator_utils.h"
#include "cmsis_os.h"

extern I2C_HandleTypeDef hi2c1;
extern osSemaphoreId GPS_Buffer_SemaphoreHandle;
extern osSemaphoreId Sensor_Buffer_SemaphoreHandle;

// BMP390 Handle
BMP390_HandleTypeDef hbmp390;


// LIS3DH Handle
#define SENSOR_BUS hi2c1

static int16_t data_raw_acceleration[3];
static float acceleration_mg[3];
static uint8_t whoamI;
stmdev_ctx_t dev_ctx;

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp,
                              uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp,
                             uint16_t len);
static void platform_delay(uint32_t ms);
static void platform_init(void);



// Sensor data Buffers
BPM390BufferData bmp390_data = {0};
LIS3DHBufferData lis3dh_highg = {0};
LIS3DHBufferData lis3dh_lowg = {0};

void sensors_init() {

    // Initialize BMP390
    hbmp390._hi2c = &hi2c1;
    BMP390_Init(&hbmp390);

//    printf("par_t1: %f\n", hbmp390._calib_data.par_t1);
//    printf("par_t2: %f\n", hbmp390._calib_data.par_t2);
//    printf("par_t3: %f\n", hbmp390._calib_data.par_t3);

    BMP390_SetTempOS(&hbmp390, BMP390_NO_OVERSAMPLING);
    BMP390_SetPressOS(&hbmp390, BMP390_OVERSAMPLING_8X);
    BMP390_SetIIRFilterCoeff(&hbmp390, BMP3_IIR_FILTER_COEFF_7);
    BMP390_SetOutputDataRate(&hbmp390, BMP3_ODR_25_HZ);


    // Initialize LIS3DH
    /* Initialize mems driver interface */
    dev_ctx.write_reg = platform_write;
    dev_ctx.read_reg = platform_read;
    dev_ctx.mdelay = platform_delay;
    dev_ctx.handle = &SENSOR_BUS;
    /* Initialize platform specific hardware */
    platform_init();
    /* Check device ID */
    lis3dh_device_id_get(&dev_ctx, &whoamI);

    if (whoamI != LIS3DH_ID) {
        indicate_error();
    }

    /*  Enable Block Data Update */
    lis3dh_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
    /* Set Output Data Rate to 25 hz */
    lis3dh_data_rate_set(&dev_ctx, LIS3DH_ODR_100Hz);
    /* Set full scale to 2 g */
    lis3dh_full_scale_set(&dev_ctx, LIS3DH_16g);
    /* Set operating mode to high resolution */
    lis3dh_operating_mode_set(&dev_ctx, LIS3DH_HR_12bit);
    /* Set FIFO watermark to 25 samples */
    lis3dh_fifo_watermark_set(&dev_ctx, 100);
    /* Set FIFO mode to Stream mode: Accumulate samples and
     * override old data */
    lis3dh_fifo_mode_set(&dev_ctx, LIS3DH_DYNAMIC_STREAM_MODE);
    /* Enable FIFO */
    lis3dh_fifo_set(&dev_ctx, PROPERTY_ENABLE);
}

static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len) {
/* Write multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Write(handle, LIS3DH_I2C_ADD_L, reg,
                      I2C_MEMADD_SIZE_8BIT, (uint8_t*) bufp, len, 1000);
    return 0;
}


static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len) {
/* Read multiple command */
    reg |= 0x80;
    HAL_I2C_Mem_Read(handle, LIS3DH_I2C_ADD_L, reg,
                     I2C_MEMADD_SIZE_8BIT, bufp, len, 1000);
    return 0;
}


static void platform_delay(uint32_t ms) {
    osDelay(ms);
}


static void platform_init(void) {}

void read_bpm390_data() {
    uint32_t raw_pressure;
    uint32_t raw_temperature;
    uint32_t time;
    BMP390_ReadRawPressTempTime(&hbmp390, &raw_pressure, &raw_temperature, &time);
//    if (xSemaphoreTake(Sensor_Buffer_SemaphoreHandle, 120) == pdTRUE) {
        BMP390_CompensateRawPressTemp(&hbmp390, raw_pressure, raw_temperature, &bmp390_data.pressure,
                                      &bmp390_data.temperature);
        osDelay(5);
//        xSemaphoreGive(Sensor_Buffer_SemaphoreHandle);
//    }
}

void read_lis3dh_data() {
    uint8_t flags;
    uint8_t num = 0;
    /* Check if FIFO level over threshold */
    lis3dh_fifo_fth_flag_get(&dev_ctx, &flags);

    if (flags) {
        lis3dh_fifo_data_level_get(&dev_ctx, &num);

        if(num < 0) return;
//        if (xSemaphoreTake(Sensor_Buffer_SemaphoreHandle, 120) == pdTRUE) {
            lis3dh_acceleration_raw_get(&dev_ctx, data_raw_acceleration);
            lis3dh_highg.accelerationX =
                    lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[0]);
            lis3dh_highg.accelerationY =
                    lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[1]);
            lis3dh_highg.accelerationZ =
                    lis3dh_from_fs2_hr_to_mg(data_raw_acceleration[2]);
//            xSemaphoreGive(Sensor_Buffer_SemaphoreHandle);
//        }
    }
}

_Noreturn void sensors_task_work() {
    for(;;) {
        read_bpm390_data();
        read_lis3dh_data();
        osDelay(600);
    }
}