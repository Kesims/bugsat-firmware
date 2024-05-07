#include <stdbool.h>
#include <string.h>
#include "sd_card_handler.h"
#include "fatfs.h"
#include "debug_printf.h"
#include "indicator_utils.h"
#include "gps_driver.h"
#include "sensors_handler.h"

//SD Card
FATFS fs;  // file system
FILINFO fno;
FRESULT fresult;  // result
UINT br, bw;  // File read/write count

static char filename[] = "data.txt";
static char data[250] = {0}; // Adjust the size as needed
static char session_split[] = "------------------------------------\n";

bool write_to_sd = false;

extern GPSData gps_buffer;
extern BPM390BufferData bmp390_data;
extern LIS3DHBufferData lis3dh_highg;
extern



//    fresult = f_mount(&fs, "/", 1);    //1=mount now
//
//    if (fresult != FR_OK)
//    {
//        debugPrintf("No SD Card found : (%i)\r\n", fresult);
//    }
//    debugPrint("SD Card Mounted Successfully!!!\r\n");
//    //Read the SD Card Total size and Free Size
//    FATFS *pfs;
//    DWORD fre_clust;
//    uint32_t totalSpace, freeSpace;
//    f_getfree("", &fre_clust, &pfs);
//    totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
//    freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);
//    printf("TotalSpace : %lu bytes, FreeSpace = %lu bytes\n", totalSpace, freeSpace);
//    //Open the file
//    fresult = f_open(&fil, "test.txt", FA_WRITE | FA_READ | FA_CREATE_ALWAYS);
//    if(fresult != FR_OK)
//    {
//        printf("File creation/open Error : (%i)\r\n", fresult);
//    }
//    printf("Writing data!!!\r\n");
//    //write the data
//    f_puts("Cansat test input on the SD card.", &fil);
//    //close your file
//    f_close(&fil);


void init_sd_card() {
    fresult = f_mount(&fs, "/", 1);    //1=mount now

    if (fresult != FR_OK)
    {
        debugPrintf("No SD Card found : (%i)\r\n", fresult);
        indicate_error();
    }
    debugPrint("SD Card Mounted Successfully!!!\r\n");
    //Read the SD Card Total size and Free Size
    FATFS *pfs;
    DWORD fre_clust;
    uint32_t totalSpace, freeSpace;
    f_getfree("", &fre_clust, &pfs);
    totalSpace = (uint32_t)((pfs->n_fatent - 2) * pfs->csize * 0.5);
    freeSpace = (uint32_t)(fre_clust * pfs->csize * 0.5);
    debugPrintf("SD CARD: TotalSpace : %lu bytes, FreeSpace = %lu bytes\n", totalSpace, freeSpace);

    // write the session split
    FIL file;
    fresult = f_open(&file, filename, FA_OPEN_APPEND | FA_WRITE);
    if (fresult != FR_OK) {
        debugPrintf("Failed to open file: %s\n", filename);
        return;
    }
    f_puts(session_split, &file);
    f_close(&file);
}

void sd_card_thread_work() {
    FIL file;
    FRESULT res;

    // Check if the file exists
    res = f_stat(filename, NULL);
    if (res == FR_NO_FILE) {
        // If the file does not exist, create it
        res = f_open(&file, filename, FA_CREATE_NEW);
        if (res != FR_OK) {
            debugPrintf("Failed to create file: %s\n", filename);
            return;
        }
        f_close(&file);
    }

    for(;;) {

        if(!write_to_sd) {
            osDelay(50);
            continue;
        }
        // Open the file in append mode
        res = f_open(&file, filename, FA_OPEN_APPEND | FA_WRITE);
        if (res != FR_OK) {
            debugPrintf("Failed to open file: %s\n", filename);
            return;
        }

        memset(data, 0, sizeof(data));

        //seconds, latitude, longitude, sattelites, altitude, hdop, temperature, pressure, highg_x, highg_y, highg_z
        sprintf(data, "%lu,%.6f,%.6f,%d,%.2hu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                gps_buffer.seconds_since_midnight, gps_buffer.latitude, gps_buffer.longitude, gps_buffer.satellites, gps_buffer.altitude, gps_buffer.hdop,
                bmp390_data.temperature, bmp390_data.pressure, lis3dh_highg.accelerationX, lis3dh_highg.accelerationY, lis3dh_highg.accelerationZ);

        // Write the data to the file
        f_puts(data, &file);

        // Close the file
        f_close(&file);
    }
}

