#include "display_handler.h"
#include "Adafruit_SSD1306.h"
#include "debug.h"
#include "device_status.h"

#define USER_BTN 34

#define SCREEN_WIDTH 128 // OLED display width, in pixels-
#define SCREEN_HEIGHT 64 // OLED display height, in pixels


// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
// The pins for I2C are defined by the Wire-library.
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3D ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

extern xSemaphoreHandle i2c_gatekeeper;

void init_display() {
    pinMode(USER_BTN, INPUT);
    if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C, OLED_RESET, true)) { // Address 0x3D for 128x64
        debugPrintln(F("SSD1306 allocation failed"));
        while(1);
    }

    // Show initial display buffer contents on the screen --
    // the library initializes this with an Adafruit splash screen.
    display.display();
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(WHITE);

    display.setCursor(8, 28);
    display.println(F("-- BugSat Ground --"));
    display.setCursor(0, 57);
    display.println(F("DELTA - SSIE PARDUBICE"));
    display.display();
    unsigned long delayTimer = millis()+1200;
    while(delayTimer > millis()); // Non blocking delay
//    displayUpdateDisplay();
}

void display_update() {

    // print status
    display_print_status_bar();

    xSemaphoreTake(i2c_gatekeeper, 250);
    display.display();
    xSemaphoreGive(i2c_gatekeeper);
}

void display_print_status_bar() {
    //Battery
    display.setCursor(0,0);
    display.fillRect(0,0,128,8, BLACK);
    int level = get_battery_level();
    //draw the battery outline
    const int batteryPosX = 5; //Top Right Corner
    const int batteryPosY = 1;
    display.drawRect(batteryPosX, batteryPosY, 15, 7, WHITE);
    display.drawRect(batteryPosX+15, batteryPosY+2, 1, 3, WHITE);
    //draw battery levels
    if (level > 0) {
        display.drawRect(batteryPosX+2, batteryPosY+2, 2, 3, WHITE);
    }
    if (level > 1) {
        display.drawRect(batteryPosX+5, batteryPosY+2, 2, 3, WHITE);
    }
    if (level > 2) {
        display.drawRect(batteryPosX+8, batteryPosY+2, 2, 3, WHITE);
    }
    if (level > 3) {
        display.drawRect(batteryPosX+11, batteryPosY+2, 2, 3, WHITE);
    }

    //draw line bellow to make it stand out from info below
    display.drawLine(0, 9, 127, 9, WHITE);
}

