#include <WiFi.h>
#include <HTTPClient.h>
#include "wifi_handler.h"
#include "debug.h"
#include "STATIC_CONFIG.h"
#include "lora_handler.h"
#include <ArduinoJson.h>

#define UPLOAD_BUFFER_SIZE 1024
char upload_buffer[UPLOAD_BUFFER_SIZE];


void keep_wifi_alive() {

    if (WiFi.isConnected()) {
        vTaskDelay(15000/portTICK_PERIOD_MS);   // Rest the thread for 15 seconds
        return; // The Wi-Fi is already connected, no need to do anything
    }

    // Else try connecting to the Wi-Fi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    unsigned long timer = millis();
    while(!WiFi.isConnected() && millis()-timer < 20000);    // Give it some time to try connecting
    vTaskDelay(30000/portTICK_PERIOD_MS); // Retry after 30 seconds
}

void http_upload(String url) {
    if(WiFi.status() == WL_CONNECTED){
        WiFiClient client;
        HTTPClient http;

        // Domain name with URL path or IP address with path
        url.replace("\r", "");
        url.replace("\n", "");
        http.begin(client, url);

        debugPrintln("Uploading data to: " + url);

        http.addHeader("Content-Type", "application/json");
        http.addHeader("Authorization", "Bearer " + String(ACCESS_TOKEN));

        int httpResponseCode = http.POST(upload_buffer); // If response code is -1, there is no internet connection and request failed

        if(httpResponseCode < 0) {
            debugPrintln(F("HTTP POST request failed"));
            return;
        }

        debugPrintln("HTTP Response code: " + String(httpResponseCode));

        // Free resources after the request
        http.end();
    }
    else {
        debugPrintln(F("Unable to upload status report, WiFi disconnected!"));
    }
}

void upload_status_data(uint8_t packet_id, LoraStatusData status_data) { // Data rn gets deleted after this is called so do something with them if I need them
    JsonDocument jdata;

    jdata["packet_id"] = packet_id;

// gps ready == first bit of status
    jdata["gps_ready"] = status_data.status & 0x01;

// second bit launch detected
    jdata["launch_detected"] = (status_data.status >> 1) & 0x01;

// third bit bugs deployed
    jdata["bugs_deployed"] = (status_data.status >> 2) & 0x01;

    memset(upload_buffer, 0, UPLOAD_BUFFER_SIZE);
    serializeJson(jdata, upload_buffer, UPLOAD_BUFFER_SIZE);
    http_upload(String(API_ADDRESS) + "upload_status_data");
}

void upload_gps_data(uint8_t packet_id, LoraGPSData gps_data) {
    JsonDocument jdata;

    jdata["packet_id"] = packet_id;
    jdata["seconds_since_midnight"] = gps_data.seconds_since_midnight;
    jdata["latitude"] = gps_data.latitude;
    jdata["longitude"] = gps_data.longitude;
    jdata["altitude"] = gps_data.altitude;
    jdata["hdop"] = gps_data.hdop;

    memset(upload_buffer, 0, UPLOAD_BUFFER_SIZE);
    serializeJson(jdata, upload_buffer, UPLOAD_BUFFER_SIZE);
    http_upload(String(API_ADDRESS) + "upload_gps_data");
}

void upload_sensor_data(uint8_t packet_id, LoraSensorData sensor_data) {
    JsonDocument jdata;

    jdata["packet_id"] = packet_id;
    jdata["temperature"] = (float) sensor_data.temperature / 100;
    jdata["pressure"] = sensor_data.pressure;
    jdata["accelerationX"] = sensor_data.accelerationX;
    jdata["accelerationY"] = sensor_data.accelerationY;
    jdata["accelerationZ"] = sensor_data.accelerationZ;

    memset(upload_buffer, 0, UPLOAD_BUFFER_SIZE);
    serializeJson(jdata, upload_buffer, UPLOAD_BUFFER_SIZE);
    http_upload(String(API_ADDRESS) + "upload_sensor_data");
}

void upload_battery_data(uint8_t packet_id, LoraBatteryData battery_data) {
    JsonDocument jdata;

    jdata["packet_id"] = packet_id;
    jdata["voltage"] = battery_data.voltage;

    memset(upload_buffer, 0, UPLOAD_BUFFER_SIZE);
    serializeJson(jdata, upload_buffer, UPLOAD_BUFFER_SIZE);
    http_upload(String(API_ADDRESS) + "upload_battery_data");
}

void upload_bugpack_data(uint8_t packet_id, LoraBugpackData bugpack_data) {
    JsonDocument jdata;

    jdata["packet_id"] = packet_id;
    jdata["bugpack_id"] = bugpack_data.bugpack_id;
    jdata["battery_voltage"] = bugpack_data.battery_voltage;

    memset(upload_buffer, 0, UPLOAD_BUFFER_SIZE);
    serializeJson(jdata, upload_buffer, UPLOAD_BUFFER_SIZE);
    http_upload(String(API_ADDRESS) + "upload_bugpack_data");
}


