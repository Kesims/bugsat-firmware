#include <WiFi.h>
#include "wifi_handler.h"


void keep_wifi_alive() {

    if (WiFi.isConnected()) {
        vTaskDelay(15000/portTICK_PERIOD_MS);   // Rest the thread for 15 seconds
        return; // The Wi-Fi is already connected, no need to do anything
    }

    // Else try connecting to the Wi-Fi
    WiFi.begin("phk", "heslophk");

    unsigned long timer = millis();
    while(!WiFi.isConnected() && millis()-timer < 20000);    // Give it some time to try connecting
    vTaskDelay(30000/portTICK_PERIOD_MS); // Retry after 30 seconds
}

void uploadStatus(byte * data, byte nodeID, int nodeType) { // Data rn gets deleted after this is called so do something with them if I need them
    if(WiFi.status()== WL_CONNECTED){
        WiFiClient client;
        HTTPClient http;

        // Domain name with URL path or IP address with path
        String url = String(config.apiAddress) + "nodes/addStatus";
        debugPrintln(url);
        url.replace("\r", "");
        url.replace("\n", "");
        http.begin(client, url);

        http.addHeader("Content-Type", "application/json");

        DynamicJsonDocument jdata(512);

        jdata["competition_id"] = String(config.competitionID);
        jdata["node_id"] = String(nodeID);
        jdata["node_type"] = String(nodeType);
        jdata["battery_level"] = String(data[0]);
        jdata["user_token"] = String(config.userToken);
        jdata["neighbour_count"] = String(data[1]);
        JsonArray neighbours  = jdata.createNestedArray("neighbours");
        for(int i = 2; i < data[1]; i++) {
            neighbours.add(String(data[i]));
        }

        char * json = new char[512];
        serializeJson(jdata, json, 512);

        int httpResponseCode = http.POST(json); // If response code is -1, there is no internet connection and request failed
        debugPrint(F("HTTP response code to status upload: "));
        debugPrintln(httpResponseCode);

        delete[](json);

        // Free resources
        http.end();
    }
    else {
        debugPrintln(F("Unable to upload status report, WiFi disconnected!"));
    }
}
