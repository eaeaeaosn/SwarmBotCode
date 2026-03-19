#include "wifi_handler.h"
#include <WiFi.h>

// WiFi credentials
const char *wifi_ssid = "Eason iPhone16";      // Enter your Wi-Fi name (2.4GHz)
const char *wifi_password = "12345678";        // Enter Wi-Fi password

// Connect to WiFi network
void wifi_connect() {
    Serial.println("Connecting to WiFi...");
    WiFi.begin(wifi_ssid, wifi_password);
    
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    
    Serial.println();
    Serial.println("Connected to the Wi-Fi network");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
}
