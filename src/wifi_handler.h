#ifndef WIFI_HANDLER_H
#define WIFI_HANDLER_H

#include <Arduino.h>

// WiFi configuration
extern const char *wifi_ssid;
extern const char *wifi_password;

// Function declarations
void wifi_connect();

#endif
