#ifndef MQTT_HANDLER_H
#define MQTT_HANDLER_H

#include <Arduino.h>
#include <PubSubClient.h>

// MQTT configuration
extern const char *mqtt_broker;
extern const char *mqtt_topic;
extern const char *mqtt_username;
extern const char *mqtt_password;
extern const int mqtt_port;

// Function declarations
void mqtt_init(PubSubClient &client);
void mqtt_connect(PubSubClient &client);
void mqtt_reconnect(PubSubClient &client);
void mqtt_callback(char *topic, byte *payload, unsigned int length);

#endif
