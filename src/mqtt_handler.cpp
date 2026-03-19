#include "mqtt_handler.h"
#include <WiFi.h>

// MQTT Broker configuration
const char *mqtt_broker = "broker.emqx.io";
const char *mqtt_topic = "emqx/esp32";
const char *mqtt_username = "emqx";
const char *mqtt_password = "public";
const int mqtt_port = 1883;

// Initialize MQTT client
void mqtt_init(PubSubClient &client) {
    client.setServer(mqtt_broker, mqtt_port);
    client.setCallback(mqtt_callback);
}

// Connect to MQTT broker
void mqtt_connect(PubSubClient &client) {
    while (!client.connected()) {
        String client_id = "esp32-client-";
        client_id += String(WiFi.macAddress());
        
        Serial.printf("The client %s connects to the public MQTT broker\n", client_id.c_str());
        
        if (client.connect(client_id.c_str(), mqtt_username, mqtt_password)) {
            Serial.println("Public EMQX MQTT broker connected");
            
            // Publish and subscribe
            client.publish(mqtt_topic, "Hi, I'm ESP32 ^^");
            client.subscribe(mqtt_topic);
        } else {
            Serial.print("Failed with state ");
            Serial.print(client.state());
            Serial.println(". Retrying in 2 seconds...");
            delay(2000);
        }
    }
}

// Reconnect to MQTT broker if connection lost
void mqtt_reconnect(PubSubClient &client) {
    if (!client.connected()) {
        Serial.println("MQTT connection lost. Reconnecting...");
        mqtt_connect(client);
    }
}

// MQTT message callback
void mqtt_callback(char *topic, byte *payload, unsigned int length) {
    Serial.print("Message arrived in topic: ");
    Serial.println(topic);
    Serial.print("Message: ");
    
    for (int i = 0; i < length; i++) {
        Serial.print((char)payload[i]);
    }
    
    Serial.println();
    Serial.println("-----------------------");
}
