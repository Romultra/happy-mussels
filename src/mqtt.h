#ifndef MQTT_H
#define MQTT_H

#include <PubSubClient.h>

extern PubSubClient mqtt;

// Function to initialize MQTT client
void initMQTT();

// Function to reconnect to MQTT broker
void reconnect();

// MQTT message callback
void onMqttMessage(char* topic, byte* payload, unsigned int len);

#endif