#ifndef MQTT_H
#define MQTT_H

#include <PubSubClient.h>

extern PubSubClient mqtt;

// Function to initialize MQTT client
void initMQTT();

// Function to reconnect to MQTT broker
void reconnect();

// MQTT message callback (now extern, can be defined in main)
extern void onMqttMessage(char* topic, byte* payload, unsigned int len);

// Function to send status updates
void sendStatus(const char* loop_id);

#endif