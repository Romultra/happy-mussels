
#include <config.h>
#include <secrets.h>
#include <AdafruitIO_WiFi.h>
#include <utils.h>
#include <network.h>
#include <PID_v1.h>

#define LED_PIN 13

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, "", "");
// set up the feeds
AdafruitIO_Feed *counter = io.feed("counter");
AdafruitIO_Feed *pumpSpeed = io.feed("Pump speed");
AdafruitIO_Feed *temp = io.feed("Temperature");
AdafruitIO_Feed *led = io.feed("Led 27");

// Function prototype for the message handler
void handleMessage(AdafruitIO_Data *data);

double tempSetpoint, tempInput, tempOutput;
PID tempPID(&tempInput, &tempOutput, &tempSetpoint,20,5,1, REVERSE);

double odSetpoint, odInput, odOutput;
PID odPID(&odInput, &odOutput, &odSetpoint, 2, 5, 1, DIRECT);

double tempValue = 25; // Simulate initial temperature value

void setup() {
  pinMode(LED_PIN, OUTPUT);

  // start the serial connection
  Serial.begin(115200);
  // wait for serial monitor to open
  while(!Serial);

  // connect to Wi-Fi
  connectToWiFi();
  connectToAdafruitIO(&io);

  led->onMessage(handleMessage);
  led->get();

  Serial.print("sending -> ");
  Serial.println(tempValue);
  temp->save(tempValue);

  tempInput = tempValue; // Set initial temperature input
  tempSetpoint = 18.0; // Set desired temperature

  //turn the PID on
  tempPID.SetMode(AUTOMATIC);
  tempPID.SetOutputLimits(0, 100); // Set output limits for the PID controller
}



void loop() {
  io.run();
  // Update the temperature input with simulated data
  //Serial.print("sending -> ");
  //Serial.println(tempInput);
  //temp->save(tempInput);

  tempPID.Compute();
  tempValue = tempValue - (tempOutput-5) * 0.01; // Simulate temperature change based on pump output
  tempInput = tempValue;
  
  Serial.print("Current Temperature: ");
  Serial.print(tempInput);
  Serial.print(" | Setpoint: ");
  Serial.print(tempSetpoint);
  Serial.print(" | Pump Speed: ");
  Serial.println(tempOutput);

  delay(1000);
}

void handleMessage(AdafruitIO_Data *data) {
  Serial.print("received <- ");
  Serial.println(data->toString());

  // Check if the received data is "on" or "off"
  if (data->toString() == "HIGH") {
    digitalWrite(LED_PIN, HIGH);
    Serial.println("LED turned ON");
  } else if (data->toString() == "LOW") {
    digitalWrite(LED_PIN, LOW);
    Serial.println("LED turned OFF");
  } else {
    Serial.println("Unknown command received");
  }
}