
#include <config.h>
#include <secrets.h>
#include <AdafruitIO_WiFi.h>
#include <utils.h>

#define LED_PIN 13

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, "", "");
// set up the feeds
AdafruitIO_Feed *counter = io.feed("counter");
AdafruitIO_Feed *pumpSpeed = io.feed("Pump speed");
AdafruitIO_Feed *temp = io.feed("Temperature");
AdafruitIO_Feed *led = io.feed("Led 27");

// Function prototype for the message handler
void handleMessage(AdafruitIO_Data *data);

void setup() {
  pinMode(LED_PIN, OUTPUT);
  // start the serial connection
  Serial.begin(115200);

  // wait for serial monitor to open
  while(!Serial);

  // connect to Wi-Fi
  connectToWiFi();

  Serial.print("Connecting to Adafruit IO");
  // connect to io.adafruit.com
  io.connect();

  // wait for a connection
  while(io.status() < AIO_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  // we are connected
  Serial.println();
  Serial.println(io.statusText());

  led->onMessage(handleMessage);
  led->get();

}

void loop() {
  // io.run(); is required for all sketches.
  // it should always be present at the top of your loop
  // function. it keeps the client connected to
  // io.adafruit.com, and processes any incoming data.
  io.run();
  /*
  long tempValue = random(20, 30); // Simulate temperature data
  long pumpSpeedValue = random(0, 101); // Simulate pump speed data in percentage
  // save count to the 'counter' feed on Adafruit IO
  Serial.print("sending -> ");
  Serial.print(tempValue);
  Serial.print(" ");
  Serial.println(pumpSpeedValue);

  temp->save(tempValue);
  pumpSpeed->save(pumpSpeedValue);

  // Adafruit IO is rate limited for publishing, so a delay is required in
  // between feed->save events. In this example, we will wait three seconds
  delay(4000);
  */
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