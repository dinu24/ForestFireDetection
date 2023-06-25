#define BLYNK_PRINT Serial
// #define csPin 5
// #define resetPin 14
// #define irqPin 2


#include "credentials.h"
// #include <WiFi.h>
// #include <WiFiClient.h>
// #include <BlynkSimpleEsp32.h>
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SPI.h>
#include <LoRa.h>
const int csPin = 15;    // LoRa radio chip select
const int resetPin = 5;  // LoRa radio reset
const int irqPin = 4;
byte localAddress = 0xBB;  // address of this device
byte destination = 0xFF;

BlynkTimer timer;
String myString;  // complete message from arduino, which consistors of sensors data

int Smoke, Light, Rain, distance, fire_alarm, rain_alarm;
float Btemp, Temp, Hum, Pressure;

// String outgoing;
String incoming;
// destination to send to

// void myTimerEvent() {
//   Blynk.virtualWrite(V0, millis() / 1000);
// }

//====================================================================

void setup() {
  Serial.begin(115200);
  LoRa.setPins(csPin, resetPin, irqPin);  // set CS, reset, IRQ pin

  Blynk.begin(BLYNK_AUTH_TOKEN, WIFI_SSID, WIFI_PASSWORD);
  LoRa.setPins(csPin, resetPin, irqPin);
  if (!LoRa.begin(433E6)) {  // initialize ratio at 433 MHz
    Serial.println("LoRa init failed. Check your connections.");
  }
}
//====================================================================
//====================================================================

void loop() {
  Blynk.run();
  timer.run();  // Initiates BlynkTimeR
  Receive(LoRa.parsePacket());
  // onReceive(LoRa.parsePacket());
}
//====================================================================


void pushReadings() {
  Blynk.virtualWrite(V1, Smoke);
  Blynk.virtualWrite(V2, Light);
  Blynk.virtualWrite(V3, Rain);
  Blynk.virtualWrite(V4, Temp);
  Blynk.virtualWrite(V5, Hum);
  Blynk.virtualWrite(V6, Pressure);
  Blynk.virtualWrite(V7, Btemp);
  Blynk.virtualWrite(V8, distance);
}


void sendMessage(String outgoing) {
  LoRa.beginPacket();             // start packet
  LoRa.write(destination);        // add destination address
  LoRa.write(localAddress);       // add sender address
  LoRa.write(outgoing.length());  // add payload length
  LoRa.print(outgoing);           // add payload
  LoRa.endPacket();               // finish packet and send it
}


void Receive(int packetSize) {
  if (packetSize == 0) return;
  int recipient = LoRa.read();        // recipient address
  byte sender = LoRa.read();          // sender address
  byte incomingLength = LoRa.read();  // incoming msg length
  if (recipient == localAddress) {
    while (LoRa.available()) {
      incoming += (char)LoRa.read();
    }
    Serial.println("Incoming: ");
    Serial.println(incoming);
    // if (incomingLength != incoming.length()) {   // check length for error
    // Serial.println("LoRa_error: message length does not match length");
    // return;                             // skip rest of function

    Smoke = getValue(incoming, ',', 0).toInt();
    Serial.println("smoke: ");
    Serial.println(Smoke);
    Light = getValue(incoming, ',', 1).toInt();
    Serial.println("pH: ");
    Serial.println(Light);
    Rain = getValue(incoming, ',', 2).toInt();
    Serial.println("Rain: ");
    Serial.println(Rain);
    Temp = getValue(incoming, ',', 3).toFloat();
    Serial.println("Temp: ");
    Serial.println(Temp);
    Hum = getValue(incoming, ',', 4).toFloat();
    Serial.println("Hum: ");
    Serial.println(Hum);
    Pressure = getValue(incoming, ',', 5).toFloat();
    Serial.println("Pressure: ");
    Serial.println(Pressure);
    Btemp = getValue(incoming, ',', 6).toFloat();
    Serial.println("Btemp: ");
    Serial.println(Btemp);
    distance = getValue(incoming, ',', 7).toInt();
    Serial.println("distance: ");
    Serial.println(distance);
    fire_alarm = getValue(incoming, ',', 8).toInt();
    Serial.println("fire_alarm: ");
    Serial.println(fire_alarm);
    rain_alarm = getValue(incoming, ',', 9).toInt();
    Serial.println("rain_alarm: ");
    Serial.println(rain_alarm);
    if (fire_alarm == 1) {  //rain detected
      Blynk.logEvent("smoke", "SMOKE DETECTED");
    }
    if (rain_alarm == 1) {  //smoke detected
      Blynk.logEvent("rain", "RAIN DETECTED");
    }
    incoming = "";
  }
  pushReadings();
}

////////////////////////////////////////////TANK
BLYNK_WRITE(V11) {
  int pinValue = param.asInt();
  if (pinValue == HIGH) {
    sendMessage("A");
    delay(10);
  } else if (pinValue == LOW) {
    sendMessage("a");
    delay(10);
  }
}

//////////////////////////////////////////PUMP
BLYNK_WRITE(V12) {
  int pinValue = param.asInt();
  if (pinValue == HIGH) {
    sendMessage("B");
    delay(10);
  } else if (pinValue == LOW) {
    sendMessage("b");
    delay(10);
  }
}



String getValue(String data, char separator, int index) {
  int found = 0;
  int strIndex[] = { 0, -1 };
  int maxIndex = data.length() - 1;

  for (int i = 0; i <= maxIndex && found <= index; i++) {
    if (data.charAt(i) == separator || i == maxIndex) {
      found++;
      strIndex[0] = strIndex[1] + 1;
      strIndex[1] = (i == maxIndex) ? i + 1 : i;
    }
  }
  return found > index ? data.substring(strIndex[0], strIndex[1]) : "";
}
