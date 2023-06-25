#define DHTPIN 4
#define DHTTYPE DHT11
#define PUMP_Relay 5
#define TANK_Relay 6
#define trigPin 3
#define echoPin 7
#define pushButton 2
#define pH_sensor A0
#define Smoke_sensor A1
#define Rain_sensor A2
#define CRITICAL_TEMP 40
// #define BMP280_ADDRESS 0x77  // <-- Check

// //LoRa pin config
#define csPin 10
#define resetPin 9
#define irqPin 8

#include <arduino-timer.h>
#include <DHT_U.h>
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_BMP280.h>

int pH = 0;
int Smoke = 0;
int Rain = 0;
float Temp = 0;
float Hum = 0;
float Btemp = 0;
float Pressure = 0;
int Altitude = 0;
int distance = 0;
bool pump_enable = false;

int fire_alarm = 0;
int rain_alarm = 0;
int tank_clear = 0;

byte localAddress = 0xFF;  // address of this device
byte destination = 0xBB;   // destination to send to

auto timer = timer_create_default();
String cdata;
Adafruit_BMP280 bmp;
DHT_Unified dht(DHTPIN, DHTTYPE);
sensors_event_t event;

//=========================================================================================================
void setup() {
  Serial.begin(115200);
  LoRa.setPins(csPin, resetPin, irqPin);
  dht.begin();  // Intiate DHT sensor

  if (!LoRa.begin(433E6)) {  // initialize ratio at 433 MHz
    Serial.println("LoRa init failed. Check your connections.");
  }
  if (!bmp.begin(0x76)) {
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
  }
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* Operating Mode. */
                  Adafruit_BMP280::SAMPLING_X2,     /* Temp. oversampling */
                  Adafruit_BMP280::SAMPLING_X16,    /* Pressure oversampling */
                  Adafruit_BMP280::FILTER_X16,      /* Filtering. */
                  Adafruit_BMP280::STANDBY_MS_500); /* Standby time. */

  pinMode(PUMP_Relay, OUTPUT);
  pinMode(TANK_Relay, OUTPUT);
  pinMode(pH_sensor, INPUT);
  pinMode(Smoke_sensor, INPUT);
  pinMode(Rain_sensor, INPUT);
  pinMode(trigPin, OUTPUT);  // Sets the trigPin as an Output
  pinMode(echoPin, INPUT);   // Sets the echoPin as an Input
  digitalWrite(PUMP_Relay, HIGH);
  digitalWrite(TANK_Relay, HIGH);
  attachInterrupt(digitalPinToInterrupt(pushButton), pushButtonISR, CHANGE);
  LoRa.onTxDone(onTxDone);
  timer.every(1500, packetSend);
  timer.every(3000, tankAndTempCheck);
}
//=========================================================================================================

//=========================================================================================================

void loop() {
  timer.tick();
  Smoke = analogRead(Smoke_sensor);
  Smoke = map(Smoke, 0, 1023, 0, 100);
  if (Smoke > 20) {  //Fire Warning
    int fire_alarm_temp = 1;
    if (fire_alarm != fire_alarm_temp) {
      fire_alarm = fire_alarm_temp;
      packetSend();
      Serial.print("Fire Detected!!!!-----");
      Serial.println(Smoke);
    }
  } else {
    fire_alarm = 0;
  }

  Rain = analogRead(Rain_sensor);
  Rain = map(Rain, 0, 1023, 100, 0);
  if (Rain > 20) {  //Rain Warning
    int rain_alarm_temp = 1;
    if (rain_alarm != rain_alarm_temp) {
      rain_alarm = rain_alarm_temp;
      packetSend();
      Serial.print("Rain Detected!!!!-----");
      Serial.println(Smoke);
    }
  } else {
    rain_alarm = 0;
  }
}
//=========================================================================================================

//=========================================================================================================
void pushButtonISR(){
  pump_enable = !pump_enable;
}
bool packetSend() {
  readSensors();
  cdata = String(Smoke) + "," + String(pH) + "," + String(Rain) + "," + String(Temp) + "," + String(Hum) + ","
          + String(Pressure) + "," + String(Btemp) + "," + String(distance) + "," + String(fire_alarm) + "," + String(rain_alarm);
  Serial.println(cdata);
  sendMessage(cdata);
}


void sendMessage(String outgoing) {
  LoRa.beginPacket();             // start packet
  LoRa.write(destination);        // add destination address
  LoRa.write(localAddress);       // add sender address
  LoRa.write(outgoing.length());  // add payload length
  LoRa.print(outgoing);           // add payload
  LoRa.endPacket();               // finish packet and send it
  Serial.println("Message send.");
  outgoing = "";
}


int onReceive(int packetSize) {
  if (packetSize == 0) return 0;
  int recipient = LoRa.read();        // recipient address
  byte sender = LoRa.read();          // sender address
  byte incomingLength = LoRa.read();  // incoming msg length

  /////////////////////////////////////////////////////////DATA from SERVER
  if (recipient == localAddress) {
    char incoming = (char)LoRa.read();
    Serial.println("Incoming: ");
    Serial.println(incoming);
    if (incoming == 'A') {  //TANK RELAY
      tank_clear = 1;
    } else if (incoming == 'a') {
      tank_clear = 0;
    }

    if (incoming == 'B') {  //PUMP RELAY
      digitalWrite(PUMP_Relay, LOW);
    } else if (incoming == 'b') {
      digitalWrite(PUMP_Relay, HIGH);
    }
    return 1;
  } else {
    Serial.println("Addresses doesn't match");
    return 0;
  }
}

void onTxDone() {
  Serial.println("TxDone");
}



void readSensors() {
  Smoke = analogRead(Smoke_sensor);
  Smoke = map(Smoke, 0, 1023, 0, 100);
  Serial.print("Smoke: ");
  Serial.println(Smoke);

  pH = analogRead(pH_sensor);
  pH = map(pH, 0, 1023, 14, 0);
  Serial.print("pH: ");
  Serial.println(pH);

  Rain = analogRead(Rain_sensor);
  Rain = map(Rain, 0, 1023, 100, 0);
  Serial.print("Rain: ");
  Serial.println(Rain);

  dht.temperature().getEvent(&event);  // Get temperature event and print its value.
  Temp = event.temperature;
  Serial.print("Temperature: ");
  Serial.println(Temp);
  if (isnan(Temp)) {  //DHT error handling
    Serial.println(F("Error reading temperature!"));
  }
  dht.humidity().getEvent(&event);
  Hum = event.relative_humidity;
  Serial.print("Humidity: ");
  Serial.println(Hum);
  if (isnan(Hum)) {  // DHT error handling
    Serial.println(F("Error reading humidity!"));
  }

  Btemp = bmp.readTemperature();
  Serial.print("Barometric Temperature: ");
  Serial.println(Btemp);

  Pressure = bmp.readPressure()/100;
  Serial.print("Pressure (hPa): ");
  Serial.println(Pressure);

  Altitude = bmp.readAltitude(1010.0);  //1008.78 Altitude +","+
  Serial.print("Altitude: ");
  Serial.println(Altitude);
}

boolean runEvery(unsigned long interval) {
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval) {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

void tankAndTempCheck() {
  digitalWrite(trigPin, LOW);  // Clears the trigPin
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);  // Sets the trigPin on HIGH state for 10 micro seconds
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  long duration = pulseIn(echoPin, HIGH);  // Reads the echoPin, returns the sound wave travel time in microseconds
  distance = duration * 0.034 / 2;         // Calculating the distance

  if (distance < 4 || tank_clear == 1) {
    digitalWrite(TANK_Relay, LOW);
    Serial.println("Valve open ");
  } else if (distance >= 4 || tank_clear == 0) {
    digitalWrite(TANK_Relay, HIGH);
  }
  if (Btemp > CRITICAL_TEMP || pump_enable == true) {
    digitalWrite(PUMP_Relay, LOW);
    Serial.println("Pump ON");
    if (runEvery(10000)) {
      digitalWrite(PUMP_Relay, HIGH);
      pump_enable == false;
      Serial.println("Pump OFF");
    }
  } else {
    digitalWrite(PUMP_Relay, HIGH);
  }

  // if (Btemp > CRITICAL_TEMP) {
  //   digitalWrite(PUMP_Relay, LOW);
  //   Serial.println("Pump ON");
  // } else {
  //   digitalWrite(PUMP_Relay, HIGH);
  // }
}
