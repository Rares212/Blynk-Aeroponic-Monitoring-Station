/* TO DO
    - Rewire liquid level sensor to D2 (Maybe??)
    - Wire ADS1115: SDA to SDA(21), SCL to SCL(22), ADDR to GND
    - Wire temp sensor to D2(IO25)
*/

#include <EEPROM.h>
#include "DHT.h"
#include <SHT1x-ESP.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include "DFRobot_ESP_PH_WITH_ADC.h"
#include "DFRobot_ESP_EC.h"
#include "DFRobot_PH.h"
#include "DFRobot_EC10.h"
#include "OneWire.h"
#include "DallasTemperature.h"
#include <Wire.h>
#include <Adafruit_ADS1015.h>

#define REF_VOLTAGE_MV 3300.0
#define ADC_MAX_VOLTAGE 4096.0
#define MAX_SERIAL_LENGTH 64
//#define BLYNK_PRINT Serial
#define CAL_MODE false

// Modifica aici cu WiFi-ul + auth tokenul tau de la blynk
char authToken[] = "YS-o_ds7KW1FcadGiFwzrd7ALxkpT3i_";
char ssid[] = "Livebox-3758";
char pass[] = "qwerty123";

BlynkTimer updateTimer;

// Seteaza pinii
const int liquidLevelPin = 17;
const int dhtPin = 3;
const int shtDataPin = 16;
const int shtSckPin = 4;
const int lightPin = A0;
const int tempPin = D2;
const int ecPin = 1;
const int phPin = 0;

// Variabile pentru valorile de la senzori
float shtHumidity, shtTemp;
float dhtHumidity, dhtTemp;
float waterTemp = 25.0;

float lightVoltage;
float lightLux;
const float lightToLux = 0.05376f;

float phVoltage;
float phValue;

float ecVoltage;
float ecValue;

int liquidLevel;

DHT dht(dhtPin, DHT22);

SHT1x sht(shtDataPin, shtSckPin);

Adafruit_ADS1115 ads;

OneWire oneWire(tempPin);
DallasTemperature tempSensor(&oneWire);

DFRobot_EC10 ec;
DFRobot_PH ph;

// Seteaza timpul dintre masuratori succesive in milisecunde
const unsigned int measurementTimeMs = 15000;

// Citeste valoarea in mV de la un pin ADC
float readMiliVolts(int pin, int nSamples = 10) {
  int adc;
  long adcSum = 0;
  for (int i = 0; i < nSamples; i++) {
    adc = analogRead(pin);
    adcSum += adc;
  }
  adc = adcSum / nSamples;
  return adc / 4095.0f * REF_VOLTAGE_MV;
}

// Citeste valorile de la senzori
void readSensorData() {
  lightVoltage = readMiliVolts(lightPin);
  lightLux = lightVoltage * lightToLux;

  liquidLevel = digitalRead(liquidLevelPin);

  tempSensor.requestTemperatures();
  waterTemp = tempSensor.getTempCByIndex(0);
  yield();

  dhtHumidity = dht.readHumidity();
  dhtTemp = dht.readTemperature();

  shtHumidity = sht.readHumidity();
  shtTemp = sht.readTemperatureC();
  yield();

  // Face conversie la mV
  ecVoltage = ads.readADC_SingleEnded(ecPin) * ADC_MAX_VOLTAGE / 32767.0;
  phVoltage = ads.readADC_SingleEnded(phPin) * ADC_MAX_VOLTAGE / 32767.0;

  ecValue = ec.readEC(ecVoltage, shtTemp);
  phValue = ph.readPH(phVoltage, shtTemp);
}

void sendSensorData() {
  Blynk.virtualWrite(V0, shtTemp);
  Blynk.virtualWrite(V1, shtHumidity);
  Blynk.virtualWrite(V2, (bool)liquidLevel);
  Blynk.virtualWrite(V3, lightLux);
  Blynk.virtualWrite(V4, phValue);
  Blynk.virtualWrite(V5, ecValue);
  Blynk.virtualWrite(V6, waterTemp);
}

void printSensorData() {
  Serial.print("Light(lux): ");
  Serial.println(lightLux);
  Serial.print("Temp(C): ");
  Serial.println(shtTemp);
  Serial.print("Humidity(%): ");
  Serial.println(shtHumidity);
  Serial.print("Liquid level(bool): ");
  Serial.println(liquidLevel);
  Serial.print("Ec value(ms/cm): ");
  Serial.println(ecValue);
  Serial.print("Ph value: ");
  Serial.println(phValue);
  Serial.print("Ec voltage(mV): ");
  Serial.println(ecVoltage);
  Serial.print("Ph voltage(mV): ");
  Serial.println(phVoltage);
  Serial.print('\n');
}

void timerEvent() {
  readSensorData();
  if (!CAL_MODE) {
    sendSensorData();
    printSensorData();
  }
}

void setup() {
  Serial.begin(9600);
  EEPROM.begin(32);
  
  Blynk.begin(authToken, ssid, pass);
  updateTimer.setInterval(measurementTimeMs, timerEvent);
  
  ec.begin();
  ph.begin();
  tempSensor.begin();

  ads.setGain(GAIN_ONE);
  ads.begin();

  pinMode(liquidLevelPin, INPUT);
  readSensorData();
}

char incomingChar = 0;
char cmdSerial[MAX_SERIAL_LENGTH];
int charIndex = 0;

void loop() {
  if (!CAL_MODE) {
    Blynk.run();
    updateTimer.run();
  }
  else {
    // Nebunie ca sa mearga calibrarea
    while (Serial.available() > 0) {
      incomingChar = Serial.read();
      if (incomingChar == '\n' || incomingChar == '\r') {
        Serial.print(cmdSerial);
        ph.calibration(phVoltage, shtTemp, cmdSerial);
        ec.calibration(ecVoltage, shtTemp, cmdSerial);
        charIndex = 0;
        cmdSerial[0] = '\0';
      }
      else {
        if (charIndex < MAX_SERIAL_LENGTH) {
          cmdSerial[charIndex++] = incomingChar;
          cmdSerial[charIndex] = '\0';
        }
      }
    }
  }
}
