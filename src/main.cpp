#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Arduino.h>

#define PRESSURE_PIN A0
#define ONE_WIRE_BUS 2

Adafruit_MPU6050 mpu;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature sensors(&oneWire);

void setup() {
  Serial.begin(9600);
  mpu.begin();
  sensors.begin();
}

void loop() {
  // Bosim sensori
  int pressureValue = analogRead(PRESSURE_PIN);
  
  // Harorat sensori
  sensors.requestTemperatures();
  float temperature = sensors.getTempCByIndex(0);
  
  // Harakat (MPU6050)
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  
  Serial.print("Bosim: ");
  Serial.print(pressureValue);
  Serial.print("\tHarorat: ");
  Serial.print(temperature);
  Serial.print(" °C\tEgilish: ");
  Serial.println(a.acceleration.x);

  delay(500);
}
