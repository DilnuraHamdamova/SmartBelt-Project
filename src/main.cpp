#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>


#define SDA_PIN 22
#define SCL_PIN 20
#define BUTTON_PIN 32
#define BUZZER_PIN 27


BLECharacteristic* pCharacteristic;
Adafruit_MPU6050 mpu;
bool buzzerOn = false; // buzzerni  holati 
bool lastButtonState = HIGH;

void i2cScan() {
  Serial.println(F("\nI2C scan:"));
  byte count = 0;
  for (byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.printf(" - 0x%02X\n", addr);
      count++;
    }
  }
  if (count == 0) Serial.println(" (no devices found)");
  Serial.println();
}

void setup() {
  Serial.begin(115200);
  BLEDevice::init("GrijaBelt");
  BLEServer* pServer = BLEDevice::createServer();
  BLEService* pService = pServer->createService(BLEUUID((uint16_t)0x180D));
  pCharacteristic =  pService->createCharacteristic(
    BLEUUID((uint16_t) 0x2A37),
    BLECharacteristic::PROPERTY_NOTIFY
  );
  pCharacteristic->addDescriptor(new BLE2902());
  pService-> start();
  pServer->getAdvertising()->start();
  Serial.println("BLE started. Scan for 'Girja belt'");

  delay(300);

  // Start I2C on custom pins
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.printf("I2C started on SDA=%d, SCL=%d\n", SDA_PIN, SCL_PIN);

  i2cScan(); // helpful the first time

  // Try to initialize MPU6050 at default 0x68, then 0x69
  if (!mpu.begin(0x68, &Wire) && !mpu.begin(0x69, &Wire)) {
    Serial.println("MPU6050 not found. Check wiring and address.");
    while (true) delay(1000);
  }
  Serial.println("MPU6050 init OK.");

  // Configure ranges / filters (tweak as needed)
  mpu.setAccelerometerRange(MPU6050_RANGE_4_G);   // 2/4/8/16G
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);        // 250/500/1000/2000 dps
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);     // 5/10/21/44/94/184 Hz


  pinMode(BUTTON_PIN,INPUT_PULLUP);
  pinMode(BUZZER_PIN,OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);
  Serial.println("Setup complete.\n");
}

void loop() {
  bool buttonState = digitalRead(BUTTON_PIN);
  if(buttonState == LOW && lastButtonState == HIGH){
    buzzerOn = !buzzerOn;
    digitalWrite(BUZZER_PIN,buzzerOn ? HIGH : LOW);
  }
  lastButtonState = buttonState;
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  char data[100];
sprintf(data, "AX:%.2f AY:%.2f AZ:%.2f", 
        a.acceleration.x, a.acceleration.y, a.acceleration.z);

  pCharacteristic->setValue((uint8_t*)data, strlen(data));
  pCharacteristic->notify();
  Serial.printf("Accel: X=%.2f Y=%.2f Z=%.2f | Buzzer=%s\n",
  a.acceleration.x,a.acceleration.y, a.acceleration.z,
  buzzerOn ? "ON" : "OFF");
  
  delay(1000);

  // Simple orientation estimate (degrees) from accel
  float roll  = atan2(a.acceleration.y, a.acceleration.z) * 180.0 / PI;
  float pitch = atan(-a.acceleration.x /
                    sqrt(a.acceleration.y * a.acceleration.y +
                         a.acceleration.z * a.acceleration.z)) * 180.0 / PI;

  Serial.printf("Accel (m/s^2): X=%6.2f Y=%6.2f Z=%6.2f | ",
                a.acceleration.x, a.acceleration.y, a.acceleration.z);
  Serial.printf("Gyro (deg/s): X=%6.2f Y=%6.2f Z=%6.2f | ",
                g.gyro.x * 180.0 / PI, g.gyro.y * 180.0 / PI, g.gyro.z * 180.0 / PI);
  Serial.printf("Pitch=%6.2f Roll=%6.2f | Temp=%.2f C\n", pitch, roll, temp.temperature);

  delay(50); // ~20 Hz output
}
