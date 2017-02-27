#include <Adafruit_BLE.h>
#include <Adafruit_BluefruitLE_UART.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include "BluefruitConfig.h"
#include <String.h>

// Connect Vin to 3-5VDC
// Connect GND to ground
// Connect SCL to I2C clock pin (A5 on UNO) (D0 on teensy)
// Connect SDA to I2C data pin (A4 on UNO) (D1 on teensy)

SoftwareSerial Genotronex(7, 8); // RX, TXs

float getTemperature();
float getHumidity();
String sensorCubeID = "GfrGUXmZY5";

void setup() {

  Genotronex.begin(9600);
//  Genotronex.println("Sent 12345");
  Wire.begin();
  // software reset
  Wire.beginTransmission(0x40);
  Wire.write(0xFE);
  Wire.endTransmission();
  delay(500);
  // begin transmission
  Wire.beginTransmission(0x40);
  Wire.write(0xE7);
  Wire.endTransmission();
  Wire.requestFrom(0x40, 1);
  delay(500);
}

void printCount() {
  int i = 0;
  while(true) {
    if (i % 100 == 0) {
      Genotronex.println(i);
    }
    i++;
  }
}

void loop() {

  float temp = getTemperature();
  float hum = getHumidity();

  int som = Genotronex.read();
    if(som != '\n'){
      Genotronex.print("{'cubeID': '");Genotronex.print(sensorCubeID);
      Genotronex.print("','Temp': '");Genotronex.print(String(temp));
      Genotronex.print("','Hum': '");Genotronex.print(String(hum));
      Genotronex.print("','Battery': '23");Genotronex.println("'}");
    }  
  delay(1000);
}

// returns the temperature in degree celcius
float getTemperature() {
  Wire.beginTransmission(0x40);
  Wire.write(0xE3);
  Wire.endTransmission();
  delay(50);

  Wire.requestFrom(0x40, 3);
  while(!Wire.available()) {}

  uint16_t t = Wire.read();
  t <<= 8;

  t |= Wire.read();
  uint8_t crc = Wire.read(); // ignored??

  // equation inside the documentation of the sensor
  float temp = t;
  temp *= 175.72;
  temp /= 65536;
  temp -= 46.85;

  return temp;
}


// returns the humidity in percentage
float getHumidity() {
  Wire.beginTransmission(0x40);
  Wire.write(0xE5);
  Wire.endTransmission();
  delay(50);

  Wire.requestFrom(0x40, 3);
  while(!Wire.available()) {}

  uint16_t h = Wire.read();
  h <<= 8;

  h |= Wire.read();
  uint8_t crc = Wire.read(); // ignored??

  // equation inside the documentation of the sensor
  float hum = h;
  hum *= 125;
  hum /= 65536;
  hum -= 6;

  return hum;
}

