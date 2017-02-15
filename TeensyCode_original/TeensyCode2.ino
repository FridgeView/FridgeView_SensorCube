//#include <NewSoftSerial.h>

//#include <Adafruit_ATParser.h>
#include <Adafruit_BLE.h>
//#include <Adafruit_BLEBattery.h>
//#include <Adafruit_BLEEddystone.h>
//#include <Adafruit_BLEGatt.h>
//#include <Adafruit_BLEMIDI.h>
//#include <Adafruit_BluefruitLE_SPI.h>
#include <Adafruit_BluefruitLE_UART.h>

/*************************************************** 
  This is an example for the HTU21D-F Humidity & Temp Sensor

  Designed specifically to work with the HTU21D-F sensor from Adafruit
  ----> https://www.adafruit.com/products/1899

  These displays use I2C to communicate, 2 pins are required to  
  interface
 ****************************************************/

 /******* PINS FOR THE BLUEFRUIT BLUETOOTH LE UART FRIEND ******
  *  THIS EXPLAINS ON HOW WE WILL WIRE IT!!!
TXO - This is the UART Transmit pin out of the breakout (Bluefruit LE --> MCU), it's at 3.3V logic level.
RXI - This is the UART Receive pin into the breakout (MCU --> Bluefruit LE). This has a logic level shifter on it, you can use 3-5V logic.


CTS - Clear to Send hardware flow control pin into the the breakou (MCU --> Bluefruit LE). 
Use this pin to tell the Bluefruit that it can send data back to the microcontroller over the TXO pin. 
This pin is pulled high by default and must be set to ground in order to enable data transfer out! If you do not need hardware flow control, 
tie this pin to ground it is a level shifted pin, you can use 3-5V logic
^ THIS ONE TO GROUND

RTS - Read to Send flow control pin out of the module (Bluefruit LE --> MCU). This pin will be low when its fine to send data to the Bluefruit. 
In general, at 9600 baud we haven't seen a need for this pin, but you can watch it for full flow control! This pin is 3.3V out
Other Pins
^ THIS ONE TO GROUND

MOD: Mode Selection. The Bluefruit has two modes, Command and Data. 
You can keep this pin disconnected, and use the slide switch to select the mode. 
Or, you can control the mode by setting this pin voltage, it will override the switch setting!  
High = Command Mode, Low = UART/DATA mode. This pin is level shifted, you can use 3-5V logic
^ THIS ONE TO GROUND

DFU: Setting this pin low when you power the device up will force the 
Bluefruit LE module to enter a special firmware update mode to update the firmware over the air.  
Once the device is powered up, this pin can also be used to perform a factory reset.  
Wire the pin to GND for >5s until the two LEDs start to blink, then release the pin 
(set it to 5V or logic high) and a factory reset will be performed.
^ DONT USE THIS ONE
 */

#include <Wire.h>
#include <SoftwareSerial.h>
#include "BluefruitConfig.h"


// Create the bluefruit object, either software serial...uncomment these lines

//SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

//Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN, BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);

// Connect Vin to 3-5VDC
// Connect GND to ground
// Connect SCL to I2C clock pin (A5 on UNO) (D0 on teensy)
// Connect SDA to I2C data pin (A4 on UNO) (D1 on teensy)

SoftwareSerial Genotronex(7, 8); // RX, TX


float getTemperature();
float getHumidity();

void setup() {

  Genotronex.begin(9600);
  Genotronex.println("Sent 12345");



  //if ( !ble.begin(VERBOSE_MODE) )
  //{
  //  error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  //}
  //Serial.println( F("OK!") );
  /* Disable command echo from Bluefruit */
 // ble.echo(false);

  //Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  //ble.info();
  
  //ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  //while (! ble.isConnected()) {
   //   delay(500);
  //}


  
  //9200
  //115200
  //Serial.begin(115200);
  //Serial.println("HTU21D-F test");

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

 //printCount();
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

  //char n, inputs[BUFSIZE+1];

  float temp = getTemperature();
  float hum = getHumidity();

  Genotronex.print("Temp: ");Genotronex.print(temp);
  Genotronex.print("\t\tHum: ");Genotronex.println(hum);

//  if(Genotronex.available()>0) {
//    
//    byte som = Genotronex.read();
//    
//  }

  //Serial.print("Temp: ");Serial.print(temp);
  //Serial.print("\t\tHum: ");Serial.println(hum);

  //ble.print(temp);
  //ble.print(hum);

//  if (Serial.available())
//  {
//    n = Serial.readBytes(inputs, BUFSIZE);
//    inputs[n] = 0;
//    // Send characters to Bluefruit
//    Serial.print("Sending: ");
//    Serial.println(inputs);
//
//    // Send input data to host via Bluefruit
//    ble.print(inputs);
//  }

  // Echo received data
//  while ( ble.available() )
//  {
//    int c = ble.read();
//
//    Serial.print((char)c);
//
//    // Hex output too, helps w/debugging!
//    Serial.print(" [0x");
//    if (c <= 0xF) Serial.print(F("0"));
//    Serial.print(c, HEX);
//    Serial.print("] ");
//  }
  
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

