/*********************************************************************
 This is a test of setting up a bluetooth LE service with 
 characteristics representing the output of multiple 9dof sensors.

 *********************************************************************
 
 This is an example for our nRF51822 based Bluefruit LE modules

 Pick one up today in the adafruit shop!

 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include <Arduino.h>
#include <SPI.h>
#if not defined (_VARIANT_ARDUINO_DUE_X_) && not defined (_VARIANT_ARDUINO_ZERO_)
  #include <SoftwareSerial.h>
#endif

#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"

#include "BluefruitConfig.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;        // will store last 'click'

// constants won't change :
const long interval = 150;           // interval at which to poll (milliseconds)

const int MOTOR_1_PIN = 10;
const int MOTOR_2_PIN = 11;

bool debug = false;
/*=========================================================================
    APPLICATION SETTINGS

    FACTORYRESET_ENABLE     Perform a factory reset when running this sketch
   
                            Enabling this will put your Bluefruit LE module
                            in a 'known good' state and clear any config
                            data set in previous sketches or projects, so
                            running this at least once is a good idea.
   
                            When deploying your project, however, you will
                            want to disable factory reset by setting this
                            value to 0.  If you are making changes to your
                            Bluefruit LE device via AT commands, and those
                            changes aren't persisting across resets, this
                            is the reason why.  Factory reset will erase
                            the non-volatile memory where config data is
                            stored, setting it back to factory default
                            values.
       
                            Some sketches that require you to bond to a
                            central device (HID mouse, keyboard, etc.)
                            won't work at all with this feature enabled
                            since the factory reset will clear all of the
                            bonding data stored on the chip, meaning the
                            central device won't be able to reconnect.
    -----------------------------------------------------------------------*/
    #define FACTORYRESET_ENABLE      1
/*=========================================================================*/


// Create the bluefruit object, either software serial...uncomment these lines
/*
SoftwareSerial bluefruitSS = SoftwareSerial(BLUEFRUIT_SWUART_TXD_PIN, BLUEFRUIT_SWUART_RXD_PIN);

Adafruit_BluefruitLE_UART ble(bluefruitSS, BLUEFRUIT_UART_MODE_PIN,
                      BLUEFRUIT_UART_CTS_PIN, BLUEFRUIT_UART_RTS_PIN);
*/

/* ...or hardware serial, which does not need the RTS/CTS pins. Uncomment this line */
// Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

/* ...hardware SPI, using SCK/MOSI/MISO hardware SPI pins and then user selected CS/IRQ/RST */
Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_CS, BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);

/* ...software SPI, using SCK/MOSI/MISO user-defined SPI pins and then user selected CS/IRQ/RST */
//Adafruit_BluefruitLE_SPI ble(BLUEFRUIT_SPI_SCK, BLUEFRUIT_SPI_MISO,
//                             BLUEFRUIT_SPI_MOSI, BLUEFRUIT_SPI_CS,
//                             BLUEFRUIT_SPI_IRQ, BLUEFRUIT_SPI_RST);


// A small helper
void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

bool isSetup = false;

/**************************************************************************/
/*!
    @brief  Sets up the HW an the BLE module (this function is called
            automatically on startup)
*/
/**************************************************************************/
void setup(void)
{
  while (!Serial);  // required for Flora & Micro
  delay(500);

  pinMode(MOTOR_1_PIN, OUTPUT);
  pinMode(MOTOR_2_PIN, OUTPUT);

  Serial.begin(115200);
  Serial.println(F("Adafruit Bluefruit AT Command Example"));
  Serial.println(F("-------------------------------------"));

  /* Initialise the module */
  Serial.print(F("Initialising the Bluefruit LE module: "));

  if ( !ble.begin(VERBOSE_MODE) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }
  Serial.println( F("OK!") );

  if ( FACTORYRESET_ENABLE )
  {
    /* Perform a factory reset to make sure everything is in a known state */
    Serial.println(F("Performing a factory reset: "));
    if ( ! ble.factoryReset() ){
      error(F("Couldn't factory reset"));
    }
  }  

  /* Disable command echo from Bluefruit */
  ble.echo(false);

  Serial.println("Requesting Bluefruit info:");
  /* Print Bluefruit information */
  ble.info();
  ble.println("AT+GATTCLEAR");
  ble.waitForOK();
  ble.println("AT+GATTADDSERVICE=UUID128=00-11-00-11-00-00-00-00-00-00-00-00-00-00-00-00");
  ble.waitForOK();
  ble.println("AT+GATTADDCHAR=UUID=0x1001,PROPERTIES=0x10,MIN_LEN=1,MAX_LEN=2,VALUE=11");
  ble.waitForOK();
  ble.println("AT+GATTADDCHAR=UUID=0x1002,PROPERTIES=0x10,MIN_LEN=1,MAX_LEN=2,VALUE=12");
  ble.waitForOK();
  ble.println("AT+GATTADDCHAR=UUID=0x1003,PROPERTIES=0x10,MIN_LEN=1,MAX_LEN=2,VALUE=13");
  ble.waitForOK();
  ble.println("AT+GATTADDCHAR=UUID=0x1004,PROPERTIES=0x08,MIN_LEN=1,MAX_LEN=2,VALUE=0");
  ble.waitForOK();
  ble.println("AT+GATTADDCHAR=UUID=0x1005,PROPERTIES=0x08,MIN_LEN=1,MAX_LEN=2,VALUE=0");
//  ble.waitForOK();
//  ble.println("AT+GATTADDCHAR=UUID=0x2001,PROPERTIES=0x10,MIN_LEN=1,MAX_LEN=2,VALUE=21");
//  ble.waitForOK();
//  ble.println("AT+GATTADDCHAR=UUID=0x2002,PROPERTIES=0x10,MIN_LEN=1,MAX_LEN=2,VALUE=22");
//  ble.waitForOK();
//  ble.println("AT+GATTADDCHAR=UUID=0x2003,PROPERTIES=0x10,MIN_LEN=1,MAX_LEN=2,VALUE=23");
//  ble.waitForOK();
//  ble.println("AT+GATTADDCHAR=UUID=0x2004,PROPERTIES=0x08,MIN_LEN=1,MAX_LEN=2,VALUE=24");
//  ble.waitForOK();
//  ble.println("AT+GATTADDCHAR=UUID=0x2005,PROPERTIES=0x08,MIN_LEN=1,MAX_LEN=2,VALUE=25");
//  ble.waitForOK();
//  ble.println("AT+GATTADDCHAR=UUID=0x3001,PROPERTIES=0x10,MIN_LEN=1,MAX_LEN=2,VALUE=31");
//  ble.waitForOK();
//  ble.println("AT+GATTADDCHAR=UUID=0x3002,PROPERTIES=0x10,MIN_LEN=1,MAX_LEN=2,VALUE=32");
//  ble.waitForOK();
//  ble.println("AT+GATTADDCHAR=UUID=0x3003,PROPERTIES=0x10,MIN_LEN=1,MAX_LEN=2,VALUE=33");
//  ble.waitForOK();
//  ble.println("AT+GATTADDCHAR=UUID=0x3004,PROPERTIES=0x08,MIN_LEN=1,MAX_LEN=2,VALUE=34");
//  ble.waitForOK();
//  ble.println("AT+GATTADDCHAR=UUID=0x3005,PROPERTIES=0x08,MIN_LEN=1,MAX_LEN=2,VALUE=35");
  ble.waitForOK();
  ble.reset();
  ble.waitForOK();
  ble.println("AT+GATTLIST");
  ble.waitForOK();

  /* Enable auto-gain */
  mag.enableAutoRange(true);

  /* Initialise the sensor */
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  
  /* Display some basic information on this sensor */
  displaySensorDetails();

}

int x1 = 0;
int Y1 = 0;
int z1 = 0;
int x2 = 0;
int Y2 = 0;
int z2 = 0;
int x3 = 0;
int Y3 = 0;
int z3 = 0;

/**************************************************************************/
/*!
    @brief  Constantly poll for new command or response data
*/
/**************************************************************************/
void loop(void)
{
  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {
    // save the last time you blinked the LED
    previousMillis = currentMillis;
 
    sendEvent(); 

    ble.println("AT+GATTCHAR=4");
    ble.waitForOK();
    char command[BUFSIZE+1];
    getUserInput(command, BUFSIZE);
    String cmdString = String(command);
    int splitAt = cmdString.indexOf("-");
    if (splitAt == -1) {
      splitAt = cmdString.indexOf("x");
    }
    
    String firstValue = cmdString.substring(splitAt-2, splitAt+3);
//    if (debug) {
      Serial.print(splitAt); Serial.print(" "); Serial.println(firstValue);
//    }
    if (firstValue == "00-00") {
      stopMotor(MOTOR_1_PIN);
      Serial.println("off");
    } else {
      runMotor(MOTOR_1_PIN);
      Serial.println("on");
    }
  }  
}

void sendEvent() {
  /* Get a new sensor event */ 
  sensors_event_t event; 
  mag.getEvent(&event);

  int x = normalizeToInt(event.magnetic.x);
  int y = normalizeToInt(event.magnetic.y);
  int z = normalizeToInt(event.magnetic.z);
  /* Display the results (magnetic vector values are in micro-Tesla (uT)) */
  if (x > 45.0) {
    runMotor(MOTOR_1_PIN);
  } else {
    stopMotor(MOTOR_1_PIN);  
  }
  
  String str1x = "AT+GATTCHAR=1," + String(x, DEC);
  String str2x = "AT+GATTCHAR=2," + String(y, DEC);
  String str3x = "AT+GATTCHAR=3," + String(z, DEC);
  ble.println(str1x);
  ble.println(str2x);
  ble.println(str3x);
}

void runMotor(int motorNumber) {
  digitalWrite(motorNumber, HIGH); 
}

void stopMotor(int motorNumber) {
  digitalWrite(motorNumber, LOW); 
}

int normalizeToInt(float in) {
  int out = (int) in;
  if (out < 0) {
    out = 360 + out;
  }

  return out;
}

void displaySensorDetails(void)
{
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void getUserInput(char buffer[], uint8_t maxSize)
{
  memset(buffer, 0, maxSize);
  while( ble.available() == 0 ) {
    delay(1);
  }

  uint8_t count=0;

  do
  {
    count += ble.readBytes(buffer+count, maxSize);
    delay(2);
  } while( (count < maxSize) && !(ble.available() == 0) );
}

