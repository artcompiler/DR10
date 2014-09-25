/*************************************************** 
  This is an example for the Adafruit CC3000 Wifi Breakout & Shield

  Designed specifically to work with the Adafruit WiFi products:
  ----> https://www.adafruit.com/products/1469

  Adafruit invests time and resources providing this open source code, 
  please support Adafruit and open-source hardware by purchasing 
  products from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ****************************************************/
 
 /*
This example does a test of the TCP client capability:
  * Initialization
  * Optional: SSID scan
  * AP connection
  * DHCP printout
  * DNS lookup
  * Optional: Ping
  * Connect to website and print out webpage contents
  * Disconnect
SmartConfig is still beta and kind of works but is not fully vetted!
It might not work on all networks!
*/
#include <Adafruit_CC3000.h>
#include <ccspi.h>
#include <SPI.h>
#include <string.h>
#include "utility/debug.h"

// These are the interrupt and control pins
#define ADAFRUIT_CC3000_IRQ   3  // MUST be an interrupt pin!
// These can be any two pins
#define ADAFRUIT_CC3000_VBAT  5
#define ADAFRUIT_CC3000_CS    10
// Use hardware SPI for the remaining pins
// On an UNO, SCK = 13, MISO = 12, and MOSI = 11
Adafruit_CC3000 cc3000 = Adafruit_CC3000(ADAFRUIT_CC3000_CS, ADAFRUIT_CC3000_IRQ, ADAFRUIT_CC3000_VBAT,
                                         SPI_CLOCK_DIVIDER); // you can change this clock speed

// EDIT ME!
#define WLAN_SSID       "underdog"           // cannot be longer than 32 characters!
#define WLAN_PASS       ""
// Security can be WLAN_SEC_UNSEC, WLAN_SEC_WEP, WLAN_SEC_WPA or WLAN_SEC_WPA2
#define WLAN_SECURITY   WLAN_SEC_WPA2

#define IDLE_TIMEOUT_MS  3000      // Amount of time to wait (in milliseconds) with no data 
                                   // received before closing the connection.  If you know the server
                                   // you're accessing is quick to respond, you can reduce this value.

// What page to grab!
#define WEBSITE      "www.graffiticode.org"
#define WEBPAGE      "/graffiti/dr10/latest"

// BEGIN STEPPER DECLS

/*-----( Import needed libraries )-----*/
#include <AccelStepper.h>
/*-----( Declare Constants and Pin Numbers )-----*/
#define FULLSTEP 4
#define HALFSTEP 8
// motor pins
#define motorPin1  A0     // Blue   - 28BYJ48 pin 1
#define motorPin2  A1     // Pink   - 28BYJ48 pin 2
#define motorPin3  A2     // Yellow - 28BYJ48 pin 3
#define motorPin4  A3     // Orange - 28BYJ48 pin 4
                        // Red    - 28BYJ48 pin 5 (VCC)
                        
#define motorPin5  6    // Blue   - 28BYJ48 pin 1
#define motorPin6  7     // Pink   - 28BYJ48 pin 2
#define motorPin7  8    // Yellow - 28BYJ48 pin 3
#define motorPin8  9    // Orange - 28BYJ48 pin 4
                        // Red    - 28BYJ48 pin 5 (VCC)
/*-----( Declare objects )-----*/
// NOTE: The sequence 1-3-2-4 is required for proper sequencing of 28BYJ48
AccelStepper stepper1(FULLSTEP, motorPin1, motorPin3, motorPin2, motorPin4);
AccelStepper stepper2(FULLSTEP, motorPin5, motorPin7, motorPin6, motorPin8);

#define BACKWARD -1
#define FORWARD 1

// END STEPPER DECLS


// BEGIN SERVO DECLS

//#include <Servo.h> 
 
//Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
int pos = 60;    // variable to store the servo position 

// END SERVO DECLS

/**************************************************************************/
/*!
    @brief  Sets up the HW and the CC3000 module (called automatically
            on startup)
*/
/**************************************************************************/

uint32_t ip;
Adafruit_CC3000_Client www;

void setup(void)
{
  Serial.begin(115200);
  // INIT STEPPERS

  stepper1.setMaxSpeed(1500.0);
  stepper2.setMaxSpeed(1500.0);

//  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 



  Serial.println(F("Hello, CC3000!\n"));
  Serial.print("Free RAM: "); Serial.println(getFreeRam(), DEC);
  
  /* Initialise the module */

  Serial.println(F("\nInitializing..."));
  if (!cc3000.begin())
  {
    Serial.println(F("Couldn't begin()! Check your wiring?"));
    while(1);
  }
  
  Serial.print(F("\nAttempting to connect to ")); Serial.println(WLAN_SSID);
  if (!cc3000.connectToAP(WLAN_SSID, WLAN_PASS, WLAN_SECURITY)) {
    Serial.println(F("Failed!"));
    while(1);
  }
   
  Serial.println(F("Connected!"));
  
  /* Wait for DHCP to complete */
  Serial.println(F("Request DHCP"));
  while (!cc3000.checkDHCP())
  {
    delay(100); // ToDo: Insert a DHCP timeout!
  }  

  ip = 0;
  // Try looking up the website's IP address
  Serial.print(WEBSITE); Serial.print(F(" -> "));
  while (ip == 0) {
    if (! cc3000.getHostByName(WEBSITE, &ip)) {
      Serial.println(F("Couldn't resolve!"));
    }
    delay(500);
  }

  cc3000.printIPdotsRev(ip);
  
  // Optional: Do a ping test on the website
//  Serial.print(F("\n\rPinging ")); cc3000.printIPdotsRev(ip); Serial.print("...");  
//  int replies = cc3000.ping(ip, 5);
//  Serial.print(replies); Serial.println(F(" replies"));

  /* Try connecting to the website.
     Note: HTTP/1.1 protocol is used to keep the server from closing the connection before all data is read.
  */
  www = cc3000.connectTCP(ip, 80);
  if (www.connected()) {
    www.fastrprint(F("GET "));
    www.fastrprint(WEBPAGE);
    www.fastrprint(F(" HTTP/1.1\r\n"));
    www.fastrprint(F("Host: ")); www.fastrprint(WEBSITE); www.fastrprint(F("\r\n"));
    www.fastrprint(F("\r\n"));
    www.println();
  } else {
    Serial.println(F("Connection failed"));    
    return;
  }

  // Read data until either the connection is closed, or the idle timeout is reached.
//  unsigned long lastRead = millis();
//  while (www.connected() && (millis() - lastRead < IDLE_TIMEOUT_MS)) {
//    while (www.available()) {
//      char c = www.read();
//      Serial.print(c);
//      lastRead = millis();
//    }
//  }
//  www.close();

  // You need to make sure to clean up after yourself or the CC3000 can freak out
  // the next time your try to connect ...
//  Serial.println(F("\n\nDisconnecting"));
//  cc3000.disconnect();

}

long hex2int(char *a, int len)
{
    int i;
    long val = 0;
    for(i=0;i<len;i++) {
       if(a[i] <= 57)
        val += (a[i]-'0')*(1<<(4*(len-1-i)));
       else
        val += (a[i]-'A'+10)*(1<<(4*(len-1-i)));
    }
    return val;
}

const float CALIBRATE = 2.910;  // The original DR10's calibration

const int START = 0x01;
int state = START;
char *buf = new char[10];
void loop() {
  if(www.available() )       // if data is available to read
  {
    long v1, v2;
    buf[4] = 0;
    char c = www.read();
    switch (state) {
    case START:
      switch (c) {
      case 'S':
        switch (www.read()) {
        case 'S':
          //Serial.print("\nCALIBRATE ");
          //Serial.print(CALIBRATE);
          buf[0] = www.read();
          buf[1] = www.read();
          buf[2] = www.read();
          buf[3] = www.read();
          v1 = hex2int(buf, 4);
          buf[0] = www.read();
          buf[1] = www.read();
          buf[2] = www.read();
          buf[3] = www.read();
          v2 = hex2int(buf, 4);
          Serial.print("\nSS ");
          Serial.print(v1);
          Serial.print(" ");
          Serial.print(v2);
          step(v1*CALIBRATE, v2*CALIBRATE);
          break;
        }
        break;
      }
      break;
    }
  }
}

// FIXME
void penUp() {
  /*
  for(; pos < 60; pos += 1)  // goes from 0 degrees to 180 degrees 
  {                                 // in steps of 1 degree 
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
                      // waits 15ms for the servo to reach the position 
  } 
  pos = 60;
  myservo.write(pos);              // tell servo to go to position in variable 'pos' 
  */
}

// FIXME
void penDown() {
  /*
  for(; pos > 0; pos -= 1)  // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
                      // waits 15ms for the servo to reach the position 
  }
  pos = 0;
  myservo.write(pos);              // tell servo to go to position in variable 'pos' 
  */
}

#define SPEED 500

void step(long lsteps, long rsteps) { // 2000, 1000
  int dirL = (lsteps > 0) ? BACKWARD : FORWARD;
  int dirR = (rsteps > 0) ? BACKWARD : FORWARD;
  lsteps = abs(lsteps);
  rsteps = abs(rsteps);
  if (lsteps >= rsteps) {
    double offset = 0;
    if (rsteps > 0) {
      double delta = double(lsteps - rsteps) / rsteps;  // 3
      for ( ; rsteps > 0; ) {
        offset += delta;
        stepOne(dirL, dirR, SPEED);
        lsteps--;
        rsteps--;
        for(; offset >= 1; offset--) {  // 3 * 0 | 3 * 1
          stepOneLeft(dirL, SPEED);
          lsteps--;
        }
      }
    }
    for(; lsteps > 0; lsteps--) {
      stepOneLeft(dirL, SPEED);
    }
  } else {
    double offset = 0;
    if (lsteps > 0) {
      double delta = double(rsteps - lsteps) / lsteps;
      for ( ; lsteps > 0; ) {
        offset += delta;
        stepOne(dirL, dirR, SPEED);
        lsteps--;
        rsteps--;
        for(; offset >= 1; offset--) {  // 3 * 0 | 3 * 1
          stepOneRight(dirR, SPEED);
          rsteps--;
        }
      }
    }
    for(; rsteps > 0; rsteps--) {
      stepOneRight(dirR, SPEED);
    }
  }
}

#define DELAY 2000  // This appears to be the minimum delay (3ms) to allow the motors to complete one step

void stepOne(int dirL, int dirR, float speed) {
//  Serial.print("\nstepOne");
  stepper1.move(dirR*1);
  stepper1.setSpeed(speed);
  stepper1.runSpeedToPosition();
  delayMicroseconds(DELAY); 
  stepper2.move(dirL*-1);
  stepper2.setSpeed(speed);
  stepper2.runSpeedToPosition();
  delayMicroseconds(DELAY); 
}

void stepOneLeft(int dirL, float speed) {
//  Serial.print("\nstepOneLeft");
  stepper2.move(dirL*-1);
  stepper2.setSpeed(speed);
  stepper2.runSpeedToPosition();
  delayMicroseconds(DELAY); 
}

void stepOneRight(int dirR, float speed) {
//  Serial.print("\nstepOneRight");
  stepper1.move(dirR);
  stepper1.setSpeed(speed);
  stepper1.runSpeedToPosition();
  delayMicroseconds(DELAY); 
}


