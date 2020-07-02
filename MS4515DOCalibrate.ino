// Code to calibrate the MS4515DO pressure sensor
// using the Perfect Prime digital Manometer as a standard
#include <Wire.h> 
//#include <Adafruit_ADS1015.h>
#include <Streaming.h>

// code for the MS4515DO pressure sensor

#define MS4515DO_ADDR 0x46

// these will go into M4515DO.h
#ifndef __MS4515DO_H__
#define __MS4515DO_H__

  #if ARDUINO >= 100
  #include "Arduino.h"
  #else
  #include "WProgram.h"
  #endif
  
  #include <Wire.h>
  
  class MS4515DO {
    protected:
    // Instance-specific properties
    uint8_t m_i2cAddress;      ///< the I2C address
  public:
    MS4515DO(uint8_t i2cAddress = MS4515DO_ADDR);
    void begin(void);
    uint16_t readPressure();
  
  };
  
#endif

// these will go into M4515DO.cpp
MS4515DO::MS4515DO(uint8_t i2cAddress) {
  m_i2cAddress = i2cAddress;
}

void MS4515DO::begin() {
  Wire.begin(); // i2c bus master
  delay(10);
}
uint16_t MS4515DO::readPressure() {
  uint8_t  data[9];
  const int numBytes = 2;
  bool done = false;
  
  while ( !done ) {
    Wire.requestFrom( m_i2cAddress, numBytes );
    //while ( Wire.available() < numBytes ) {
    //}
    for( int i = 0; i< numBytes; i++ ) {
      while( ! Wire.available() ) {};
      data[i] = Wire.read();
    }
  
    // debug info
//#define DEBUG
    #ifdef DEBUG
    Serial << "data";
    for ( int i = 0; i < numBytes; i++ ) {
      Serial <<"  " << _HEX(data[i]); 
    }
    Serial << endl;
    #endif
    /*
     * Status Bits
     * (2 MSB of Output Data Packet) 
     * 00       Normal Operation. Good Data Packet
     * 01       Reserved
     * 10       Stale Data. Data has been fetched since last measurement cycle.
     * 11       Fault Detected
     */
     // done if we have good data
    done = ( data[0] <=0x3F );
  }
  
  return ((data[0] & 0x3F) << 8) + data[1] ;
}

// This is the main program
// globals for this program
MS4515DO ms45;
int nSamples;
long sampMsec;
float pInitial, pFinal;


void setup()
{
  int i;
  
 //set pins as outputs
 pinMode(LED_BUILTIN, OUTPUT);
// pressure 
  ms45.begin();

  Serial.begin(115200);
  Serial.setTimeout(1000000); // "disable" serial input timeout
  Serial << "MS4515DO test" << endl;  

/*
  // ask for #samples, sample rate and start pressure
  Serial << "Number of samples? ";
  nSamples = Serial.parseInt();
  Serial << "Sample rate (msec)? ";
  sampMsec = Serial.parseInt();
  Serial << "Initial pressure? ";
  pInitial = Serial.parseFloat();

  Serial << nSamples << " Samples at " << sampMsec << "msec" << endl;
  Serial << "Initial pressure " << pInitial << endl;
  Serial << "sample\ttime\tcounts\tmV\tpsi\tcm H2O\r\n";
  */
} 

void loop()
{
  int results, sample=1;
  long initTime = 0, msec = 0;
  float mv, psi, cm;
  char t = '\t'; // for tab separator

  results = ms45.readPressure();
  Serial << "result " << results << endl;
  delay(100);

/*  
  while ( sample <= nSamples ) {
    results = ms45.readPressure();
    mv = results * 0.0078125;
    psi = mv * 1.45/35;
    cm = psi * 70.307;
    Serial << sample <<t << (float)msec/1000. << t << results << t << mv <<t << psi<< t << cm << endl;
    if ( initTime == 0 ) {
      initTime = millis();
    }
    msec += sampMsec;
    while ( millis() -initTime <= msec ) {} // wait
    sample++;
  }
  Serial << "Final pressure? ";
  pFinal = Serial.parseFloat();
  Serial << "Final pressure "<< pFinal << endl;
  while (1) {
    digitalWrite( LED_BUILTIN, !digitalRead( LED_BUILTIN )); // blink LED
    delay(500);
  }
  */
}
