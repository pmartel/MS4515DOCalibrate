// Code to calibrate the MPX10 pressure sensor
// using the Perfect Prime digital Manometer as a standard
#include <Wire.h> 
#include <Adafruit_ADS1015.h>
#include <Streaming.h>

// The A/D we have is an ADS1015 (12-bit) but with the '1115 object(16-bit)
// and the '1115 gain, the results are right 
Adafruit_ADS1115 ads1115;
int nSamples;
long sampMsec;
float pInitial, pFinal;

void setup()
{
  int i;
  
 //set pins as outputs
 pinMode(LED_BUILTIN, OUTPUT);
// a2d
  ads1115.begin();
  ads1115.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV

  Serial.begin(115200);
  Serial.setTimeout(1000000); // "disable" serial input timeout
  Serial << "Differential mode" << endl;  
  Serial << "gain set to: " << ads1115.getGain() << endl;

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
} 

void loop()
{
  int results, sample=1;
  long initTime = 0, msec = 0;
  float mv, psi, cm;
  char t = '\t'; // for tab separator
  
  while ( sample <= nSamples ) {
    results = ads1115.readADC_Differential_0_1();
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
}

