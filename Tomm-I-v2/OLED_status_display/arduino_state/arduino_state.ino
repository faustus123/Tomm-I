/*
  Analog Input

  Demonstrates analog input by reading an analog sensor on analog pin 0 and
  turning on and off a light emitting diode(LED) connected to digital pin 13.
  The amount of time the LED will be on and off depends on the value obtained
  by analogRead().

  The circuit:
  - potentiometer
    center pin of the potentiometer to the analog input 0
    one side pin (either one) to ground
    the other side pin to +5V
  - LED
    anode (long leg) attached to digital output 13
    cathode (short leg) attached to ground

  - Note: because most Arduinos have a built-in LED attached to pin 13 on the
    board, the LED is optional.

  created by David Cuartielles
  modified 30 Aug 2011
  By Tom Igoe

  This example code is in the public domain.

  http://www.arduino.cc/en/Tutorial/AnalogInput
*/

int battery_pin = 15;    // select the input pin for the potentiometer
int analogInputs[16];

float Vnominal = 7.4;
float Vfull = 1.125*Vnominal*0.98; // 0.98 is to give some head room for full
float Vempty = 0.8*Vnominal;

void setup() {

  Serial.begin(9600);
  Serial1.begin(9600);
  
}

void loop() {

  // read all analog inputs
  for(int ipin=0; ipin<16; ipin++){
    analogInputs[A0 + ipin] = analogRead(ipin);
  }
  
  // convert battery value to calibrated voltage
  float voltage = analogInputs[A0+battery_pin]*(5.0/1023.0)*5.0;
  voltage = 1.007*voltage + 0.224; // based on linear fit

  // calculate percentage charge left on battery
  float percent_full = (voltage - Vempty)/(Vfull - Vempty)*100.0;
  if( percent_full > 100.0) percent_full = 100.0;

  String json_str("");
  char Vstr[16];
  char Percentstr[16];
  dtostrf(voltage, 4, 2, Vstr);
  dtostrf(percent_full, 5, 1, Percentstr);
  json_str += String("{");
  json_str += String("\"battery_voltage\":") + Vstr;
  json_str += String(", \"battery_percent\":") + Percentstr;
  for(int ipin=0; ipin<16; ipin++){
    json_str += String(", \"A") + ipin + String("\":") + analogInputs[A0+ipin];
  }
  json_str += String("}");
  //sprintf(json_str, "{ \"battery_voltage\":%s,  \"battery_percent\":%s", Vstr, Percentstr);
  
  // Write serial1 (not USB which is mirrored to serial 0!)
  Serial1.println(json_str);

  Serial.println(json_str);

  delay(500);
 }


 
