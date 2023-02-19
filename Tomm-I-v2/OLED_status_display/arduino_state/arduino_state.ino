
#include <Wire.h>


int battery_pin = 15;    // select the input pin for the potentiometer
int analogInputs[16];

float Vnominal = 7.4;
float Vfull = 1.125*Vnominal*0.98; // 0.98 is to give some head room for full
float Vempty = 0.8*Vnominal;

unsigned long last_time; // updated to millis() every iteration of loop()
unsigned long last_print_time = 0; // time in millis since last printed to serial monitor
unsigned int last_data_size = 0;

float last_distance_cm = -1.0;

void MPU6050_setup();
void calculate_IMU_error();

void GetAnalogInputsState(String &str);
void GetBatteryState(String &str);
void GetMPU6050State(String &str);

//-------------------------------
// setup
//-------------------------------
void setup() {

  Serial.begin(9600); // USB serial connection used for arduino serial monitor and programming arduino
  Serial1.begin(115200); // TX1,RX1 on arduion used for high speed communication
//  Serial1.begin(9600); // TX1,RX1 on arduion used for high speed communication

  MPU6050_setup();

  last_time = millis();
}

//-------------------------------
// loop
//-------------------------------
void loop() {

  unsigned long now = millis();
  unsigned long loop_time_ms = now - last_time;
  last_time = now;

  // Get state of various pieces in forma of JSON-compatible strings.
  // All should be prefixed with a comma "," so they are easily appended
  // to the JSON string.
  String analogInputs_str;
  String battery_str;
  String mpu_str;
  String dist_str;
  GetAnalogInputsState(analogInputs_str);
  GetBatteryState(battery_str);
  GetMPU6050State(mpu_str);
  GetHCSR04State(dist_str);

  unsigned long device_read_time_ms = millis() - now; // how long did it take to read all devices

  // Create JSSON state string and send it
  String json_str;
  json_str = "{";
  json_str += String("\"millis\":") + now;
  json_str += String(", \"device_read_time_ms\":") + device_read_time_ms;
  json_str += String(", \"loop_time_ms\":") + loop_time_ms; // time elapsed since last iteration of loop
  json_str += String(", \"last_data_size\":") + last_data_size; // bytes in last message sent
  json_str += battery_str;
  json_str += mpu_str;
  json_str += dist_str;
  json_str += analogInputs_str;
  json_str += String("}");

  last_data_size = json_str.length();
  
  // Write serial1 (not USB which is mirrored to serial 0!)
  Serial1.println(json_str);

  // Print to the main serial device once every two second2
  // so it can be monitored via the serial monitor
  if( (now - last_print_time) > 2000 ){
    // Serial.println(json_str);   // n.b. this can add 300ms to the loop time!
    last_print_time = now;
  }

  //delay(500);
 }

//-------------------------------
// GetAnalogInputsState
//
/// This will read all of the analog input pins and store
/// them in the global analogInputs[] array. It will then
/// write the values in JSON-compatible format into the
/// provided String.
///
///
/// n.b. the MUST be called before anything that uses the
/// analog input values (e.g. GetBatteryState)
//-------------------------------
void GetAnalogInputsState(String &str)
{
  // read all analog inputs
  for(int ipin=0; ipin<16; ipin++){
    analogInputs[ipin] = analogRead(A0+ipin);
  }

  for(int ipin=0; ipin<16; ipin++){
    str += String(", \"A") + ipin + String("\":") + analogInputs[ipin];
  }
}

//-------------------------------
// GetBatteryState
//
/// This will convert the analog input from A15 into a
/// voltage and a percent full value for the battery.
/// It assumes GetAnalogInputsState() has already been
/// called so that analogInputs[battery_pin] is valid.
///
/// The voltage is calculated using a calibration obtained
/// by measuring the batteyr with a DMM and recording the
/// value read from the analog input.
//-------------------------------
void GetBatteryState(String &str)
{
  char Vstr[16];
  char Percentstr[16];

  // convert battery value to calibrated voltage
  float voltage = analogInputs[battery_pin]*(5.0/1023.0)*5.0;
  voltage = 1.007*voltage + 0.224; // based on linear fit

  // calculate percentage charge left on battery
  float percent_full = (voltage - Vempty)/(Vfull - Vempty)*100.0;
  if( percent_full > 100.0) percent_full = 100.0;

  // Print values into string that can be inserted into JSON
  // e.g. ', "battery_voltage":7.22, "battery_percent":62.1'
  dtostrf(voltage, 4, 2, Vstr);
  dtostrf(percent_full, 5, 1, Percentstr);
  str = String(", \"battery_voltage\":") + Vstr;
  str += String(", \"battery_percent\":") + Percentstr;
}

//-------------------------------
// GetHCSR04State
//-------------------------------
void GetHCSR04State(String &str) {
  // This is based on information in the description here:
  // https://www.adafruit.com/product/4742
  //
  // I later found some example code here:
  // http://www.mikroelec.com/product/2182/rcwl-9610-ultrasonic-sensor-2022-version-3-to-5v-ultrasonic-distance-module-replace-for-hc-sr04-with
  //
  // Note that in the comments of the code at the second site
  // they says we need to wait at least 30ms between readings
  // and wait at least 150ms for the return signal. That blows
  // our timing budget. The use case here is for distances that
  // aren't more than about 40cm => 0.40/343*1000 = 1.1ms.
  // The maximum distance the device can measure is ~400cm which
  // would correspond to ~11ms so I don't know where they are
  // getting 150ms from. I do know that if I try reading the device
  // when no response is recieved, it takes about 100ms extra to
  // complete the read.
  const int HCSR04 = 0x57; // MPU6050 I2C address
  Wire.beginTransmission(HCSR04);
  Wire.write(0x01); // Send pulse
  Wire.endTransmission(false);
  delay(2);
  Wire.requestFrom(HCSR04, 3, true); // Read 3 bytes
  // n.b. Because other devices use the I2C bus, we can't
  // leave other bytes in the Wire buffer. Thus, if there
  // is no return signal then we have to wait for the
  // device to timeout and return 3 values of 0XFF.
  unsigned long byte0 = Wire.read() & 0xFF;
  unsigned long byte1 = Wire.read() & 0xFF;
  unsigned long byte2 = Wire.read() & 0xFF;
  unsigned long dist = (byte0 << 16) + (byte1 << 8) + (byte2 << 0);
  if( (dist != 0x00FFFFFF) && (dist != 0x00000000) ){
    last_distance_cm = dist / 10000.0;
  }
  if( last_distance_cm > 400.0 ) last_distance_cm = -1;
  str = String(", \"dist_front\":") + last_distance_cm;

}


//==========================================================================
// The following code was copied from the following website (and slightly modified):
// https://howtomechatronics.com/tutorials/arduino/arduino-and-mpu6050-accelerometer-and-gyroscope-tutorial/

/*
   Arduino and MPU6050 Accelerometer and Gyroscope Sensor Tutorial
   by Dejan, https://howtomechatronics.com
*/
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ;
float GyroX, GyroY, GyroZ;
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ;
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

//-------------------------------
// MPU6050_setup
//-------------------------------
void MPU6050_setup() {
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  
  // Configure Accelerometer Sensitivity - Full Scale Range (default +/- 2g)
  Wire.beginTransmission(MPU);
  Wire.write(0x1C);                  //Talk to the ACCEL_CONFIG register (1C hex)
  Wire.write(0x00);                  //Set the register bits as 00000000 (+/- 2g full scale range)
  Wire.endTransmission(true);
  // Configure Gyro Sensitivity - Full Scale Range (default +/- 250deg/s)
  Wire.beginTransmission(MPU);
  Wire.write(0x1B);                   // Talk to the GYRO_CONFIG register (1B hex)
  Wire.write(0x00);                   // Set the register bits as 00000000 (250deg/s full scale)
  Wire.endTransmission(true);
  delay(20);
  
  // Call this function if you need to get the IMU error values for your module
  calculate_IMU_error();
  delay(20);
}

//-------------------------------
// GetMPU6050State
//-------------------------------
void GetMPU6050State(String &str) {
  // === Read acceleromter data === //
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
  // Calculating Roll and Pitch from the accelerometer data
  accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - 0.58; // AccErrorX ~(0.58) See the calculate_IMU_error()custom function for more details
  accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) + 1.58; // AccErrorY ~(-1.58)
  // === Read gyroscope data === //
  previousTime = currentTime;        // Previous time is stored before the actual time read
  currentTime = millis();            // Current time actual time read
  elapsedTime = (currentTime - previousTime) / 1000; // Divide by 1000 to get seconds
  Wire.beginTransmission(MPU);
  Wire.write(0x43); // Gyro data first register address 0x43
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 4 registers total, each axis value is stored in 2 registers
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0; // For a 250deg/s range we have to divide first the raw value by 131.0, according to the datasheet
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
  // Correct the outputs with the calculated error values
  GyroX = GyroX - GyroErrorX; // GyroErrorX ~(-0.56)
  GyroY = GyroY - GyroErrorY; // GyroErrorY ~(2)
  GyroZ = GyroZ - GyroErrorZ; // GyroErrorZ ~ (-0.8)
  // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
  gyroAngleX = gyroAngleX + GyroX * elapsedTime; // deg/s * s = deg
  gyroAngleY = gyroAngleY + GyroY * elapsedTime;
  yaw =  yaw + GyroZ * elapsedTime;
  // Complementary filter - combine acceleromter and gyro angle values
  roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
  pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
  
  // Print the values to the given string
  str = String(", \"roll_front\":") + roll;
  str += String(", \"pitch_front\":") + pitch;
  str += String(", \"yaw_front\":") + yaw;
//  Serial.print(roll);
//  Serial.print("/");
//  Serial.print(pitch);
//  Serial.print("/");
//  Serial.println(yaw);
}

//-------------------------------
// calculate_IMU_error
//-------------------------------
void calculate_IMU_error() {
  // We can call this funtion in the setup section to calculate the accelerometer and gyro data error. From here we will get the error values used in the above equations printed on the Serial Monitor.
  // Note that we should place the IMU flat in order to get the proper values, so that we then can the correct values
  // Read accelerometer values 200 times
  while (c < 20) {
    Wire.beginTransmission(MPU);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    AccX = (double)(Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccY = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0 ;
    Serial.print("AccX: "); Serial.println(AccX);
    // Sum all readings
    AccErrorX = AccErrorX + ((atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI));
    AccErrorY = AccErrorY + ((atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI));
    c++;
  }
  //Divide the sum by 200 to get the error value
  AccErrorX = AccErrorX / 20;
  AccErrorY = AccErrorY / 200;
  c = 0;
  // Read gyro values 200 times
  while (c < 200) {
    Wire.beginTransmission(MPU);
    Wire.write(0x43);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true);
    GyroX = Wire.read() << 8 | Wire.read();
    GyroY = Wire.read() << 8 | Wire.read();
    GyroZ = Wire.read() << 8 | Wire.read();
    // Sum all readings
    GyroErrorX = GyroErrorX + (GyroX / 131.0);
    GyroErrorY = GyroErrorY + (GyroY / 131.0);
    GyroErrorZ = GyroErrorZ + (GyroZ / 131.0);
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  // Print the error values on the Serial Monitor
  Serial.print("AccErrorX: ");
  Serial.println(AccErrorX);
  Serial.print("AccErrorY: ");
  Serial.println(AccErrorY);
  Serial.print("GyroErrorX: ");
  Serial.println(GyroErrorX);
  Serial.print("GyroErrorY: ");
  Serial.println(GyroErrorY);
  Serial.print("GyroErrorZ: ");
  Serial.println(GyroErrorZ);
}
