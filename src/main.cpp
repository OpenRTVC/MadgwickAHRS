
#include "I2Cdev.h"
#include "MPU6050.h"
#include <MadgwickAHRS.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

#define frequency_hz 25

Madgwick filter;
MPU6050 accelgyro;

unsigned long millisPerReading, millisPrevious;
float accelScale, gyroScale;

float convertRawAcceleration(int16_t aRaw);
float convertRawGyro(int16_t gRaw);

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA, SCL, 400000);

  // start the IMU and filter
  Serial.println("Initializing I2C devices...");
  accelgyro.initialize();

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

  Serial.println("Calibrate Gyro and accel...");
  delay(3000);
  accelgyro.CalibrateGyro(10);
  accelgyro.CalibrateAccel(10);
  accelgyro.PrintActiveOffsets();
  delay(3000);

  filter.begin(frequency_hz);

  // Set the accelerometer range to 2G
  // CurieIMU.setAccelerometerRange(2);
  // Set the gyroscope range to 250 degrees/second
  // CurieIMU.setGyroRange(250);

  // initialize variables to pace updates to correct rate
  millisPerReading = 1000/frequency_hz;
  millisPrevious = millis();
}

void loop() {
  int16_t aix, aiy, aiz;
  int16_t gix, giy, giz;
  float ax, ay, az;
  float gx, gy, gz;
  float roll, pitch, yaw;
  unsigned long millisNow;

  // check if it's time to read data and update the filter
  millisNow = millis();
  if (millisNow - millisPrevious >= millisPerReading) {

    accelgyro.getMotion6(&aix, &aiy, &aiz, &gix, &giy, &giz);

    // convert from raw data to gravity and degrees/second units
    ax = convertRawAcceleration(aix);
    ay = convertRawAcceleration(aiy);
    az = convertRawAcceleration(aiz);
    gx = convertRawGyro(gix);
    gy = convertRawGyro(giy);
    gz = convertRawGyro(giz);

    // update the filter, which computes orientation
    // filter.updateIMU(gx, gy, gz, ax, ay, az);
    // filter.updateIMU(gy, gx, gz, ay,ax,az);
    // filter.updateIMU(gx, gy, gz, 0,0,0);  // Fonctionne
    filter.updateIMU(gx, gy, gz, ax, ay, az);  // Discutable avec Accel 
 
    // print the yaw, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    yaw = filter.getYaw();
    
    // // Serial.print("Orientation: ");
    Serial.print(yaw); Serial.print(",");
    Serial.print(pitch); Serial.print(",");
    Serial.println(roll);

    // Serial.print("Accelerometer: ");
    // Serial.print(ax); Serial.print(",");
    // Serial.print(ay); Serial.print(",");
    // Serial.println(az);

    // Print Quaternions
    // Serial.print(filter.q0); Serial.print(",");
    // Serial.print(filter.q1); Serial.print(",");
    // Serial.print(filter.q2);Serial.print(",");
    // Serial.println(filter.q3);

    // increment previous time, so we keep proper pace
    millisPrevious = millisPrevious + millisPerReading;
  }
}

float convertRawAcceleration(int16_t aRaw) {
  // since we are using 2G range
  // -2g maps to a raw value of -32768
  // +2g maps to a raw value of 32767
  
  float a = (aRaw * 2.0) / 32768.0; 
  return a;
}

float convertRawGyro(int16_t gRaw) {
  // since we are using 250 degrees/seconds range
  // -250 maps to a raw value of -32768
  // +250 maps to a raw value of 32767
  
  float g = (gRaw * 250.0) / 32768.0;
  return g;
}