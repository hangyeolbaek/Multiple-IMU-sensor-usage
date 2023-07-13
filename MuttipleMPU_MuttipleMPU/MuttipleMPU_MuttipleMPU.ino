
#include <Wire.h>
#include <Kalman.h> // Source: https://github.com/TKJElectronics/KalmanFilter

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

Kalman kalmanX2;
Kalman kalmanY2;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t timer;
uint8_t i2cData[14]; // Buffer for I2C data

const int MPU2 = 0x69, MPU1 = 0x68;

/* IMU Data2 */
double accX2, accY2, accZ2;
double gyroX2, gyroY2, gyroZ2;
int16_t tempRaw2;

double gyroXangle2, gyroYangle2; // Angle calculate using the gyro only
double compAngleX2, compAngleY2; // Calculated angle using a complementary filter
double kalAngleX2, kalAngleY2; // Calculated angle using a Kalman filter

uint32_t timer2;
uint8_t i2cData2[14]; // Buffer for I2C data




//-------------------------------------------------\setup loop\------------------------------------------------------------
void setup() {
  Serial.begin(9600);
  Wire.begin();
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode

  while (i2cRead(0x75, i2cData, 1));
  if (i2cData[0] != 0x68) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor 1"));
    while (1);
  }

  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead(0x3B, i2cData, 6));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  kalmanX.setAngle(roll); // Set starting angle
  kalmanY.setAngle(pitch);
  gyroXangle = roll;
  gyroYangle = pitch;
  compAngleX = roll;
  compAngleY = pitch;

  timer = micros();

//---------------------------------------------------
   i2cData2[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData2[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData2[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData2[3] = 0x00; // Set Accelerometer Full Scale Range to ±2g
  while (i2cWrite2(0x19, i2cData2, 4, false)); // Write to all four registers at once
  while (i2cWrite2(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
/*
  while (i2cRead2(0x75, i2cData2, 1));
  if (i2cData2[0] != 0x69) { // Read "WHO_AM_I" register
    Serial.print(F("Error reading sensor 2"));
    while (1);
  }
*/
  delay(100); // Wait for sensor to stabilize

  /* Set kalman and gyro starting angle */
  while (i2cRead2(0x3B, i2cData2, 6));
  accX2 = (int16_t)((i2cData2[0] << 8) | i2cData2[1]);
  accY2 = (int16_t)((i2cData2[2] << 8) | i2cData2[3]);
  accZ2 = (int16_t)((i2cData2[4] << 8) | i2cData2[5]);

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH2 // Eq. 25 and 26
  double roll2  = atan2(accY2, accZ2) * RAD_TO_DEG;
  double pitch2 = atan(-accX2 / sqrt(accY2 * accY2 + accZ2 * accZ2)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll2  = atan(accY2 / sqrt(accX2 * accX2 + accZ2 * accZ2)) * RAD_TO_DEG;
  double pitch2 = atan2(-accX2, accZ2) * RAD_TO_DEG;
#endif

  kalmanX2.setAngle(roll2); // Set starting angle
  kalmanY2.setAngle(pitch2);
  gyroXangle2 = roll2;
  gyroYangle2 = pitch2;
  compAngleX2 = roll2;
  compAngleY2 = pitch2;

  timer = micros();
}

//---------------------------------------------------\void loop\------------------------------------------------------------
void loop() {

  while (i2cRead(0x3B, i2cData, 14));
  accX = (int16_t)((i2cData[0] << 8) | i2cData[1]);
  accY = (int16_t)((i2cData[2] << 8) | i2cData[3]);
  accZ = (int16_t)((i2cData[4] << 8) | i2cData[5]);
  tempRaw = (int16_t)((i2cData[6] << 8) | i2cData[7]);
  gyroX = (int16_t)((i2cData[8] << 8) | i2cData[9]);
  gyroY = (int16_t)((i2cData[10] << 8) | i2cData[11]);
  gyroZ = (int16_t)((i2cData[12] << 8) | i2cData[13]);;

  double dt = (double)(micros() - timer) / 1000000; // Calculate delta time
  timer = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
    kalmanX.setAngle(roll);
    compAngleX = roll;
    kalAngleX = roll;
    gyroXangle = roll;
  } else
    kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
    kalmanY.setAngle(pitch);
    compAngleY = pitch;
    kalAngleY = pitch;
    gyroYangle = pitch;
  } else
    kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

  gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = kalAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = kalAngleY;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX); Serial.print("\t");
  Serial.print(accY); Serial.print("\t");
  Serial.print(accZ); Serial.print("\t");

  Serial.print(gyroX); Serial.print("\t");
  Serial.print(gyroY); Serial.print("\t");
  Serial.print(gyroZ); Serial.print("\t");

  Serial.print("\t");
#endif

  //Serial.print(roll); Serial.print("\t");
  // Serial.print(gyroXangle); Serial.print("\t");
  // Serial.print(compAngleX); Serial.print("\t");
  Serial.print("     roll Angle : ");
  Serial.print(kalAngleX);






  while (i2cRead2(0x3B, i2cData2, 14));
  accX2 = (int16_t)((i2cData2[0] << 8) | i2cData2[1]);
  accY2 = (int16_t)((i2cData2[2] << 8) | i2cData2[3]);
  accZ2 = (int16_t)((i2cData2[4] << 8) | i2cData2[5]);
  tempRaw2 = (int16_t)((i2cData2[6] << 8) | i2cData2[7]);
  gyroX2 = (int16_t)((i2cData2[8] << 8) | i2cData2[9]);
  gyroY2 = (int16_t)((i2cData2[10] << 8) | i2cData2[11]);
  gyroZ2 = (int16_t)((i2cData2[12] << 8) | i2cData2[13]);;
  double dt2 = (double)(micros() - timer2) / 1000000; // Calculate delta time
  timer2 = micros();

  // Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
  // atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
  // It is then converted from radians to degrees
#ifdef RESTRICT_PITCH_2 // Eq. 25 and 26
  double roll2  = atan2(accY2, accZ2) * RAD_TO_DEG;
  double pitch2 = atan(-accX2 / sqrt(accY2 * accY2 + accZ2 * accZ2)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll2  = atan(accY2 / sqrt(accX2 * accX2 + accZ2 * accZ2)) * RAD_TO_DEG;
  double pitch2 = atan2(-accX2, accZ2) * RAD_TO_DEG;
#endif

  double gyroXrate2 = gyroX2 / 131.0; // Convert to deg/s
  double gyroYrate2 = gyroY2 / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll2 < -90 && kalAngleX2 > 90) || (roll2 > 90 && kalAngleX2 < -90)) {
    kalmanX2.setAngle(roll2);
    compAngleX2 = roll2;
    kalAngleX2 = roll2;
    gyroXangle2 = roll2;
  } else
    kalAngleX2 = kalmanX2.getAngle(roll2, gyroXrate2, dt2); // Calculate the angle using a Kalman filter

  if (abs(kalAngleX2) > 90)
    gyroYrate2 = -gyroYrate2; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleY2 = kalmanY2.getAngle(pitch2, gyroYrate2, dt2);
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch2 < -90 && kalAngleY2 > 90) || (pitch2 > 90 && kalAngleY2 < -90)) {
    kalmanY2.setAngle(pitch2);
    compAngleY2 = pitch2;
    kalAngleY2 = pitch2;
    gyroYangle2 = pitch2;
  } else
    kalAngleY2 = kalmanY2.getAngle(pitch2, gyroYrate2, dt2); // Calculate the angle using a Kalman filter

  if (abs(kalAngleY2) > 90)
    gyroXrate2 = -gyroXrate2; // Invert rate, so it fits the restriced accelerometer reading
  kalAngleX2 = kalmanX2.getAngle(roll2, gyroXrate2, dt2); // Calculate the angle using a Kalman filter
#endif

  gyroXangle2 += gyroXrate2 * dt2; // Calculate gyro angle without any filter
  gyroYangle2 += gyroYrate2 * dt2;
  //gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
  //gyroYangle += kalmanY.getRate() * dt;

  compAngleX2 = 0.93 * (compAngleX2 + gyroXrate2 * dt2) + 0.07 * roll2; // Calculate the angle using a Complimentary filter
  compAngleY2 = 0.93 * (compAngleY2 + gyroYrate2 * dt2) + 0.07 * pitch2;

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle2 < -180 || gyroXangle2 > 180)
    gyroXangle2 = kalAngleX2;
  if (gyroYangle2 < -180 || gyroYangle2 > 180)
    gyroYangle2 = kalAngleY2;

  /* Print Data */
#if 0 // Set to 1 to activate
  Serial.print(accX2); Serial.print("\t");
  Serial.print(accY2); Serial.print("\t");
  Serial.print(accZ2); Serial.print("\t");

  Serial.print(gyroX2); Serial.print("\t");
  Serial.print(gyroY2); Serial.print("\t");
  Serial.print(gyroZ2); Serial.print("\t");

  Serial.print("\t");
#endif


  Serial.print("  rear Angle : ");
  Serial.println(kalAngleX2);

}
