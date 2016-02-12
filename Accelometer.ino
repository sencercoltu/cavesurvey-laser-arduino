#include <ADXL345.h>

ADXL345 sensor_accel;

void Accel_Init()
{
  sensor_accel.initialize(); 
  delay(20);
  if (!sensor_accel.testConnection())
    Bluetooth.println("[S]Acc err");
  else
    Bluetooth.println("[S]Acc OK");
  //sensor_accel.setFullResolution(1);
  //sensor_accel.setRange(ADXL345_RANGE_2G);
  //sensor_accel.setRate(ADXL345_RATE_50);  
}

// Reads x, y and z accelerometer registers
void Read_Accel()
{
  sensor_accel.getAcceleration(&accel[0], &accel[1], &accel[2]);
}

