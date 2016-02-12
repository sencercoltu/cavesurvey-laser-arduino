#include <L3G4200D.h>

L3G4200D sensor_gyroscope;

void Gyro_Init()
{
  sensor_gyroscope.initialize(); delay(20);
  if (!sensor_gyroscope.testConnection())
    Bluetooth.println("[S]Gyro err");
  else
    Bluetooth.println("[S]Gyro OK");  
  sensor_gyroscope.setOutputDataRate(L3G4200D_RATE_100);
  sensor_gyroscope.setFullScale(L3G4200D_FS_2000);  delay(20);
}

// Reads x, y and z gyroscope registers
void Read_Gyro()
{
  sensor_gyroscope.getAngularVelocity(&gyro[0], &gyro[1], &gyro[2]);
}

