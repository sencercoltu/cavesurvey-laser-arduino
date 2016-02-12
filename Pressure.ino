#include <BMP085.h>

BMP085 sensor_pressure;

void Pressure_Init() {
  sensor_pressure.initialize(); delay(20);
  if (!sensor_pressure.testConnection())
    Bluetooth.println("[S]Baro err");
  else
    Bluetooth.println("[S]Baro OK");    
}

void Read_Pressure() {  
  int32_t lastMicros;
  sensor_pressure.setControl(BMP085_MODE_TEMPERATURE);
  lastMicros = micros();
  while (micros() - lastMicros < sensor_pressure.getMeasureDelayMicroseconds());
  temperature = sensor_pressure.getTemperatureC();
  sensor_pressure.setControl(BMP085_MODE_PRESSURE_3);
  while (micros() - lastMicros < sensor_pressure.getMeasureDelayMicroseconds());
  pressure = sensor_pressure.getPressure();
  altitude = sensor_pressure.getAltitude(pressure, parameters[ALT_SEA_LEVEL_PRESSURE]);
}
