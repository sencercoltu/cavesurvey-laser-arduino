
#include <EEPROM.h>

#define EEPROM_MAGIC_VALUE (0xa5)

void setEEPROMDirty()
{
  EEPROM.write(0, 0x00);
}  

void writeEEPROM() {
  Bluetooth.println("[D]Wr params");
  byte *b = (byte *) parameters;
  for(int i=0; i<sizeof(parameters); i++) {
    EEPROM.write(i+1, b[i]); 
  }    
  EEPROM.write(0, EEPROM_MAGIC_VALUE);
}

void setEEPROMDefaults() {
  Bluetooth.println("[D]Set defs");
  parameters[ACCEL_X_MIN] = (-250.0f);
  parameters[ACCEL_X_MAX] = (250.0f);
  parameters[ACCEL_Y_MIN] = (-250.0f);
  parameters[ACCEL_Y_MAX] = (250.0f);
  parameters[ACCEL_Z_MIN] = (-250.0f);
  parameters[ACCEL_Z_MAX] = (250.0f);
  // Magnetometer
  // "magn x,y,z (min/max) = X_MIN/X_MAX  Y_MIN/Y_MAX  Z_MIN/Z_MAX"
  parameters[MAGN_X_MIN] = (-600.0f);
  parameters[MAGN_X_MAX] = (600.0f);
  parameters[MAGN_Y_MIN] = (-600.0f);
  parameters[MAGN_Y_MAX] = (600.0f);
  parameters[MAGN_Z_MIN] = (-600.0f);
  parameters[MAGN_Z_MAX] = (600.0f);

  parameters[ACCEL_X_DEV] = 0.0f;
  parameters[ACCEL_Y_DEV] = 0.0f;
  parameters[ACCEL_Z_DEV] = 0.0f;
  
  parameters[MAG_X_DEV] = 0.0f;
  parameters[MAG_Y_DEV] = 0.0f;
  parameters[MAG_Z_DEV] = 0.0f;

  // Gyroscope
  // "gyro x,y,z (current/average) = .../OFFSET_X  .../OFFSET_Y  .../OFFSET_Z
  parameters[GYRO_X_OFFSET] = (0.0f);
  parameters[GYRO_Y_OFFSET] = (0.0f);
  parameters[GYRO_Z_OFFSET] = (0.0f);

  // Altimeter
  parameters[ALT_SEA_LEVEL_PRESSURE] = 102133;

  // Gain for gyroscope
  parameters[GYRO_GAIN_X] = (0.06957f);
  parameters[GYRO_GAIN_Y] = (0.06957f);
  parameters[GYRO_GAIN_Z] = (0.06957f);
  
  writeEEPROM();  
}

void readEEPROM() {
  Bluetooth.println("[D]Eprom Chk");
  byte b = EEPROM.read(0);
  if (b != EEPROM_MAGIC_VALUE) {
    setEEPROMDefaults();
    return;
  }
  Bluetooth.println("[D]Rd params");      
  byte *p = (byte *) parameters;
  for(int i=0; i<sizeof(parameters); i++)
    p[i] = EEPROM.read(i+1); //ilk byte magic idi        
}

