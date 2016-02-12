/***************************************************************************************************************
 * Razor AHRS Firmware v1.4.0
 * 9 Degree of Measurement Attitude and Heading Reference System
 * for Sparkfun "9DOF Razor IMU" (SEN-10125 and SEN-10736)
 * and "9DOF Sensor Stick" (SEN-10183, 10321 and SEN-10724)
 *
 * Released under GNU GPL (General Public License) v3.0
 * Copyright (C) 2011 Quality & Usability Lab, Deutsche Telekom Laboratories, TU Berlin
 *
 * Infos, updates, bug reports and feedback:
 *     http://dev.qu.tu-berlin.de/projects/sf-razor-9dof-ahrs
 *
 *
 * History:
 *   * Original code (http://code.google.com/p/sf9domahrs/) by Doug Weibel and Jose Julio,
 *     based on ArduIMU v1.5 by Jordi Munoz and William Premerlani, Jose Julio and Doug Weibel. Thank you!
 *
 *   * Updated code (http://groups.google.com/group/sf_9dof_ahrs_update) by David Malik (david.zsolt.malik@gmail.com)
 *     for new Sparkfun 9DOF Razor hardware (SEN-10125).
 *
 *   * Updated and extended by Peter Bartz (peter-bartz@gmx.de):
 *     * v1.3.0
 *       * Cleaned up, streamlined and restructured most of the code to make it more comprehensible.
 *       * Added sensor calibration (improves precision and responsiveness a lot!).
 *       * Added binary yaw/pitch/roll output.
 *       * Added basic serial command interface to set output modes/calibrate sensors/synch stream/etc.
 *       * Added support to synch automatically when using Rovering Networks Bluetooth modules (and compatible).
 *       * Wrote new easier to use test program (using Processing).
 *       * Added support for new version of "9DOF Razor IMU": SEN-10736.
 *       --> The output of this code is not compatible with the older versions!
 *       --> A Processing sketch to test the tracker is available.
 *     * v1.3.1
 *       * Initializing rotation matrix based on start-up sensor readings -> orientation OK right away.
 *       * Adjusted gyro low-pass filter and output rate settings.
 *     * v1.3.2
 *       * Adapted code to work with new Arduino 1.0 (and older versions still).
 *     * v1.3.3
 *       * Improved synching.
 *     * v1.4.0
 *       * Added support for SparkFun "9DOF Sensor Stick" (versions SEN-10183, SEN-10321 and SEN-10724).
 *
 * TODOs:
 *   * Allow optional use of EEPROM for storing and reading calibration values.
 *   * Use self-test and temperature-compensation features of the sensors.
 *   * Add binary output of unfused sensor data for all 9 axes.
 ***************************************************************************************************************/

#include <Wire.h>
#include <I2Cdev.h>
#include <AltSoftSerial.h>
#define FIRMWARE_VERSION "2014.09.09"

AltSoftSerial Bluetooth;

// OUTPUT OPTIONS
/*****************************************************************/
/*
1---------1200
 2---------2400
 3---------4800
 4---------9600
 5---------19200
 6---------38400
 7---------57600
 8---------115200
 */

// Set your serial port baud rate used to send out data here!
#define INPUT_BAUD_RATE 115200
#define OUTPUT_BAUD_RATE 38400
//#define BT_BAUD_COMMAND "AT+BAUD6"
#define LED_PIN 13 // (Arduino is 13, Teensy is 6)

// Sensor data output interval in milliseconds
// This may not work, if faster than 20ms (=50Hz)
// Code is tuned for 20ms, so better leave it like that
#define OUTPUT_DATA_INTERVAL 20  // in milliseconds
#define HEARTBEAT_INTERVAL 10000

enum {
  ACCEL_X_MIN = 0,
  ACCEL_X_MAX,
  ACCEL_Y_MIN,
  ACCEL_Y_MAX,
  ACCEL_Z_MIN,
  ACCEL_Z_MAX,
  MAGN_X_MIN,
  MAGN_X_MAX,
  MAGN_Y_MIN,
  MAGN_Y_MAX,
  MAGN_Z_MIN,
  MAGN_Z_MAX,
  GYRO_X_OFFSET,
  GYRO_Y_OFFSET,
  GYRO_Z_OFFSET,
  GYRO_GAIN_X,
  GYRO_GAIN_Y,
  GYRO_GAIN_Z,
  ALT_SEA_LEVEL_PRESSURE,
  ACCEL_X_DEV,
  ACCEL_Y_DEV,
  ACCEL_Z_DEV,
  MAG_X_DEV,
  MAG_Y_DEV,
  MAG_Z_DEV,    
  PARAMS_MAX
};

float parameters[PARAMS_MAX] = {0};

// DCM parameters
#define GRAVITY (256.0f)
#define Kp_ROLLPITCH (0.02f)
#define Ki_ROLLPITCH (0.00002f)
#define Kp_YAW (1.2f)
#define Ki_YAW (0.00002f)

// Sensor calibration scale and offset values
#define ACCEL_X_OFFSET (((parameters[ACCEL_X_MIN] + parameters[ACCEL_X_MAX]) / 2.0f) + parameters[ACCEL_X_DEV])
#define ACCEL_Y_OFFSET (((parameters[ACCEL_Y_MIN] + parameters[ACCEL_Y_MAX]) / 2.0f) + parameters[ACCEL_Y_DEV])
#define ACCEL_Z_OFFSET (((parameters[ACCEL_Z_MIN] + parameters[ACCEL_Z_MAX]) / 2.0f) + parameters[ACCEL_Z_DEV])
#define ACCEL_X_SCALE (GRAVITY / (parameters[ACCEL_X_MAX] - ACCEL_X_OFFSET))
#define ACCEL_Y_SCALE (GRAVITY / (parameters[ACCEL_Y_MAX] - ACCEL_Y_OFFSET))
#define ACCEL_Z_SCALE (GRAVITY / (parameters[ACCEL_Z_MAX] - ACCEL_Z_OFFSET))

#define MAGN_X_OFFSET (((parameters[MAGN_X_MIN] + parameters[MAGN_X_MAX]) / 2.0f) + parameters[MAG_X_DEV])
#define MAGN_Y_OFFSET (((parameters[MAGN_Y_MIN] + parameters[MAGN_Y_MAX]) / 2.0f) + parameters[MAG_Y_DEV])
#define MAGN_Z_OFFSET (((parameters[MAGN_Z_MIN] + parameters[MAGN_Z_MAX]) / 2.0f) + parameters[MAG_Z_DEV])
#define MAGN_X_SCALE (100.0f / (parameters[MAGN_X_MAX] - MAGN_X_OFFSET))
#define MAGN_Y_SCALE (100.0f / (parameters[MAGN_Y_MAX] - MAGN_Y_OFFSET))
#define MAGN_Z_SCALE (100.0f / (parameters[MAGN_Z_MAX] - MAGN_Z_OFFSET))

#define GYRO_X_SCALE (TO_RAD(parameters[GYRO_GAIN_X]))
#define GYRO_Y_SCALE (TO_RAD(parameters[GYRO_GAIN_Y]))
#define GYRO_Z_SCALE (TO_RAD(parameters[GYRO_GAIN_Z]))

#define TO_RAD(x) (x * 0.01745329252)  // *pi/180
#define TO_DEG(x) (x * 57.2957795131)  // *180/pi

// RAW sensor data
int16_t accel[3] = {0};  // Actually stores the NEGATED acceleration (equals gravity, if board not moving).
int16_t accel_mapped[3] = {0};

int16_t magnetom[3] = {0};
int16_t magnetom_mapped[3] = {0};

int16_t gyro[3] = {0};
int16_t gyro_mapped[3] = {0};

float temperature;
float pressure;
float altitude;

// DCM variables
float MAG_Heading;
float Magn_Vector[3] = {0}; // Store the magnetometer turn rate in a vector
float Accel_Vector[3] = {0}; // Store the acceleration in a vector
float Gyro_Vector[3] = {0}; // Store the gyros turn rate in a vector
float Omega_Vector[3] = {0}; // Corrected Gyro_Vector data
float Omega_P[3] = {0}; // Omega Proportional correction
float Omega_I[3] = {0}; // Omega Integrator
float Omega[3] = {0};
float errorRollPitch[3] = {0};
float errorYaw[3] = {0};
float DCM_Matrix[3][3] = {
  {1, 0, 0}, 
  {0, 1, 0}, 
  {0, 0, 1}
};
float Update_Matrix[3][3] = {
  {0, 1, 2}, 
  {3, 4, 5}, 
  {6, 7, 8}
};
float Temporary_Matrix[3][3] = {0};
//  {0, 0, 0}, 
//  {0, 0, 0},
//  {0, 0, 0}
//};

// Euler angles
float yaw, pitch, roll;

// DCM timing in the main loop
long currMillis;
long heartstamp = 0;
long timestamp;
long timestamp_old;
long pressurestamp = 0;
float G_Dt; // Integration time for DCM algorithm

// More output-state variables
//int num_accel_errors = 0;
//int num_magn_errors = 0;
//int num_gyro_errors = 0;

String laserBuffer = "";
String bluetoothBuffer = "";

//float raw_min[3];
//float raw_max[3];

float distance = 0.0f;

void (*SoftReset)(void) = 0;

bool rawMode = false;
bool continiousMode = false;

void ReadSensors() {
  if (currMillis - pressurestamp >= 1000)
  {
    pressurestamp = currMillis;
    Read_Pressure(); //Read pressure only every second
  }
  Read_Gyro(); // Read gyroscope
  Read_Accel(); // Read accelerometer
  Read_Magn(); // Read magnetometer
  ApplySensorMapping();
}

// Read every sensor and record a time stamp
// Init DCM with unfiltered orientation
// TODO re-init global vars?
void reset_sensor_fusion()
{
  float temp1[3];
  float temp2[3];
  float xAxis[] ={1.0f, 0.0f, 0.0f};

  ReadSensors();
  timestamp = millis();

  // GET PITCH
  // Using y-z-plane-component/x-component of gravity vector
  pitch = -atan2(Accel_Vector[0], sqrt(Accel_Vector[1] * Accel_Vector[1] + Accel_Vector[2] * Accel_Vector[2]));

  // GET ROLL
  // Compensate pitch of gravity vector
  Vector_Cross_Product(temp1, Accel_Vector, xAxis);
  Vector_Cross_Product(temp2, xAxis, temp1);
  // Normally using x-z-plane-component/y-component of compensated gravity vector
  // roll = atan2(temp2[1], sqrt(temp2[0] * temp2[0] + temp2[2] * temp2[2]));
  // Since we compensated for pitch, x-z-plane-component equals z-component:
  roll = atan2(temp2[1], temp2[2]);

  // GET YAW
  Compass_Heading();
  yaw = MAG_Heading;

  // Init rotation matrix
  init_rotation_matrix(DCM_Matrix, yaw, pitch, roll);
}

// Apply calibration to raw sensor readings
void ApplySensorMapping()
{
  // Magnetometer axis mapping
  magnetom_mapped[0] = -magnetom[1];
  magnetom_mapped[1] = -magnetom[0];
  magnetom_mapped[2] = -magnetom[2];

  Magn_Vector[0] = magnetom_mapped[0];
  Magn_Vector[1] = magnetom_mapped[1];
  Magn_Vector[2] = magnetom_mapped[2];

  // Magnetometer values mapping
  Magn_Vector[0] -= MAGN_X_OFFSET;
  Magn_Vector[0] *= MAGN_X_SCALE;
  Magn_Vector[1] -= MAGN_Y_OFFSET;
  Magn_Vector[1] *= MAGN_Y_SCALE;
  Magn_Vector[2] -= MAGN_Z_OFFSET;
  Magn_Vector[2] *= MAGN_Z_SCALE;

  // Accelerometer axis mapping
  accel_mapped[0] = accel[1]; //OK
  accel_mapped[1] = accel[0]; //OK
  accel_mapped[2] = accel[2];  //OK

  Accel_Vector[0] = accel_mapped[0]; //OK
  Accel_Vector[1] = accel_mapped[1]; //OK
  Accel_Vector[2] = accel_mapped[2];  //OK

  // Accelerometer values mapping
  Accel_Vector[0] -= ACCEL_X_OFFSET;
  Accel_Vector[0] *= ACCEL_X_SCALE;
  Accel_Vector[1] -= ACCEL_Y_OFFSET;
  Accel_Vector[1] *= ACCEL_Y_SCALE;
  Accel_Vector[2] -= ACCEL_Z_OFFSET;
  Accel_Vector[2] *= ACCEL_Z_SCALE;

  // Gyroscope axis mapping
  gyro_mapped[0] = -gyro[1];
  gyro_mapped[1] = -gyro[0];
  gyro_mapped[2] = -gyro[2];

  Gyro_Vector[0] = gyro_mapped[0];
  Gyro_Vector[1] = gyro_mapped[1];
  Gyro_Vector[2] = gyro_mapped[2];

  // Gyroscope values mapping
  Gyro_Vector[0] -= parameters[GYRO_X_OFFSET];
  Gyro_Vector[0] *= GYRO_X_SCALE;
  Gyro_Vector[1] -= parameters[GYRO_Y_OFFSET];
  Gyro_Vector[1] *= GYRO_Y_SCALE;
  Gyro_Vector[2] -= parameters[GYRO_Z_OFFSET];
  Gyro_Vector[2] *= GYRO_Z_SCALE;
}

void setup()
{
  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  //read eeprom
  readEEPROM();

  // Init serial output
  Serial.begin(INPUT_BAUD_RATE);
  Bluetooth.begin(OUTPUT_BAUD_RATE);
  delay(200);
#ifdef BT_BAUD_COMMAND
  Bluetooth.print(BT_BAUD_COMMAND);
  delay(1000);
#endif //BT_BAUD_COMMAND
  Bluetooth.println("\r\n[S]Start v"FIRMWARE_VERSION);

  // Init sensors
  Wire.begin();
  delay(200);  // Give sensors enough time to start
  currMillis = millis();  
  Accel_Init();
  Magn_Init();
  Gyro_Init();
  Pressure_Init();

  // Read sensors, init DCM algorithm
  delay(200);  // Give sensors enough time to collect data
  reset_sensor_fusion();
  Bluetooth.println("[S]OK");
}

// Main loop
void loop()
{
  currMillis = millis();
  // Time to read the sensors again?
  if ((currMillis - heartstamp) >= HEARTBEAT_INTERVAL) {
    Bluetooth.println("[H]");
    heartstamp = currMillis;
  }
  
  if ((currMillis - timestamp) >= OUTPUT_DATA_INTERVAL) {
    //Serial.write(".");
    timestamp_old = timestamp;
    timestamp = currMillis;
    if (timestamp > timestamp_old)
      G_Dt = (float) (timestamp - timestamp_old) / 1000.0f; // Real time of loop run. We use this on the DCM algorithm (gyro integration time)
    else
      G_Dt = 0;

    ReadSensors();

    // Run DCM algorithm
    Compass_Heading(); // Calculate magnetic heading
    Matrix_update();
    Normalize();
    Drift_correction();
    Euler_angles();

    if (continiousMode)
    {
      heartstamp = currMillis; //data gönderirsek heartbeat atma
      Bluetooth.print("[O]");
      printReading();
    }

    if (rawMode) {
      heartstamp = currMillis; //data gönderirsek heartbeat atma
      Bluetooth.print("[C]");
      Bluetooth.print("A=");
      Bluetooth.print((float)accel_mapped[0], 0); //X
      Bluetooth.print(";");
      Bluetooth.print((float)accel_mapped[1], 0); //Y
      Bluetooth.print(";");
      Bluetooth.print((float)accel_mapped[2], 0); //Z
      Bluetooth.print(" M=");
      Bluetooth.print((float)magnetom_mapped[0], 0);
      Bluetooth.print(";");
      Bluetooth.print((float)magnetom_mapped[1], 0);
      Bluetooth.print(";");
      Bluetooth.print((float)magnetom_mapped[2], 0);
      Bluetooth.print(" G=");
      Bluetooth.print((float)gyro_mapped[0], 0);
      Bluetooth.print(";");
      Bluetooth.print((float)gyro_mapped[1], 0);
      Bluetooth.print(";");
      Bluetooth.print((float)gyro_mapped[2], 0);
      Bluetooth.print(" P=");
      Bluetooth.print(temperature, 0);
      Bluetooth.print(";");
      Bluetooth.print(pressure, 2);
      Bluetooth.print(";");
      Bluetooth.print(altitude, 2);
      Bluetooth.println();
    }
  }

  char serialChar;
  int serialLoop;
  int bytes = Serial.available();
  if (bytes > 0) {
    for (serialLoop = 0; serialLoop < bytes; serialLoop++)
    {
      serialChar = Serial.read();
      if (serialChar == -1)
        break;
      if (serialChar == '\r' || serialChar == '\n') {
        processCommand(laserBuffer);
        laserBuffer = "";
      }
      else
        laserBuffer += serialChar;
    }
  }

  bytes = Bluetooth.available();
  if (Bluetooth.available()) {
    heartstamp = currMillis; //bluetooth'tan data gelirse de heartbeat atma
    for (serialLoop = 0; serialLoop < bytes; serialLoop++)
    {
      serialChar = Bluetooth.read();
      if (serialChar == -1)
        break;
      if (serialChar == '\r' || serialChar == '\n') {
        processCommand(bluetoothBuffer);
        bluetoothBuffer = "";
      }
      else
        bluetoothBuffer += serialChar;
    }
  }
}

void processCommand(String command)
{
  if (command.equals("\r") || command.equals("\n")) return;
  char numbuf[24]; // make this at least big enough for the whole string  
  int pos;
  String subcommand;
  
  digitalWrite(LED_PIN, 1);

  if (command.startsWith("Dist: "))
  {
    //command from lasermeter
    pos = command.indexOf(',');
    if (pos > -1)
    {
      subcommand = command.substring(6, pos);
      subcommand.toCharArray(numbuf, sizeof(numbuf));
      distance = atof(numbuf) / 1000.0f;
      Bluetooth.print("[R]");
      printReading();
    }
  }
  else if (command.equalsIgnoreCase("read"))
  {
    Serial.print("*11114#");
  }
  else if (command.startsWith("cmd "))
  {
    subcommand = command.substring(4);
    Serial.print(subcommand);
  }
  else if (command.equalsIgnoreCase("test"))
  {
    //distance = 1000;
    Bluetooth.print("[O]");
    printReading();
  }
  else if (command.equalsIgnoreCase("reset"))
  {
    setEEPROMDirty();
    SoftReset();
  }
  else if (command.equalsIgnoreCase("reboot"))
  {
    SoftReset();
  }
  else if (command.equalsIgnoreCase("param"))
  {    
    for (int i = 0; i < PARAMS_MAX; i++)
    {
      Bluetooth.print("[P]");
      Bluetooth.print(i+1);
      Bluetooth.print("=");
      Bluetooth.print(parameters[i], 6);
      Bluetooth.println();
    }
  }
  else if (command.startsWith("cont "))
  {
    subcommand = command.substring(5);
    if (subcommand.equalsIgnoreCase("on"))
    {
      continiousMode = true;
    }
    else
    {
      continiousMode = false;
      Bluetooth.println("[D]End cont");
    }
  }
  else if (command.startsWith("calib "))
  {
    subcommand = command.substring(6);
    if (subcommand.equalsIgnoreCase("on"))
    {
      rawMode = true;
    }
    else if (subcommand.equalsIgnoreCase("save"))
    {
        writeEEPROM();
    }
    else
    {
      rawMode = false;
      Bluetooth.println("[D]End calib");
    }
  }
  else if (command.startsWith("set "))
  {
    subcommand = command.substring(4);
    pos = subcommand.indexOf('=');
    if (pos > -1)
    {
      String szIndex = subcommand.substring(0, pos);
      String szValue = subcommand.substring(pos + 1);

      szIndex.toCharArray(numbuf, sizeof(numbuf));
      pos = atoi(numbuf);
      szValue.toCharArray(numbuf, sizeof(numbuf));
      float value = atof(numbuf);
      if (pos > 0 && pos <= PARAMS_MAX)
      {
        parameters[pos-1] = value;
        Bluetooth.print("[D]");
        Bluetooth.print(pos);
        Bluetooth.print("=");
        Bluetooth.print(value, 6);
        Bluetooth.println();
      }
      else
      {
        Bluetooth.print("[E]Inv param idx ");
        Bluetooth.print(pos);
        Bluetooth.println();
      }
    }
  }
  digitalWrite(LED_PIN, 0);
}

void printReading() {
  Bluetooth.print("1=");
  Bluetooth.print(distance);
  Bluetooth.print(";");
  Bluetooth.print("2=");
  Bluetooth.print(180.0f + TO_DEG(yaw), 2);
  Bluetooth.print(";");
  Bluetooth.print("3=");
  Bluetooth.print(-TO_DEG(pitch), 2);
  Bluetooth.print(";");
  Bluetooth.print("4=");
  Bluetooth.print(TO_DEG(roll), 2);
  Bluetooth.print(";");
  Bluetooth.print("5=");
  Bluetooth.print(temperature, 0);
  Bluetooth.print(";");
  Bluetooth.print("6=");
  Bluetooth.print(pressure, 2);
  Bluetooth.print(";");
  Bluetooth.print("7=");
  Bluetooth.print(altitude, 2);
  Bluetooth.println();
}





