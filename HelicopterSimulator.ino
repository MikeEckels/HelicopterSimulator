#include "MPU.h"

#define SDA0 19
#define SCL0 18
#define SDA1 16
#define SCL1 17

bool timePrint = false;
bool fullPrint = false;
bool rawPrint = false;
unsigned long timeMicros;

/*
 ####PRACTICE SIM####
   MPU0: Collective      X: Y: Z:
   MPU1: Throttle (MPU9) X: Y: Z:
   MPU2: Cyclic          X: Y: Z:
   External YAW control using pedals
   
 ####HELLICOPTER####
   MPU0: Collecttive/Throttle      X:N/A  Y:Throttle   Z:Collective
   MPU1: Cyclic                    X:N/A  Y:Pitch/Roll Z:N/A
   MPU2: Rudder                    X:YAW  Y:N/A        Z:N/A
 */

MPU mpu0 = MPU(0x68, MPU6050_GYRO_FS_250, MPU6050_ACCEL_FS_2, SDA0, SCL0);
MPU mpu1 = MPU(0x69, MPU6050_GYRO_FS_250, MPU6050_ACCEL_FS_2, SDA0, SCL0);
MPU mpu2 = MPU(0x68, MPU6050_GYRO_FS_250, MPU6050_ACCEL_FS_2, SDA1, SCL1);

VectorKalmanFilter collectiveThrottleKF = VectorKalmanFilter(0.05, 40);
VectorKalmanFilter cyclicKF = VectorKalmanFilter(0.05, 40);
VectorKalmanFilter yawKF = VectorKalmanFilter(0.05, 40);

void setup() {
  pinMode(SCL1, OUTPUT);
  pinMode(SDA1, INPUT);

  Wire.begin(I2C_MASTER, 0x00, SDA0, SCL0);
  Wire.setClock(400000);

  Serial.end();
  Serial.begin(1000000);

  mpu0.initMPU();
  mpu1.initMPU();
  mpu2.initMPU();
}

void parseCommand(String value) {
  if (value.equals("FullPrint:True")) {
    fullPrint = true;
    Serial.println("FILTERED_PRINT_ENABLED");
  }
  else if (value.equals("FullPrint:False")) {
    fullPrint = false;
    Serial.println("FILTERED_PRINT_DISABLED");
  }
  else if (value.equals("RawPrint:True")) {
    rawPrint = true;
    Serial.println("RAW_PRINT_ENABLED");
  }
  else if (value.equals("RawPrint:False")) {
    rawPrint = false;
    Serial.println("RAW_PRINT_DISABLED");
  }
  else if (value.equals("TimePrint:True")) {
    timePrint = true;
    Serial.println("TIME_PRINT_TRUE");
  }
  else if (value.equals("TimePrint:False")) {
    timePrint = false;
    Serial.println("TIME_PRINT_FALSE");
  }
  else {
    Serial.println("COMMAND_INVALID");
  }
}

void loop() {
 
  if (Serial.available() > 0) {
    parseCommand(Serial.readStringUntil('\n'));
  }

  timeMicros = micros();

  Vector3D collThrottle = mpu0.readMPU();
  Vector3D cyclic       = mpu1.readMPU();
  Vector3D yaw          = mpu2.readMPU();

  collThrottle = collectiveThrottleKF.Filter(collThrottle.Add(32768).Divide(64));
  cyclic       = cyclicKF.Filter(cyclic.Add(32768).Divide(64));
  yaw          = yawKF.Filter(yaw.Add(32768).Divide(64));


  Joystick.X(collThrottle.Y);
  Joystick.Y(collThrottle.Z);
  Joystick.Z(cyclic.Z);
  Joystick.Zrotate(yaw.X);
  Joystick.sliderLeft(cyclic.Y);

  if(!rawPrint){
    if (!fullPrint){
      Serial.print(cyclic.Y);
      Serial.print(",");
      Serial.print(cyclic.Z);
      Serial.print("\t");
      Serial.print(collThrottle.Z);
      Serial.print("\t");
      Serial.print(collThrottle.Y);
      Serial.print("\t");
      Serial.print(yaw.X);
    }else{
      Serial.print(cyclic.ToString());
      Serial.print("\t");
      Serial.print(collThrottle.ToString());
      Serial.print("\t");
      Serial.print(yaw.ToString());
    }
  }else {
    if (rawPrint){
      //Print Raw MPU Readings for debug. 
      //Add later
    }
  }
  
  Serial.println();
  Serial.flush();

  if (timePrint) {
    Serial.print(micros() - timeMicros);
    Serial.print("\t");
  }
}
