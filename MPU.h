#include "i2c_t3.h"
#include "I2Cdev.h"
#include "VectorKalmanFilter.h"
#include "QuaternionKalmanFilter.h"
#include "Rotation.h"
#include "MPU6050.h"

typedef struct MPU {
  //Used for MPU9 3D/4D Quaternian math.
  /*
    VectorKalmanFilter mpuVKF = VectorKalmanFilter(0.1, 40);
    QuaternionKalmanFilter magKF = QuaternionKalmanFilter(0.6,10);
    QuaternionKalmanFilter mpuKF = QuaternionKalmanFilter(0.6,10);
  */
    uint8_t address;
    uint8_t devStatus;
    uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;     // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64]; // FIFO storage buffer
    int16_t d[6];
    uint8_t sda = 19, scl = 18, gAcc, aAcc;
    MPU6050 dev;

    MPU(uint8_t address, uint8_t gAcc, uint8_t aAcc, uint8_t sda = 19, uint8_t scl = 18) {
      this->address = address;
      this->sda = sda;
      this->scl = scl;

      this->dev = MPU6050(address);
    }

    void setAccuracy(uint8_t gAcc, uint8_t aAcc){
      selectMPU();
      
      dev.setFullScaleGyroRange(gAcc);
      dev.setFullScaleAccelRange(aAcc);
    }

    void setGyroAccuracy(uint8_t gAcc){
      selectMPU();
      
      dev.setFullScaleGyroRange(gAcc);
    }

    void setAccelAccuracy(uint8_t aAcc){
      selectMPU();
      
      dev.setFullScaleAccelRange(aAcc);
    }

    void selectMPU() {
      Wire.pinConfigure(sda, scl);
    }

    void initMPU() {
      selectMPU();

      dev.initialize();
      Serial.println(dev.testConnection() ? "MPU connection successful" : "MPU connection failed");

      dev.resetSensors();
      
      dev.resetGyroscopePath();
      dev.resetAccelerometerPath();
      dev.resetTemperaturePath();

      dev.setAccelerometerPowerOnDelay(0);

      dev.setFIFOEnabled(false);

      dev.setDHPFMode(0);
      dev.setSlaveEnabled(0, false);
      dev.setFSyncInterruptEnabled(false);
      dev.setI2CBypassEnabled(false);
      dev.setClockOutputEnabled(false);
      dev.setIntFreefallEnabled(false);
      dev.setIntMotionEnabled(false);
      dev.setIntZeroMotionEnabled(false);
      dev.setIntFIFOBufferOverflowEnabled(false);
      dev.setIntI2CMasterEnabled(false);
      dev.setIntDataReadyEnabled(false);
      dev.setIntEnabled(false);
      dev.setSleepEnabled(false);
      dev.setWakeCycleEnabled(false);
      dev.setTempSensorEnabled(false);
      dev.setClockSource(0);

      setAccuracy(gAcc, aAcc);

      dev.setRate(0);
      
      dev.setXGyroOffset(0);  dev.setYGyroOffset(0);  dev.setZGyroOffset(0);
      dev.setXAccelOffset(0); dev.setYAccelOffset(0); dev.setZAccelOffset(0);
    }

    //Not Currently working/Used. 3D/4D math for MPU9
/*
    Vector3D getAccelVector(){
      Vector3D filtered = mpuVKF.Filter(Vector3D(d[0], d[1], d[2])) / 16834.0f;

      filtered.Constrain(-1.0, 1.0);
      //filtered.UnitSphere();

      filtered.X *= -1.0;

      filtered = filtered.Add((double)1.0);
      filtered = filtered.Multiply((double)(1023.0 / 2.0));

      filtered.Y+= 461;

      return filtered;
    }

    Vector3D getThrottle(Vector3D collective){
      Quaternion mag = magKF.Filter(Rotation(Vector3D(-0.70, -0.625, -0.23).UnitSphere(), Vector3D(d[3], d[4], d[5]).UnitSphere()).GetQuaternion().UnitQuaternion());//filtered magnetometer as a rotation from north vector
      Quaternion acc = mpuKF.Filter(Rotation(Vector3D(0, 0, -16834).UnitSphere(),         Vector3D(d[0], d[1], d[2]).UnitSphere()).GetQuaternion().UnitQuaternion());//filtered accel as a rotation from gravity

      Vector3D magEA = Rotation(mag).GetEulerAngles(EulerConstants::EulerOrderZXYR).Angles;//convert quaternion to yaw, pitch and roll
      Vector3D accEA = Rotation(acc).GetEulerAngles(EulerConstants::EulerOrderZXYR).Angles;//convert quaternion to yaw, pitch and roll

      Quaternion fusedQuat = Rotation(EulerAngles(Vector3D(magEA.X, accEA.Y, accEA.Z), EulerConstants::EulerOrderZXYR)).GetQuaternion().UnitQuaternion();//mpu9 fused rotation
      Quaternion coll = Rotation(Vector3D(0, 16384, 0).UnitSphere(), collective.UnitSphere()).GetQuaternion().UnitQuaternion();//collective rotation

      Quaternion rotatedFused = fusedQuat * coll.Conjugate();//remove collective rotation from mpu9 fused rotation

      Vector3D base = Vector3D(0, 0, 1);

      return rotatedFused.RotateVector(base);
    }
*/

    Vector3D readMPU() {
      selectMPU();
      dev.getAcceleration(&d[0], &d[1], &d[2]);

      return Vector3D(d[0], d[1], d[2]);
    }
    
//Used in 3D/4D math for MPU9
/*
    void readFusedMPU(Vector3D *mag, Vector3D *acc){
      int16_t del;
      
      selectMPU();
      dev.getMotion9(&d[0], &d[1], &d[2], &del, &del, &del, &d[4], &d[5], &d[6]);

      mag->X = d[0];
      mag->Y = d[1];
      mag->Z = d[2];

      acc->X = d[3];
      acc->Y = d[4];
      acc->Z = d[5];
    }
*/
} MPU;
