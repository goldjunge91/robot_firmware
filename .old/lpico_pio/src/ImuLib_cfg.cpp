/**
 * @file ImuLib_cfg.cpp
 * @author Maciej Kurcius
 * @brief
 * @version 0.1
 * @date 2022-02-15
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "ImuLib_cfg.h"
#include <Wire.h>
#include "bsp.h"

#define DEGREESPERSEC_TO_RADPERSEC 0.017453293

extern TwoWire I2cBus;

ImuDriver ImuBno(IMU_ID, IMU_ADDR_B, &I2cBus);

ImuDriver::ImuDriver(uint8_t ImuId_, uint8_t ImuAddr_, TwoWire *ImuWire_)
{
  this->ImuBno = new Adafruit_BNO055(ImuId_, ImuAddr_, ImuWire_);
}

ImuDriver::~ImuDriver() { ; }

bool ImuDriver::Init()
{
  // OPERATION_MODE_IMUPLUS fuses accelerometer and gyroscope data for
  // orientation
  if (this->ImuBno->begin(OPERATION_MODE_IMUPLUS))
  {
    this->ImuBno->setAxisRemap(Adafruit_BNO055::REMAP_CONFIG_P1);
    this->ImuBno->setAxisSign(Adafruit_BNO055::REMAP_SIGN_P4);
    this->ImuBno->setExtCrystalUse(true);
    return false;
  }
  return true;
}

imu_queue_t ImuDriver::LoopHandler()
{
  imu_queue_t ImuQueue;
  imu::Quaternion Quaternion;

  // Get accelerometer data
  imu::Vector<3> accel = this->ImuBno->getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  ImuQueue.LinearAcceleration[0] = (float)accel.x();
  ImuQueue.LinearAcceleration[1] = (float)accel.y();
  ImuQueue.LinearAcceleration[2] = (float)accel.z();

  // Get gyroscope data
  imu::Vector<3> gyro = this->ImuBno->getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  // by default angular velocity is in deg/s
  ImuQueue.AngularVelocity[0] = (float)gyro.x() * DEGREESPERSEC_TO_RADPERSEC;
  ImuQueue.AngularVelocity[1] = (float)gyro.y() * DEGREESPERSEC_TO_RADPERSEC;
  ImuQueue.AngularVelocity[2] = (float)gyro.z() * DEGREESPERSEC_TO_RADPERSEC;

  Quaternion = this->ImuBno->getQuat();
  ImuQueue.Orientation[0] = Quaternion.x();
  ImuQueue.Orientation[1] = Quaternion.y();
  ImuQueue.Orientation[2] = Quaternion.z();
  ImuQueue.Orientation[3] = Quaternion.w();
  return ImuQueue;
}
