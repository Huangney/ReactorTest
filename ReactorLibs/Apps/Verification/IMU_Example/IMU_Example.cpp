#include "IMU_Example.hpp"
#include "Monitor.hpp"
#include "spi.h"
#include "bsp_log.hpp"

// 0: 使用阻塞 SPI 测试；1: 使用 DMA SPI 测试
#ifndef IMU_EXAMPLE_USE_DMA
#define IMU_EXAMPLE_USE_DMA 0
#endif

void IMU_Example::Start()
{
  // 配置 SPI 实例: CS1_Accel -> PA4, CS1_Gyro -> PB0
  spi_acc_inst_.Init((BSP::SPI::SpiID)&hspi1, {'A', 4});
  spi_gyro_inst_.Init((BSP::SPI::SpiID)&hspi1, {'B', 0}); // 假设加速度计和陀螺仪共用SPI1，仅CS不同

  // 初始化 BMI088 (包含自动标定)
  imu_.Init(&spi_acc_inst_, &spi_gyro_inst_);

  Monitor &monit = Monitor::GetInstance();
  monit.Track(imu_.gyro[0]);
  monit.Track(imu_.gyro[1]);
  monit.Track(imu_.gyro[2]);

#if IMU_EXAMPLE_USE_DMA
  BspLog_LogInfo("IMU_Example start in DMA SPI mode.");
#else
  BspLog_LogInfo("IMU_Example start in normal SPI mode.");
#endif
}

void IMU_Example::Update()
{
  Monitor &monit = Monitor::GetInstance();

  // 打印初始化状态
  if (!imu_.IsOnline())
  {
    monit.LogWarning(
        "IMU is offline! Init Failed. Acc_Err:0x%02X, Gyro_Err:0x%02X",
        imu_.GetInitError() >> 8, imu_.GetInitError() & 0xFF);
    return;
  }

  // 周期性更新数据
#if IMU_EXAMPLE_USE_DMA
  // 最小可用 DMA 测试链路：触发并解析陀螺仪与加速度计数据
  imu_.TriggerAccelDMA();
  imu_.ParseAccelDMA();
  imu_.TriggerGyroDMA();
  imu_.ParseGyroDMA();
#else
  imu_.Update();
#endif

  tick_cnt_++;
  if (tick_cnt_ > 9)
  {
    // 打印陀螺仪数据进行测试
    const float *gyro = imu_.GetGyro();
    monit.LogInfo("IMU_Gyro, x:%.3f, y:%.3f, z:%.3f", gyro[0], gyro[1], gyro[2]);
    tick_cnt_ = 0;
  }
}
