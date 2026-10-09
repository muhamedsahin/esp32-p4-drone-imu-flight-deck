#pragma once

#include "attitude_types.hpp"
#include "imu_types.hpp"
#include "quaternion.hpp"

/** Kalibre edilmiş IMU verisinden gövde yönelimini hesaplar. */
class AttitudeEstimator
{
public:
  /**
   * Sabit veya yavaş hareket eden sensörde yerçekiminden roll/pitch hesaplar.
   * Doğrusal ivme sırasında sonuç eğim değil, toplam ivme yönüdür.
   */
  AccelAngles calculateAccelAngles(const IMUData &data) const;
  /** İlk güvenilir ivme yönünden roll/pitch başlatılır; yaw referansı sıfırdır. */
  bool gyroinitialize(const IMUData &data);
  /** Normal kullanım: gyro takibi + güvenilir ivmeyle eğim düzeltmesi. */
  bool update(const IMUData& data);
  /** Karşılaştırma/öğrenme için yalnız gyro ile takip. */
  bool updateGyro(const IMUData& data);
  bool isInitialized() const;
  EulerAngles getAngles() const;
  const Quaternion& getQuaternion() const;
  // Eski çağrılar korunur; güncel quaternion'dan çıkarılan açıları yazdırır.
  void printGyroAngles() const;

private:
  bool updateOrientation(const IMUData& data, bool use_accelerometer);

  // Zaman yönetimi estimator'da, quaternion matematiği QuaternionUtils'tedir.
  bool initialized_ = false;
  int64_t last_timestamp_us_ = 0;
  QuaternionUtils orientation_;
};
