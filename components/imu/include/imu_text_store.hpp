#pragma once

#include "imu_calibration_text.hpp"

namespace imu_calibration
{
/** PC dosyasının derlemeye gömülen kopyası + karttaki son metin kaydı.
 * Kaynak dosya değişmediyse NVS'deki daha yeni ölçümler korunur.
 * Kaynak içeriği değişmişse düzenlenen dosya bir kez içeri alınır.
 */
class TextStore
{
  public:
    bool load(const std::string &source, TextCalibration &state, std::string &error);
    bool save(const TextCalibration &state);
    static bool checkpoint(const TextCalibration &state, void *context)
    {
        return static_cast<TextStore *>(context)->save(state);
    }

  private:
    uint32_t source_checksum_ = 0;
};
} // namespace imu_calibration
