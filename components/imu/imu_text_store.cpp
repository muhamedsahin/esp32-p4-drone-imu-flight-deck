#include "imu_text_store.hpp"

#include "nvs.h"
#include "nvs_flash.h"
#include <cstring>
#include <vector>

namespace imu_calibration
{
namespace
{
constexpr const char *SPACE = "mpu6050_cal";
constexpr const char *KEY = "text_v1";
// Sabit boyutlu başlık; ardından UTF-8 metin. CRC metin ve kaynak kimliğini kapsar.
struct Header
{
    uint32_t version, source, checksum, length;
};
static_assert(sizeof(Header) == 16, "Metin kayit basligi degisti");
bool readBlob(std::vector<uint8_t> &blob, bool &missing)
{
    missing = false;
    nvs_handle_t h = 0;
    esp_err_t err = nvs_open(SPACE, NVS_READONLY, &h);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        missing = true;
        return true;
    }
    if (err != ESP_OK)
        return false;
    // NVS API'sinin size sorgusu yerine sınırlandırılmış tampon; fakes ile de aynı yol.
    blob.resize(8192 + sizeof(Header));
    size_t length = blob.size();
    err = nvs_get_blob(h, KEY, blob.data(), &length);
    nvs_close(h);
    if (err == ESP_ERR_NVS_NOT_FOUND)
    {
        missing = true;
        blob.clear();
        return true;
    }
    if (err != ESP_OK)
        return false;
    blob.resize(length);
    return true;
}
uint32_t checksum(uint32_t source, const std::string &text)
{
    return textChecksum(std::to_string(source) + "\n" + text);
}
} // namespace

bool TextStore::load(const std::string &source, TextCalibration &state, std::string &error)
{
    TextCalibration seed{};
    if (!parseCalibrationText(source, seed, error))
        return false;
    source_checksum_ = textChecksum(calibrationText(seed)); // Yorum değişikliği kalibrasyonu sıfırlamaz.
    if (nvs_flash_init() != ESP_OK)
    {
        error = "NVS acilamadi; otomatik silme yapilmadi.";
        return false;
    }
    std::vector<uint8_t> blob;
    bool missing = false;
    if (!readBlob(blob, missing))
    {
        error = "NVS metin kaydi okunamadi.";
        return false;
    }
    if (!missing)
    {
        Header header{};
        if (blob.size() < sizeof(header))
        {
            error = "NVS metin basligi bozuk.";
            return false;
        }
        std::memcpy(&header, blob.data(), sizeof(header));
        if (header.version != 1 || header.length != blob.size() - sizeof(header))
        {
            error = "NVS metin boyutu/surumu bozuk.";
            return false;
        }
        const std::string saved(reinterpret_cast<const char *>(blob.data() + sizeof(header)), header.length);
        if (header.checksum != checksum(header.source, saved))
        {
            error = "NVS metin CRC hatasi.";
            return false;
        }
        TextCalibration stored{};
        if (!parseCalibrationText(saved, stored, error))
            return false;
        if (stored.identity != seed.identity)
        {
            error = "Dosya ve kart kaydinin sensor profili farkli.";
            return false;
        }
        if (header.source == source_checksum_)
        {
            state = stored;
            return true;
        }
    }
    state = seed;
    applyCalibrationEdits(state);
    if (!save(state))
    {
        error = "Dosya kart hafizasina kaydedilemedi.";
        return false;
    }
    return true;
}

bool TextStore::save(const TextCalibration &state)
{
    const std::string text = calibrationText(state);
    if (text.size() > 8192 || nvs_flash_init() != ESP_OK)
        return false;
    const Header header{1, source_checksum_, checksum(source_checksum_, text),
                        static_cast<uint32_t>(text.size())};
    std::vector<uint8_t> blob(sizeof(header) + text.size());
    std::memcpy(blob.data(), &header, sizeof(header));
    std::memcpy(blob.data() + sizeof(header), text.data(), text.size());
    nvs_handle_t h = 0;
    esp_err_t err = nvs_open(SPACE, NVS_READWRITE, &h);
    if (err == ESP_OK)
    {
        err = nvs_set_blob(h, KEY, blob.data(), blob.size());
        if (err == ESP_OK)
            err = nvs_commit(h);
        nvs_close(h);
    }
    if (err != ESP_OK)
        return false;
    std::vector<uint8_t> check;
    bool missing = false;
    return readBlob(check, missing) && !missing && check == blob;
}
} // namespace imu_calibration
