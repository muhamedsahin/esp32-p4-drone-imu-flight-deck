#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "orientation_system.hpp"
#include "drone_dashboard.hpp"

// Web/ağ ayrıntıları component içinde; ana dosya yalnız sonucu yayımlar.
static DroneDashboard dashboard;

void imuTask(void *)
{
    // Sürücü, dosya kaydı ve rapor aynı görevden kullanılır.
    {
        OrientationSystem orientation;
        unsigned print_divider = 0;
        if (orientation.init())
        {
            while (true)
            {
                const bool updated = orientation.update();
                dashboard.publish(orientation.getOrientation());
                if (updated)
                {
                    // Filtre ~100 Hz çalışır; konsolu yalnız ~10 Hz güncelleriz.
                    if (++print_divider >= 10)
                    {
                        print_divider = 0;
                        //orientation.print();
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(10));
            }
        }
    } // Görev silinmeden önce I2C kaynakları serbest bırakılır.
    vTaskDelete(nullptr);
}

extern "C" void app_main()
{
    if (!dashboard.start()) ESP_LOGE("DRONE_WEB", "Web gorevi icin yeterli bellek yok.");
    if (xTaskCreate(imuTask, "imu", 12288, nullptr, 5, nullptr) != pdPASS){
        ESP_LOGE("IMU_APP", "IMU gorevi icin yeterli bellek yok.");
    }
}
