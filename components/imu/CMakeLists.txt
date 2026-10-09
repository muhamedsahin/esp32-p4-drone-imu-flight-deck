idf_component_register(
    SRCS
        "mpu6050.cpp"
        "imu_calibration.cpp"
        "mpu6050_calibration.cpp"
        "imu_calibration_record.cpp"
        "mpu6050_storage.cpp"
        "imu_calibration_text.cpp"
        "mpu6050_text_calibration.cpp"
        "imu_text_store.cpp"
    INCLUDE_DIRS "include"
    REQUIRES esp_driver_i2c
    PRIV_REQUIRES esp_timer nvs_flash
)
