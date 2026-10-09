idf_component_register(
    SRCS
        "orientation_system.cpp"
        "imu_application.cpp"
    INCLUDE_DIRS "include"
    REQUIRES imu attitude
    PRIV_REQUIRES log
)

# Kalibrasyon dosyasının gömülmesi ve yüklenmesi API katmanının sorumluluğudur.
target_add_binary_data(${COMPONENT_LIB} "${PROJECT_DIR}/calibration.txt" TEXT)
