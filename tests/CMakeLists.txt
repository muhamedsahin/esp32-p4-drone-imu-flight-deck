cmake_minimum_required(VERSION 3.22)
project(imu_calibration_tests LANGUAGES CXX)
enable_testing()
add_executable(imu_calibration_tests
    calibration_tests.cpp
    ../components/imu/imu_calibration.cpp
    ../components/imu/imu_calibration_record.cpp
    ../components/imu/imu_calibration_text.cpp
)
target_include_directories(imu_calibration_tests PRIVATE ../components/imu/include)
target_compile_features(imu_calibration_tests PRIVATE cxx_std_17)
if(MSVC)
    target_compile_options(imu_calibration_tests PRIVATE /W4 /WX /utf-8 /EHsc)
else()
    target_compile_options(imu_calibration_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME calibration_math_and_record COMMAND imu_calibration_tests)

# Aynı sürücü/oturum/storage kodu, simüle I2C/FreeRTOS/NVS ile çalıştırılır.
add_executable(imu_driver_tests
    driver_tests.cpp fakes/fake_esp.cpp
    ../components/imu/mpu6050.cpp
    ../components/imu/mpu6050_calibration.cpp
    ../components/imu/mpu6050_storage.cpp
    ../components/imu/imu_calibration.cpp
    ../components/imu/imu_calibration_record.cpp
    ../components/imu/imu_calibration_text.cpp
    ../components/imu/mpu6050_text_calibration.cpp
    ../components/imu/imu_text_store.cpp
)
target_include_directories(imu_driver_tests PRIVATE fakes ../components/imu/include)
target_compile_features(imu_driver_tests PRIVATE cxx_std_17)
if(MSVC)
    target_compile_options(imu_driver_tests PRIVATE /W4 /WX /utf-8 /EHsc)
else()
    target_compile_options(imu_driver_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME calibration_driver_and_workflow COMMAND imu_driver_tests)

add_executable(imu_text_tests text_workflow_tests.cpp fakes/fake_esp.cpp
    ../components/imu/mpu6050.cpp ../components/imu/mpu6050_calibration.cpp
    ../components/imu/mpu6050_storage.cpp ../components/imu/mpu6050_text_calibration.cpp
    ../components/imu/imu_calibration.cpp ../components/imu/imu_calibration_record.cpp
    ../components/imu/imu_calibration_text.cpp ../components/imu/imu_text_store.cpp)
target_include_directories(imu_text_tests PRIVATE fakes ../components/imu/include)
target_compile_features(imu_text_tests PRIVATE cxx_std_17)
if(MSVC)
    target_compile_options(imu_text_tests PRIVATE /W4 /WX /utf-8 /EHsc)
else()
    target_compile_options(imu_text_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME calibration_text_and_partial_resume COMMAND imu_text_tests)

add_executable(attitude_tests
    attitude_tests.cpp
    ../components/attitude/attitude_estimator.cpp
    ../components/attitude/quaternion.cpp)
target_include_directories(attitude_tests PRIVATE
    ../components/attitude/include ../components/imu/include)
target_compile_features(attitude_tests PRIVATE cxx_std_17)
if(MSVC)
    target_compile_options(attitude_tests PRIVATE /W4 /WX /utf-8 /EHsc)
else()
    target_compile_options(attitude_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME attitude_accelerometer_foundation COMMAND attitude_tests)

# API, gerçek IMU/attitude/kalibrasyon koduyla; yalnız ESP donanımı simüle edilir.
add_executable(orientation_system_tests
    orientation_system_tests.cpp fakes/fake_esp.cpp
    ../components/orientation/orientation_system.cpp
    ../components/orientation/imu_application.cpp
    ../components/attitude/attitude_estimator.cpp
    ../components/attitude/quaternion.cpp
    ../components/imu/mpu6050.cpp
    ../components/imu/mpu6050_calibration.cpp
    ../components/imu/mpu6050_storage.cpp
    ../components/imu/mpu6050_text_calibration.cpp
    ../components/imu/imu_calibration.cpp
    ../components/imu/imu_calibration_record.cpp
    ../components/imu/imu_calibration_text.cpp
    ../components/imu/imu_text_store.cpp)
target_include_directories(orientation_system_tests PRIVATE
    fakes ../components/orientation/include
    ../components/attitude/include ../components/imu/include)
target_compile_features(orientation_system_tests PRIVATE cxx_std_17)
if(MSVC)
    target_compile_options(orientation_system_tests PRIVATE /W4 /WX /utf-8 /EHsc)
else()
    target_compile_options(orientation_system_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME orientation_api_and_sensor_workflow COMMAND orientation_system_tests)

# ESP donanımı gerektirmeyen yeni Wi-Fi TXT sözleşmesi.
add_executable(network_config_tests network_config_tests.cpp
    ../components/drone_web/network_config.cpp)
target_include_directories(network_config_tests PRIVATE ../components/drone_web/private)
target_compile_features(network_config_tests PRIVATE cxx_std_17)
if(MSVC)
    target_compile_options(network_config_tests PRIVATE /W4 /WX /utf-8 /EHsc)
else()
    target_compile_options(network_config_tests PRIVATE -Wall -Wextra -Werror)
endif()
add_test(NAME drone_web_wifi_configuration COMMAND network_config_tests)
