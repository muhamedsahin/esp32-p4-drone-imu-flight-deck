# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "app.js.S"
  "bootloader\\bootloader.bin"
  "bootloader\\bootloader.elf"
  "bootloader\\bootloader.map"
  "calibration.txt.S"
  "config\\sdkconfig.cmake"
  "config\\sdkconfig.h"
  "esp-idf\\mbedtls\\x509_crt_bundle"
  "flash_app_args"
  "flash_bootloader_args"
  "flash_project_args"
  "flasher_args.json"
  "flasher_args.json.in"
  "index.html.S"
  "ldgen_libraries"
  "ldgen_libraries.in"
  "math.js.S"
  "mpu6050.bin"
  "mpu6050.map"
  "project_elf_src_esp32p4.c"
  "renderer.js.S"
  "styles.css.S"
  "wifi.txt.S"
  "x509_crt_bundle.S"
  )
endif()
