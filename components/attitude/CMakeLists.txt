idf_component_register(
    SRCS
        "quaternion.cpp"
        "attitude_estimator.cpp"
    INCLUDE_DIRS "include"
    REQUIRES imu
)
