idf_component_register(SRCS "main.c"
                            "i2c_bus.c"
                            "sensor_manager.c"
                            "wifi_link.c"
                            "sensors/sensor_imu_6axis.c"
                    INCLUDE_DIRS "."
                                 "sensors"
                    REQUIRES driver)
