idf_component_register(SRCS "VL53L4CD_ULD_Driver/VL53L4CD_api.c" "VL53L4CD_ULD_Driver/VL53L4cd_calibration.c" "platform/platform.c"
    INCLUDE_DIRS "VL53L4CD_ULD_Driver" "platform" 
    REQUIRES "driver" "freertos"
)