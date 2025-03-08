#include "vl53l5cx_api.h"
#include "debug.h"
#include <stdbool.h>
#include <string.h>
#include <stdint.h>
bool config_sensors(VL53L5CX_Configuration *p_dev, uint16_t new_i2c_address);
bool initialize_sensors_I2C(VL53L5CX_Configuration *p_dev, uint8_t mode);
bool get_sensor_data(VL53L5CX_Configuration *p_dev,VL53L5CX_ResultsData *p_results);
