#define DRIVER_STATE_INPUT_VOLTAGE_LOW_MASK ((uint8_t)0x01)
#define DRIVER_STATE_MOTOR_OVERHEAT_MASK ((uint8_t)0x02)
#define DRIVER_STATE_DRIVER_OVERLOAD_MASK ((uint8_t)0x04)
#define DRIVER_STATE_DRIVER_OVERHEAT_MASK ((uint8_t)0x08)
#define DRIVER_STATE_SENSOR_FAULT_MASK ((uint8_t)0x10)
#define DRIVER_STATE_DRIVER_FAULT_MASK ((uint8_t)0x20)
#define DRIVER_STATE_DRIVER_ENABLED_MASK ((uint8_t)0x40)
#define DRIVER_STATE_DRIVER_RESET_MASK ((uint8_t)0x80)

uint8 motor_id
int16 rpm
float64 current
int32 position

float32 driver_voltage
uint8 error_code