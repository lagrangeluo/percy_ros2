std_msgs/Header header

int8 MOTOR_ID_FRONT_RIGHT = 0
int8 MOTOR_ID_FRONT_LEFT = 1
int8 MOTOR_ID_REAR_RIGHT = 2
int8 MOTOR_ID_REAR_LEFT = 3

int8 LIGHT_ID_FRONT = 0
int8 LIGHT_ID_REAR = 1

# system state feedback
uint8 vehicle_state
uint8 control_mode
uint16 error_code

uint8 fan_1_percentage
uint8 fan_2_percentage
uint8 fan_3_percentage
uint8 fan_4_percentage

# motion state feedback
float64 linear_velocity
float64 angular_velocity

#light state feedback
uint8 light_control_state
uint8 front_light_mode
uint8 back_light_mode
PercyRgb front_left
PercyRgb front_right
PercyRgb back_left
PercyRgb back_right

#remote data feedback 
PercyRCState rc_state

# motor state
#TODO
PercyActuatorState[2] actuator_states

#sensor state
PercyBmsState bms_state

uint8 powerbutton

uint32 wheel_circumference
uint32 wheel_track