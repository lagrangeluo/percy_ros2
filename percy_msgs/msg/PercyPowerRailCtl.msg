#Power Rail Control instructions

# #byte[0]
# uint16 EXTERNAL_48V = 0x0001
# uint16 EXTERNAL_5V = 0x0002
# uint16 FAN_12V = 0x0004

# #byte[1]
# uint16 JETSON1_12V = 0x0100
# uint16 JETSON2_12V = 0x0200
# uint16 ULTRASONIC_SENSOR_12V = 0x0400
# uint16 CAMERA_12V = 0x0800
# uint16 ROUTER_5G_12V = 0x1000
# uint16 SWITCHBOARD_12V = 0x2000
# uint16 USB_HUB_12V = 0x4000
# uint16 SICK_SYSTEM_24V = 0x8000

bool external_48v
bool external_5v
bool fan_12v

bool jetson1_12v
bool jetson2_12v
bool ultrasonic_sensor_12v
bool camera_12v
bool router_5g_12v
bool switchboard_12v
bool usb_hub_12v
bool sick_system_24v