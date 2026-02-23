# Frame of reference for angles sent by the user.
uint8 BODY=0   # X (roll) forward, Y (pitch) left, Z (yaw) up.
uint8 GLOBAL=1 # Yaw is assumed to be compass heading in NED (0-360 deg) with 0 pointing North.

# Camera optical channel.
uint8 RBG=0
uint8 INFRARED=1

# TODO: resolution enum.

uint8 frame # [0,1]

float64 roll  # [deg]
float64 pitch # [deg]
float64 yaw   # [deg]

# Point of interest (POI).
geographic_msgs/GeoPoint poi

uint8 channel    # [0,1]
uint8 resolution # TODO
