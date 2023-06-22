#pdk_SensorMsgHeader t_header
Header header

uint32 u_ObjId # Object ID

float32 f_DistX # X-position in AUTOSAR vehicle coordinate system [m]
float32 f_DistY # Y-position in AUTOSAR vehicle coordinate system [m]

float32 f_VrelX # Relative velocity in x-direction [m/s]
float32 f_VrelY # Relative velocity in y-direction [m/s]

float32 f_ArelX # Relative acceleration in x-direction [m/s^2]
float32 f_ArelY # Relative acceleration in y-direction [m/s^2]

float32 f_DistXStd # Standard deviation of x-position [m]
float32 f_DistYStd # Standard deviation of y-position [m]

float32 f_VrelXStd # Standard deviation of relative velocity in x direction [m/s]
float32 f_VrelYStd # Standard deviation of relative velocity in y direction [m/s]

float32 f_ArelXStd # Standard deviation of relative acceleration in x direction [m/s^2]
float32 f_ArelYStd # Standard deviation of relative acceleration in y direction [m/s^2]

float32 f_LDeltaX_left # X-distance between reference point and bounding box (left, mid, right) [m]
float32 f_LDeltaX_mid 
float32 f_LDeltaX_right

float32 f_LDeltaY_left # Y-distance between reference point and bounding box (left, mid, right) [m]
float32 f_LDeltaY_mid
float32 f_LDeltaY_right

# LDeltaQual e_LDeltaQual

float32 f_RCS # Object RCS [dBm^2]
float32 f_ObjectScore # Object score, range: 0 - 1


uint32 u_LifeCycles # Number of life cycles, range: 0 - 65534

# DynamicProperty e_DynamicProperty

# ObjState e_ObjState

float32 f_VabsX # Absolute velocity in x-direction [m/s]
float32 f_VabsY # Absolute velocity in y-direction [m/s]
float32 f_AabsX # Absolute acceleration in x-direction [m/s^2]
float32 f_AabsY # Absolute acceleration in y-direction [m/s^2]
float32 f_VabsXStd # Standard deviation of absolute velocity in x-direction [m/s]
float32 f_VabsYStd # Standard deviation of absolute velocity in y-direction [m/s]

float32 f_AabsXStd # Standard deviation of absolute acceleration in x-direction [m/s^2]
float32 f_AabsYStd # Standard deviation of absolute acceleration in y-direction [m/s^2]
