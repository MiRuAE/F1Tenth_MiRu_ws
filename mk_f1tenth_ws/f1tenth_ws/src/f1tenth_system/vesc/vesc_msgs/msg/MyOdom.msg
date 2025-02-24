# MyOdometry.msg
# 기본 헤더(타임스탬프, 프레임 정보 등)
std_msgs/Header header

# 로봇의 위치와 회전 정보
float64 x
float64 y
float64 yaw     # degree

# 선형 및 각속도 정보
float64 linear_velocity
float64 angular_velocity    # degree/s