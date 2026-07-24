# Real-time plant disturbance / interference status (wind + IMU noise).
# Published by drone_dynamics for the stability / interference monitor UI.
std_msgs/Header header

bool wind_enable
float64 wind_force_x          # Instantaneous wind force in map frame [N]
float64 wind_force_y
float64 wind_force_z
float64 wind_force_norm
float64 wind_const_x          # Configured constant wind components [N]
float64 wind_const_y
float64 wind_const_z
float64 wind_sin_amp          # Sinusoidal wind amplitude [N]
float64 wind_sin_freq         # Sinusoidal wind frequency [Hz]

bool imu_noise_enable
float64 imu_accel_noise_std   # Configured accel noise σ [m/s^2]
float64 imu_gyro_noise_std    # Configured gyro noise σ [rad/s]
float64 imu_accel_x           # Latest noisy IMU linear accel (body) [m/s^2]
float64 imu_accel_y
float64 imu_accel_z
float64 imu_gyro_x            # Latest noisy IMU angular rate (body) [rad/s]
float64 imu_gyro_y
float64 imu_gyro_z

float64 sim_time              # Plant simulation time [s]
