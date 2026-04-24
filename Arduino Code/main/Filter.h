#ifndef FILTER_H
#define FILTER_H

#include "MadgwickAHRS.h"

class Filter {
public:
    // Constructor - pass in sample rate and beta tuning parameter
    Filter(float sample_freq_hz, float beta);

    // Initialize the filter and set tuning
    void begin();

    // Set/change tuning parameter. Higher = faster correction, more noise.
    // Typical range: 0.02 (conservative) to 0.3 (aggressive). Default: 0.086
    void setBeta(float beta);

    // Update filter with a new IMU sample.
    // Units: gyro in rad/s, accel in m/s^2 (will be converted internally)
    void update(float gx, float gy, float gz, float ax, float ay, float az);

    // Get current orientation estimates
    float getRoll();   // radians
    float getPitch();  // radians
    float getYaw();    // radians

    // Calibrate gyro bias - keep IMU still during this call
    // Takes samples from a provided callback function
    void calibrateGyroBias(void (*readGyro)(float&, float&, float&), int num_samples = 10000);

    // Reset filter state to identity orientation
    void reset();

private:
    Madgwick _madgwick;
    float _sample_freq;
    float _beta;

    // Gyro bias (subtracted from every reading, in rad/s)
    float _gx_bias;
    float _gy_bias;
    float _gz_bias;
};

#endif
