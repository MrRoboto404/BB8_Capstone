#include <Arduino.h>
#include "Filter.h"

Filter::Filter(float sample_freq_hz, float beta)
    : _sample_freq(sample_freq_hz), _beta(beta),
      _gx_bias(0.0f), _gy_bias(0.0f), _gz_bias(0.0f) {}

void Filter::begin() {
    _madgwick.begin(_sample_freq);
    _madgwick.setBeta(_beta);
}

void Filter::setBeta(float beta) {
    _beta = beta;
    _madgwick.setBeta(beta);
}

void Filter::update(float gx, float gy, float gz, float ax, float ay, float az) {
    // Subtract gyro bias (input in rad/s, bias stored in rad/s)
    gx -= _gx_bias;
    gy -= _gy_bias;
    gz -= _gz_bias;

    // Madgwick expects gyro in deg/s, accel in any consistent unit (it normalizes)
    // RAD_TO_DEG is defined by Arduino core as 57.295...
    _madgwick.updateIMU(
        gx * (float)RAD_TO_DEG,
        gy * (float)RAD_TO_DEG,
        gz * (float)RAD_TO_DEG,
        ax, ay, az
    );
}

float Filter::getRoll() {
    return _madgwick.getRollRadians();
}

float Filter::getPitch() {
    return _madgwick.getPitchRadians();
}

float Filter::getYaw() {
    return _madgwick.getYawRadians();
}

void Filter::calibrateGyroBias(void (*readGyro)(float&, float&, float&), int num_samples) {
    float gx, gy, gz;
    float sum_x = 0.0f, sum_y = 0.0f, sum_z = 0.0f;

    for (int i = 0; i < num_samples; i++) {
        readGyro(gx, gy, gz);
        sum_x += gx;
        sum_y += gy;
        sum_z += gz;
        delayMicroseconds(2000); // 2ms between samples
    }

    _gx_bias = sum_x / num_samples;
    _gy_bias = sum_y / num_samples;
    _gz_bias = sum_z / num_samples;
}

void Filter::reset() {
    // Re-initialize Madgwick (resets quaternion to identity)
    _madgwick = Madgwick();
    _madgwick.begin(_sample_freq);
    _madgwick.setBeta(_beta);
}
