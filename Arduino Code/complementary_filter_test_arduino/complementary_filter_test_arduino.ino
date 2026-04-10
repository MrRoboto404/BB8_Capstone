#include <Wire.h>
#include <SparkFun_ISM330DHCX.h>

SparkFun_ISM330DHCX myISM;
sfe_ism_data_t accelData;
sfe_ism_data_t gyroData;

// Break frequency in rad/s — crossover between trusting accel vs gyro.
float wb = 1.1;
float tau = 1 / wb;

// Target loop rate 
const unsigned long LOOP_MS = 10;   // 100 Hz

// Sensor Scaling
const float ACCEL_SCALE = 0.122e-3 * 9.81;   // LSB -> m/s^2
const float GYRO_SCALE  = 17.5e-3 * PI/180;  // LSB -> rad/s


// Filter state
float roll_filtered  = 0.0;
float pitch_filtered = 0.0;
float gx_bias = 0.0;
float gy_bias = 0.0;

// Timing
unsigned long last_time_ms = 0;

// Returns angle in RADIANS.
float accel_roll(float ax, float ay, float az) {
    return atan2(ay, sqrt(ax*ax + az*az));
}

float accel_pitch(float ax, float ay, float az) {
    return atan2(-ax, sqrt(ay*ay + az*az));
}

// alpha is computed from wb and dt using the 1st order time constant formula.
float comp_filter(float angle_prev, float gyro_rate, float accel_angle, float dt) {
    // Compute alpha from wb and dt each call
    float alpha = tau / (tau + dt);

    float gyro_prediction = angle_prev + gyro_rate * dt;
    return alpha * gyro_prediction + (1.0 - alpha) * accel_angle;
}


void setup() {
    Serial.begin(115200);
    while (!Serial);

    Wire.begin();
    Wire.setClock(400000);

    if (!myISM.begin()) { while (1); }

    myISM.setAccelDataRate(ISM330DHCX_XL_ODR_104Hz);
    myISM.setGyroDataRate(ISM330DHCX_GY_ODR_104Hz);
    myISM.setAccelFullScale(ISM330DHCX_4g);
    myISM.setGyroFullScale(ISM330DHCX_500dps);

    last_time_ms = millis();

    int num_samples = 10000;
    for (int i = 0; i < num_samples; i++) {
        myISM.getGyro(&gyroData);
        gx_bias += gyroData.xData * GYRO_SCALE;
        gy_bias += gyroData.yData * GYRO_SCALE;
        delay(2);
}
gx_bias /= num_samples;
gy_bias /= num_samples;

    myISM.getAccel(&accelData);
    float ax0 = accelData.xData * ACCEL_SCALE;
    float ay0 = accelData.yData * ACCEL_SCALE;
    float az0 = accelData.zData * ACCEL_SCALE;
    roll_filtered  = accel_roll(ax0, ay0, az0);
    pitch_filtered = accel_pitch(ax0, ay0, az0);  
}

void loop() {

    unsigned long now = millis();
    if (now - last_time_ms < LOOP_MS) return;
    float dt = (now - last_time_ms) / 1000.0;
    last_time_ms = now;

    if (dt > 0.1) dt = 0.01; // Guard 

    myISM.getAccel(&accelData);
    myISM.getGyro(&gyroData);

    // Scale raw integers to physical units
    float ax = accelData.xData * ACCEL_SCALE; // m/s^2
    float ay = accelData.yData * ACCEL_SCALE;
    float az = accelData.zData * ACCEL_SCALE;

    float gx = gyroData.xData * GYRO_SCALE - gx_bias; // rad/s
    float gy = gyroData.yData * GYRO_SCALE - gy_bias;

    // Unfiltered Accel Angle
    float accel_roll_raw  = accel_roll(ax, ay, az);    // radians
    float accel_pitch_raw = accel_pitch(ax, ay, az);   // radians


    // Complementary filter 
    roll_filtered  = comp_filter(roll_filtered,  gx, accel_roll_raw,  dt);
    pitch_filtered = comp_filter(pitch_filtered, gy, accel_pitch_raw, dt);

    // Serial output 
    Serial.print(accel_roll_raw  * 180.0/PI); Serial.print(",");
    Serial.print(roll_filtered   * 180.0/PI); Serial.print(",");
    Serial.print(accel_pitch_raw * 180.0/PI); Serial.print(",");
    Serial.println(pitch_filtered  * 180.0/PI);
