/*========================INCLUDES========================*/
#include <FlexCAN_T4.h>
#include <stdio.h>
#include <Wire.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <Bounce2.h>
#include <math.h>
#include "Filter.h"
#include <SPI.h>
#include <SD.h>





/*========================GLOBAL DECLARATIONS========================*/
//____________CAN Setup____________


//------Constants------
// CAN IDs correctly bit-shifted
#define MOTOR_1 (0x01 << 5)
#define MOTOR_2 (0x02 << 5)
#define MOTOR_3 (0x03 << 5)
// CAN ODrive commands
#define SET_TORQUE (0x00E)
#define GET_ENCODER (0x09)
#define MOTOR_STATE (0x07)
#define SET_ABS_POS (0x19)
#define GET_ERROR (0x03)
#define CLEAR_ERRORS (0x018)
// Control loop switch
#define SWITCH_PIN 14
// ODrive error codes
#define ERROR_ESTOP_REQ (0x02)
#define ERROR_OVERVOLTAGE (0x00000100)
#define ERROR_UNDERVOLTAGE (0x00000200)
#define ERROR_DC_OVERCURRENT (0x00000400)
#define ERROR_OVERREGEN (0x00000800)
#define ERROR_OVERCURRENT (0x00001000)


//------Global Variables------
//____________CAN____________
//float motor_true_torques; DISABLED, only needed for data collection
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2; // wired to CAN2
int MOTOR_IDS[3] = {MOTOR_1, MOTOR_2, MOTOR_3};
volatile float motor_true_vels[3]; // index 0 is motor 1, and so on
int axis_state = 1; // default to idle
float vel; // placeholder variables for recieving data from encoders
bool errored = false;


//____________IMU____________
SparkFun_ISM330DHCX myISM;
sfe_ism_data_t accelData; // accel data storage variable filtered
sfe_ism_data_t gyroData; // gyro data storage variable filtered

IntervalTimer myTimer;
int imu_timer_freq = 160; //hz
int imu_period = 1000000/imu_timer_freq; //convert to seconds, rounds to floor of quintent, us

//____________IMU Filter____________
const float ACCEL_SCALE = 1.0f / 1000.0f * 9.81f;          // SparkFun outputs mg -> m/s^2
const float GYRO_SCALE  = (1.0f / 1000.0f) * PI / 180.0f;  // SparkFun outputs mdps -> rad/s
const float BETA = 0.02f;                                   // Madgwick tuning param
Filter imu_filter((float)imu_timer_freq, BETA);
volatile bool imu_ready = false;

// Controller inputs — updated at 160 Hz in loop()
float filtered_roll  = 0.0f;  // radians
float filtered_pitch = 0.0f;  // radians
float gyro_x         = 0.0f;  // rad/s
float gyro_y         = 0.0f;  // rad/s
float gyro_z         = 0.0f;  // rad/s (yaw rate, bias-subtracted)

//____________LQR & Controller Setup____________
uint32_t last_time = 0; 
float phi_x = 0.0;
float phi_y = 0.0;

// Kinematic Params
const float rW = 0.048;         // wheel radius (m)
const float rB = 0.12;          // ball radius (m)
const float alpha_rad = 0.785;  // 45 degrees
const float beta_rad = 0.0;     // Alignment offset
const float MAX_TORQUE = 0.20f;  // N*m

// Precompute constants
const float SQRT_2 = 1.41421356f;
const float SQRT_3 = 1.73205081f;
const float SQRT_6 = 2.44948974f;

// LQR Controller Gains
const float gain_mod = 0.2;
const float K_xy[4] = {-0.2000,  -29.6151,   -0.5554,  -14.6805};
// const float K_xy[4] = {gain_mod*-1.2527, gain_mod*-140.9692, gain_mod*-3.2801, gain_mod*-70.3089};
// const float K_xy[4] = {gain_mod*0, gain_mod*-140.9692, gain_mod*-0, gain_mod*-70.3089};
const float K_z[2]  = {gain_mod*-0.9188, gain_mod*-0.9553};

//____________Switches____________
Bounce debouncer = Bounce();
bool control_run = false; // false by default

// SD Card Data
File roll_data_file;
File pitch_data_file;
File gyro_x_data_file;
File gyro_y_data_file;
File gyro_z_data_file;





/*========================DEFINITIONS========================*/

//==============Core stuff==============

/**
 * @brief Initializes IMU communication, CAN communication, motor states,
                determines gyro bias, [LIST OTHERS]
 * 
 * @return None
 * 
 * @note Errors not yet implemented
 */
void setup() {
  delay(1000);
  Serial.begin(9600);
  Serial.println("------------Beginning Setup------------");
  /*____________________________SPI/IMU____________________________*/
  
  Wire.begin();

  // if connection fails say so
  while(!myISM.begin()){
    Serial.println("Failed to connect to IMU!");
  }

  // Reset device to default settings
  myISM.deviceReset();
  // wait for reset to complete
  while(!myISM.getDeviceReset()){
    delay(1);
  }

  Serial.println("IMU successfully reset");
  Serial.println("Applying settings");
  delay(100);

  myISM.setDeviceConfig();
	myISM.setBlockDataUpdate();
	
	// Set the output data rate and precision of the accelerometer
	myISM.setAccelDataRate(ISM_XL_ODR_208Hz);
	myISM.setAccelFullScale(ISM_2g); 

	// Set the output data rate and precision of the gyroscope
	myISM.setGyroDataRate(ISM_GY_ODR_208Hz);
	myISM.setGyroFullScale(ISM_500dps);

  /*____________________________IMU CALIBRATION____________________________*/
  // Keep IMU perfectly still during this period
  imu_filter.begin();
  imu_filter.setBias(0.0062981257f, -0.0121280579f, 0.0001833806f); 

  myTimer.begin(IMU_ISR, imu_period);

  /*_________________________CAN/MOTORS_______________________*/
  // Start bus on existing CAN2
  Can2.begin();
  Can2.setBaudRate(500000); // 500KB/sec

  // Enable recieving messages
  Can2.enableMBInterrupts();
  Can2.onReceive(can_sniff);

  //______Motor Care______
  motors_reset_position(); // sets absolute positions to 0, even if not needed
  axis_state = 1; // idles
  set_motors_states(axis_state); // change from idle -> ready (flashing green)

  /*_________________________SWITCHES_______________________*/
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  debouncer.attach(SWITCH_PIN);
  debouncer.interval(2); // debounce time, in ms


  /*_________________________ACK SETUP_______________________*/
  Serial.println("------------Completed Setup.------------");

  /*_______________SD CARD SETUP_____________________________*/
  // Serial.println("Starting SD card");

  // while(!SD.begin(10)){
  //   Serial.println("FISH");
  // }

  // SD.remove("roll_data_file.csv");
  // SD.remove("pitch_data_file.csv");
  // SD.remove("gyro_x_data_file.csv");
  // SD.remove("gyro_y_data_file.csv");
  // SD.remove("gyro_z_data_file.csv");

  // roll_data_file = SD.open("roll_data_file.csv", FILE_WRITE);
  // pitch_data_file = SD.open("pitch_data_file.csv", FILE_WRITE);
  // gyro_x_data_file = SD.open("gyro_x_data.csv", FILE_WRITE);
  // gyro_y_data_file = SD.open("gyro_y_data.csv", FILE_WRITE);
  // gyro_z_data_file = SD.open("gyro_z_data.csv", FILE_WRITE);

}

/** 
* @brief Main loop that is in all .ino files
*
* @return None
*
* @note Errors not yet implemented
*/
void loop() {
  // Always listen for CAN events regardless of state
  Can2.events(); 

  // Detect if control switch has been flicked
  debouncer.update();
  if (debouncer.changed()) {
    // INPUT_PULLUP: LOW = pressed
    control_run = (debouncer.read() == LOW);
    
    if (!control_run) {
      Serial.println("Control Switch is OFF - Disabling Torques");
      shutdown(false); // not an error, normal shutdown
    } 
    else {
      Serial.println("Control Switch is ON - Engaging LQR");
      clear_errors(); // clear any existing errors on the motors
      errored = false;

      // Reset integration and timing when switched on to prevent jolts
      axis_state = 8; // ready motors
      set_motors_states(axis_state);
      phi_x = 0.0;
      phi_y = 0.0;
      last_time = micros() - imu_period;
    }
  }

  // 160Hz Execution Block triggered by the Timer ISR
  if (imu_ready) {
    imu_ready = false; // Reset the flag
    
    // Get IMU Data and Filter 
    myISM.getAccel(&accelData);
    myISM.getGyro(&gyroData);

    float ax = accelData.xData * ACCEL_SCALE;
    float ay = accelData.yData * ACCEL_SCALE;
    float az = accelData.zData * ACCEL_SCALE;
    float gx = gyroData.xData * GYRO_SCALE;
    float gy = gyroData.yData * GYRO_SCALE;
    float gz = gyroData.zData * GYRO_SCALE;

    imu_filter.update(gx, gy, gz, ax, ay, az);

    // Update global variables for the controller
    filtered_roll  = -imu_filter.getRoll();
    filtered_pitch = -imu_filter.getPitch();
    gyro_x         = -gx;
    gyro_y         = -gy;
    gyro_z         = -(gz - 0.0001833806f); // bias-subtracted

    /*___WRITE TO SD CARD____*/
    
    // roll_data_file.print(filtered_roll);
    // roll_data_file.print(", ");

    // pitch_data_file.print(filtered_pitch);
    // pitch_data_file.print(", ");
    
    // gyro_x_data_file.print(gyro_x);
    // gyro_x_data_file.print(", ");
    
    // gyro_y_data_file.print(gyro_y);
    // gyro_y_data_file.print(", ");
    
    // gyro_z_data_file.print(gyro_z);
    // gyro_z_data_file.print(", ");
    
    
    
    
    
    
    // 2. Run the controller (Only if the switch is ON)
    if (control_run){
      run_controller();
    }
  }
}

/**
 * @brief Executes LQR math and outputs torque to the motors
 */
void run_controller() {
  uint32_t current_time = micros();
  
  // Calculate dynamic dt
  float dt = (current_time - last_time) / 1000000.0f;
  if (last_time == 0) dt = 1.0/160.0; 
  last_time = current_time;

  // Wheel speeds (psi_dot) - Gear ratio: n=27
  float psd1 = motor_true_vels[0] / 27.0;
  float psd2 = motor_true_vels[1] / 27.0;
  float psd3 = motor_true_vels[2] / 27.0;

  // Trig precalculation
  float sX = sin(filtered_roll);
  float cX = cos(filtered_roll);
  float sY = sin(filtered_pitch);
  float cY = cos(filtered_pitch);

  // Ball rolling rate (phi_dot)
  float phi_dot_x = calc_phi_dot_x(psd1, psd2, psd3, gyro_x, sX, cX, sY, cY);
  float phi_dot_y = calc_phi_dot_y(psd1, psd2, psd3, gyro_y, sX, cX);
  float phi_dot_z = calc_phi_dot_z(psd1, psd2, psd3, gyro_x, gyro_z, sX, cX, sY, cY);

  // Integrate velocity
  phi_x += phi_dot_x * dt;
  phi_y += phi_dot_y * dt;

  // LQR calculation (u = -Kx)
  float Tx = -(K_xy[0]*phi_x + K_xy[1]*filtered_roll  + K_xy[2]*phi_dot_x + K_xy[3]*gyro_x);
  float Ty = (K_xy[0]*phi_y + K_xy[1]*filtered_pitch + K_xy[2]*phi_dot_y + K_xy[3]*gyro_y);
  float Tz = 0;

  // Torque conversion
  float cA = cos(alpha_rad);
  float cB = cos(beta_rad);
  float sB = sin(beta_rad);

  float T1 = (1.0/(3.0*27)) * (Tz + (2.0/cA) * (Tx * cB - Ty * sB));
  float T2 = (1.0/(3.0*27)) * (Tz + (1.0/cA) * (sB * (-SQRT_3*Tx + Ty) - cB * (Tx + SQRT_3*Ty)));
  float T3 = (1.0/(3.0*27)) * (Tz + (1.0/cA) * (sB * (SQRT_3*Tx + Ty) + cB * (-Tx + SQRT_3*Ty)));

  // Applying saturation
  T1 = constrain(T1, -MAX_TORQUE, MAX_TORQUE);
  T2 = constrain(T2, -MAX_TORQUE, MAX_TORQUE);
  T3 = constrain(T3, -MAX_TORQUE, MAX_TORQUE);

  // Command the drives
  send_torque(MOTOR_1, 1*T1);
  send_torque(MOTOR_2, 1*T2);
  send_torque(MOTOR_3, 1*T3);

  // Overrun check
  uint32_t execution_time = micros() - current_time;
  if (execution_time > 6250) {
    Serial.print("CRITICAL: Overrun detected! Execution took (us): ");
    Serial.println(execution_time);
  }
}


/**
 * @brief Used to cleanly shutdown components under normal or errorconditions
 * 
 * @note Prints to serial
 */
void shutdown(bool errored){
  Serial.println("------------Exit Requested------------");
  //------Motors------
  send_torque(MOTOR_1, 0.0);
  send_torque(MOTOR_2, 0.0);
  send_torque(MOTOR_3, 0.0);
  
  if (!errored){
    axis_state = 1;
    set_motors_states(axis_state); // idle
  }

  //------IMU Stuff------


  //-----SD Card------
  // roll_data_file.close();
  // pitch_data_file.close();
  // gyro_x_data_file.close();
  // gyro_y_data_file.close();
  // gyro_z_data_file.close();


  //------Acknowledge------
  Serial.println("------------Safely Exited Program------------");
}

//==============IMU stuff==============

/*
Behavior: interrupt service routine that runs every time the imu timer ends
Errors: none
Returns: none
Arguments: none
*/
void IMU_ISR(){
  imu_ready = true;
}

//==============CAN stuff==============
/**
 * @brief Translates incoming CAN messages. Right now, just the required motor velocities are decoded.
 * 
 * @param msg not needed in use, as it is a global declaration
 * 
 * @return None
 * 
 * @note Printing CAN messages (and logging torque) aren't included. See 'can_read' file for printing examples.
 * 
 */
void can_sniff(const CAN_message_t &msg) {
    /*
    ODRIVE CAN FRAME ID (11 bits)
    10  9  8  7  6  5 |  4  3  2  1  0
          msg ID           cmd_id 
    */
    uint8_t node = msg.id >> 5; // shift over to find the node/origin
    uint8_t cmd  = msg.id & 0x1F; // only look at the cmd_id region

    // Directly assign data from the message buffer
    if (cmd == GET_ENCODER) {
        // Use an array to map node numbers to velocity variables
        motor_true_vels[node - 1] = *reinterpret_cast<const float*>(msg.buf + 4);
    }
    else if (cmd == GET_ERROR){
      // ignore cyclic message if it is zero
      uint32_t errors = *reinterpret_cast<const uint32_t*>(msg.buf);
      if (errors == 0) {
          // No error → ignore completely
          return;
      }

      // error detected, kill ISR if it hasn't already done so. print once
      if (!errored){
        shutdown(true);
        errored = true;
        Serial.println("!!!!!!!!!!!!ERROR DETECTED!!!!!!!!!!!!");
      }
      
      // print error message(s)
      Serial.print("Motor ");
      Serial.print(node);
      Serial.println(":");
      if (errors & ERROR_OVERVOLTAGE){
        Serial.println("  - OVERVOLTAGE");
      }
      if (errors & ERROR_UNDERVOLTAGE){
        Serial.println("  - UNDERVOLTAGE");
      }
      if (errors & ERROR_DC_OVERCURRENT){
        Serial.println("  - DC_OVERCURRENT");
      }
      if (errors & ERROR_OVERREGEN){
        Serial.println("  - OVER_REGEN");
      }
      if (errors & ERROR_OVERCURRENT){
        Serial.println("  - OVERCURRENT");
      }
      if (errors & ERROR_ESTOP_REQ){
        Serial.println("  - ESTOP REQ");
      }



      Serial.println();
      
    }
    
}

/**
 * @brief Constructs and sends a CAN frame to a target motor with specified torque.
 * 
 * @param MOTOR Hex 'node' identifier
 * @param torque Desired motor torque in N*m
 * 
 * @return None
 */
void send_torque(int MOTOR, float torque){
    CAN_message_t msg;
    msg.id = MOTOR | SET_TORQUE; // eg 0x03 shifted | 0x09 = 000011 01110 bin = 110 dec = 0x6E (0x4E for 2, 0x2E for 1)
    msg.len = 4;
    memcpy(msg.buf, &torque, 4);
    Can2.write(msg);
}

void clear_errors(void){
  CAN_message_t msg;
    msg.len = 4;
    int flash = 0; // flashes when identifying. 0 = no, 1 = true
    memcpy(msg.buf, &flash, 4);
    for (int i = 0; i<3; ++i){
        msg.id = MOTOR_IDS[i] | CLEAR_ERRORS;
        Can2.write(msg);
        delay(10);
    }
}

/**
 * @brief Aids in initialization. Changes the state of all motors to ready.
 * 
 * @param axis_state 1 = idle, 8 = ready
 * @return None
 * 
 * @note Error messages printed to Serial
 */
void set_motors_states(int axis_state){
    CAN_message_t msg;
    msg.len = 4;
    memcpy(msg.buf, &axis_state, 4);
    for (int i = 0; i<3; ++i){
        msg.id = MOTOR_IDS[i] | MOTOR_STATE;
        Can2.write(msg);
        delay(10);
    }
}

/**
 * @brief Aids in initialization. Makes all positions of the motors equal to 0.
 * 
 * @return None
 * 
 * @note Prints error messages to Serial
 */
void motors_reset_position(void){
    float set_zero = 0;
    CAN_message_t msg;
    msg.len = 4;
    memcpy(msg.buf, &set_zero, 4);
    for (int i = 0; i<3; ++i){
        msg.id = MOTOR_IDS[i] | SET_ABS_POS;
        Can2.write(msg);
        delay(10);
    }
}


//==============KINEMATIC HELPERS==============
float calc_phi_dot_x(float dp1, float dp2, float dp3, float dthx, float sX, float cX, float sY, float cY) {
  return (1.0/(3.0*rB)) * ( (SQRT_6*rW*sX*sY*(-dp2+dp3)) + (SQRT_2*rW*cX*sY*(dp1+dp2+dp3)) + (cY*(SQRT_2*rW*(-2.0*dp1+dp2+dp3) + 3.0*rB*dthx)) );
}

float calc_phi_dot_y(float dp1, float dp2, float dp3, float dthy, float sX, float cX) {
  return (1.0/(3.0*rB)) * ( (SQRT_6*rW*cX*(-dp2+dp3)) - (SQRT_2*rW*sX*(dp1+dp2+dp3)) + (3.0*rB*dthy) );
}

float calc_phi_dot_z(float dp1, float dp2, float dp3, float dthx, float dthz, float sX, float cX, float sY, float cY) {
  return (1.0/(3.0*rB)) * ( (SQRT_2*rW*(cX*cY + 2.0*sY)*dp1) + (SQRT_2*rW*(SQRT_3*cY*sX*(-dp2+dp3) + cX*cY*(dp2+dp3) - sY*(dp2+dp3))) + (3.0*rB*(-sY*dthx + dthz)) );
}

