//=============================================================================================
//              UW Mechanical Engineering Dept — Mechatronics 2026 Ballbot
//=============================================================================================
/*  This is the code used to flash the Teensy 4.0 microcontroller for the Ballbot group.
    Controller Design:      Jonathan Pham (hieupham0927@gmail.com)
    Motor Communication:    Ryan Stokes (rtstokes05@gmail.com)
    IMU filtering:          Benedict Chandra
    IMU & SD communication: Dante Rieger
    Readability:            Ryan Stokes  */
//=============================================================================================


/*================================================ INCLUDES ================================================*/
// Generic
#include <stdio.h>
#include <math.h>

// IMU communication (I2C)
#include <Wire.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>

// IMU Filtering
#include "Filter.h"

// CAN bus 
#include <FlexCAN_T4.h>

// Software-based debouncer
#include <Bounce2.h>

// SD card (SPI)
#include <SPI.h>
#include <SdFat.h>



/*================================================ GLOBAL DECLARATIONS ================================================*/
//________________________ CAN Setup ________________________
//------CAN Constants------
#define MOTOR_1 (0x01 << 5) // Configured in ODrive GUI
#define MOTOR_2 (0x02 << 5) // Configured in ODrive GUI
#define MOTOR_3 (0x03 << 5) // Configured in ODrive GUI

// Commands on ODrive documentation
#define MOTOR_STATE (0x07)
#define CLEAR_ERRORS (0x018)
#define SET_TORQUE (0x00E) 
#define SET_ABS_POS (0x19)
#define GET_ENCODER (0x09)
#define GET_ERROR (0x03)
#define GET_TORQUE (0x01C)


#define ERROR_DC_OVERCURRENT (0x00100000) // Verified error code
#define ERROR_UNDERVOLTAGE (0x00000200)   // Verified error code

#define ERROR_ESTOP_REQ (0x00000002)   // Unverified guess of error code
#define ERROR_OVERVOLTAGE (0x00000100) // Unverified guess of error code
#define ERROR_OVERREGEN (0x00000800)   // Unverified guess of error code
#define ERROR_OVERCURRENT (0x00001000) // Unverified guess of error code

//------CAN Global Variables------
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;  // Wired to CAN2 physically

// arrays to be passed around
int MOTOR_IDS[3] = { MOTOR_1, MOTOR_2, MOTOR_3 }; 
volatile float motor_true_vels[3]; // rev/s
volatile float motor_true_pos[3];  // rev
volatile float motor_true_torqs[3];  // Nm, pre-gearing

int axis_state = 1; // Defaults to idle state
bool errored = false; // Monitors if any drivers have errored



//________________________ IMU ________________________
SparkFun_ISM330DHCX myISM;
sfe_ism_data_t accelData; // Holds readings 
sfe_ism_data_t gyroData; // Holds readings

//!!!THIS TIMER CONTROLS THE SPEED OF THE ENTIRE THING!!!
IntervalTimer myTimer;
const float imu_timer_freq = 200.0;  // Hz 
int imu_period = 1000000 / imu_timer_freq;  // µs

//------IMU Filter------
const float ACCEL_SCALE = 1.0f / 1000.0f * 9.81f; 
const float GYRO_SCALE = (1.0f / 1000.0f) * PI / 180.0f;  
const float BETA = 0.02f; // Tuned weight

Filter imu_filter((float)imu_timer_freq, BETA);

volatile bool imu_ready = false; // Defaults to unready. Readies only when the timer ISR is trigged

// Used by controller
float filtered_roll = 0.0f;   
float filtered_pitch = 0.0f;  
float gyro_x = 0.0f;          
float gyro_y = 0.0f;          
float gyro_z = 0.0f;   

//------IMU Buffers for Data------
int buff_pointer = 0;
const int max_buff_size = imu_timer_freq * 10; // Always 10 seconds of data 
unsigned long time_buffer[max_buff_size];
unsigned long save_time;

float roll_buffer[max_buff_size];
float pitch_buffer[max_buff_size];

float gyroX_buffer[max_buff_size];
float gyroY_buffer[max_buff_size];
float gyroZ_buffer[max_buff_size];

float T1_buffer[max_buff_size];
float T2_buffer[max_buff_size];
float T3_buffer[max_buff_size];

float phi_x_buffer[max_buff_size];
float phi_y_buffer[max_buff_size];

float phi_dx_buffer[max_buff_size];
float phi_dy_buffer[max_buff_size];

float motor_vel_1_buffer[max_buff_size];
float motor_vel_2_buffer[max_buff_size];
float motor_vel_3_buffer[max_buff_size];

float motor_pos_1_buffer[max_buff_size];
float motor_pos_2_buffer[max_buff_size];
float motor_pos_3_buffer[max_buff_size];

float motor_torq_1_buffer[max_buff_size];
float motor_torq_2_buffer[max_buff_size];
float motor_torq_3_buffer[max_buff_size];



//________________________ LQR & Controller Setup ________________________
uint32_t last_time = 0;
float phi_x = 0.0;
float phi_y = 0.0;

// Kinematic Params
const float rW = 0.048;          // wheel radius (m)
const float rB = 0.12;           // ball radius (m)
const float alpha_rad = 0.785;   // 45 degrees
const float beta_rad = 0.0;      // Alignment offset
float MAX_TORQUE = 0.23f;        // saturation torque of motors (Nm, pre-gearing)
const float gear_ratio = 5.177;

// Precompute constants
const float SQRT_2 = 1.41421356f;
const float SQRT_3 = 1.73205081f;
const float SQRT_6 = 2.44948974f;
const float CSC_35 = 1.74344679f; 
const float SEC_35 = 1.22077458f;
const float cA = cos(alpha_rad); // radians
const float cB = cos(beta_rad); // radians
const float sB = sin(beta_rad); // radians

//------Direct LQR Tuning Matrix------
// Initialized to your original baseline values (Gain multipliers pre-applied)
float K_xy[4] = { 0.0f, -100.0f, -0.03f, -3.0f }; // Indicies: [0]=Ball Pos (phi), [1]=Tilt (theta), [2]=Ball Vel (phi_dot), [3]=Tilt Rate (theta_dot)
float K_z[2] = { -0.2f, -0.6f }; // Indicies: [0]=Ball Pos (phi), [1]=Tilt (theta)



//________________________ Switches ________________________
#define SWITCH_PIN 14 // control switch wired to pin 14 on teensy 4.0
Bounce debouncer = Bounce(); // software-based switch debouncer
bool control_run = false; // defaults to inactive controller



//________________________ Oscilloscope Verification ________________________
#define OSC_PIN 16 // oscilloscope output wired to pin 16 on teensy 4.0
bool osc_state = false; // defaults to low



//________________________ SD Card ________________________
SdFat sd;
FsFile my_file;





/*================================================ FUNCTION DEFINITIONS ================================================*/

/**
 * @brief Initializes all types of device communications, readies motors and control switch, and connects to the SD card.
 * 
 * @return None
 * 
 * @note  Must have SD card inserted to exit setup.
 */
void setup() {
  delay(1000);
  Serial.begin(9600);
  Serial.println("Beginning Setup...");


  /*____________________________IMU____________________________*/
  Wire.begin();
  Wire.setClock(400000); // Hz (Fast mode)
  while (!myISM.begin()) {
    Serial.println(" |   Failed to connect to IMU!");
    delay(500);
  }

  // Reset device to default settings
  myISM.deviceReset();
  // wait for reset to complete
  while (!myISM.getDeviceReset()) {
    delay(1);
  }

  Serial.println(" |   IMU successfully reset.\n |   Applying settings...");
  delay(100);

  myISM.setDeviceConfig();
  myISM.setBlockDataUpdate();
  myISM.setAccelDataRate(ISM_XL_ODR_833Hz);
  myISM.setAccelFullScale(ISM_2g);
  myISM.setGyroDataRate(ISM_GY_ODR_833Hz);
  myISM.setGyroFullScale(ISM_500dps);
  Serial.println(" |   Done.");


  /*____________________________IMU CALIBRATION____________________________*/
  imu_filter.begin();
  imu_filter.setBias(0.0062981257f, -0.0121280579f, 0.0001833806f);

  myTimer.begin(IMU_ISR, imu_period);


  /*_________________________CAN/MOTORS_______________________*/
  Can2.begin();
  Can2.setBaudRate(1000000);  
  Can2.enableMBInterrupts();
  Can2.onReceive(can_sniff);

  motors_reset_position();        
  axis_state = 1;                 
  set_motors_states(axis_state);  


  /*_________________________SWITCHES_______________________*/
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  debouncer.attach(SWITCH_PIN);
  debouncer.interval(2);  

  pinMode(OSC_PIN, OUTPUT);


  /*_______________SD CARD SETUP_____________________________*/
  Serial.println(" |   Connecting to SD card...");
  while (!sd.begin(10, SD_SCK_MHZ(1))) {
    Serial.println(" |   Failed to connect to SD Card!");
    delay(100);
  }
  Serial.println(" |   Connected to SD.");

  // Ack
  Serial.println("____________Completed Setup____________");
  
  // Print initial active tuning matrix
  printActiveGains();
}

/**
 * @brief Main loop.
 * 
 * @return None
 * 
 * @note When ON, allows IMU ISR to run the controller. When OFF, IMU ISR still runs but does not enage controller.
 */
void loop() {
  /*____________________________ Checks outside of ISR ____________________________*/
  Can2.events(); // CAN messages
  checkSerialCommands(); // Serial gain changes


  /*____________________________ Control Switch Updating ____________________________*/
  debouncer.update();
  if (debouncer.changed()) {
    control_run = (debouncer.read() == LOW);

    if (!control_run) {
      Serial.println("Control Switch is OFF - Disabling Torques");
      shutdown();
    } else {
      Serial.println("Control Switch is ON - Engaging LQR");
      clear_errors();  
      errored = false;

      axis_state = 8;  
      set_motors_states(axis_state);
      phi_x = 0.0;
      phi_y = 0.0;
      last_time = micros() - imu_period;
    }
  }


  /*____________________________ ISR-Dependent ____________________________*/
  if (imu_ready) { // when ISR flag switches
    imu_ready = false;  

    // Retrieve data
    myISM.getAccel(&accelData);
    myISM.getGyro(&gyroData);

    // Filter data
    float ax = accelData.xData * ACCEL_SCALE;
    float ay = accelData.yData * ACCEL_SCALE;
    float az = accelData.zData * ACCEL_SCALE;
    float gx = gyroData.xData * GYRO_SCALE;
    float gy = gyroData.yData * GYRO_SCALE;
    float gz = gyroData.zData * GYRO_SCALE;

    imu_filter.update(gx, gy, gz, ax, ay, az);

    filtered_roll = -imu_filter.getRoll();
    filtered_pitch = -imu_filter.getPitch();
    gyro_x = -gx;
    gyro_y = -gy;
    gyro_z = -(gz - 0.0001833806f);  

    // If control switch is on, enable action on the readings
    if (control_run) {
      run_controller();
    }
  }
}

/**
 * @brief Executes LQR math and outputs torque to the motors using live K_xy gains
 */
void run_controller() {
  uint32_t current_time = micros();

  osc_state = !osc_state;
  digitalWriteFast(OSC_PIN, osc_state);

  float dt = (current_time - last_time) / 1000000.0f;
  if (last_time == 0) dt = 1.0 / imu_timer_freq;
  last_time = current_time;

  float psd1 = motor_true_vels[0] / gear_ratio;
  float psd2 = motor_true_vels[1] / gear_ratio;
  float psd3 = motor_true_vels[2] / gear_ratio;

  float sX = sin(filtered_roll);
  float cX = cos(filtered_roll);
  float sY = sin(filtered_pitch);
  float cY = cos(filtered_pitch);

  float phi_dot_x = calc_phi_dot_x(psd1, psd2, psd3, gyro_x, sX, cX, sY, cY);
  float phi_dot_y = calc_phi_dot_y(psd1, psd2, psd3, gyro_y, sX, cX);

  phi_x += phi_dot_x * dt;
  phi_y += phi_dot_y * dt;

  // LQR calculation using the live modified elements of K_xy
  float Tx = (K_xy[0] * phi_x + K_xy[1] * filtered_roll + K_xy[2] * phi_dot_x + K_xy[3] * gyro_x);
  float Ty = -(K_xy[0] * phi_y + K_xy[1] * filtered_pitch + K_xy[2] * phi_dot_y + K_xy[3] * gyro_y);
  float Tz = -(K_z[1] * gyro_z);

  float T1 = (1.0 / (3.0 * gear_ratio)) * (Tz + (2.0 / cA) * (Tx * cB - Ty * sB));
  float T2 = (1.0 / (3.0 * gear_ratio)) * (Tz + (1.0 / cA) * (sB * (-SQRT_3 * Tx + Ty) - cB * (Tx + SQRT_3 * Ty)));
  float T3 = (1.0 / (3.0 * gear_ratio)) * (Tz + (1.0 / cA) * (sB * (SQRT_3 * Tx + Ty) + cB * (-Tx + SQRT_3 * Ty)));
  

  T1 = constrain(T1, -MAX_TORQUE, MAX_TORQUE);
  T2 = constrain(T2, -MAX_TORQUE, MAX_TORQUE);
  T3 = constrain(T3, -MAX_TORQUE, MAX_TORQUE);

  send_torque(MOTOR_1, 1 * T1);
  send_torque(MOTOR_2, 1 * T2);
  send_torque(MOTOR_3, 1 * T3);

  if (buff_pointer < max_buff_size) {
    time_buffer[buff_pointer] = micros();
    roll_buffer[buff_pointer] = filtered_roll;
    pitch_buffer[buff_pointer] = filtered_pitch;
    gyroX_buffer[buff_pointer] = gyro_x;
    gyroY_buffer[buff_pointer] = gyro_y;
    gyroZ_buffer[buff_pointer] = gyro_z;
    T1_buffer[buff_pointer] = T1;
    T2_buffer[buff_pointer] = T2;
    T3_buffer[buff_pointer] = T3;
    phi_x_buffer[buff_pointer] = phi_x;
    phi_y_buffer[buff_pointer] = phi_y;
    phi_dx_buffer[buff_pointer] = phi_dot_x;
    phi_dy_buffer[buff_pointer] = phi_dot_y;
    motor_vel_1_buffer[buff_pointer] = motor_true_vels[0];
    motor_vel_2_buffer[buff_pointer] = motor_true_vels[1];
    motor_vel_3_buffer[buff_pointer] = motor_true_vels[2];
    motor_pos_1_buffer[buff_pointer] = motor_true_pos[0];
    motor_pos_2_buffer[buff_pointer] = motor_true_pos[1];
    motor_pos_3_buffer[buff_pointer] = motor_true_pos[2];
    motor_torq_1_buffer[buff_pointer] = motor_true_torqs[0];
    motor_torq_2_buffer[buff_pointer] = motor_true_torqs[1];
    motor_torq_3_buffer[buff_pointer] = motor_true_torqs[2];
    buff_pointer++;
  }

  uint32_t execution_time = micros() - current_time;
  if (execution_time > 6250) {
    Serial.print("CRITICAL: Overrun detected! Execution took (us): ");
    Serial.println(execution_time);
  }
}

/**
 * @brief Parses direct gain inputs asynchronously via Serial
 */
void checkSerialCommands() {
  if (!Serial.available()) return;

  String input = Serial.readStringUntil('\n');
  input.trim();

  if (input.length() == 0) return;

  int index = input.indexOf('=');
  if (index == -1) {
    Serial.println("Invalid format! Use: k0=val, k1=val, k2=val, k3=val, or tb=val");
    return;
  }

  String cmd = input.substring(0, index);
  float val = input.substring(index + 1).toFloat();

  if (cmd.equalsIgnoreCase("k0")) {
    K_xy[0] = val;
    printActiveGains();
  } else if (cmd.equalsIgnoreCase("k1")) {
    K_xy[1] = val;
    printActiveGains();
  } else if (cmd.equalsIgnoreCase("k2")) {
    K_xy[2] = val;
    printActiveGains();
  } else if (cmd.equalsIgnoreCase("k3")) {
    K_xy[3] = val;
    printActiveGains();
  } else if (cmd.equalsIgnoreCase("T_max")){
    MAX_TORQUE = val;
    printActiveGains();  
  }
    else if (cmd.equalsIgnoreCase("kz1")){
    K_z[0] = val;
    printActiveGains();  
  }
    else if (cmd.equalsIgnoreCase("kz2")){
    K_z[1] = val;
    printActiveGains();  
  } else {
    Serial.print("Unknown parameter: ");
    Serial.println(cmd);
  }
}

/**
 * @brief Prints current active state matrix configuration
 */
void printActiveGains() {
  Serial.println("\n--- Active LQR Matrix (K_xy) & Params ---");
  Serial.printf("k0 (Ball Position): %.4f\n", K_xy[0]);
  Serial.printf("k1 (Tilt Angle):    %.4f\n", K_xy[1]);
  Serial.printf("k2 (Ball Velocity): %.4f\n", K_xy[2]);
  Serial.printf("k3 (Gyro Rate):     %.4f\n", K_xy[3]);
  Serial.printf("T_max :             %.4fN\n", MAX_TORQUE);
  Serial.printf("kz1 (Yaw Angle):    %.4f\n", K_z[0]);
  Serial.printf("kz2 (Yaw Rate):     %.4f\n", K_z[1]);
  Serial.println("-----------------------------------------");
}

void shutdown(bool has_errored) {
  if (!has_errored){
    Serial.println("------------Safe Exit Requested------------");
  } else{
    Serial.println("------------Error Detected------------");
  }
  Serial.println(" |  Shutting down...");
  send_torque(MOTOR_1, 0.0);
  send_torque(MOTOR_2, 0.0);
  send_torque(MOTOR_3, 0.0);

  axis_state = 1;
  set_motors_states(axis_state);
  save_all_data_to_one_CSV();
  buff_pointer = 0;
  Serial.println("------------Safely Exited Program------------");
}

//==============IMU stuff==============

/*
Behavior: interrupt service routine that runs every time the imu timer ends
Errors: none
Returns: none
Arguments: none
*/
void IMU_ISR() {
  imu_ready = true;
}

void can_sniff(const CAN_message_t& msg) {
  uint8_t node = msg.id >> 5;   
  uint8_t cmd = msg.id & 0x1F;  

  if (cmd == GET_ENCODER) {
    motor_true_pos[node - 1] = *reinterpret_cast<const float*>(msg.buf);
    motor_true_vels[node - 1] = *reinterpret_cast<const float*>(msg.buf + 4);
  } else if (cmd == GET_TORQUE){
    motor_true_torqs[node - 1] = *reinterpret_cast<const float*>(msg.buf + 4);
  } else if (cmd == GET_ERROR) {
    uint32_t errors_raw = *reinterpret_cast<const uint32_t*>(msg.buf + 4);
    uint32_t errors = __builtin_bswap32(errors_raw);  
    if (errors == 0) return;

    if (!errored) {
      shutdown(true);
      Serial.println("!!!!!!!!!!!!ERROR DETECTED!!!!!!!!!!!!");
      Serial.printf("Motor %d:\n", node);
      if (errors & ERROR_OVERVOLTAGE)    Serial.println("  - OVERVOLTAGE");
      if (errors & ERROR_UNDERVOLTAGE)   Serial.println("  - UNDERVOLTAGE");
      if (errors & ERROR_DC_OVERCURRENT) Serial.println("  - DC_OVERCURRENT");
      if (errors & ERROR_OVERREGEN)      Serial.println("  - OVER_REGEN");
      if (errors & ERROR_OVERCURRENT)    Serial.println("  - OVERCURRENT");
      if (errors & ERROR_ESTOP_REQ)      Serial.println("  - ESTOP REQ");
      Serial.println();
      errored = true;
    }


  }
}

void send_torque(int MOTOR, float torque) {
  CAN_message_t msg;
  msg.id = MOTOR | SET_TORQUE;  
  msg.len = 4;
  memcpy(msg.buf, &torque, 4);
  Can2.write(msg);
}

void clear_errors(void) {
  CAN_message_t msg;
  msg.len = 4;
  int flash = 0;  
  memcpy(msg.buf, &flash, 4);
  for (int i = 0; i < 3; ++i) {
    msg.id = MOTOR_IDS[i] | CLEAR_ERRORS;
    Can2.write(msg);
    delay(10);
  }
}

void set_motors_states(int axis_state) {
  CAN_message_t msg;
  msg.len = 4;
  memcpy(msg.buf, &axis_state, 4);
  for (int i = 0; i < 3; ++i) {
    msg.id = MOTOR_IDS[i] | MOTOR_STATE;
    Can2.write(msg);
    delay(10);
  }
}

void motors_reset_position(void) {
  float set_zero = 0;
  CAN_message_t msg;
  msg.len = 4;
  memcpy(msg.buf, &set_zero, 4);
  for (int i = 0; i < 3; ++i) {
    msg.id = MOTOR_IDS[i] | SET_ABS_POS;
    Can2.write(msg);
    delay(10);
  }
}

float calc_phi_dot_x(float dp1, float dp2, float dp3, float dthx, float sX, float cX, float sY, float cY) {
    float term1 = rW * (-2.0f * cY * CSC_35 + cX * SEC_35 * sY) * dp1;
    float term2 = rW * sY * (SQRT_3 * CSC_35 * sX * (-dp2 + dp3) + cX * SEC_35 * (dp2 + dp3));
    float term3 = cY * (rW * CSC_35 * (dp2 + dp3) + 3.0f * rB * dthx);
    return (1.0f / (3.0f * rB)) * (term1 + term2 + term3);
}

float calc_phi_dot_y(float dp1, float dp2, float dp3, float dthy, float sX, float cX) {
    float term1 = SQRT_3 * cX * CSC_35 * (dp2 - dp3);
    float term2 = SEC_35 * sX * (dp1 + dp2 + dp3);
    return - (rW * (term1 + term2)) / (3.0f * rB) + dthy;
}

void save_all_data_to_one_CSV() {
  unsigned long save_time = micros();
  char filename[40];
  sprintf(filename, "%lu_motor_data.csv", save_time);

  Serial.print(" |  Creating master log: ");
  Serial.println(filename);

  my_file = sd.open(filename, FILE_WRITE);
  if (!my_file) {
    Serial.println(" |  Failed to open master log!");
    return;
  }

  my_file.println("Time_us,Roll,Pitch,GyroX,GyroY,GyroZ,T1,T2,T3,PhiX,PhiY,PhiDX,PhiDY,motor_vel_1,motor_vel_2,motor_vel_3,motor_pos_1,motor_pos_2,motor_pos_3,motor_torq_1,motor_torq_2,motor_torq_3");

  for (int i = 0; i < buff_pointer; i++) {
    my_file.print(time_buffer[i]);     my_file.print(",");
    my_file.print(roll_buffer[i], 4);  my_file.print(",");
    my_file.print(pitch_buffer[i], 4); my_file.print(",");
    my_file.print(gyroX_buffer[i], 4); my_file.print(",");
    my_file.print(gyroY_buffer[i], 4); my_file.print(",");
    my_file.print(gyroZ_buffer[i], 4); my_file.print(",");
    my_file.print(T1_buffer[i], 4);    my_file.print(",");
    my_file.print(T2_buffer[i], 4);    my_file.print(",");
    my_file.print(T3_buffer[i], 4);    my_file.print(",");
    my_file.print(phi_x_buffer[i], 4); my_file.print(",");
    my_file.print(phi_y_buffer[i], 4); my_file.print(",");
    my_file.print(phi_dx_buffer[i], 4);my_file.print(",");
    my_file.print(phi_dy_buffer[i], 4); my_file.print(",");
    my_file.print(motor_vel_1_buffer[i], 4); my_file.print(",");
    my_file.print(motor_vel_2_buffer[i], 4); my_file.print(",");
    my_file.print(motor_vel_3_buffer[i], 4); my_file.print(",");
    my_file.print(motor_pos_1_buffer[i], 4); my_file.print(",");
    my_file.print(motor_pos_2_buffer[i], 4); my_file.print(",");
    my_file.print(motor_pos_3_buffer[i], 4); my_file.print(",");
    my_file.print(motor_torq_1_buffer[i], 4); my_file.print(",");
    my_file.print(motor_torq_2_buffer[i], 4); my_file.print(",");
    my_file.println(motor_torq_3_buffer[i], 4);

    
  }

  my_file.close();
  Serial.println(" |  Master log saved.");
}