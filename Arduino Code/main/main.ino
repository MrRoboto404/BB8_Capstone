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
#include <SdFat.h>

/*========================GLOBAL DECLARATIONS========================*/
//____________CAN Setup____________
//------Constants------
#define MOTOR_1 (0x01 << 5)
#define MOTOR_2 (0x02 << 5)
#define MOTOR_3 (0x03 << 5)

#define SET_TORQUE (0x00E)
#define GET_ENCODER (0x09)
#define MOTOR_STATE (0x07)
#define SET_ABS_POS (0x19)
#define GET_ERROR (0x03)
#define CLEAR_ERRORS (0x018)

#define SWITCH_PIN 14
#define OSC_PIN 16

#define ERROR_ESTOP_REQ (0x00000002)
#define ERROR_OVERVOLTAGE (0x00000100)
#define ERROR_UNDERVOLTAGE (0x00000200)
#define ERROR_DC_OVERCURRENT (0x00100000)
#define ERROR_OVERREGEN (0x00000800)
#define ERROR_OVERCURRENT (0x00001000)

//------Global Variables------
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;  
int MOTOR_IDS[3] = { MOTOR_1, MOTOR_2, MOTOR_3 };
volatile float motor_true_vels[3]; 
volatile float motor_true_pos[3];  
int axis_state = 1;                 
float vel;                          
bool errored = false;

//____________IMU____________
SparkFun_ISM330DHCX myISM;
sfe_ism_data_t accelData;  
sfe_ism_data_t gyroData;   

IntervalTimer myTimer;
float imu_timer_freq = 200.0;                   // hz
int imu_period = 1000000 / imu_timer_freq;      // us

//____________IMU Filter____________
const float ACCEL_SCALE = 1.0f / 1000.0f * 9.81f;         
const float GYRO_SCALE = (1.0f / 1000.0f) * PI / 180.0f;  
const float BETA = 0.02f;                                 
Filter imu_filter((float)imu_timer_freq, BETA);
volatile bool imu_ready = false;

//_____________IMU Buffers________________
int buff_pointer = 0;
const int max_buff_size = 200 * 10;  
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

// Controller inputs
float filtered_roll = 0.0f;   
float filtered_pitch = 0.0f;  
float gyro_x = 0.0f;          
float gyro_y = 0.0f;          
float gyro_z = 0.0f;          

//____________LQR & Controller Setup____________
uint32_t last_time = 0;
float phi_x = 0.0;
float phi_y = 0.0;

// Kinematic Params
const float rW = 0.048;          // wheel radius (m)
const float rB = 0.12;           // ball radius (m)
const float alpha_rad = 0.785;   // 45 degrees
const float beta_rad = 0.0;      // Alignment offset
float MAX_TORQUE = 0.21f;  

// Precompute constants
const float SQRT_2 = 1.41421356f;
const float SQRT_3 = 1.73205081f;
const float SQRT_6 = 2.44948974f;
const float CSC_35 = 1.74344679f; 
const float SEC_35 = 1.22077458f;
const float cA = cos(alpha_rad);
const float cB = cos(beta_rad);
const float sB = sin(beta_rad);

//____________Direct LQR Tuning Matrix____________
float K_xy[4] = { 0.0f, -115.0f, -2.2f, -5.0f };
const float K_z[2] = { -1.0f, -1.0357f };

//____________Backlash Compensation____________
// Note: These are no longer const so they can be modified via Serial
float KICK_TORQUE = 0.05f;      // Accelerate torque amplitude (Nm)
int KICK_CYCLES = 3;            // How many 200Hz cycles to accelerate
float BRAKE_TORQUE = 0.02f;     // Decelerate torque amplitude (Nm)
int BRAKE_CYCLES = 2;           // How many 200Hz cycles to brake
float TORQUE_DEADBAND = 0.002f; // Minimum torque required to trigger a reversal

int backlash_state[3] = {1, 1, 1};    // 1 = engaged positive, -1 = engaged negative
int kick_timer[3] = {0, 0, 0};        // Countdown timers for active kicks

//____________Switches____________
Bounce debouncer = Bounce();
bool control_run = false;  

//____________Oscilloscope Verification____________
bool osc_state = false;

//______SD Card Reading______
SdFat sd;
FsFile my_file;

/*========================CORE LOGIC========================*/

void setup() {
  delay(1000);
  Serial.begin(9600);
  Serial.println("------------Beginning Setup------------");

  /*____________________________IMU____________________________*/
  Wire.begin();
  while (!myISM.begin()) {
    Serial.println("Failed to connect to IMU!");
    delay(500);
  }

  myISM.deviceReset();
  while (!myISM.getDeviceReset()) {
    delay(1);
  }

  Serial.println("IMU successfully reset\nApplying settings");
  delay(100);

  myISM.setDeviceConfig();
  myISM.setBlockDataUpdate();
  myISM.setAccelDataRate(ISM_XL_ODR_208Hz);
  myISM.setAccelFullScale(ISM_2g);
  myISM.setGyroDataRate(ISM_GY_ODR_208Hz);
  myISM.setGyroFullScale(ISM_500dps);

  /*____________________________IMU CALIBRATION____________________________*/
  imu_filter.begin();
  imu_filter.setBias(0.0062981257f, -0.0121280579f, 0.0001833806f);

  myTimer.begin(IMU_ISR, imu_period);

  /*_________________________CAN/MOTORS_______________________*/
  Can2.begin();
  Can2.setBaudRate(500000);  
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

  Serial.println("------------Completed Setup.------------");

  /*_______________SD CARD SETUP_____________________________*/
  Serial.println("Starting SD card");
  while (!sd.begin(10, SD_SCK_MHZ(1))) {
    Serial.println("FISH");
    delay(100);
  }
  Serial.println("CAUGHT SOME FISH!");
  
  printActiveGains();
}

void loop() {
  Can2.events();
  checkSerialCommands();

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

  if (imu_ready) {
    imu_ready = false;  

    myISM.getAccel(&accelData);
    myISM.getGyro(&gyroData);

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

    if (control_run) {
      run_controller();
    }
  }
}

/**
 * @brief Executes LQR math and outputs torque to the motors using live K_xy gains
 * Includes Feed-Forward Backlash Compensation (Kick & Brake)
 */
void run_controller() {
  uint32_t current_time = micros();

  osc_state = !osc_state;
  digitalWriteFast(OSC_PIN, osc_state);

  float dt = (current_time - last_time) / 1000000.0f;
  if (last_time == 0) dt = 1.0 / imu_timer_freq;
  last_time = current_time;

  // 1. Read true velocities
  float psd[3] = {
    motor_true_vels[0] / 27.0f,
    motor_true_vels[1] / 27.0f,
    motor_true_vels[2] / 27.0f
  };

  // 2. ENCODER SKIP: Mask velocity if the motor is currently kicking through the deadzone
  for (int i = 0; i < 3; i++) {
    if (kick_timer[i] > 0) {
      psd[i] = 0.0f; 
    }
  }

  float sX = sin(filtered_roll);
  float cX = cos(filtered_roll);
  float sY = sin(filtered_pitch);
  float cY = cos(filtered_pitch);

  // Calculate ball kinematics using the (potentially masked) velocities
  float phi_dot_x = calc_phi_dot_x(psd[0], psd[1], psd[2], gyro_x, sX, cX, sY, cY);
  float phi_dot_y = calc_phi_dot_y(psd[0], psd[1], psd[2], gyro_y, sX, cX);

  phi_x += phi_dot_x * dt;
  phi_y += phi_dot_y * dt;

  // LQR calculation 
  float Tx = -(K_xy[0] * phi_x + K_xy[1] * filtered_roll + K_xy[2] * phi_dot_x + K_xy[3] * gyro_x);
  float Ty = (K_xy[0] * phi_y + K_xy[1] * filtered_pitch + K_xy[2] * phi_dot_y + K_xy[3] * gyro_y);
  float Tz = (K_z[1] * gyro_z);

  float T_cmd[3];
  T_cmd[0] = (1.0 / (3.0 * 27)) * (Tz + (2.0 / cA) * (Tx * cB - Ty * sB));
  T_cmd[1] = (1.0 / (3.0 * 27)) * (Tz + (1.0 / cA) * (sB * (-SQRT_3 * Tx + Ty) - cB * (Tx + SQRT_3 * Ty)));
  T_cmd[2] = (1.0 / (3.0 * 27)) * (Tz + (1.0 / cA) * (sB * (SQRT_3 * Tx + Ty) + cB * (-Tx + SQRT_3 * Ty)));
  
  // 3. FEED-FORWARD KICK & BRAKE LOGIC
  for (int i = 0; i < 3; i++) {
    
    // ----- ARE WE CURRENTLY IN A KICK/BRAKE SEQUENCE? -----
    if (kick_timer[i] > 0) {
      
      // Phase 1: The Acceleration Kick
      if (kick_timer[i] > BRAKE_CYCLES) {
        if (backlash_state[i] == 1) {
          T_cmd[i] = KICK_TORQUE;  
        } else {
          T_cmd[i] = -KICK_TORQUE; 
        }
      } 
      // Phase 2: The Braking Pulse (Reverse Torque)
      else {
        if (backlash_state[i] == 1) {
          T_cmd[i] = -BRAKE_TORQUE; 
        } else {
          T_cmd[i] = BRAKE_TORQUE;  
        }
      }
      
      kick_timer[i]--; // Countdown the timer
    } 
    
    // ----- IF NORMAL, WATCH FOR DIRECTION REVERSALS -----
    else {
      // Reversing from Positive to Negative
      if (backlash_state[i] == 1 && T_cmd[i] < -TORQUE_DEADBAND) {
        backlash_state[i] = -1;                                // 1. Flip state
        kick_timer[i] = KICK_CYCLES + BRAKE_CYCLES;            // 2. Start total timer
        T_cmd[i] = -KICK_TORQUE;                               // 3. Start kicking
      } 
      // Reversing from Negative to Positive
      else if (backlash_state[i] == -1 && T_cmd[i] > TORQUE_DEADBAND) {
        backlash_state[i] = 1;                                 // 1. Flip state
        kick_timer[i] = KICK_CYCLES + BRAKE_CYCLES;            // 2. Start total timer
        T_cmd[i] = KICK_TORQUE;                                // 3. Start kicking
      }
    }
    
    // Constrain final torques for safety
    T_cmd[i] = constrain(T_cmd[i], -MAX_TORQUE, MAX_TORQUE);
  }

  // Send the compensated torques
  send_torque(MOTOR_1, T_cmd[0]);
  send_torque(MOTOR_2, T_cmd[1]);
  send_torque(MOTOR_3, T_cmd[2]);

  // Buffer logging 
  if (buff_pointer < max_buff_size) {
    time_buffer[buff_pointer] = micros();
    roll_buffer[buff_pointer] = filtered_roll;
    pitch_buffer[buff_pointer] = filtered_pitch;
    gyroX_buffer[buff_pointer] = gyro_x;
    gyroY_buffer[buff_pointer] = gyro_y;
    gyroZ_buffer[buff_pointer] = gyro_z;
    T1_buffer[buff_pointer] = T_cmd[0];
    T2_buffer[buff_pointer] = T_cmd[1];
    T3_buffer[buff_pointer] = T_cmd[2];
    phi_x_buffer[buff_pointer] = phi_x;
    phi_y_buffer[buff_pointer] = phi_y;
    phi_dx_buffer[buff_pointer] = phi_dot_x;
    phi_dy_buffer[buff_pointer] = phi_dot_y;
    
    // Log the TRUE unmasked velocities for later dead-zone measurement analysis
    motor_vel_1_buffer[buff_pointer] = motor_true_vels[0];
    motor_vel_2_buffer[buff_pointer] = motor_true_vels[1];
    motor_vel_3_buffer[buff_pointer] = motor_true_vels[2];
    motor_pos_1_buffer[buff_pointer] = motor_true_pos[0];
    motor_pos_2_buffer[buff_pointer] = motor_true_pos[1];
    motor_pos_3_buffer[buff_pointer] = motor_true_pos[2];
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
    Serial.println("Invalid format! Use: k0, k1, k2, k3, T_max, kt, kc, bt, bc, td = val");
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
  } else if (cmd.equalsIgnoreCase("T_max")) {
    MAX_TORQUE = val;
    printActiveGains();  
  } else if (cmd.equalsIgnoreCase("kt")) {
    KICK_TORQUE = val;
    printActiveGains();  
  } else if (cmd.equalsIgnoreCase("kc")) {
    KICK_CYCLES = (int)val;
    printActiveGains();  
  } else if (cmd.equalsIgnoreCase("bt")) {
    BRAKE_TORQUE = val;
    printActiveGains();  
  } else if (cmd.equalsIgnoreCase("bc")) {
    BRAKE_CYCLES = (int)val;
    printActiveGains();  
  } else if (cmd.equalsIgnoreCase("td")) {
    TORQUE_DEADBAND = val;
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
  Serial.printf("k0 (Ball Position):  %.4f\n", K_xy[0]);
  Serial.printf("k1 (Tilt Angle):     %.4f\n", K_xy[1]);
  Serial.printf("k2 (Ball Velocity):  %.4f\n", K_xy[2]);
  Serial.printf("k3 (Gyro Rate):      %.4f\n", K_xy[3]);
  Serial.printf("T_max:               %.4f N\n", MAX_TORQUE);
  Serial.println("--- Backlash Compensation ---");
  Serial.printf("Kick Torque  (kt):   %.4f Nm\n", KICK_TORQUE);
  Serial.printf("Kick Cycles  (kc):   %d\n", KICK_CYCLES);
  Serial.printf("Brake Torque (bt):   %.4f Nm\n", BRAKE_TORQUE);
  Serial.printf("Brake Cycles (bc):   %d\n", BRAKE_CYCLES);
  Serial.printf("Torq Deadband(td):   %.4f Nm\n", TORQUE_DEADBAND);
  Serial.println("-----------------------------------------");
}

void shutdown() {
  Serial.println("------------Exit Requested------------");
  send_torque(MOTOR_1, 0.0);
  send_torque(MOTOR_2, 0.0);
  send_torque(MOTOR_3, 0.0);

  axis_state = 1;
  set_motors_states(axis_state);  

  save_all_data_to_one_CSV();
  buff_pointer = 0;
  Serial.println("------------Safely Exited Program------------");
}

void IMU_ISR() {
  imu_ready = true;
}

void can_sniff(const CAN_message_t& msg) {
  uint8_t node = msg.id >> 5;   
  uint8_t cmd = msg.id & 0x1F;  

  if (cmd == GET_ENCODER) {
    motor_true_pos[node - 1] = *reinterpret_cast<const float*>(msg.buf);
    motor_true_vels[node - 1] = *reinterpret_cast<const float*>(msg.buf + 4);
  } else if (cmd == GET_ERROR) {
    uint32_t errors_raw = *reinterpret_cast<const uint32_t*>(msg.buf + 4);
    uint32_t errors = __builtin_bswap32(errors_raw);  
    if (errors == 0) return;

    if (!errored) {
      shutdown();
      Serial.println("!!!!!!!!!!!!ERROR DETECTED!!!!!!!!!!!!");
      errored = true;
    }

    Serial.printf("Motor %d:\n", node);
    if (errors & ERROR_OVERVOLTAGE)    Serial.println("  - OVERVOLTAGE");
    if (errors & ERROR_UNDERVOLTAGE)   Serial.println("  - UNDERVOLTAGE");
    if (errors & ERROR_DC_OVERCURRENT) Serial.println("  - DC_OVERCURRENT");
    if (errors & ERROR_OVERREGEN)      Serial.println("  - OVER_REGEN");
    if (errors & ERROR_OVERCURRENT)    Serial.println("  - OVERCURRENT");
    if (errors & ERROR_ESTOP_REQ)      Serial.println("  - ESTOP REQ");
    Serial.println();
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

  Serial.print("Creating master log: ");
  Serial.println(filename);

  my_file = sd.open(filename, FILE_WRITE);
  if (!my_file) {
    Serial.println("Failed to open master log!");
    return;
  }

  my_file.println("Time_us,Roll,Pitch,GyroX,GyroY,GyroZ,T1,T2,T3,PhiX,PhiY,PhiDX,PhiDY,motor_vel_1,motor_vel_2,motor_vel_3,motor_pos_1,motor_pos_2,motor_pos_3");

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
    my_file.println(motor_pos_3_buffer[i], 4);
    
  }

  my_file.close();
  Serial.println("Master log saved. FISH ARE IN ONE BUCKET!");
}