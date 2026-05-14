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

SdFat sd;
FsFile my_file;



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
#define ERROR_ESTOP_REQ (0x00000002)
#define ERROR_OVERVOLTAGE (0x00000100)
#define ERROR_UNDERVOLTAGE (0x00000200)
#define ERROR_DC_OVERCURRENT (0x00000400)
#define ERROR_OVERREGEN (0x00000800)
#define ERROR_OVERCURRENT (0x00001000)
// Oscilloscope
#define OSC_PIN 16


//------Global Variables------
//float motor_true_torques; DISABLED, only needed for data collection
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;  // wired to CAN2
int MOTOR_IDS[3] = { MOTOR_1, MOTOR_2, MOTOR_3 };
volatile float motor_true_vels[3];  // index 0 is motor 1, and so on
int axis_state = 1;                 // default to idle
float vel;                          // placeholder variables for recieving data from encoders
bool errored = false;


//____________IMU____________
SparkFun_ISM330DHCX myISM;
sfe_ism_data_t accelData;  // accel data storage variable filtered
sfe_ism_data_t gyroData;   // gyro data storage variable filtered

IntervalTimer myTimer;
int imu_timer_freq = 160;                   //hz
int imu_period = 1000000 / imu_timer_freq;  //convert to seconds, rounds to floor of quintent, us

//____________IMU Filter____________
const float ACCEL_SCALE = 1.0f / 1000.0f * 9.81f;         // SparkFun outputs mg -> m/s^2
const float GYRO_SCALE = (1.0f / 1000.0f) * PI / 180.0f;  // SparkFun outputs mdps -> rad/s
const float BETA = 0.02f;                                 // Madgwick tuning param
Filter imu_filter((float)imu_timer_freq, BETA);
volatile bool imu_ready = false;

//_____________IMU Buffers________________
int buff_pointer = 0;
const int max_buff_size = 160 * 5;  //the number after 160 is the number of seconds
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
// float phi_dz_buffer[max_buff_size];

// Controller inputs — updated at 160 Hz in loop()
float filtered_roll = 0.0f;   // radians
float filtered_pitch = 0.0f;  // radians
float gyro_x = 0.0f;          // rad/s
float gyro_y = 0.0f;          // rad/s
float gyro_z = 0.0f;          // rad/s (yaw rate, bias-subtracted)

//____________LQR & Controller Setup____________
uint32_t last_time = 0;
float phi_x = 0.0;
float phi_y = 0.0;

// Kinematic Params
const float rW = 0.048;          // wheel radius (m)
const float rB = 0.12;           // ball radius (m)
const float alpha_rad = 0.785;   // 45 degrees
const float beta_rad = 0.0;      // Alignment offset
const float MAX_TORQUE = 0.20f;  // N*m

// Precompute constants
const float SQRT_2 = 1.41421356f;
const float SQRT_3 = 1.73205081f;
const float SQRT_6 = 2.44948974f;
const float CSC_35 = 1.74344679f; // 1.0 / sin(35 deg)
const float SEC_35 = 1.22077458f; // 1.0 / cos(35 deg)

// LQR Controller Gains
const float gain_mod = 0.65;
const float gain_mod_2 = 0.35;
const float K_xy[4] = { 0, -gain_mod*29.6151, -gain_mod*0.5554, -gain_mod_2*14.6805 };
// const float K_xy[4] = {gain_mod*-1.2527, gain_mod*-140.9692, gain_mod*-3.2801, gain_mod*-70.3089};
// const float K_xy[4] = {gain_mod*0, gain_mod*-140.9692, gain_mod*-0, gain_mod*-70.3089};
const float K_z[2] = { -1, -1.0357 };

//____________Switches____________
Bounce debouncer = Bounce();
bool control_run = false;  // false by default

//____________Oscilloscope Verification____________
bool osc_state = false;


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
  /*____________________________IMU____________________________*/

  Wire.begin();

  // if connection fails say so
  while (!myISM.begin()) {
    Serial.println("Failed to connect to IMU!");
  }

  // Reset device to default settings
  myISM.deviceReset();
  // wait for reset to complete
  while (!myISM.getDeviceReset()) {
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

	// Enable hardware low-pass filters
	myISM.setGyroFilterLP1(true);
	myISM.setGyroLP1Bandwidth(ISM_VERY_LIGHT);  // ~58 Hz cutoff at 208 Hz ODR
	myISM.setAccelSlopeFilter(ISM_LP_ODR_DIV_10);  // accel LPF at ODR/10 ≈ 21 Hz

  /*____________________________IMU CALIBRATION____________________________*/
  // Keep IMU perfectly still during this period
  imu_filter.begin();
  imu_filter.setBias(0.0062981257f, -0.0121280579f, 0.0001833806f);

  myTimer.begin(IMU_ISR, imu_period);

  /*_________________________CAN/MOTORS_______________________*/
  // Start bus on existing CAN2
  Can2.begin();
  Can2.setBaudRate(500000);  // 500KB/sec

  // Enable recieving messages
  Can2.enableMBInterrupts();
  Can2.onReceive(can_sniff);

  //______Motor Care______
  motors_reset_position();        // sets absolute positions to 0, even if not needed
  axis_state = 1;                 // idles
  set_motors_states(axis_state);  // change from idle -> ready (flashing green)

  /*_________________________SWITCHES_______________________*/
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  debouncer.attach(SWITCH_PIN);
  debouncer.interval(2);  // debounce time, in ms

  pinMode(OSC_PIN, OUTPUT);


  /*_________________________ACK SETUP_______________________*/
  Serial.println("------------Completed Setup.------------");

  /*_______________SD CARD SETUP_____________________________*/
  Serial.println("Starting SD card");

  while (!sd.begin(10, SD_SCK_MHZ(1))) {
    Serial.println("FISH");
    delay(100);
  }

  Serial.println("CAUGHT SOME FISH!");
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
      shutdown();
    } else {
      Serial.println("Control Switch is ON - Engaging LQR");
      clear_errors();  // clear any existing errors on the motors
      errored = false;

      // Reset integration and timing when switched on to prevent jolts
      axis_state = 8;  // ready motors
      set_motors_states(axis_state);
      phi_x = 0.0;
      phi_y = 0.0;
      last_time = micros() - imu_period;
    }
  }

  // 160Hz Execution Block triggered by the Timer ISR
  if (imu_ready) {
    imu_ready = false;  // Reset the flag

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
    filtered_roll = -imu_filter.getRoll();
    filtered_pitch = -imu_filter.getPitch();
    gyro_x = -gx;
    gyro_y = -gy;
    gyro_z = -(gz - 0.0001833806f);  // bias-subtracted


    // 2. Run the controller (Only if the switch is ON)
    if (control_run) {
      run_controller();
    }
  }
}

/**
 * @brief Executes LQR math and outputs torque to the motors
 */
void run_controller() {
  uint32_t current_time = micros();

  // Invert state of oscilloscope pin. this should have a freq of 1 BTI
  osc_state = !osc_state;
  digitalWriteFast(OSC_PIN, osc_state);

  // Calculate dynamic dt
  float dt = (current_time - last_time) / 1000000.0f;
  if (last_time == 0) dt = 1.0 / 160.0;
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
  // float phi_dot_z = calc_phi_dot_z(psd1, psd2, psd3, gyro_x, gyro_z, sX, cX, sY, cY);

  // Integrate velocity
  phi_x += phi_dot_x * dt;
  phi_y += phi_dot_y * dt;

  // LQR calculation (u = -Kx)
  float Tx = -(K_xy[0] * phi_x + K_xy[1] * filtered_roll + K_xy[2] * phi_dot_x + K_xy[3] * gyro_x);
  float Ty = (K_xy[0] * phi_y + K_xy[1] * filtered_pitch + K_xy[2] * phi_dot_y + K_xy[3] * gyro_y);
  float Tz = (K_z[1] * gyro_z);

  // Torque conversion
  float cA = cos(alpha_rad);
  float cB = cos(beta_rad);
  float sB = sin(beta_rad);

  float T1 = (1.0 / (3.0 * 27)) * (Tz + (2.0 / cA) * (Tx * cB - Ty * sB));
  float T2 = (1.0 / (3.0 * 27)) * (Tz + (1.0 / cA) * (sB * (-SQRT_3 * Tx + Ty) - cB * (Tx + SQRT_3 * Ty)));
  float T3 = (1.0 / (3.0 * 27)) * (Tz + (1.0 / cA) * (sB * (SQRT_3 * Tx + Ty) + cB * (-Tx + SQRT_3 * Ty)));

  // Applying saturation
  T1 = constrain(T1, -MAX_TORQUE, MAX_TORQUE);
  T2 = constrain(T2, -MAX_TORQUE, MAX_TORQUE);
  T3 = constrain(T3, -MAX_TORQUE, MAX_TORQUE);

  // Command the drives
  send_torque(MOTOR_1, 1 * T1);
  send_torque(MOTOR_2, 1 * T2);
  send_torque(MOTOR_3, 1 * T3);

  // save a point to the buffer if buffers are not maxed out
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
    // phi_dz_buffer[buff_pointer] = phi_dot_z;

    buff_pointer++;
  }

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
void shutdown() {
  Serial.println("------------Exit Requested------------");
  //------Motors------
  send_torque(MOTOR_1, 0.0);
  send_torque(MOTOR_2, 0.0);
  send_torque(MOTOR_3, 0.0);

  axis_state = 1;
  set_motors_states(axis_state);  // idle

  // Save all data to one file
  save_all_data_to_one_CSV();

  // Clear buffers and reset pointer
  buff_pointer = 0;

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
void IMU_ISR() {
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
void can_sniff(const CAN_message_t& msg) {
  /*
    ODRIVE CAN FRAME ID (11 bits)
    10  9  8  7  6  5 |  4  3  2  1  0
          msg ID           cmd_id 
    */
  uint8_t node = msg.id >> 5;   // shift over to find the node/origin
  uint8_t cmd = msg.id & 0x1F;  // only look at the cmd_id region

  // Directly assign data from the message buffer
  if (cmd == GET_ENCODER) {
    // Use an array to map node numbers to velocity variables
    motor_true_vels[node - 1] = *reinterpret_cast<const float*>(msg.buf + 4);
  } else if (cmd == GET_ERROR) {
    // ignore cyclic message if it is zero
    uint32_t errors_raw = *reinterpret_cast<const uint32_t*>(msg.buf + 4);
    uint32_t errors = __builtin_bswap32(errors_raw);  // swap for endianness
    if (errors == 0) {
      // No error → ignore completely
      return;
    }

    // error detected, kill ISR if it hasn't already done so. print once
    if (!errored) {
      shutdown();
      Serial.println("!!!!!!!!!!!!ERROR DETECTED!!!!!!!!!!!!");
    }

    // print error message(s)
    Serial.print("Motor ");
    Serial.print(node);
    Serial.println(":");
    if (errors & ERROR_OVERVOLTAGE) {
      Serial.println("  - OVERVOLTAGE");
    }
    if (errors & ERROR_UNDERVOLTAGE) {
      Serial.println("  - UNDERVOLTAGE");
    }
    if (errors & ERROR_DC_OVERCURRENT) {
      Serial.println("  - DC_OVERCURRENT");
    }
    if (errors & ERROR_OVERREGEN) {
      Serial.println("  - OVER_REGEN");
    }
    if (errors & ERROR_OVERCURRENT) {
      Serial.println("  - OVERCURRENT");
    }
    if (errors & ERROR_ESTOP_REQ) {
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
void send_torque(int MOTOR, float torque) {
  CAN_message_t msg;
  msg.id = MOTOR | SET_TORQUE;  // eg 0x03 shifted | 0x09 = 000011 01110 bin = 110 dec = 0x6E (0x4E for 2, 0x2E for 1)
  msg.len = 4;
  memcpy(msg.buf, &torque, 4);
  Can2.write(msg);
}

void clear_errors(void) {
  CAN_message_t msg;
  msg.len = 4;
  int flash = 0;  // flashes when identifying. 0 = no, 1 = true
  memcpy(msg.buf, &flash, 4);
  for (int i = 0; i < 3; ++i) {
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

/**
 * @brief Aids in initialization. Makes all positions of the motors equal to 0.
 * 
 * @return None
 * 
 * @note Prints error messages to Serial
 */
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


//==============KINEMATIC HELPERS==============
float calc_phi_dot_x(float dp1, float dp2, float dp3, float dthx, float sX, float cX, float sY, float cY) {
    // Broken into terms for readability and to match the equation structure perfectly
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

// float calc_phi_dot_x(float dp1, float dp2, float dp3, float dthx, float sX, float cX, float sY, float cY) {
//   return (1.0/(3.0*rB)) * ( (SQRT_6*rW*sX*sY*(-dp2+dp3)) + (SQRT_2*rW*cX*sY*(dp1+dp2+dp3)) + (cY*(SQRT_2*rW*(-2.0*dp1+dp2+dp3) + 3.0*rB*dthx)) );
// }
// float calc_phi_dot_y(float dp1, float dp2, float dp3, float dthy, float sX, float cX) {
//   return (1.0/(3.0*rB)) * ( (SQRT_6*rW*cX*(-dp2+dp3)) - (SQRT_2*rW*sX*(dp1+dp2+dp3)) + (3.0*rB*dthy) );
// }

// float calc_phi_dot_z(float dp1, float dp2, float dp3, float dthx, float dthz, float sX, float cX, float sY, float cY) {
//   return (1.0/(3.0*rB)) * ( (SQRT_2*rW*(cX*cY + 2.0*sY)*dp1) + (SQRT_2*rW*(SQRT_3*cY*sX*(-dp2+dp3) + cX*cY*(dp2+dp3) - sY*(dp2+dp3))) + (3.0*rB*(-sY*dthx + dthz)) );
// }

// SD Card Helper
/**
 * @brief Saves buffer to a csv file
 * 
 * @return None
 * 
 * @note T data: the buffer to save, must be an array, can be of any type.
        const char* half_name: the name of the file to save the data into, remember to include"
 */
// template<typename T>
// void save_data_2_SD(T data, const char* half_name, unsigned long save_time) {
//   Serial.print("Checking the quality of the fish in: ");  // letting the user know what file is currently being openned
//   Serial.println(half_name);

//   char name[40];
//   sprintf(name, "%lu_%s.csv", save_time, half_name);
//   Serial.println(name);

//   if (sd.exists(name)) {
//     sd.remove(name);  // delete the previous file so it is clean
//   }

//   my_file = sd.open(name, FILE_WRITE);
//   if (!my_file) {
//     Serial.println("File failed to open");
//   }

//   // writing all of the data to the respective file
//   int writing_pointer = 0;
//   while (writing_pointer < buff_pointer) {
//     my_file.println(data[writing_pointer]);
//     writing_pointer++;
//   }

//   my_file.close();
//   Serial.println("THROW THE FISH INTO THE OCEAN!");  //SD card is safe to remove
//   delay(100);
// }

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

  // 1. Write the Header Row
  my_file.println("Time_us,Roll,Pitch,GyroX,GyroY,GyroZ,T1,T2,T3,PhiX,PhiY,PhiDX,PhiDY");

  // 2. Write the Data Rows
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
    my_file.println(phi_dy_buffer[i], 4); // println for the end of the row
  }

  my_file.close();
  Serial.println("Master log saved. FISH ARE IN ONE BUCKET!");
}