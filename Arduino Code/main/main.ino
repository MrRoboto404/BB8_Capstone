/*========================INCLUDES========================*/
#include <FlexCAN_T4.h>
#include <stdio.h>
#include <SPI.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>
#include <Bounce2.h>
#include "Filter.h"





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
// Control loop switch
#define SWITCH_PIN 14

//------Global Variables------
//____________CAN____________
//float motor_true_torques; DISABLED, only needed for data collection
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2; // wired to CAN2
int MOTOR_IDS[3] = {MOTOR_1, MOTOR_2, MOTOR_3};
float motor_true_vels[3]; // index 0 is motor 1, and so on
int axis_state = 1; // default to idle
float vel; // placeholder variables for recieving data from encoders


//____________IMU____________
SparkFun_ISM330DHCX myISM;
sfe_ism_data_t accelData; // accel data storage variable filtered
sfe_ism_data_t gyroData; // gyro data storage variable filtered

IntervalTimer myTimer;
int imu_timer_freq = 160; //hz
int imu_period = 1000000/imu_timer_freq; //convert to seconds, rounds to floor of quintent, us

//____________IMU Filter____________
const float wb  = 1.1;
const float tau = 1.0 / wb;
const float dt  = 1.0 / 160;
const int CALIB_SAMPLES = 10000; // gyro bias calibration samples

const float ACCEL_SCALE = 0.061e-3 * 9.81;  // ±2g: 0.061 mg/LSB
const float GYRO_SCALE  = 17.5e-3 * PI/180; // ±500 dps

float pitch_filtered = 0.0;
float roll_filtered  = 0.0;
float gx_bias = 0.0;
float gy_bias = 0.0;
bool filter_initialized = false;

//____________Switches____________
Bounce debouncer = Bounce();
bool control_run = false; // false by default



/*========================DEFINITIONS========================*/

// Core stuff

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
  SPI.begin();

  // pinMode(mag_CS, OUTPUT); // set mag chip select pin to output mode
  pinMode(ism_CS,OUTPUT); // set accel chip select pin to output mode
  // digitalWrite(mag_CS, HIGH); // set mag chip select pin to high as mag is low active
  digitalWrite(ism_CS, HIGH); // set ism chip select pin to high as ism is low active

  // if connection fails say so
  while(!myISM.begin(ism_CS)){
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
  Can2.setBaudRate(250000); // 250KB/sec

  // Enable recieving messages
  Can2.enableMBInterrupts();
  Can2.onReceive(can_sniff);

  //______Motor Care______
  motors_reset_position(); // sets absolute positions to 0, even if not needed
  axis_state = 8; // ready
  set_motors_states(axis_state); // change from idle -> ready (flashing green)

  /*_________________________SWITCHES_______________________*/
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  debouncer.attach(SWITCH_PIN);
  debouncer.interval(25); // debounce time, in ms


  /*_________________________ACK SETUP_______________________*/
  Serial.println("------------Completed Setup.------------");
}

/** 
* @brief Main loop that is in all .ino files
*
* @return None
*
* @note Errors not yet implemented
*/
void loop() {
  /* 
  controller should create copy of data stored in accelData and gyroData 
  so that way values are not changing halfway through calculations
  */

  // Update Madgwick filter when ISR has ticked
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
  }
	

  // Detect if control switch has been flicked
  debouncer.update();
  if (debouncer.changed()) {
    // INPUT_PULLUP: LOW = pressed
    control_run = (debouncer.read() == LOW);
  }


  //______Main Loopable stuff______
  if (control_run){
    // controller code goes here
    Can2.events();

    // note: motor velocities are global vars and are updated up to 1kHz automatically
    // 
  }
  else if (!control_run){
    Serial.println("Control Switch is OFF");
  }

}

/**
 * @brief Used to cleanly shutdown components under normal conditions
 * 
 * @note Prints to serial
 */
void cleanup(void){
  Serial.println("------------Safe Exit Requested------------");
  //------Motors------
  axis_state = 1;
  set_motors_states(axis_state);

  //------IMU Stuff------

  //------Acknowledge------
  Serial.println("------------Safely Exited Program------------");
}

// IMU stuff

/*
Behavior: interrupt service routine that runs every time the imu timer ends
Errors: none
Returns: none
Arguments: none
*/
void IMU_ISR(){
  imu_ready = true;
}

// CAN stuff

/**
 * @brief Allows the Teensy to update global encoder variables at calling speed. Optional, as default cyclic messages can go up to 1kHz
 * 
 * @returns None
 */
void demand_all_encoders(){
    CAN_message_t msg;
    msg.len = 0; // no payload
    msg.flags.remote = 1; // sets RTR

    for {int i = 0; i<3; ++i}{
        msg.id = MOTOR_IDS[i] | GET_ENCODER;
        Can2.write(msg);
    }
}

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
    
}

/**
 * @brief Constructs and sends a CAN frame to a target motor with specified torque.
 * 
 * @param MOTOR Hex 'node' identifier
 * @param torque Desired motor torque in N*m
 * 
 * @return None
 * 
 * @note Prints error message to serial if unable to send frame.
 */
void send_torque(int MOTOR, float torque){
    CAN_message_t msg;
    msg.id = MOTOR | SET_TORQUE; // eg 0x03 shifted | 0x09 = 000011 01110 bin = 110 dec = 0x6E (0x4E for 2, 0x2E for 1)
    msg.len = 4;
    memcpy(msg.buf, &torque, 4);
    Can2.write(msg);
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
    // Motor 1
    CAN_message_t msg;
    msg.id = MOTOR_1 | MOTOR_STATE;
    msg.len = 4;
    memcpy(msg.buf, &axis_state, 4);
    if (Can2.write(msg)) {
        print_CAN_frame(msg);
    } 
    else {
        Serial.println("CAN axis state send failed: M1");
    }
    delay(10);

    // Motor 2
    msg.id = MOTOR_2 | MOTOR_STATE;
    msg.len = 4;
    if (Can2.write(msg)) {
        print_CAN_frame(msg);
    } 
    else {
        Serial.println("CAN axis state send failed: M2");
    }
    delay(10);

    // Motor 3
    msg.id = MOTOR_3 | MOTOR_STATE;
    msg.len = 4;
    if (Can2.write(msg)) {
        print_CAN_frame(msg);
    } 
    else {
        Serial.println("CAN axis state send failed: M3");
    }
    delay(10);
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

    // Motor 1
    CAN_message_t msg;
    msg.id = MOTOR_1 | SET_ABS_POS;
    msg.len = 4;
    memcpy(msg.buf, &set_zero, 4);
    if (Can2.write(msg)) {
        print_CAN_frame(msg);
    } 
    else {
        Serial.println("CAN pos reset send failed: M1");
    }

    // Motor 2
    msg.id = MOTOR_2 | SET_ABS_POS;
    if (Can2.write(msg)) {
        print_CAN_frame(msg);
    } 
    else {
        Serial.println("CAN pos reset send failed: M2");
    }

    // Motor 3
    msg.id = MOTOR_3 | SET_ABS_POS;
    if (Can2.write(msg)) {
        print_CAN_frame(msg);
    } 
    else {
        Serial.println("CAN pos reset send failed: M3");
    }
}

