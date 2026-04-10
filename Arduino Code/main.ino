/*____________________________INCLUDES_________________________*/
#include <FlexCAN_T4.h>
#include <stdio.h>
#include <SPI.h>
#include <SparkFun_ISM330DHCX.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>


/*_____________________________Variables_____________________*/
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;
// CAN IDs correctly bit-shifted
#define MOTOR_1 (0x01 << 5)
#define MOTOR_2 (0x02 << 5)
#define MOTOR_3 (0x03 << 5)
#define SET_TORQUE (0x00E)
#define GET_ENCODER (0x009)
#define GET_TORQUE (0x01C)
#define CLEAR_ERROR (0x018)

float m1_true_torque;
float m2_true_torque;
float m3_true_torque;
float m1_true_vel;
float m2_true_vel;
float m3_true_vel;

// byte mag_CS = 9;
byte ism_CS = 10;

SparkFun_ISM330DHCX_SPI myISM; // our accelerameter


sfe_ism_data_t accelData; // accel data storage variable filtered
sfe_ism_data_t gyroData; // gyro data storage variable filtered

IntervalTimer myTimer;
int imu_timer_freq = 160; //hz
int imu_period = 1000000/imu_timer_freq; //convert to seconds, rounds to floor of quintent, us

// ======= Complementary filter =======
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

void setup() {
  Serial.begin(9600);
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
  Serial.println("Calibrating gyro bias...");
  for (int i = 0; i < CALIB_SAMPLES; i++) {
    myISM.getGyro(&gyroData);
    gx_bias += gyroData.xData * GYRO_SCALE;
    gy_bias += gyroData.yData * GYRO_SCALE;
    delay(2);
  }
  gx_bias /= CALIB_SAMPLES;
  gy_bias /= CALIB_SAMPLES;
  Serial.println("Calibration complete");

  myTimer.begin(IMU_ISR, imu_period);

  /*_________________________CAN/MOTORS_______________________*/
  Can0.begin();
  Can0.setBaudRate(250000);

  // clear any possible errors
  clear_errors(MOTOR_1);
  clear_errors(MOTOR_2);
  clear_errors(MOTOR_3);

  // Upon recieving a message, sniff
  // NOTE: mailboxes used are default and not set up manually
  Can0.onReceive(can_sniff);

}

void loop() {
  /* 
  controller should create copy of data stored in accelData and gyroData 
  so that way values are not changing halfway through calculations
  */

  Can0.events();

  /*
  1. read encoder states & write to m1,2,3_true_torque,vel
  2. use corresponding control loop stuff here to calc torques
  3. use send_torque for each motor
  */


}

/*
Behavior: interrupt service routine that runs every time the imu timer ends
Errors: none
Returns: none
Arguments: none
*/
void IMU_ISR(){
  myISM.getAccel(&accelData);
  myISM.getGyro(&gyroData);

  // Scale to physical units
  float ax = accelData.xData * ACCEL_SCALE;
  float ay = accelData.yData * ACCEL_SCALE;
  float az = accelData.zData * ACCEL_SCALE;

  float gx = gyroData.xData * GYRO_SCALE - gx_bias;
  float gy = gyroData.yData * GYRO_SCALE - gy_bias;

  // Accelerometer angle estimates (radians)
  float accel_roll_raw  = atan2(ay, sqrt(ax*ax + az*az));
  float accel_pitch_raw = atan2(-ax, sqrt(ay*ay + az*az));

  // Initialize filter on first run
  if (!filter_initialized) {
    roll_filtered  = accel_roll_raw;
    pitch_filtered = accel_pitch_raw;
    filter_initialized = true;
    return;
  }

  // Complementary filter
  float alpha    = tau / (tau + dt);
  roll_filtered  = alpha * (roll_filtered  + gx * dt) + (1.0 - alpha) * accel_roll_raw;
  pitch_filtered = alpha * (pitch_filtered + gy * dt) + (1.0 - alpha) * accel_pitch_raw;
}

/*  
  Function:   send_torque
  Behavior:   Constructs and sends a CAN frame to a target motor a specified torque in N*m
  Arguments:  int MOTOR    - hex 'node' identifier
              float torque - desired motor torque, in N*m
  Returns:    None
  Errors:     Not yet implemented 
*/
void send_torque(int MOTOR, float torque){
  CAN_message_t msg;
  msg.id = MOTOR | SET_TORQUE;
  msg.len = 4;
  memcpy(msg.buf, &torque, 4);

  Can0.write(msg);
}

/*  
  Function:   clear_errors
  Behavior:   Constructs and sends a CAN frame to clear possible errors from a target drive
  Arguments:  int MOTOR    - hex 'node' identifier
  Returns:    None
  Errors:     Not yet implemented 
*/
void clear_errors(int MOTOR){
  CAN_message_t msg;
  msg.id = MOTOR | CLEAR_ERROR;
  msg.len = 4;
  msg.buf = [0, 0, 0, 0];

  Can0.write(msg)
}

/*  
  Function:   can_sniff
  Behavior:   Processes CAN frames sent by a node to the teensy
  Arguments:  CAN_message_t &msg  - the address of the incoming message
  Returns:    None
  Errors:     Not yet implemented 
*/
void can_sniff(const CAN_message_t &msg) {
  /*
  ODRIVE CAN FRAME ID (11 bits)
  10  9  8  7  6  5 |  4  3  2  1  0
        msg ID           cmd_id 
  */
  float pos, vel, torq;
  uint8_t node = msg.id >> 5; // shift over to find the node/origin
  uint8_t cmd  = msg.id & 0x1F; // only look at the cmd_id region by setting those to 1 and the rest 0, then &
  if (cmd == GET_ENCODER){
    // Extract data
    memcpy(&pos, msg.buf, 4);
    memcpy(&vel, msg.buf + 4, 4);

    // Assign data
    if(node == 1) m1_true_vel = vel;
    if(node == 2) m2_true_vel = vel;
    if(node == 3) m3_true_vel = vel;

    // FOR PRINTING ONLY
    char serial_buffer = [50];
    sprintf(serial_buffer, "Motor %d   Pos: %f   Vel %f", node, pos, vel);
    Serial.println(serial_buffer);

  }
  // Torque data: disabled. should only be enabled while collecting data for plotting (not needed for control loop)
  /*
  if (cmd == GET_TORQUE){
      // Extract data
      memcpy(&torq, msg.buf, 4);

      // Assign data
      if(node == 1) m1_true_torque = torq;
      if(node == 2) m2_true_torque = torq;
      if(node == 3) m3_true_torque = torq;

      // FOR PRINTING ONLY
      char serial_buffer = [50];
      sprintf(serial_buffer, "Motor %d   Torque: %f", node, torq);
      Serial.println(serial_buffer);
  }
  */
    
}
