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

//float motor_true_torques; DISABLED, only needed for data collection
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;  // wired to CAN2
int MOTOR_IDS[3] = { MOTOR_1, MOTOR_2, MOTOR_3 };
volatile float motor_true_vels[3];  // index 0 is motor 1, and so on
int axis_state = 1;                 // default to idle
float vel;                          // placeholder variables for recieving data from encoders
bool errored = false;

//______SD Card______
SdFat sd;
FsFile my_file;


/*_________________________SWITCHES_______________________*/
Bounce debouncer = Bounce();
bool control_run = false;  // false by default






/*========================Core stuff========================*/

void setup(){
  delay(1000);
  Serial.begin(9600);
  Serial.println("------------Beginning Setup------------");
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
  
  //____Switches____
  pinMode(SWITCH_PIN, INPUT_PULLUP);
  debouncer.attach(SWITCH_PIN);
  debouncer.interval(2);  // debounce time, in ms

  /*_______________SD CARD SETUP_____________________________*/
  Serial.println("Starting SD card");

  while (!sd.begin(10, SD_SCK_MHZ(1))) {
    Serial.println("FISH");
    delay(100);
  }

  Serial.println("CAUGHT SOME FISH!");

  /*_________________________ACK SETUP_______________________*/
  Serial.println("------------Completed Setup.------------");
}


void loop(){
  Can2.events();
  
  debouncer.update();
  if (debouncer.changed()){
    control_run = (debouncer.read() == LOW);

    if (!control_run){
      Serial.println("Control Switch is OFF - Disabling Torques");
      shutdown();
    }
    else{
      Serial.println("Control Switch is ON - Starting Impulse in 3...");
      delay(1000);
      Serial.println("2...");
      delay(1000);
      Serial.println("1...");
      delay(1000);
    }
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

