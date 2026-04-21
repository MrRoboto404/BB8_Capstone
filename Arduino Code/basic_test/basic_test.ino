//========================INCLUDES========================
#include <FlexCAN_T4.h>
#include <stdio.h>


//========================GLOBAL STUFF========================
FlexCAN_T4<CAN1, RX_SIZE_256, TX_SIZE_16> Can0;

//______Constants______
// CAN IDs correctly bit-shifted
#define MOTOR_1 (0x01 << 5)
#define MOTOR_2 (0x02 << 5)
#define MOTOR_3 (0x03 << 5)
#define SET_TORQUE (0x00E)
#define GET_ENCODER (0x009)
#define GET_TORQUE (0x01C)
#define CLEAR_ERROR (0x018)

//______Global Variables______
float m1_true_torque;
float m2_true_torque;
float m3_true_torque;
float m1_true_vel;
float m2_true_vel;
float m3_true_vel;

//========================DEFINITIONS========================
// Setup of communication, serial port, and CAN
void setup() {
  Serial.begin(9600); // USB is always 12 or 480 Mbit/sec
}

void loop() {
  Serial.println("Hello World...");
  delay(1000);  // do not print too fast!
}