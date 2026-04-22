//========================INCLUDES========================
#include <FlexCAN_T4.h>
#include <stdio.h>

//========================GLOBAL STUFF========================
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;

//______Constants______
// CAN IDs correctly bit-shifted
#define MOTOR_1 (0x01 << 5)
#define MOTOR_2 (0x02 << 5)
#define MOTOR_3 (0x03 << 5)
// CAN ODrive commands
#define SET_TORQUE (0x00E)
#define GET_ENCODER (0x009)
#define GET_TORQUE (0x01C)
#define CLEAR_ERROR (0x018)
#define E_STOP (0x02)
#define READY_MOTOR (0x07)
#define SET_ABS_POS (0x19)

//______Global Variables______
float m1_true_torque;
float m2_true_torque;
float m3_true_torque;
float m1_true_vel;
float m2_true_vel;
float m3_true_vel;

int runcount = 0;
int runs_total = 1;
int frequency = 2; // Hz
float peak_torque = 0.1; // N*m, don't change

int sleep_time = 1000 / frequency;

//______Constants______
const double PI = 3.14159265358979323846;

//========================DEFINITIONS========================
// Setup of communication, serial port, and CAN
void setup() {
    delay(1000);
    Serial.begin(115200); // serial monitor
    Serial.println("------------Beginning Setup------------");

    Can2.begin();
    Can2.setBaudRate(250000);

    // set absolute position to 0
    reset_positions();

    // Activate motors for input
    ready_motors();

    Serial.println("------------Completed Setup.------------");
    Serial.println("Beginning Testing in");
    Serial.println("3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);
}

// main loop
void loop() {
    Can2.events();
    
    if (runcount < runs_total){
        static unsigned long start_time = millis();
        float t = (millis() - start_time) / 1000.0;

        float cmd = peak_torque * sin(2 * PI * frequency * t);

        send_torque(MOTOR_2, cmd);

        delay(sleep_time);
    }
    
}


/*  Function:   send_torque
    Behavior:   Constructs and sends a CAN frame to a target motor a specified torque in N*m
    Arguments:  int MOTOR    - hex 'node' identifier
                float torque - desired motor torque, in N*m
    Returns:    None
    Errors:     Not yet implemented 
*/
void send_torque(int MOTOR, float torque){
    CAN_message_t msg;
    msg.id = MOTOR | SET_TORQUE; // eg 0x03 shifted | 0x09 = 000011 01110 bin = 110 dec = 0x6E (0x4E for 2, 0x2E for 1)
    msg.len = 4;
    memcpy(msg.buf, &torque, 4);

    if (Can2.write(msg)) {
        print_CAN_frame(msg);
    } 
    else {
        Serial.println("CAN torque send failed");
    }
}

/*  Function:    ready_motors
    Behavior:    Aids in initialization. Changes the state of all drivers to "ready" (flashing green) instead of "idle" (blue)
    Arguments:   None
    Returns:     None
    Errors:      None
*/
void ready_motors(void){
    // set axis state. 1 = idle, 8 = ready

    // Motor 1
    CAN_message_t msg;
    msg.id = MOTOR_1 | READY_MOTOR;
    msg.len = 4;
    int axis_state = 8;
    memcpy(msg.buf, &axis_state, 4);
    if (Can2.write(msg)) {
        print_CAN_frame(msg);
    } 
    else {
        Serial.println("CAN ready send failed: M1");
    }
    delay(10);

    // Motor 2
    msg.id = MOTOR_2 | READY_MOTOR;
    msg.len = 4;
    if (Can2.write(msg)) {
        print_CAN_frame(msg);
    } 
    else {
        Serial.println("CAN ready send failed: M2");
    }
    delay(10);

    // Motor 3
    msg.id = MOTOR_3 | READY_MOTOR;
    msg.len = 4;
    if (Can2.write(msg)) {
        print_CAN_frame(msg);
    } 
    else {
        Serial.println("CAN ready send failed: M3");
    }
    delay(10);
}


/*  Function:    reset_positions
    Behavior:    Aids in initialization. Makes all positions of the motors = 0
    Arguments:   None
    Returns:     None
    Errors:      None
*/
void reset_positions(void){
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
