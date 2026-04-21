//========================INCLUDES========================
#include <FlexCAN_T4.h>
#include <stdio.h>


//========================GLOBAL STUFF========================
FlexCAN_T4<CAN2, RX_SIZE_256, TX_SIZE_16> Can2;
const CAN_message_t incoming_message;

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

//========================DEFINITIONS========================
// Setup of communication, serial port, and CAN
void setup() {
    delay(1000);
    Serial.begin(115200); // serial monitor
    Serial.println("------------Beginning Setup------------");

    Can2.begin();
    Can2.setBaudRate(250000);

    // Upon recieving a message, sniff
    // NOTE: mailboxes used are default and not set up manually
    Can2.onReceive(can_sniff);

    // set absolute position to 0
    reset_positions();

    // Activate motors for input
    ready_motors();

    Serial.println("------------Completed Setup.------------");
}

// main loop
void loop() {
    Can2.events();
    
    if (runcount < runs_total){
        // Benchtesting
        Serial.println("Beginning Testing in");
        Serial.println("3...");
        delay(1000);
        Serial.println("2...");
        delay(1000);
        Serial.println("1...");
        delay(1000);

        // Send 0.1N*m
        float torque = 0.1; // N*m
        send_torque(MOTOR_3, torque);
        delay(2000); // in ms

        // Send 0.2 N*m
        torque = 0.2;
        send_torque(MOTOR_3, torque);
        delay(2000);

        // Turn off
        send_torque(MOTOR_3, 0);
        delay(1000);

        Serial.println("Done sending");
        if (runcount == (runs_total - 1)){
            Serial.println("DONE WITH ALL LOOPS");
            idle_motors();
        }

        runcount++;
        delay(1000);
    }
    
}

/*  Function:    print_CAN_frame
    Behavior:    Prints to the serial the message ID and data sent over CAN (debugging)
    Arguments:   CAN_message_t msg  - CAN message being sent
    Returns:     None
    Errors:      None
*/
void print_CAN_frame(CAN_message_t msg){
    char serial_buffer[100];
        sprintf(serial_buffer, "ID: 0x%03X  Data:", msg.id);

        Serial.print(serial_buffer);
        for (int i = 0; i < msg.len; i++) {
            Serial.print(" 0x");
            if (msg.buf[i] < 0x10) Serial.print("0"); // leading zero
            Serial.print(msg.buf[i], HEX);
        }
        Serial.println();
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


/*  Function:    ready_motors
    Behavior:    Aids in initialization. Changes the state of all drivers to "ready" (flashing green) instead of "idle" (blue)
    Arguments:   None
    Returns:     None
    Errors:      None
*/
void idle_motors(void){
    // set axis state. 1 = idle, 8 = ready

    // Motor 1
    CAN_message_t msg;
    msg.id = MOTOR_1 | READY_MOTOR;
    msg.len = 1; // one byte
    int32_t axis_state = 1;
    memcpy(msg.buf, &axis_state, 1); // copy one byte
    if (Can2.write(msg)) {
        print_CAN_frame(msg);
    } 
    else {
        Serial.println("CAN ready send failed: M1");
    }
    delay(10);

    // Motor 2
    msg.id = MOTOR_2 | READY_MOTOR;
    msg.len = 1;
    if (Can2.write(msg)) {
        print_CAN_frame(msg);
    } 
    else {
        Serial.println("CAN ready send failed: M2");
    }
    delay(10);

    // Motor 3
    msg.id = MOTOR_3 | READY_MOTOR;
    msg.len = 1;
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


/*  Function:   can_sniff
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
    Serial.println(node);
    Serial.println(cmd);
    if (cmd == GET_ENCODER){
        // Extract data
        memcpy(&pos, msg.buf, 4);
        memcpy(&vel, msg.buf + 4, 4);

        // Assign data
        if(node == 1) m1_true_vel = vel;
        if(node == 2) m2_true_vel = vel;
        if(node == 3) m3_true_vel = vel;

        // FOR PRINTING ONLY
        char serial_buffer[50];
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