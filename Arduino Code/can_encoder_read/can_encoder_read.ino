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
    Serial.begin(115200); // serial monitor

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

// main loop
void loop() {
    Can0.events();

    /*
    1. read encoder states & write to m1,2,3_true_torque,vel
    2. use corresponding control loop stuff here to calc torques
    3. use send_torque for each motor
    */
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
    msg.id = MOTOR | SET_TORQUE;
    msg.len = 4;
    memcpy(msg.buf, &torque, 4);

    Can0.write(msg);
}

/*  Function:   clear_errors
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