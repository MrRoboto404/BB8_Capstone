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
#define MOTOR_STATE (0x07)
#define SET_ABS_POS (0x19)

//______Global Variables______
unsigned long test_start_time = 0; // Stores the start of the motion

int MOTOR_IDS[3] = {MOTOR_1, MOTOR_2, MOTOR_3};
float motor_true_vels[3]; // index 0 is motor 1, and so on
float motor_true_torqs[3];
float motor_torq_commands[3];

int runcount = 0;
int t_total = 2;
float frequency = 0.9; // Hz, cannot exceed 0.9
float peak_torque = 0.1; // N*m, don't change
int flag = 1;

int sleep_time = 1000 / frequency;

//____________ PI Controller Setup ____________
IntervalTimer controlTimer;
float dt_ctrl = 1.0/160.0; // (160Hz)
float BTI = 1000000*dt_ctrl; // 

// PI Controller coef.
float Kp = 0.001; 
float Ki = 0.003;

// Pre-calculate Tustin coefficients (eq.7.32 - Garbini et al.)
float b0 = Kp + (0.5 * Ki * dt_ctrl);
float b1 = -Kp + (0.5 * Ki * dt_ctrl);

// State variables for discrete PI
// e - error signal (angular velocity)
// u - torque command
float e_k_1 = 0.0;
float e_k_1_minus_1 = 0.0;
float u_k_1 = 0.0;
float u_k_1_minus_1 = 0.0;

float e_k_2 = 0.0;
float e_k_2_minus_1 = 0.0;
float u_k_2 = 0.0;
float u_k_2_minus_1 = 0.0;

float e_k_3 = 0.0;
float e_k_3_minus_1 = 0.0;
float u_k_3 = 0.0;
float u_k_3_minus_1 = 0.0;
// Targets and Limits
int n_gear = 27;
float target_vel_rads_1 = 0.0;
float target_vel_rads_2 = 0.0;
float target_vel_rads_3 = 0.0;
float max_torque = 0.26;        // SAFETY LIMIT: Max Nm commanded to ODrive


//========================DEFINITIONS========================
// Setup of communication, serial port, and CAN
void setup() {
    delay(1000);
    Serial.begin(115200); // serial monitor
    Serial.println("------------Beginning Setup------------");

    // Start CAN bus
    Can2.begin();
    Can2.setBaudRate(250000);

    // Enable reading the CAN bus
    // Upon recieving a message, sniff
    Can2.enableMBInterrupts(); // enable interrupts
    Can2.onReceive(can_sniff); // allows all FIFO/message box messages to be received in the supplied callback.

    // set absolute position to 0
    reset_positions();

    // Activate motors for input
    ready_motors();

    // PI CONTROLLER TIMER 
    // Start the 5ms (5000 microsecond) PI control loop
    controlTimer.begin(PI_Control_ISR, BTI);


    Serial.println("------------Completed Setup.------------");
    Serial.println("Beginning Testing in");
    Serial.println("3...");
    delay(1000);
    Serial.println("2...");
    delay(1000);
    Serial.println("1...");
    delay(1000);
    Serial.println("Time, rad_t_1, rad_a_1, T_cmd_1, rad_t_2, rad_a_2, T_cmd_2, rad_t_3, rad_a_3, T_cmd_3");
    test_start_time = millis(); // <--- Add this here
}

// main loop
void loop() {
    Can2.events();

    static unsigned long start_time = millis();
    float t = (millis() - start_time) / 1000.0;

    if (t < t_total){
        if (millis() > 4500) {
            target_vel_rads_1 = n_gear * 20 * PI / 30;
            target_vel_rads_2 = n_gear * 20 * PI / 30;
            target_vel_rads_3 = n_gear * 20 * PI / 30;
        }

        // Print data to the Serial Plotter every 10ms (100Hz)
        static uint32_t last_print = 0;
        if (millis() - last_print > 10) {
            last_print = millis();
            // Calculate seconds as a float (e.g., 1.02 seconds)
            float elapsed_seconds = (millis() - test_start_time) / 1000.0;

            // Print Time first
            Serial.print(elapsed_seconds, 3); // 3 decimal places for millisecond precision
            Serial.print(", ");

            // Print format for Arduino Serial Plotter: "Var1:value Var2:value"
            Serial.print(target_vel_rads_1);
            Serial.print(", ");
            Serial.print(motor_true_vels[0] * 2.0 * PI);
            Serial.print(", ");
            Serial.print(u_k_1, 5);
            Serial.print(", ");
            Serial.print(target_vel_rads_2);
            Serial.print(", ");
            Serial.print(motor_true_vels[1] * 2.0 * PI);
            Serial.print(", ");
            Serial.print(u_k_2, 5);
            Serial.print(", ");
            Serial.print(target_vel_rads_3);
            Serial.print(", ");
            Serial.print(motor_true_vels[2] * 2.0 * PI);
            Serial.print(", ");
            Serial.println(u_k_3, 5);
        }
    }
    if ((t > t_total) && (flag == 1)){
        controlTimer.end(); // Stop the PI loop interrupt
        send_torque(MOTOR_1, 0);
        send_torque(MOTOR_2, 0);
        send_torque(MOTOR_3, 0);
        idle_motors();
        Serial.println("~~~~~~~~~~Done with test~~~~~~~~~~");
        flag = 0;
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
        //print_CAN_frame(msg);
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
    msg.id = MOTOR_1 | MOTOR_STATE;
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
    msg.id = MOTOR_2 | MOTOR_STATE;
    msg.len = 4;
    if (Can2.write(msg)) {
        print_CAN_frame(msg);
    } 
    else {
        Serial.println("CAN ready send failed: M2");
    }
    delay(10);

    // Motor 3
    msg.id = MOTOR_3 | MOTOR_STATE;
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
    msg.id = MOTOR_1 | MOTOR_STATE;
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
    msg.id = MOTOR_2 | MOTOR_STATE;
    msg.len = 1;
    if (Can2.write(msg)) {
        print_CAN_frame(msg);
    } 
    else {
        Serial.println("CAN ready send failed: M2");
    }
    delay(10);

    // Motor 3
    msg.id = MOTOR_3 | MOTOR_STATE;
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
void can_sniff(const CAN_message_t &msg) { // global declaration
    /*
    ODRIVE CAN FRAME ID (11 bits)
    10  9  8  7  6  5 |  4  3  2  1  0
          msg ID           cmd_id 
    */
    node = msg.id >> 5; // shift over to find the node/origin
    cmd  = msg.id & 0x1F; // only look at the cmd_id region by setting those to 1 and the rest 0, then &
    if (cmd == GET_ENCODER) { // as minimal code as possble
        motor_true_vels[node - 1] = *reinterpret_cast<const float*>(msg.buf + 4);
    }
    else if (cmd == GET_TORQUE) { // as minimal code as possble
        motor_torq_commands[node - 1] = *reinterpret_cast<const float*>(msg.buf);
        motor_true_torqs[node - 1] = *reinterpret_cast<const float*>(msg.buf + 4);
    }
}

// MAIN MOTOR CONTROL LOOP
void PI_Control_ISR() {
    // Get current velocity and convert rev/s to rad/s
    float actual_vel_rads_1 = motor_true_vels[0] * 2.0 * PI;
    float actual_vel_rads_2 = motor_true_vels[1] * 2.0 * PI;
    float actual_vel_rads_3 = motor_true_vels[2] * 2.0 * PI; 

    // Error signal
    e_k_1 = target_vel_rads_1 - actual_vel_rads_1;
    e_k_2 = target_vel_rads_2 - actual_vel_rads_2;
    e_k_3 = target_vel_rads_3 - actual_vel_rads_3;

    // Evaluate Tustin difference equation
    u_k_1 = u_k_1_minus_1 + (b0 * e_k_1) + (b1 * e_k_1_minus_1);
    u_k_2 = u_k_2_minus_1 + (b0 * e_k_2) + (b1 * e_k_2_minus_1);
    u_k_3 = u_k_3_minus_1 + (b0 * e_k_3) + (b1 * e_k_3_minus_1);

    // Saturate
    u_k_1 = constrain(u_k_1, -max_torque, max_torque);
    u_k_2 = constrain(u_k_2, -max_torque, max_torque);
    u_k_3 = constrain(u_k_3, -max_torque, max_torque);
    
    // Send Torque to ODrive
    send_torque(MOTOR_1, u_k_1);
    send_torque(MOTOR_2, u_k_2);
    send_torque(MOTOR_3, u_k_3);

    // Shift states
    e_k_1_minus_1 = e_k_1;
    u_k_1_minus_1 = u_k_1;
    e_k_2_minus_1 = e_k_2;
    u_k_2_minus_1 = u_k_2;
    e_k_3_minus_1 = e_k_3;
    u_k_3_minus_1 = u_k_3;
}