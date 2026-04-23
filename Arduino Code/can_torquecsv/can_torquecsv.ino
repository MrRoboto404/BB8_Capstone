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

float resp_times[] = {
    0.000,0.001,0.004,0.008,0.012,0.016,0.020,0.023,0.027,0.031,0.035,0.039,0.043,0.047,0.051,0.055,0.058,0.062,0.066,0.070,0.074,0.078,0.082,0.086,0.090,0.094,0.098,0.102,0.106,0.110,0.114,0.118,0.122,0.126,0.130,0.134,0.138,0.142,0.146,0.150,0.154,0.158,0.162,0.166,0.170,0.174,0.178,0.182,0.186,0.190,0.194,0.198,0.202,0.206,0.210,0.214,0.218,0.222,0.226,0.230,0.234,0.238,0.242,
    0.246,0.250,0.254,0.258,0.262,0.266,0.270,0.274,0.278,0.282,0.286,0.290,0.294,0.298,0.302,0.306,0.310,0.314,0.318,0.322,0.326,0.330,0.334,0.338,0.342,0.346,0.350,0.354,0.358,0.362,0.366,0.370,0.374,0.378,0.382,0.386,0.390,0.394,0.398,0.402,0.406,0.410,0.414,0.418,0.422,0.426,0.430,0.434,0.438,0.442,0.446,0.450,0.454,0.458,0.462,0.466,0.470,0.474,0.478,0.482,0.486,0.490,0.494,
    0.498,0.502,0.506,0.510,0.514,0.518,0.522,0.526,0.530,0.534,0.538,0.542,0.546,0.550,0.554,0.558,0.562,0.566,0.570,0.574,0.578,0.582,0.586,0.590,0.594,0.598,0.602,0.606,0.610,0.614,0.618,0.622,0.626,0.630,0.634,0.638,0.642,0.646,0.650,0.654,0.658,0.662,0.666,0.670,0.674,0.678,0.682,0.686,0.690,0.694,0.698,0.702,0.706,0.710,0.714,0.718,0.722,0.726,0.730,0.734,0.738,0.742,0.746,
    0.750,0.754,0.758,0.762,0.766,0.770,0.774,0.778,0.782,0.786,0.790,0.794,0.798,0.802,0.806,0.810,0.814,0.818,0.822,0.826,0.830,0.834,0.838,0.842,0.846,0.850,0.854,0.858,0.862,0.866,0.870,0.874,0.878,0.882,0.886,0.890,0.894,0.898,0.902,0.906,0.910,0.914,0.918,0.922,0.926,0.930,0.934,0.938,0.942,0.946,0.950,0.954,0.958,0.962,0.966,0.970,0.974,0.978,0.982,0.986,0.990,0.994,0.998,
    1.002,1.006,1.010,1.014,1.018,1.022,1.026,1.030,1.034,1.038,1.042,1.046,1.050,1.054,1.058,1.062,1.066,1.070,1.074,1.078,1.082,1.086,1.090,1.094,1.098,1.102,1.106,1.110,1.114,1.118,1.122,1.126,1.130,1.134,1.138,1.142,1.146,1.150,1.154,1.158,1.162,1.166,1.170,1.174,1.178,1.182,1.186,1.190,1.194,1.198,1.202,1.206,1.210,1.214,1.218,1.222,1.226,1.230,1.234,1.238,1.242,1.246,1.250,
    1.254,1.258,1.262,1.266,1.270,1.274,1.278,1.282,1.286,1.290,1.294,1.298,1.302,1.306,1.310,1.314,1.318,1.322,1.326,1.330,1.334,1.338,1.342,1.346,1.350,1.354,1.358,1.362,1.366,1.370,1.374,1.378,1.382,1.386,1.390,1.394,1.398,1.402,1.406,1.410,1.414,1.418,1.422,1.426,1.430,1.434,1.438,1.442,1.446,1.450,1.454,1.458,1.462,1.466,1.470,1.474,1.478,1.482,1.486,1.490,1.494,1.498,1.502,
    1.506,1.510,1.514,1.518,1.522,1.526,1.530,1.534,1.538,1.542,1.546,1.550,1.554,1.558,1.562,1.566,1.570,1.574,1.578,1.582,1.586,1.590,1.594,1.598,1.602,1.606,1.610,1.614,1.618,1.622,1.626,1.630,1.634,1.638,1.642,1.646,1.650,1.654,1.658,1.662,1.666,1.670,1.674,1.678,1.682,1.686,1.690,1.694,1.698,1.702,1.706,1.710,1.714,1.718,1.722,1.726,1.730,1.734,1.738,1.742,1.746,1.750,1.754,
    1.758,1.762,1.766,1.770,1.774,1.778,1.782,1.786,1.790,1.794,1.798,1.802,1.806,1.810,1.814,1.818,1.822,1.826,1.830,1.834,1.838,1.842,1.846,1.850,1.854,1.858,1.862,1.866,1.870,1.874,1.878,1.882,1.886,1.890,1.894,1.898,1.902,1.906,1.910,1.914,1.918,1.922,1.926,1.930,1.934,1.938,1.942,1.946,1.950,1.954,1.958,1.962,1.966,1.970,1.974,1.978,1.982,1.986,1.990,1.994,1.998,2.000
};

float resp_torques[] = {
    -0.02,-0.02,-0.1,-0.1,-0.1,-0.2,-0.257462687,-0.257462687,-0.218330857,-0.184173307,-0.154338608,-0.128260747,-0.105447877,-0.085472567,-0.067963343,-0.052597343,-0.04034988,-0.028606351,-0.018784217,-0.010584935,-0.003755411,0.001918856,0.006619587,0.010500665,0.013692332,0.016304765,0.018431107,0.02015005,0.021528028,0.022621087,
    0.023476465,0.024133946,0.024627001,0.024983761,0.025227845,0.025379056,0.025453981,0.025466489,0.025428164,0.025348659,0.025236009,0.025096884,0.024936814,0.024760366,0.024571308,0.024372732,0.024167174,0.023956698,0.023742983,0.023527382,0.023310981,0.023094644,0.022879054,0.022664742,0.022452117,0.022241487,0.022033078,0.021827051,0.021623513,0.021422529,0.021224128,0.021028317,
    0.020835079,0.020644383,0.020456186,0.020270437,0.020087077,0.019906047,0.019727283,0.01955072,0.019376295,0.019203945,0.019033607,0.018865223,0.018698734,0.018534084,0.018371222,0.018210097,0.01805066,0.017892866,0.017736674,0.017582041,0.01742893,0.017277304,0.017127129,0.016978374,0.016831007,0.016685,0.016540326,0.016396959,0.016254875,0.016114051,0.015974466,0.015836098,0.015698929,
    0.015562939,0.015428111,0.015294428,0.015161874,0.015030434,0.014900093,0.014770836,0.01464265,0.014515523,0.014389441,0.014264392,0.014140365,0.014017349,0.013895332,0.013774304,0.013654255,0.013535175,0.013417054,0.013299882,0.013183651,0.013068351,0.012953975,0.012840512,0.012727955,0.012616296,0.012505526,0.012395638,0.012286624,0.012178477,0.012071188,0.011964752,0.011859159,
    0.011754405,0.011650481,0.01154738,0.011445097,0.011343623,0.011242954,0.011143081,0.011043999,0.010945702,0.010848183,0.010751437,0.010655456,0.010560235,0.010465768,0.01037205,0.010279074,0.010186834,0.010095325,0.010004542,0.009914478,0.009825128,0.009736487,0.00964855,0.00956131,0.009474763,0.009388903,0.009303725,0.009219224,0.009135395,0.009052232,0.008969731,0.008887887,0.008806694,
    0.008726148,0.008646244,0.008566976,0.008488341,0.008410333,0.008332948,0.008256181,0.008180028,0.008104482,0.008029542,0.0079552,0.007881454,0.007808298,0.007735728,0.00766374,0.007592329,0.007521491,0.007451222,0.007381517,0.007312372,0.007243783,0.007175745,0.007108255,0.007041308,0.006974901,0.006909028,0.006843687,0.006778873,0.006714581,0.006650809,0.006587552,0.006524806,0.006462567,
    0.006400832,0.006339597,0.006278857,0.00621861,0.00615885,0.006099576,0.006040782,0.005982466,0.005924623,0.00586725,0.005810343,0.005753899,0.005697915,0.005642386,0.005587309,0.005532681,0.005478499,0.005424758,0.005371456,0.005318589,0.005266154,0.005214148,0.005162566,0.005111406,0.005060665,0.00501034,0.004960426,0.004910922,0.004861823,0.004813128,0.004764831,0.004716931,0.004669425,
    0.004622309,0.004575581,0.004529236,0.004483273,0.004437689,0.00439248,0.004347643,0.004303176,0.004259076,0.00421534,0.004171965,0.004128948,0.004086286,0.004043977,0.004002018,0.003960407,0.00391914,0.003878214,0.003837628,0.003797378,0.003757462,0.003717877,0.003678621,0.003639691,0.003601085,0.003562799,0.003524832,0.003487181,0.003449843,0.003412817,0.003376098,0.003339687,0.003303578,
    0.003267771,0.003232263,0.003197052,0.003162135,0.00312751,0.003093175,0.003059127,0.003025364,0.002991884,0.002958685,0.002925764,0.00289312,0.002860749,0.002828651,0.002796822,0.002765261,0.002733965,0.002702933,0.002672162,0.002641651,0.002611396,0.002581397,0.002551651,0.002522156,0.002492911,0.002463912,0.002435159,0.002406649,0.002378381,0.002350352,0.002322561,0.002295005,0.002267684,
    0.002240594,0.002213734,0.002187103,0.002160698,0.002134518,0.002108561,0.002082825,0.002057309,0.00203201,0.002006927,0.001982059,0.001957403,0.001932957,0.001908721,0.001884693,0.00186087,0.001837252,0.001813836,0.001790621,0.001767606,0.001744788,0.001722167,0.001699741,0.001677507,0.001655465,0.001633614,0.001611951,0.001590475,0.001569184,0.001548078,0.001527154,0.001506412,0.001485849,
    0.001465465,0.001445257,0.001425225,0.001405367,0.001385682,0.001366168,0.001346824,0.001327649,0.001308641,0.001289799,0.001271122,0.001252608,0.001234256,0.001216065,0.001198033,0.001180159,0.001162442,0.001144881,0.001127475,0.001110221,0.00109312,0.001076169,0.001059368,0.001042715,0.001026209,0.001009849,0.000993635,0.000977564,0.000961635,0.000945848,0.000930201,0.000914693,0.000899323,
    0.00088409,0.000868993,0.000854031,0.000839203,0.000824507,0.000809942,0.000795508,0.000781204,0.000767028,0.000752979,0.000739057,0.00072526,0.000711587,0.000698038,0.000684611,0.000671306,0.000658121,0.000645055,0.000632108,0.000619279,0.000606566,0.000593969,0.000581486,0.000569118,0.000556862,0.000544718,0.000532686,0.000520764,0.000508951,0.000497246,0.000485649,0.000474159,0.000462774,
    0.000451495,0.00044032,0.000429248,0.000418279,0.000407411,0.000396645,0.000385978,0.00037541,0.000364941,0.00035457,0.000344296,0.000334117,0.000324034,0.000314046,0.000304151,0.000294349,0.00028464,0.000275022,0.000265495,0.000256058,0.000246711,0.000237452,0.000228281,0.000219197,0.0002102,0.000201288,0.000192462,0.00018372,0.000175061,0.000166486,0.000157993,0.000149582,0.000141252,
    0.000133003,0.000124833,0.000116742,0.00010873,0.000100795,9.29378E-05,8.51571E-05,7.74524E-05,6.9823E-05,6.22683E-05,5.47877E-05,4.73807E-05,4.00466E-05,3.27848E-05,2.55947E-05,1.84758E-05,1.14275E-05,4.44916E-06,-2.45972E-06,-9.29973E-06,-1.60714E-05,-2.27754E-05,-2.94121E-05,-3.59822E-05,-4.24862E-05,-4.89247E-05,-5.5298E-05,-6.16069E-05,-6.78518E-05,-7.40333E-05,-8.01518E-05,-8.62079E-05,
    -9.22021E-05,-9.81349E-05,-0.000104007,-0.000109818,-0.00011557,-0.000121262,-0.000126895,-0.00013247,-0.000137987,-0.000143446,-0.000148848,-0.000154194,-0.000159484,-0.000164718,-0.000169897,-0.000175021,-0.000180091,-0.000185107,-0.000190069,-0.000194979,-0.000199837,-0.000204642,-0.000209395,-0.000214098,-0.000218749,-0.00022335,-0.000227902,-0.000232404,-0.000236856,-0.00024126,-0.000245616,
    -0.000249923,-0.000254183,-0.000258396,-0.000262563,-0.000266683,-0.000270757,-0.000274785,-0.000278768,-0.000282707,-0.000286601,-0.000290451,-0.000294257,-0.00029802,-0.000301739,-0.000305417,-0.000306953
};

#define DATA_LEN (sizeof(resp_times) / sizeof(resp_times[0]))
uint32_t traj_start_us = 0;   // use micros for precision
int traj_index = 0;
bool traj_running = false;

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
}

// main loop
void loop() {
    Can2.events();

    static bool started = false;

    if (!started) {
        Serial.println("Beginning Testing in");
        Serial.println("3...");
        delay(1000);
        Serial.println("2...");
        delay(1000);
        Serial.println("1...");
        delay(1000);
        start_trajectory();
        started = true;
    }

    run_trajectory(MOTOR_2);
}

void start_trajectory() {
    traj_start_us = micros();
    traj_index = 0;
    traj_running = true;

    Serial.println("Trajectory started");
}

void run_trajectory(int MOTOR) {
    if (!traj_running) return;

    uint32_t now_us = micros();
    float elapsed_s = (now_us - traj_start_us) / 1e6; // convert to seconds

    // Send all points that are due
    while (traj_index < DATA_LEN && elapsed_s >= resp_times[traj_index]) {

        float torque = resp_torques[traj_index];
        send_torque(MOTOR, torque);

        /* Debug print (optional)
        Serial.print("t=");
        Serial.print(elapsed_s, 6);
        Serial.print("  cmd=");
        Serial.println(torque, 6);
        */
        traj_index++;
    }

    // End condition
    if (traj_index >= DATA_LEN) {
        Serial.println("Trajectory complete");
        traj_running = false;

        send_torque(MOTOR, 0); // safety stop
        idle_motors();
        
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
void can_sniff(const CAN_message_t &msg) { // global declaration
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