/************************************************************************************
 This program is used to control the motors in the 'Follow a Trajectory'
 mode where the trajectory can be a simple cosine function or a complex
 trajectory like that of a human hip while walking.

 In this mode, you can record the motor's position as it follows the
 reference trajectory and plot it to verify.

Author(s):
  Sai Hein Su Thu
  Alexander Martin-Ginnold
  Joshua Ramayrat

Reference:
  https://store.cubemars.com/images/file/20211201/1638329381542610.pdf
  https://store.tmotor.com/images/file/20220216/1644991915235842.pdf
  https://docs.espressif.com/projects/esp-idf/en/release-v3.3/api-reference/peripherals/can.html

  servo mode has 6 control modes:

  duty cycle
  current loop
  current brake
  velocity
  position
  position velocity
************************************************************************************/

/**********************          WHEN UPLOADING CODE          **********************/
// If you're uploading this code to ESP32 from VS Code, you must hold the EN
// button. Orient your ESP so that the microUSB port is on the top. The EN
// button will then be the button on the left of the mciroUSB port. Hold the
// button while uploading the code. You can let go of the button when you see
// "Writing (xx %)...." in the terminal.


#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
#include <stdint.h>
#include <time.h>
#include <iostream>
#include <math.h>
#include <string>
#include <HardwareSerial.h>

using namespace std;


/*******  Global Variables  *******/
CAN_device_t CAN_cfg;          // for configuring our CAN comm
const int rx_queue_size = 10;  // num of CAN msgs that can be in queue
CAN_frame_t tx_frame;  // tx_frame is the variable tht represents the CAN msg to be sent to motors

// Following variables are used in the loop() func below
//long t_pre_loop;  // can be declared as static var later: t_pre_loop, now, last_time
//long now;
//long last_time;
//int run_time = 3*PI * pow(10, 6);      // run for x seconds (s)
//float dt = 20 * pow(10, 3);           // dt required between each motor command, in ms
//const int SIZE = (run_time / dt) + 1;  // one extra just in case


/*******  Constants  *******/
// These constants match the AK80-64 motor. Change these to match your motor.
const float P_MIN = -12.5;
const float P_MAX = 12.5;
const float V_MIN = -8;
const float V_MAX = 8;
const float T_MIN = -144;
const float T_MAX = 144;
const float Kp_MIN = 0;
const float Kp_MAX = 500;
const float Kd_MIN = 0;
const float Kd_MAX = 5;
const float Test_Pos= 0.0;



/********************* Start Servo mode code ***************************/
typedef enum {
  CAN_PACKET_SET_DUTY = 0,      // Duty cycle mode
  CAN_PACKET_SET_CURRENT,       // Current loop mode
  CAN_PACKET_SET_CURRENT_BRAKE, // Current brake mode
  CAN_PACKET_SET_RPM,           // Velocity mode
  CAN_PACKET_SET_POS,           // Position mode
  CAN_PACKET_SET_ORIGIN_HERE,   // Set origin mode
  CAN_PACKET_SET_POS_SPD,       // Position velocity loop mode
} CAN_PACKET_ID;


typedef enum {
  COMM_FW_VERSION = 0,
  COMM_JUMP_TO_BOOTLOADER,
  COMM_ERASE_NEW_APP,
  COMM_WRITE_NEW_APP_DATA,
  COMM_GET_VALUES,            // Get motor running parameters
  COMM_SET_DUTY,              // Motor runs in duty cycle mode
  COMM_SET_CURRENT,           // Motor runs in current loop mode
  COMM_SET_CURRENT_BRAKE,     // Motor current brake mode operation
  COMM_SET_RPM,               // Motor runs in current loop mode
  COMM_SET_POS,               // Motor runs in position loop mode
  COMM_SET_HANDBRAKE,         // Motor runs in handbrake current loop mode
  COMM_SET_DETECT,            // Motor real-time feedback current position command
  COMM_ROTOR_POSITION=22,     // Motor feedback current position
  COMM_GET_VALUES_SETUP=50,   // Motor single or multiple parameter acquisition instructions
  COMM_SET_POS_SPD=91,        // Motor runs in position velocity loop mode
  COMM_SET_POS_MULTI=92,      // Set the motor movement to single-turn mode
  COMM_SET_POS_SINGLE=93,     // Set the motor motion to multi-turn mode, the range is +/- 100 turns
  COMM_SET_POS_UNLIMITED=94,  // Save
  COMM_SET_POS_ORIGIN=95,     // Set the motor origin
} COMM_PACKET_ID;


void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
  buffer[(*index)++] = number >> 24;
  buffer[(*index)++] = number >> 16;
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}


void buffer_append_int16(uint8_t* buffer, int16_t number, int16_t *index) {
  buffer[(*index)++] = number >> 8;
  buffer[(*index)++] = number;
}



void comm_can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
  uint8_t i=0;
  if (len > 8) {
    len = 8;
  }
  CAN_frame_t TxMessage;
  //TxMessage.StdId = 0;
  //TxMessage.IDE = CAN_ID_EXT;
  //TxMessage.ExtId = id;
  //TxMessage.RTR = CAN_RTR_DATA;
  TxMessage.FIR.B.FF = CAN_frame_ext;  // Standard or extended CAN frame
  TxMessage.MsgID = id;
  TxMessage.FIR.B.DLC = len;
  //memcpy(txmsg.data8, data, len);
  for(i=0; i<len; i++)
    TxMessage.data.u8[i]=data[i];  //wht if data is only 4 bytes? we using the .u8 of data
  //CAN_Transmit(CHASSIS_CAN, &TxMessage);
  ESP32Can.CANWriteFrame(&TxMessage);
}



/* Duty cycle mode */
void comm_can_set_duty(uint8_t controller_id, float duty) {
  int32_t send_index = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
  comm_can_transmit_eid(controller_id |((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
}



/* Position and Velocity loop mode */
void comm_can_set_pos_spd(uint8_t controller_id, float pos, int16_t spd, int16_t RPA ) {
  int32_t send_index = 0;
  int16_t send_index1 = 0;
  uint8_t buffer[4];
  buffer_append_int32(buffer, (int32_t)(pos * 10000.0), &send_index);
  buffer_append_int16(buffer,spd, & send_index1);
  buffer_append_int16(buffer,RPA, & send_index1);
  comm_can_transmit_eid(controller_id |
  ((uint32_t)CAN_PACKET_SET_POS_SPD << 8), buffer, send_index);
}



void motor_receive(float* motor_pos,float* motor_spd,float* cur, int8_t* temp, int8_t* error, CAN_frame_t* rx_message)
// originally not int8_t, it was int_8
{
  int16_t pos_int = (rx_message->data.u8[0] << 8) | (rx_message->data.u8[1]);
  int16_t spd_int = (rx_message->data.u8[2] << 8) | (rx_message->data.u8[3]);
  int16_t cur_int = (rx_message->data.u8[4] << 8) | (rx_message->data.u8[5]);
  *motor_pos = (float)( pos_int * 0.1f); //motor position
  *motor_spd = (float)( spd_int * 10.0f);//motor speed
  *cur = (float) ( cur_int * 0.01f);//motor current
  *temp = (rx_message)->data.u8[6] ;//motor temperature
  *error = (rx_message)->data.u8[7] ;//motor error mode
}

/********************* End Servo mode code ***************************/




/*******************************************************************
 Definition of float_to_uint function

 This function convertes a float to an unsigned int given range and 
 number of bits of the uint number. Conversion to [uint] needs to be
 done before sending the data to the motor.

 It is basically an interpolation calculator where value x from one
 range: [x_min, x_max] is mapped to another range: [0, 2^bits].
*******************************************************************/
int float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
  float span = x_max - x_min;
  if(x < x_min) x = x_min;
  else if(x > x_max) x = x_max;
  
  // '1 << bits' is simply 2^bits, it's multiplying 1 by 2^bits
  return (int) ((x- x_min)*((float)((1<<bits)/span)));
}



/*******************************************************************
 Definition of pack_cmd function

 This function packs user's input command into a CAN message, which
 can be sent to the motor using another function.
*******************************************************************/
void pack_cmd(CAN_frame_t *msg, float p, float v, float kp, float kd, float torq) {
  // Limits data to be within bounds
  float p_flt = fminf(fmaxf(P_MIN, p), P_MAX);
  float v_flt = fminf(fmaxf(V_MIN, v), V_MAX);
  float kp_flt = fminf(fmaxf(Kp_MIN, kp), Kp_MAX);
  float kd_flt = fminf(fmaxf(Kd_MIN, kd), Kd_MAX);
  float t_flt = fminf(fmaxf(T_MIN, torq), T_MAX);
  
  // Converts floats to unsigned ints [uint]
  int p_uint = float_to_uint(p_flt, P_MIN, P_MAX, 16);
  int v_uint = float_to_uint(v_flt, V_MIN, V_MAX, 12);
  int kp_uint = float_to_uint(kp_flt, Kp_MIN, Kp_MAX, 12);
  int kd_uint = float_to_uint(kd_flt, Kd_MIN, Kd_MAX, 12);
  int t_uint = float_to_uint(t_flt, T_MIN, T_MAX, 12);

  // Pack [uint] values into CAN message in accordance with the motor's
  // "receive data definition" from the motor manual (p.48-49)
  // 0xF is 1111, 0xFF is 11111111, each F appends 1111; F or f doesnt matter
  msg->data.u8[0] = p_uint>>8;                       // Position 8 higher
  msg->data.u8[1] = p_uint&0xFF;                     // Position 8 lower
  msg->data.u8[2] = v_uint>>4;                       // Speed 8 higher
  msg->data.u8[3] = ((v_uint&0xF)<<4)|(kp_uint>>8);  // Speed 4 bit lower Kp 4 bit higher
  msg->data.u8[4] = kp_uint&0xFF;                    // Kp 8 bit lower
  msg->data.u8[5] = kd_uint>>4;                      // Kd 8 bit higher
  msg->data.u8[6] = ((kd_uint&0xF)<<4)|(t_uint>>8);  // Kd 4 bit lower torque 4 bit higher
  msg->data.u8[7] = t_uint&0xff;                     // torque 8 bit lower
}



/*******************************************************************
 Definition of uint_to_float function

 This function converts an [uint] value sent from the motor to a
 float value given the range and number of bits of the uint number.
 It does interpolation like the float_to_uint() func as well.
*******************************************************************/
float uint_to_float(int x_int, float x_min, float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;

  /* This function is performing as expected. uint to float conversion
  has been tested. Data obtained from test is shown below:
  1. p_int = 38086 -> 2.03   (expected value = 2.029)
  2. v_int = 2818  -> 3.01   (expected value = -0.002)
  3. t_int = 2040  -> -0.53  (expected value = -0.527)
  */
}



/*******************************************************************
 Definition of unpack_reply function
 
 This function unpacks the reply from the motor. 
 
 Parameters:
 CAN_frame_t msg:
   This is the motor's return msg. It is not passed as reference since
   we do not need to modify the msg itself, we only need a copy of
   its content.

 state[]:
   The motor's position, speed, and torque will be stored in an arrary
   named 'state'.
*******************************************************************/
void unpack_reply(CAN_frame_t msg, float state[]){
  // Deconstructing motor's return message, which are [uint] values
  int id = msg.data.u8[0];                                // Motor ID
  int p_uint = (msg.data.u8[1]<<8)|msg.data.u8[2];        // Motor position data
  int v_uint = (msg.data.u8[3]<<4)|(msg.data.u8[4]>>4);   // Motor speed data
  int t_uint = ((msg.data.u8[4]&0xF)<<8)|msg.data.u8[5];  // Motor torque data

  /* As stated in the manual (p.49), the motor return code is 6 bytes. 
   DATA[6] and DATA[7], which are motor temp and error code, always
   return as 0. */
  //int temp_int = msg.data.u8[6];     // Motor temperature
  //int errcode_int = msg.data.u8[7];  // Error code

  // Convert to float and store in array named state[]
  state[0] = uint_to_float(p_uint, P_MIN, P_MAX, 16);
  state[1] = uint_to_float(v_uint, V_MIN, V_MAX, 12);
  state[2] = uint_to_float(t_uint, T_MIN, T_MAX, 12);
}



/***************************************************************
 Definition of to_motors function

 Send commands to the motor specified by ID. The ID of a motor can
 be set using CubeMars' software.

 Parameters and Variables:
 int id:
    This is the ID of the motor that the CAN message will go to. It
    can be set from 0 to 0x7FF (2047); the HIGHER the value,
    the LOWER the priority of the motor. It's only till 2047
    because AK80-64 only supports MIT mode, which uses the Standard
    Frame CAN msg, which has an 11-bit identifier (ID). 2047 is the
    largest 11-bit number. Extended Frame CAN msg has a 29-bit
    identifier (ID) so it can go more than 2047.
 
 FIR.B.DLC:
    This is the size of the data frame in a CAN message (in bytes).
    Motor manual outlined "receives data definition" which is how
    the motor recives the CAN msg. So, that is how we have to format
    our CAN message. DLC = 8 bytes in this case and 6 bytes for 
    "send data definition" which is how the motor sends back a reply.
***************************************************************/
void to_motor(int id, float pos, float vel, int kp, int kd, float torq, CAN_frame_t *msg){
  // Define properties of the CAN msg that will be sent
  msg->FIR.B.FF = CAN_frame_std;  // Standard or extended CAN frame
  msg->MsgID = id;
  msg->FIR.B.DLC = 8;  // MIT mode command is 8 bytes, see "receives data definition" (p.48)

  // Insert command into the CAN msg
  pack_cmd(msg, pos, vel, kp, kd, torq);
  
  // The function CANWriteFrame() sends the CAN mesg to ESP32's CAN controller
  ESP32Can.CANWriteFrame(msg);
}



/*******************************************************************
 Definition of enter_control_mode

 Puts the motor specified by ID into control mode. 
*******************************************************************/
void enter_control_mode(int id, CAN_frame_t *msg) {
  // Sending the following CAN msg puts the motor into control mode.
  // In other words, this is the first command you send to make the 
  // motor start listening for other commands. (Source: p.48)
  msg->FIR.B.FF = CAN_frame_std;
  msg->MsgID = id;
  msg->FIR.B.DLC = 8;
  msg->data.u8[0] = 0xFF;
  msg->data.u8[1] = 0xFF;
  msg->data.u8[2] = 0xFF;
  msg->data.u8[3] = 0xFF;
  msg->data.u8[4] = 0xFF;
  msg->data.u8[5] = 0xFF;
  msg->data.u8[6] = 0xFF;
  msg->data.u8[7] = 0xFC;
  ESP32Can.CANWriteFrame(msg);
}



int num;
int id = 1;  // ID of the motor you want to control
HardwareSerial mySerial2(2);  // DEBUG serial port 2
//const int runtime = 20;
//const int dt = 100 * pow(10, -3);
const int SIZE = 2000;  //change this according to runtime and dt in .py script
//float command_theta[SIZE];
//float encoder_theta[SIZE];
int step = 0;


/*******************************************************************
 Set up function for the ESP32
*******************************************************************/
void setup() {
  Serial.begin(56600);                  // VS code's default is 9600
  mySerial2.begin(56600, SERIAL_8N1, 17, 18);  // DEBUG
  
  CAN_cfg.speed = CAN_SPEED_1000KBPS;  // matches manual-specified 1 Mbps CAN bus speed
  CAN_cfg.tx_pin_id = GPIO_NUM_5;      // TX-RX pins might vary based on ESP32 model
  CAN_cfg.rx_pin_id = GPIO_NUM_4;
  CAN_cfg.rx_queue = xQueueCreate(rx_queue_size, sizeof(CAN_frame_t));  // data receive queue
  
  ESP32Can.CANInit();    // Initialize CAN Module
  
  // DEBUG
  //while(Serial.available() == 0) {}
  //String num_str = Serial.readString();
  //num = atoi(num_str.c_str());

  // for testing if data is receiving from .py
  //pinMode(2, OUTPUT);  //pin 2 is the LED pin confirmed
  

  // Put the motors into control mode, shown below are motors w/ ID = 1 and ID = 2
  //enter_control_mode(id, &tx_frame);
  //enter_control_mode(2, &tx_frame);
}



/*******************************************************************
 Loop function for the ESP32
*******************************************************************/
void loop() {
  
  /***********************************************************************************
    Mode: Follow a Trajectory

    The motor follows the given trajectory (function). Incremental position commands 
    are given at time step dt apart, making the motor follow any function we want.
  ***********************************************************************************/
  // Need dynamic memory alloc since 'SIZE' is not a fixed constant;
  // it can change based on dt and run_time, both of which are arbitrary
  //float* command_theta = new float[SIZE];
  //float* encoder_theta = new float[SIZE];

  CAN_frame_t motor_returnMsg;

  // For walking at freq 18*w, Kp=420 and Kd=4.5 were the best values.
  //int kp = 200;  //set back to 420
  //int kd = 3;  //set back to 4.5
  //int step = 0;  // keeps track of the number of commands sent
  //float motor_state[3];  // motor's position, velocity, and torque


  //Serial.println("\n===============================================================");
  //Serial.println("Press any key to start: ");  // prevents motor jumps
  //while(Serial.available() == 0) {}
  //Serial.readString();

  // Set the motor to initial hip position if needed
  //float ini_hip_angle = 48.1229 * PI / 180;
  //to_motor(1, ini_hip_angle, 0, kp, kd, 0, &rx_frame);

  // Some of these variables were declared at the start of the code

      
  // Simple trajectory
  //theta_d = 1 - cos(2 * (dt*step) / pow(10,6));

  // Trajectory of hip from sit to stand (from Eddie)
  /* while(Serial.available() == 0){}
  String theta_str = Serial.readString();
  int theta_int = atoi(theta_str.c_str());
  
  digitalWrite(theta_int, HIGH);
  delay(200);
  digitalWrite(theta_int, LOW);
  delay(200); */


  /* static int i = 0;
  numbers[i] = theta_int;
  i++;

  if (theta_int == 99) {
    for (int k=0; k <= 50; k++) {
      
    }
  } */
  
  //String theta_str;
  while (Serial.available()==0) {
    //theta_str = Serial.readStringUntil('\n');
  }
  float pos = Serial.parseFloat();
  comm_can_set_pos_spd(1, pos, 5000, 30000);
  Serial.println(pos);

  //String theta_str = Serial.readStringUntil('\n');
  //String theta_str = Serial.readString();
  //int theta_int = atoi(theta_str.c_str());
  //float theta_d = theta_int / 1000.000;  //int divided by int will give an int, so has to be 1000.000

  // DEBUG printing command theta values
  //mySerial2.printf("%.3f,\r\n", theta_d);
  //mySerial2.println(theta_d);

  //Print out command and encoder when .py script sends -999 (not working)
  /* if (theta_d <= -998) {
    mySerial2.println("\n\n================ BELOW IS COMMAND THETA ========================");
    
    for (int i = 0; i < step; i++) {
      mySerial2.printf("%.2f,", command_theta[i]);
    }

    mySerial2.println("\n\n================ BELOW IS ENCODER THETA ========================");
    
    for (int i = 0; i < step; i++) {
      mySerial2.printf("%.2f,", encoder_theta[i]);
    }

    return;
  } */



  // id, pos, vel, kp, kd, torq, CAN msg
  //to_motor(id, theta_d, 0, kp, kd, 0, &tx_frame);  //theta_d is in radians
  
  // The motor will give feedback for each command it receives
  //CHANGED FROM 1 TO 3*portTICK_PERIOD_MS (change back maybe)
  //if(xQueueReceive(CAN_cfg.rx_queue, &motor_returnMsg, 1*portTICK_PERIOD_MS)==pdTRUE){
  //  unpack_reply(motor_returnMsg, motor_state);
  //  //encoder_theta[step] = motor_state[0];  // only want position from motor_state
  //}

  // DEBUG printing encoder theta values
  //mySerial2.printf("%.3f,\r\n", encoder_theta[step]);
  //mySerial2.printf("%.3f,\r\n", motor_state[0]);


  //command_theta[step] = theta_d;

  //step++;
    

  //Serial.print("step = ");
  //Serial.println(step); 


  //Serial.println("\n\n================ BELOW IS COMMAND THETA ========================");
  //for (int i=0; i <= step; i++) {
  //  //Serial.printf("%.3f,", command_theta[i]);  // prints w 3 dec places
  //  // do not add a 'space' after ',' because that adds random '\n' making it hard to use data in matlab
  //  // if you do println, it truncates the float to 2 dec places
  //  // if you do %d, it prints weird numbers
  //}

  //Serial.println("\n\n================ BELOW IS ENCODER THETA ========================");
  //for (int i=0; i <= step; i++) {
    //Serial.printf("%.3f,", encoder_theta[i]);
  //}



  // Free up the memory
  //delete[] command_theta;
  //delete[] encoder_theta;
}