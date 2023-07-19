/*******************************************************************************
 Source file for CubeMars AK Series motor control classes. The control functions
 are based off of the information provided in the motor manual.

 By: Sai Hein Si Thu and Alexander Martin-Ginnold
 Updated: 2023-JUN-28
*******************************************************************************/
#include "AK_Motors.h"

/*! ****************************************************************************
* @brief Constructor for the AK_Motor base class. Sets the intial stored values
        for the unique motor constants but does not send them to the motor.
* @param motor_id The integer CAN ID for the motor
* @param proportional The integer proportional constant for the PD controller
* @param derivative The integer derivative constant for the PD controller
* @param theta_scaling The floating point constant to convert the software
                software commanded integer theta to a value in radians
*******************************************************************************/
AK_Motor::AK_Motor(int motor_id,
              int proportional,
              int derivative,
              float theta_scaling){
  int id = motor_id;
  int kp = proportional;
  int kd = derivative;
  float scale = theta_scaling;
}

/*! ****************************************************************************
* @brief Initializer for a motor object. Puts the motor in control mode and sets
         the motor origin using the stored motor constants.
*******************************************************************************/
void AK_Motor::init(){
  enter_control_mode(id, &tx_frame);
  set_origin(id, &tx_frame);
}

/*! ****************************************************************************
* @brief Overloaded function to set the motor position
* @param theta The floating point radians to command the motor to
*******************************************************************************/
void AK_Motor::set_theta(float theta){
  to_motor(id, theta, 0, kp, kd, 0, &tx_frame);  //theta_d is in radians

  if(xQueueReceive(CAN_cfg.rx_queue, &motor_returnMsg, 1*portTICK_PERIOD_MS)==pdTRUE){
    unpack_reply(motor_returnMsg);
  }
}

/*! ****************************************************************************
* @brief Overloaded function to set the motor position
* @param theta Integer value, unitless that must be scaled to radians
*******************************************************************************/
void AK_Motor::set_theta(int theta){
  set_theta(theta/scale); //Divide by scaling factor if want to round\
                            to 3dp but you must change scaling on PC program\
                            too to reflect the correct rounding config
}


/*! ****************************************************************************
* @brief Get the most recent theta feedback from the motor without CAN message
* @return The currently stored floating point theta feedback value
*******************************************************************************/
float AK_Motor::get_theta(){
  return feedback_theta;
}


/*! ****************************************************************************
* @brief Overloaded function to set the motor velocity
* @param velocity The floating point velocity to set the motor to
*******************************************************************************/
void AK_Motor::set_velocity(float velocity)
{
  to_motor(id, 0, velocity, kp, kd, 0, &tx_frame);  //theta_d is in radians

  if(xQueueReceive(CAN_cfg.rx_queue, &motor_returnMsg, 1*portTICK_PERIOD_MS)==pdTRUE){
    unpack_reply(motor_returnMsg);
  }
}

/*! ****************************************************************************
* @brief Get the most recent velocity feedback from the motor without CAN message
* @return The currently stored floating point velocity feedback value
*******************************************************************************/
float AK_Motor::get_velocity(){
  return feedback_vel;
}


/*! ****************************************************************************
* @brief Set the integer conversion constants unique to each motor model
* @param v_min The minimum motor velocity in rad/s
* @param v_max The maximum motor velocity in rad/s
* @param t_min The minimum motor torque in N-m
* @param t_max The maximum motor torque in N-m
*******************************************************************************/
void AK_Motor::set_motor_consts(float v_min, float v_max, float t_min, float t_max){
  V_MIN = v_min;
  V_MAX = v_max;
  T_MIN = t_min;
  T_MAX = t_max;

}

/*! ****************************************************************************
Converts a float to an unsigned int given range and
number of bits of the uint number. Conversion to [uint] needs to be
done before sending the data to the motor.

It is basically an interpolation calculator where value x from one
range: [x_min, x_max] is mapped to another range: [0, 2^bits].

* @brief Converts float to unsigned int in the way expected by the motors
* @param x The float to be converted
* @param x_min The minumum possible value of x
* @param x_max The maximum possible value of x
* @param bits Number of bits in the unisigned int
* @return An unsigned integer of length bits equal to the floating point x
*******************************************************************************/
int AK_Motor::float_to_uint(float x, float x_min, float x_max, unsigned int bits) {
  float span = x_max - x_min;
  if(x < x_min) x = x_min;
  else if(x > x_max) x = x_max;

  // '1 << bits' is simply 2^bits, it's multiplying 1 by 2^bits
  return (int) ((x- x_min)*((float)((1<<bits)/span)));
}

/*******************************************************************************
* @brief Packs input command into a CAN message structured for the motor
* @param msg A pointer to the location of the CAN message
* @param p The target position to send in the message
* @param v The target velocity to send in the message
* @param kp The proportional control constant to use for the motion
* @param kd The derivative control constant to use for the motion
* @param torq The (peak? not sure!) torque for the motion
*******************************************************************************/
void AK_Motor::pack_cmd(CAN_frame_t *msg, float p, float v, float kp, float kd, float torq) {
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

  /*
   Pack [uint] values into CAN message in accordance with the motor's
   "receive data definition" from the motor manual (p.48-49)
   0xF is 1111, 0xFF is 11111111, each F appends 1111; F or f doesnt matter.
  */
  msg->data.u8[0] = p_uint>>8;                       // Position 8 higher
  msg->data.u8[1] = p_uint&0xFF;                     // Position 8 lower
  msg->data.u8[2] = v_uint>>4;                       // Speed 8 higher
  msg->data.u8[3] = ((v_uint&0xF)<<4)|(kp_uint>>8);  // Speed 4 bit lower Kp 4 bit higher
  msg->data.u8[4] = kp_uint&0xFF;                    // Kp 8 bit lower
  msg->data.u8[5] = kd_uint>>4;                      // Kd 8 bit higher
  msg->data.u8[6] = ((kd_uint&0xF)<<4)|(t_uint>>8);  // Kd 4 bit lower torque 4 bit higher
  msg->data.u8[7] = t_uint&0xff;                     // torque 8 bit lower
}



/*! ****************************************************************************
Converts an [uint] value sent from the motor to a
float value given the range and number of bits of the uint number.
It does interpolation like the float_to_uint() func as well.

* @brief Converts unsigned int output by motor to standard float
* @param x_int The unsigned integer to be converted
* @param x_min The minumum possible value of x
* @param x_max The maximum possible value of x
* @param bits Number of bits in the unisigned int
* @return A float equal to the unsigned x_int of length bits
*******************************************************************************/
float AK_Motor::uint_to_float(int x_int, float x_min, float x_max, int bits){
  float span = x_max - x_min;
  float offset = x_min;
  return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;

  /*
   This function is performing as expected. uint to float conversion
   has been tested. Data obtained from test is shown below:
   1. p_int = 38086 -> 2.03   (expected value = 2.029)
   2. v_int = 2818  -> 3.01   (expected value = -0.002)
   3. t_int = 2040  -> -0.53  (expected value = -0.527)
  */
}



/*! ****************************************************************************
* @brief  This function unpacks the feedback (reply) message sent by the motor.
* @param msg This is the motor's return msg. It is not passed as reference since
   we do not need to modify the msg itself, we only need a copy of its content.
*******************************************************************************/
void AK_Motor::unpack_reply(CAN_frame_t msg){
  // Deconstructing motor's return message, which are [uint] values
  feedback_id = msg.data.u8[0];                                // Motor ID
  int p_uint = (msg.data.u8[1]<<8)|msg.data.u8[2];        // Motor position data
  int v_uint = (msg.data.u8[3]<<4)|(msg.data.u8[4]>>4);   // Motor speed data
  int t_uint = ((msg.data.u8[4]&0xF)<<8)|msg.data.u8[5];  // Motor torque data

  /*
   As stated in the manual (p.49), the motor return code is 6 bytes.
   DATA[6] and DATA[7], which are motor temp and error code, always
   return as 0, so commented out. Perhaps there is a better way to retrive
   these values.
  */
  //int temp_int = msg.data.u8[6];     // Motor temperature
  //int errcode_int = msg.data.u8[7];  // Error code

  // Convert to float and store in array named state[]
  feedback_theta = uint_to_float(p_uint, P_MIN, P_MAX, 16);
  feedback_vel = uint_to_float(v_uint, V_MIN, V_MAX, 12);
  feedback_curr = uint_to_float(t_uint, T_MIN, T_MAX, 12);
}



/*! ****************************************************************************
Send commands to the motor specified by 'id'. The ID of a motor can
be set using CubeMars' software called 'CubeMarstool_V1.32.exe' (find it in the
shared drive since the official website's copy is corrupted as of 2023-06-13).
You need to connect PC to the motor using an R-Link in order to interface with
the motor using CubeMarstool.

* @brief Send commands to the motor specified by 'id'.
* @param id This is the ID of the motor that the CAN message will be sent to. It
    can be set from 0 to 0x7FF (2047); the HIGHER the value,
    the LOWER the priority of the motor. It's only till 2047
    because AK80-64 only supports MIT mode, which uses the Standard
    Frame CAN msg, which has an 11-bit identifier (ID). 2047 is the
    largest 11-bit number. Extended Frame CAN msg has a 29-bit
    identifier (ID) so it can go more than 2047.
* @param pos The target position to send in the message
* @param vvel The target velocity to send in the message
* @param kp The proportional control constant to use for the motion
* @param kd The derivative control constant to use for the motion
* @param torq The (peak? not sure!) torque for the motion
* @param msg A pointer to the location of the CAN message
* @var FIR.B.DLC This is the size of the data frame in a CAN message (in bytes).
    Motor manual outlined "receives data definition" which is how
    the motor recives the CAN msg. So, that is how we have to format
    our CAN message. DLC = 8 bytes in this case and 6 bytes for
    "send data definition" which is how the motor sends back a reply.
*******************************************************************************/
void AK_Motor::to_motor(int id, float pos, float vel, int kp, int kd, float torq, CAN_frame_t *msg){
  // Define properties of the CAN msg that will be sent
  msg->FIR.B.FF = CAN_frame_std;  // Standard or extended CAN frame
  msg->MsgID = id;
  msg->FIR.B.DLC = 8;  // MIT mode command is 8 bytes, \
                          see "receives data definition" (p.48)

  // Insert command into the CAN msg
  pack_cmd(msg, pos, vel, kp, kd, torq);

  // The function CANWriteFrame() sends the CAN msg to the motor by first sending\
     it to ESP32's CAN controller, which then sends to the CAN transceiver.
  ESP32Can.CANWriteFrame(msg);
}



/*! ****************************************************************************
* @brief Puts the motor specified by ID into control mode (aka listening mode).
* @param id Integer motor ID for this motor
* @param msg A pointer to the location of the CAN message
*******************************************************************************/
void AK_Motor::enter_control_mode(int id, CAN_frame_t *msg) {
  /*
   Sending the following CAN msg puts the motor into control mode.
   In other words, this is the first command you send to make the
   motor start listening for other commands. (See motor manual p.48)
  */
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



/*! ****************************************************************************
* @brief Sets the current position of the motor to be the origin
    (i.e. 0 rad position).
* @param id Integer motor ID for this motor
* @param msg A pointer to the location of the CAN message
*******************************************************************************/
void AK_Motor::set_origin(int id, CAN_frame_t *msg) {
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
  msg->data.u8[7] = 0xFE;
  ESP32Can.CANWriteFrame(msg);
}

/*! ****************************************************************************
* @brief Constructor for the derived AK80_64 motor class. Sets the intial stored
        values for the motor constants specific to the AK80_64 motor and calls
        the constructor for the general motor class.
* @param motor_id The integer CAN ID for the motor
* @param proportional The integer proportional constant for the PD controller
* @param derivative The integer derivative constant for the PD controller
* @param theta_scaling The floating point constant to convert the software
                software commanded integer theta to a value in radians
*******************************************************************************/
AK80_64::AK80_64(int motorid,
            int proportional,
            int derivative,
            float theta_scaling) :
            AK_Motor(motorid, proportional, derivative, theta_scaling){
  set_motor_consts(const_V_MIN, const_V_MAX, const_T_MIN, const_T_MAX);
}

/*! ****************************************************************************
* @brief Constructor for the derived AK70_10 motor class. Sets the intial stored
        values for the motor constants specific to the AK70_10 motor and calls
        the constructor for the general motor class.
* @param motor_id The integer CAN ID for the motor
* @param proportional The integer proportional constant for the PD controller
* @param derivative The integer derivative constant for the PD controller
* @param theta_scaling The floating point constant to convert the software
                software commanded integer theta to a value in radians
*******************************************************************************/
AK70_10::AK70_10(int motorid,
            int proportional,
            int derivative,
            float theta_scaling) :
            AK_Motor(motorid, proportional, derivative, theta_scaling){
  set_motor_consts(const_V_MIN, const_V_MAX, const_T_MIN, const_T_MAX);
}