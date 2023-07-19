/*******************************************************************************
 Header file for CubeMars AK Series motor control classes.

 Author(s): 
  Sai Hein Si Thu
  Alexander Martin-Ginnold
  Joshua Ramayrat
*******************************************************************************/
#pragma once

#include <Arduino.h>
#include <ESP32CAN.h>
#include <CAN_config.h>
class AK_Motor
{
    public:
        AK_Motor(int motor_id,
                 int proportional,
                 int derivative,
                 float theta_scaling);

        void init();

        void set_theta(float theta);

        void set_theta(int theta);

        float get_theta();

        void set_velocity(float velocity);
        
        float get_velocity();

    protected:
        void set_motor_consts(float v_min, float v_max, float t_min, float t_max);

    private:
        int float_to_uint(float x, float x_min, float x_max, unsigned int bits);
        void pack_cmd(CAN_frame_t *msg, float p, float v, float kp, float kd, float torq);
        float uint_to_float(int x_int, float x_min, float x_max, int bits);
        void unpack_reply(CAN_frame_t msg);
        void to_motor(int id, float pos, float vel, int kp, int kd, float torq, CAN_frame_t *msg);
        void enter_control_mode(int id, CAN_frame_t *msg);
        void set_origin(int id, CAN_frame_t *msg);

        int id;
        int kp;
        int kd;
        float scale;
        CAN_frame_t tx_frame;          //the CAN msg to be sent to motors
        CAN_frame_t motor_returnMsg;
        float setpoint_theta;
        int feedback_id;
        float feedback_theta;
        float feedback_vel;
        float feedback_curr;

        /**********  Constants  **********/
        // Same for these motors: AK10-9, 60-6, 70-10, 80-6, 80-9, 80-80/64
        const float P_MIN = -12.5;  //see motor manual (p. 49-50) for these constants
        const float P_MAX = 12.5;
        const float Kp_MIN = 0;
        const float Kp_MAX = 500;
        const float Kd_MIN = 0;
        const float Kd_MAX = 5;
        const float Test_Pos = 0.0;

        // Different for each motor type. Default to Ak80-64 Values
        float V_MIN = -8;
        float V_MAX = 8;
        float T_MIN = -144;
        float T_MAX = 144;

};

class AK80_64 : public AK_Motor
{
    public:
        AK80_64(int motorid,
                int proportional,
                int derivative,
                float theta_scaling);
    protected:
        const float const_V_MIN = -8;
        const float const_V_MAX = 8;
        const float const_T_MIN = -144;
        const float const_T_MAX = 144;
};

class AK70_10 : public AK_Motor
{
    public:
        AK70_10(int motorid,
                int proportional,
                int derivative,
                float theta_scaling);
    private:
        const float const_V_MIN = -50;
        const float const_V_MAX = 50;
        const float const_T_MIN = -25;
        const float const_T_MAX = 25;
};

class AK10_9 : public AK_Motor
{
    public:
        AK10_9(int motorid,
               int proportional,
               int derivative,
               float theta_scaling);
    private:

        /// @todo: Need to update these numbers:
        const float const_V_MIN = -50;
        const float const_V_MAX = 50;
        const float const_T_MIN = -25;
        const float const_T_MAX = 25;
};