#ifndef T4KB3_OTP_H
#define T4KB3_OTP_H

#include "msm_sensor.h"

extern uint16_t  t4kb3_af_macro_value; 
extern uint16_t  t4kb3_af_inifity_value;
extern uint16_t  t4kb3_af_otp_status;

void t4kb3_update_otp_para(struct msm_sensor_ctrl_t *s_ctrl);
#endif