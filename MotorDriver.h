/**********************************************************************************************************************
 * \file MotorDriver.h
 * \brief DC Motor (Door / Window) PWM + Quadrature Encoder Driver
 *********************************************************************************************************************/
#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include "Ifx_Types.h"
#include "IfxPort.h"
#include "IfxPort_PinMap.h"
#include "IfxGtm.h"
#include "IfxGtm_PinMap.h"
#include "IfxGtm_Tom_Pwm.h"
#include "Stm/Std/IfxStm.h"
#include "Gtm/Std/IfxGtm_Tim.h"
#include "Gtm/Tim/In/IfxGtm_Tim_In.h"

/*============================================================
 * PWM
 *============================================================*/
#define DOOR_PWM_PERIOD     50000
#define WINDOW_PWM_PERIOD   50000
#define DOOR_PWM_PIN        IfxGtm_TOM0_1_TOUT1_P02_1_OUT
#define WINDOW_PWM_PIN      IfxGtm_TOM0_3_TOUT105_P10_3_OUT

#define MIN_PWM             20.0f

/*============================================================
 * Encoder
 *============================================================*/
#define CPR                 30
#define ENCODER_MODE        1
#define COUNTS_PER_REV      (CPR * ENCODER_MODE)
#define COUNTS_PER_DEG      (COUNTS_PER_REV / 360.0f)

// Door Encoder Pins (P13.1 = A, P13.2 = B)
#define DOOR_ENC_A_PIN      IfxGtm_TIM2_6_P13_1_IN
#define DOOR_ENC_B_PIN      IfxGtm_TIM2_7_P13_2_IN

// Window Encoder Pins (P02.4 = A, P02.5 = B)
#define WINDOW_ENC_A_PIN    IfxGtm_TIM0_4_P02_4_IN
#define WINDOW_ENC_B_PIN    IfxGtm_TIM0_5_P02_5_IN

/*============================================================
 * Interrupt Priority
 *============================================================*/
#define DOOR_ENC_A_ISR_PRIO    10
#define DOOR_ENC_B_ISR_PRIO    11
#define WINDOW_ENC_A_ISR_PRIO  12
#define WINDOW_ENC_B_ISR_PRIO  13

/*============================================================
 * 도달 판정 허용 오차 (count)
 *============================================================*/
#define DOOR_ARRIVE_THRESH      1
#define WINDOW_ARRIVE_THRESH    1

/*============================================================
 * 공유 엔코더 카운트 (ISR에서 갱신)
 *============================================================*/
extern volatile sint32 door_encoder_count;
extern volatile sint32 window_encoder_count;

/*============================================================
 * 디버그용 전역변수
 *============================================================*/
extern volatile sint32 dbg_target;
extern volatile sint32 dbg_error;
extern volatile sint32 dbg_abs;
extern volatile uint32 dbg_duty;

/*============================================================
 * 초기화
 *============================================================*/
void Motor_Init(void);

/*============================================================
 * Door Motor API
 *============================================================*/
void Door_Motor_Start_CW(float percent);
void Door_Motor_Start_CCW(float percent);
void Door_Motor_Stop(void);
void Door_Motor_Home(void);
void Door_Motor_Angle_Control(float target_angle);

/*============================================================
 * Window Motor API
 *============================================================*/
void Window_Motor_Start_CW(float percent);
void Window_Motor_Start_CCW(float percent);
void Window_Motor_Stop(void);
void Window_Motor_Home(void);
void Window_Motor_Angle_Control(float target_angle);

/*============================================================
 * 유틸
 *============================================================*/
void Motor_Delay_ms(uint32 ms);

#endif /* MOTOR_DRIVER_H */
