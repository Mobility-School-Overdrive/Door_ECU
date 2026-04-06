/**********************************************************************************************************************
 * \file MotorDriver.h
 * \brief DC Motor (Door / Window) PWM + Quadrature Encoder Driver
 *        논블로킹 상태머신 구조 - 센서 드라이버와 연동 가능
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
#define GEAR_RATIO          8
#define CPR                 30
#define ENCODER_MODE        1
#define COUNTS_PER_REV      (CPR * ENCODER_MODE * GEAR_RATIO)  /* 30*1*8 = 240 */
#define COUNTS_PER_DEG      (COUNTS_PER_REV / 360.0f)          /* = 0.6667 */

#define DOOR_ENC_A_PIN      IfxGtm_TIM2_6_P13_1_IN
#define DOOR_ENC_B_PIN      IfxGtm_TIM2_7_P13_2_IN
#define WINDOW_ENC_A_PIN    IfxGtm_TIM0_4_P02_4_IN
#define WINDOW_ENC_B_PIN    IfxGtm_TIM0_5_P02_5_IN

/*============================================================
 * Angle Limit
 *============================================================*/
#define DOOR_MAX_ANGLE      80.0f
#define WINDOW_MAX_ANGLE    90.0f  /* 실제 창문 물리적 최대치에 맞게 조정 */

/*============================================================
 * Interrupt Priority
 *============================================================*/
#define DOOR_ENC_A_ISR_PRIO    10
#define DOOR_ENC_B_ISR_PRIO    11
#define WINDOW_ENC_A_ISR_PRIO  12
#define WINDOW_ENC_B_ISR_PRIO  13

/*============================================================
 * 도달 판정 허용 오차
 *============================================================*/
#define DOOR_ARRIVE_THRESH      1
#define WINDOW_ARRIVE_THRESH    1

/*============================================================
 * 모터 상태머신
 *============================================================*/
typedef enum {
    MOTOR_IDLE,      // 정지 대기 중
    MOTOR_RUNNING,   // 목표 각도로 이동 중
    MOTOR_DONE,      // 목표 도달 완료
    MOTOR_STOPPED    // 외부 요청(센서 등)으로 강제 정지
} MotorState_t;

/*============================================================
 * 공유 엔코더 카운트 (ISR에서 갱신)
 *============================================================*/
extern volatile sint32 door_encoder_count;
extern volatile sint32 window_encoder_count;

/*============================================================
 * 디버그용 전역변수
 *============================================================*/
extern volatile sint32 dbg_door_target;
extern volatile sint32 dbg_door_error;
extern volatile sint32 dbg_door_abs;
extern volatile sint32 dbg_win_target;
extern volatile sint32 dbg_win_error;
extern volatile sint32 dbg_win_abs;

/*============================================================
 * 초기화
 *============================================================*/
void Motor_Init(void);

/*============================================================
 * Door Motor API
 *============================================================*/
void         Door_Motor_SetTarget(float target_angle); // 목표 각도 설정 (논블로킹)
void         Door_Motor_Update(void);                  // 메인루프에서 매 주기 호출
void         Door_Motor_Stop(void);                    // 즉시 정지 (센서 등 외부 호출용)
void         Door_Motor_Home(void);                    // 블로킹 홈 복귀 (초기화 시 1회만)
MotorState_t Door_Motor_GetState(void);                // 현재 상태 반환

/*============================================================
 * Window Motor API
 *============================================================*/
void         Window_Motor_SetTarget(float target_angle);
void         Window_Motor_Update(void);
void         Window_Motor_Stop(void);
void         Window_Motor_Home(void);
MotorState_t Window_Motor_GetState(void);

/*============================================================
 * 유틸
 *============================================================*/
void Motor_Delay_ms(uint32 ms);
void Door_Motor_SetDuty(uint32 duty);
void Window_Motor_SetDuty(uint32 duty);

#endif /* MOTOR_DRIVER_H */
