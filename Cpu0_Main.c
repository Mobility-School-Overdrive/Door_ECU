/**********************************************************************************************************************
 * \file Cpu0_Main.c
 *********************************************************************************************************************/
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "Ifx_Cfg_Ssw.h"
#include "MotorDriver.h"

#include "Driver_Stm.h"
#include "Buzzer.h"
#include "DFPlayer.h"
#include "UltraSonic.h"

/* CUSTOM MACRO */


/***********************/

/* CUSTOM GLOBAL VALUE */

// App Scheduling
typedef struct {
    uint32 u32nuCnt1ms;
    uint32 u32nuCnt10ms;
    uint32 u32nuCnt100ms;
    uint32 u32nuCnt1000ms;
} StTestCnt;

StTestCnt stTestCnt;

/***********************/

/* Function Prototypes */

// App Scheduling
void AppTask1ms(void);
void AppTask10ms(void);
void AppTask100ms(void);
void AppTask1000ms(void);
void AppScheduling(void);

/****************************/

/* Function Implementations */

IFX_ALIGN(4) IfxCpu_syncEvent cpuSyncEvent = 0;

void core0_main(void)
{
    IfxCpu_enableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
    IfxCpu_emitEvent(&cpuSyncEvent);
    IfxCpu_waitEvent(&cpuSyncEvent, 1);

    /* Init */
    Motor_Init();
    Door_Motor_Home();
    Window_Motor_Home();

    Driver_Stm_Init();
    initDFPlayer();
    initUltraSonic();
    initBUZ();

    /********/

    Ifx_TickTime ticksFor1000ms = IfxStm_getTicksFromMilliseconds(&MODULE_STM0, 1000);

    DFPlayer_Task();

    while (1)
    {
        AppScheduling();

        // ── 모터 상태머신 업데이트 (논블로킹) ───────────────
        Door_Motor_Update();
        Window_Motor_Update();

        // ── 상태 전환 예시 ───────────────────────────────────
        // DONE 상태가 되면 다음 목표로 전환
        if (Door_Motor_GetState()   == MOTOR_DONE &&
            Window_Motor_GetState() == MOTOR_DONE)
        {
            Motor_Delay_ms(2000);
            Door_Motor_SetTarget(0.0f);
            Window_Motor_SetTarget(0.0f);
        }

        Motor_Delay_ms(5);  // 루프 주기 5ms
    }
}


/******************/

/* App Scheduling */

void AppTask1ms(void)
{
    Ultrasonic_Task();

    stTestCnt.u32nuCnt1ms++;
}

void AppTask10ms(void)
{
    stTestCnt.u32nuCnt10ms++;
}


void AppTask100ms(void)
{
    stTestCnt.u32nuCnt100ms++;
}

void AppTask1000ms(void)
{
    stTestCnt.u32nuCnt1000ms++;
}

void AppScheduling(void)
{
    if (stSchedulingInfo.u8nuScheduling1msFlag == 1u) {
        stSchedulingInfo.u8nuScheduling1msFlag = 0u;
        AppTask1ms();

        if (stSchedulingInfo.u8nuScheduling10msFlag == 1u) {
            stSchedulingInfo.u8nuScheduling10msFlag = 0u;
            AppTask10ms();
        }

        if (stSchedulingInfo.u8nuScheduling100msFlag == 1u) {
            stSchedulingInfo.u8nuScheduling100msFlag = 0u;
            AppTask100ms();
        }

        if (stSchedulingInfo.u8nuScheduling1000msFlag == 1u) {
            stSchedulingInfo.u8nuScheduling1000msFlag = 0u;
            AppTask1000ms();
        }
    }
}

