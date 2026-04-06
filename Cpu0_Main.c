/**********************************************************************************************************************
 * \file Cpu0_Main.c
 *********************************************************************************************************************/
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "Ifx_Cfg_Ssw.h"
#include "MotorDriver.h"

#include "Driver_Stm.h"
#include "BT.h"
#include "FSR.h"
#include "MCMCAN.h"
#include "Buzzer.h"
#include "DFPlayer.h"
#include "RGB_LED.h"
#include "UltraSonic.h"
#include "Current_Sensor.h"

//#include "PinTest.c"

/* CUSTOM MACRO */

/* MOTER ID */
#define DOOR_MOTOR_ID   1
#define WINDOW_MOTOR_ID 2

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

static volatile uint8 g_btLine[64];
static volatile uint8 g_btWorkLine[64];
static volatile boolean g_btLineReady = FALSE;

void BT_LineTask(void)
{
    static uint8 idx = 0;
    uint8 ch;

    while (BT_GetChar(&ch))
    {
        if ((ch != '\r') && (ch != '\n'))
        {
            if (idx < sizeof(g_btWorkLine) - 1)
            {
                g_btWorkLine[idx++] = ch;
            }
        }

        if ((ch == '\r') || (ch == '\n'))
        {
            if (idx > 0)
            {
                g_btWorkLine[idx] = '\0';
                strcpy((char *)g_btLine, (char *)g_btWorkLine);
                g_btLineReady = TRUE;
                idx = 0;
            }
        }
    }
}

boolean raw = 1;

void core0_main(void)
{
    IfxCpu_enableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
    IfxCpu_emitEvent(&cpuSyncEvent);
    IfxCpu_waitEvent(&cpuSyncEvent, 1);

    /* Init */
    Driver_Stm_Init();
    EVADC_Manager_initModule();
    EVADC_Group0_init();

    Motor_Init();
    Button_Init();
    Door_Motor_Home();
    Window_Motor_Home();

    initDFPlayer();
    initUltraSonic();
    initBUZ();
    initRGB();
    initFSR();
    initCurrentSensor();
    initMcmcan();
    initBT();

    EVADC_Group0_start();

    /********/

    Ifx_TickTime ticksFor1000ms = IfxStm_getTicksFromMilliseconds(&MODULE_STM0, 1000);

    // 블루투스 이름 설정
    BT_SendString("AT+NAMEOVERDRIVE");

    float door_target_angle   = 0.0f;
    float window_target_angle = 0.0f;

    dfSetVolume(0x02);

    delayMs(1000);

    DFPlayer_Task();

    delayMs(1000);

    CurrentSensor_ClearFault(DOOR_MOTOR_ID);
    CurrentSensor_StartSense(DOOR_MOTOR_ID);

    CurrentSensor_ClearFault(WINDOW_MOTOR_ID);
    CurrentSensor_StartSense(WINDOW_MOTOR_ID);

    while (1)
    {
        CurrentSensor_Task(DOOR_MOTOR_ID);
        CurrentSensor_Task(WINDOW_MOTOR_ID);
        AppScheduling();
        BT_Task();

//        // ── 모터 상태머신 업데이트 (논블로킹) ───────────────
//        Door_Motor_Update();
//        Window_Motor_Update();
//
//        // ── 상태 전환 예시 ───────────────────────────────────
//        // DONE 상태가 되면 다음 목표로 전환
//        if (Door_Motor_GetState()   == MOTOR_DONE &&
//            Window_Motor_GetState() == MOTOR_DONE)
//        {
//            Motor_Delay_ms(2000);
//            Door_Motor_SetTarget(0.0f);
//            Window_Motor_SetTarget(0.0f);
//        }
//
//        Motor_Delay_ms(5);  // 루프 주기 5ms
//
//        Door_Motor_Update();
//        Window_Motor_Update();
//
//        if (Door_Motor_GetState() == MOTOR_DONE)
//        {
//            CurrentSensor_StopSense(DOOR_MOTOR_ID);
//
//            Motor_Delay_ms(1000);
//            door_target_angle = (door_target_angle == 0.0f) ? 90.0f : 0.0f;
//
//            CurrentSensor_ClearFault(DOOR_MOTOR_ID);
//            Door_Motor_SetTarget(door_target_angle);
//            CurrentSensor_StartSense(DOOR_MOTOR_ID);
//        }
//
//        if (Window_Motor_GetState() == MOTOR_DONE)
//        {
//            CurrentSensor_StopSense(WINDOW_MOTOR_ID);
//
//            Motor_Delay_ms(1000);
//            window_target_angle = (window_target_angle == 0.0f) ? 90.0f : 0.0f;
//
//            CurrentSensor_ClearFault(WINDOW_MOTOR_ID);
//            Window_Motor_SetTarget(window_target_angle);
//            CurrentSensor_StartSense(WINDOW_MOTOR_ID);
//        }
//
//        Motor_Delay_ms(10);
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
    BT_LineTask();
    FSR_Task();

//    if (CurrentSensor_GetFault(DOOR_MOTOR_ID) == CURRENT_FAULT_MOTOR)
//    {
//        Door_Motor_Stop();
//        CurrentSensor_StopSense(DOOR_MOTOR_ID);
//    }
//
//    if (CurrentSensor_GetFault(WINDOW_MOTOR_ID) == CURRENT_FAULT_MOTOR)
//    {
//        Window_Motor_Stop();
//        CurrentSensor_StopSense(WINDOW_MOTOR_ID);
//    }
    Button_Update();
    Door_Motor_Update();
    Window_Motor_Update();

    stTestCnt.u32nuCnt10ms++;
}


void AppTask100ms(void)
{
    stTestCnt.u32nuCnt100ms++;
}

void AppTask1000ms(void)
{
    RGB_Task();

    if (g_btLineReady)
    {
        g_btLineReady = FALSE;

        if (strstr((char *)g_btLine, "+ADDR") != NULL)
        {
            // IfxPort_setPinHigh(&MODULE_P00, 5);
        }
    }

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

