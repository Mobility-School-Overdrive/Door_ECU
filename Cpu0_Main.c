/**********************************************************************************************************************
 * \file Cpu0_Main.c
 *********************************************************************************************************************/
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "Ifx_Cfg_Ssw.h"
#include "MotorDriver.h"

#include "can_type_def.h"
#include "Driver_Stm.h"
#include "FSR.h"
#include "MCMCAN.h"
#include "Buzzer.h"
#include "DFPlayer.h"
#include "RGB_LED.h"
#include "UltraSonic.h"
#include "Current_Sensor.h"
#include "ButtonHandler.h"
#include "DoorApp.h"
#include <string.h>

//#include "PinTest.c"

/* CUSTOM MACRO */

/* MOTER ID */

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

static void CanEvent_ServiceTask(void);
static void ControlState_ApplyTask(void);

/* helper */
static void Apply_LockState(LockCmd_t state);
static void Apply_DoorState(OpenClose_t state);
static void Apply_WindowState(OpenClose_t state);
static void Apply_SpeakerState(OnOff_t state);

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
    Driver_Stm_Init();
    EVADC_Manager_initModule();
    EVADC_Group0_init();

    Motor_Init();
    Button_Init();
    DoorLock_Init();
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

    // 스피커 볼륨 설정
    dfSetVolume(0x02);
    delayMs(1000);

    playDFPlayerMusicLoop(g_rxCurrentUserId);
    delayMs(1000);

    // 문, 창문 전류 센서 초기화
    CurrentSensor_ClearFault(DOOR_MOTOR_ID);
    CurrentSensor_StartSense(DOOR_MOTOR_ID);

    CurrentSensor_ClearFault(WINDOW_MOTOR_ID);
    CurrentSensor_StartSense(WINDOW_MOTOR_ID);

    while (1)
    {
        CurrentSensor_Task(DOOR_MOTOR_ID);
        CurrentSensor_Task(WINDOW_MOTOR_ID);
        AudioControl_Task();
        AppScheduling();
        BT_Task();
    }
}

/*************/

/* Functions */

static void CanEvent_ServiceTask(void)
{
    /* 1) 도난 감지 */
    if (g_reqTheftWarn == TRUE)
    {
        Buzzer_Request(BUZ_EVENT_THEFT);
        g_reqTheftWarn = FALSE;
    }

    /* 2) 공통 버저 요청 */
    if (g_reqBuzzer == TRUE)
    {
        if (g_rxSafeExitAssist == TRUE)
        {
            Buzzer_Request(BUZ_EVENT_SAFE_EXIT);
            g_rxSafeExitAssist = FALSE;
        }
        else if (g_rxTheftDetected == TRUE)
        {
            Buzzer_Request(BUZ_EVENT_THEFT);
        }
        else if (g_canObstacleDetected == TRUE)
        {
            Buzzer_Request(BUZ_EVENT_OBSTACLE);
            g_canObstacleDetected = FALSE;
        }
        else
        {
            Buzzer_Request(BUZ_EVENT_OBSTACLE);
        }

        g_reqBuzzer = FALSE;
    }

    /* 4) 스피커 / DFPlayer */
    if (g_reqSpeakerPlay == TRUE)
    {
        g_ctrlState.speaker = ON_OFF_ON;
        g_reqSpeakerPlay = FALSE;
    }

    /* 5) 시동 ON */
    if (g_reqIgnitionOnAction == TRUE)
    {
        /* AudioControl_Task()가 별도로 재생 처리하므로
           여기서는 필요 시 추가 상태 초기화만 */
        g_reqIgnitionOnAction = FALSE;
    }

    /* 6) 시동 OFF */
    if (g_reqIgnitionOffAction == TRUE)
    {
        /* 필요하면 OFF 시 오디오 정리/LED 정리 */
        g_reqIgnitionOffAction = FALSE;
    }
}

static void ControlState_ApplyTask(void)
{
    static OpenClose_t prevDoorState = OPEN_CLOSE_CLOSE;
    static OpenClose_t prevWindowState = OPEN_CLOSE_CLOSE;
    static LockCmd_t   prevLockState = LOCK_CMD_LOCK;
    static OnOff_t     prevSpeakerState = ON_OFF_OFF;
    static Ignition_t  prevIgnitionState = IGNITION_OFF;
    static boolean     prevTheftActive = FALSE;
    static boolean     prevSafeExitActive = FALSE;
    static UserId_t    prevCurrentUser = USER_ID_1;

    /* 1) 현재 사용자 변경 */
    if (g_ctrlState.current_user != prevCurrentUser)
    {
        prevCurrentUser = g_ctrlState.current_user;
    }

    /* 2) 잠금 상태 적용 */
    if (g_ctrlState.lock_state != prevLockState)
    {
        Apply_LockState(g_ctrlState.lock_state);
        prevLockState = g_ctrlState.lock_state;
    }

    /* 3) 문 상태 적용 */
    if (g_ctrlState.door_state != prevDoorState)
    {
        Apply_DoorState(g_ctrlState.door_state);
        prevDoorState = g_ctrlState.door_state;
    }

    /* 4) 창문 상태 적용 */
    if (g_ctrlState.window_state != prevWindowState)
    {
        Apply_WindowState(g_ctrlState.window_state);
        prevWindowState = g_ctrlState.window_state;
    }

    /* 5) 스피커 상태 적용 */
    if (g_ctrlState.speaker != prevSpeakerState)
    {
        Apply_SpeakerState(g_ctrlState.speaker);

        /* 1회성 재생이면 다시 OFF로 내림 */
        if (g_ctrlState.speaker == ON_OFF_ON)
        {
            g_ctrlState.speaker = ON_OFF_OFF;
        }

        prevSpeakerState = g_ctrlState.speaker;
    }

    /* 6) 시동 상태 변화 */
    if (g_ctrlState.ignition_state != prevIgnitionState)
    {
        prevIgnitionState = g_ctrlState.ignition_state;
    }

    /* 7) 도난 상태 변화 */
    if (g_ctrlState.theft_active != prevTheftActive)
    {
        prevTheftActive = g_ctrlState.theft_active;
    }

    /* 8) safe exit 상태 변화 */
    if (g_ctrlState.safe_exit_active != prevSafeExitActive)
    {
        prevSafeExitActive = g_ctrlState.safe_exit_active;
    }
}

static void Apply_LockState(LockCmd_t state)
{
    DoorLock_SetState(state);
}

static void Apply_DoorState(OpenClose_t state)
{
    if (state == OPEN_CLOSE_OPEN)
    {
        if (g_ctrlState.door_state == OPEN_CLOSE_OPEN &&
            g_ultraReady == TRUE && g_distanceCm < OBSTACLE_THRESHOLD_CM)
        {
            g_ctrlState.door_state = OPEN_CLOSE_CLOSE;
            g_rxDoorOpenCmd        = OPEN_CLOSE_CLOSE;
            Door_Motor_Stop();
            g_reqBuzzer = TRUE;
        }

        Door_Motor_SetTarget((float32)g_user_settings.door_angle);
        RGB_SetColor(g_user_settings.led_color);
    }
    else
    {
        Door_Motor_SetTarget(0.0f);
        RGB_Reset();
    }
}

static void Apply_WindowState(OpenClose_t state)
{
    if (state == OPEN_CLOSE_OPEN)
    {
        Window_Motor_SetTarget(WINDOW_MAX_ANGLE);
    }
    else
    {
        Window_Motor_SetTarget(0.0f);
    }
}

static void Apply_SpeakerState(OnOff_t state)
{
    if (state == ON_OFF_ON)
    {
        playDFPlayerMusic(1, 1);
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
    /* 초음파 감지 시 즉시 정지 — Door_Motor_Update() 호출 전에 위치 */
    if (g_ctrlState.door_state == OPEN_CLOSE_OPEN &&
        g_ultraReady == TRUE && g_distanceCm < OBSTACLE_THRESHOLD_CM)
    {
        g_ctrlState.door_state = OPEN_CLOSE_CLOSE;
        g_rxDoorOpenCmd        = OPEN_CLOSE_CLOSE;
        Door_Motor_Stop();
        g_reqBuzzer = TRUE;
    }

    BT_ProcessTask();

    CanEvent_ServiceTask();
    ControlState_ApplyTask();
    BUZ_Task();
    // Speaker_Task();

    // can_send_UserSettingValue(70U);

    FSR_Task();
    HandlePressure_Task();

    DoorButton_Task();
    WindowButton_Task();
    PinchMonitor_Task();

    // 끼임 감지 시 모터 중단
    if (CurrentSensor_GetFault(DOOR_MOTOR_ID) == CURRENT_FAULT_MOTOR)
    {
        Door_Motor_Stop();
        CurrentSensor_StopSense(DOOR_MOTOR_ID);
    }

    if (CurrentSensor_GetFault(WINDOW_MOTOR_ID) == CURRENT_FAULT_MOTOR)
    {
        Window_Motor_Stop();
        CurrentSensor_StopSense(WINDOW_MOTOR_ID);
    }

    Button_Update();
    Door_Motor_Update();
    Window_Motor_Update();

    // 문이 닫혔을 시 끼임 감지 초기화
    if (Door_Motor_GetState() == MOTOR_DONE)
    {
        CurrentSensor_StopSense(DOOR_MOTOR_ID);
        CurrentSensor_ClearFault(DOOR_MOTOR_ID);
        CurrentSensor_StartSense(DOOR_MOTOR_ID);
    }

    if (Window_Motor_GetState() == MOTOR_DONE)
    {
        CurrentSensor_StopSense(WINDOW_MOTOR_ID);
        CurrentSensor_ClearFault(WINDOW_MOTOR_ID);
        CurrentSensor_StartSense(WINDOW_MOTOR_ID);
    }

    stTestCnt.u32nuCnt10ms++;
}


void AppTask100ms(void)
{
    UserSetting_Task();

    /* 0x115 Door Status 주기 송신 */
    can_send_DoorStatus();

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

