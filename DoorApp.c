/**********************************************************************************************************************
 * \file DoorApp.c
 *********************************************************************************************************************/

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "IfxCpu.h"
#include "IfxPort.h"
#include "MotorDriver.h"
#include "DoorApp.h"
#include "MCMCAN.h"
#include "ButtonHandler.h"
#include "UltraSonic.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
/* 실제 하드웨어 연결 핀 */
#define DOOR_LOCK_PORT      &MODULE_P21
#define DOOR_LOCK_PIN       5


/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
static volatile LockCmd_t g_doorLockHwState = LOCK_CMD_UNLOCK;

/*********************************************************************************************************************/
/*---------------------------------------------Function Implementations----------------------------------------------*/
/*********************************************************************************************************************/

float clampFloat32(float value, float minVal, float maxVal)
{
    if (value < minVal)
    {
        return minVal;
    }

    if (value > maxVal)
    {
        return maxVal;
    }

    return value;
}

/* -------------------------------------------------
 * Door Lock
 * ------------------------------------------------- */
void DoorLock_Init(void)
{
    IfxPort_setPinModeOutput(DOOR_LOCK_PORT,
                             DOOR_LOCK_PIN,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);

    /* 기본 상태: unlock */
    IfxPort_setPinLow(DOOR_LOCK_PORT, DOOR_LOCK_PIN);
    g_doorLockHwState = LOCK_CMD_UNLOCK;
}

void DoorLock_SetState(LockCmd_t cmd)
{
    g_doorLockHwState = cmd;

    if (cmd == LOCK_CMD_LOCK)
    {
        IfxPort_setPinHigh(DOOR_LOCK_PORT, DOOR_LOCK_PIN);
    }
    else
    {
        IfxPort_setPinLow(DOOR_LOCK_PORT, DOOR_LOCK_PIN);
    }
}

LockCmd_t DoorLock_GetState(void)
{
    return g_doorLockHwState;
}

/* -------------------------------------------------
 * Door / Window Button
 * ------------------------------------------------- */
static boolean Door_OpenButton_IsPressed(void)
{
    /* Active Low 버튼 가정: 눌리면 0 */
    return (IfxPort_getPinState(BTN_DOOR_PORT, BTN_DOOR_PIN) == 0U) ? TRUE : FALSE;
}

void DoorButton_Task(void)
{
    static uint8 debounceCnt = 0U;
    static boolean latched = FALSE;

    boolean pressed = Door_OpenButton_IsPressed();

    if (pressed == TRUE)
    {
        if (debounceCnt < BUTTON_DEBOUNCE_COUNT)
        {
            debounceCnt++;
        }
        else if (latched == FALSE)
        {
            if (Door_Motor_GetState() == MOTOR_DONE)
            {
                if (g_ctrlState.door_state == OPEN_CLOSE_CLOSE)
                {
                    can_send_DoorButtonEvent(BTN_STATE_OPEN);
                }
                else
                {
                    can_send_DoorButtonEvent(BTN_STATE_CLOSE);
                }
            }

            latched = TRUE;
        }
    }
    else
    {
        debounceCnt = 0U;
        latched = FALSE;
    }
}

#define OBSTACLE_THRESHOLD_CM  30U

void DoorControl_Task(void)
{
    if (Door_Motor_GetState() == MOTOR_DONE)
    {
        return;
    }

    if (g_ctrlState.door_state == OPEN_CLOSE_OPEN)
    {
        Door_Motor_SetTarget((float32)g_currentDoorAngle);
    }
    else
    {
        Door_Motor_SetTarget(0.0f);
    }
}

static boolean Window_OpenButton_IsPressed(void)
{
    /* Active Low 버튼 가정 */
    return (IfxPort_getPinState(BTN_WIN_UP_PORT, BTN_WIN_UP_PIN) == 0U) ? TRUE : FALSE;
}

static boolean Window_CloseButton_IsPressed(void)
{
    /* Active Low 버튼 가정 */
    return (IfxPort_getPinState(BTN_WIN_DOWN_PORT, BTN_WIN_DOWN_PIN) == 0U) ? TRUE : FALSE;
}

void WindowButton_Task(void)
{
    static uint8 openDebounceCnt = 0U;
    static uint8 closeDebounceCnt = 0U;
    static boolean openLatched = FALSE;
    static boolean closeLatched = FALSE;

    boolean openPressed = Window_OpenButton_IsPressed();
    boolean closePressed = Window_CloseButton_IsPressed();

    if (openPressed == TRUE)
    {
        if (openDebounceCnt < BUTTON_DEBOUNCE_COUNT)
        {
            openDebounceCnt++;
        }
        else if (openLatched == FALSE)
        {
            can_send_WindowButtonEvent(BTN_STATE_OPEN);
            openLatched = TRUE;
        }
    }
    else
    {
        openDebounceCnt = 0U;
        openLatched = FALSE;
    }

    if (closePressed == TRUE)
    {
        if (closeDebounceCnt < BUTTON_DEBOUNCE_COUNT)
        {
            closeDebounceCnt++;
        }
        else if (closeLatched == FALSE)
        {
            can_send_WindowButtonEvent(BTN_STATE_CLOSE);
            closeLatched = TRUE;
        }
    }
    else
    {
        closeDebounceCnt = 0U;
        closeLatched = FALSE;
    }
}

void UserSetting_Task(void)
{
    static uint8 prevDoorAngle = 90U;
    uint8 currentDoorAngle = (uint8)g_currentDoorAngle;

    if (currentDoorAngle != prevDoorAngle)
    {
        can_send_UserSettingValue(currentDoorAngle);
        prevDoorAngle = currentDoorAngle;
    }
}

void DoorSmartKeyUnlockSequence(void)
{
    /* 1) unlock */
    DoorLock_SetState(LOCK_CMD_UNLOCK);

    /* 2) unlock 안내는 기존 lock 로직에서 blink 처리됨 */
    /* 추가 동작이 필요하면 여기 확장 */
}

void DoorSmartKeyLockSequence(void)
{
    /* 문 닫힘 이후 lock을 거는 흐름용 */
    DoorLock_SetState(LOCK_CMD_LOCK);
}
