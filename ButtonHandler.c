#include "ButtonHandler.h"
#include "IfxCpu.h"

/* 문/창문 모터 제어 */
static InBtnState_t btn_win_up;
static InBtnState_t btn_win_down;
static InBtnState_t btn_door;

/* 문 잠금 제어 */
//static InBtnState_t btn_door_lock;
//volatile LockCmd_t g_doorLockState = LOCK_CMD_UNLOCK;
//
///* 스피커 제어 */
//static InBtnState_t btn_speaker;
//volatile SpeakerCmd_t g_speakerState = SPEAKER_CMD_OFF;

// ButtonHandler.c 파일 상단에 추가
volatile boolean dbg_raw_up   = FALSE;
volatile boolean dbg_raw_down = FALSE;
volatile boolean dbg_raw_door = FALSE;

/* 10ms 주기 호출 기준 디바운스 */
static void Btn_Debounce(InBtnState_t *b, boolean raw_active)
{
    b->edge_detected = FALSE;

    if (raw_active)
    {
        if (b->timer_ms < BTN_DEBOUNCE_MS)
            b->timer_ms += 10u;
    }
    else
    {
        b->timer_ms  = 0u;
        b->debounced = FALSE;   // 손 떼면 리셋 → 다음 누름 준비
    }

    if (b->timer_ms >= BTN_DEBOUNCE_MS && b->debounced == FALSE)
    {
        b->edge_detected = TRUE;
        b->debounced     = TRUE; // 누르고 있는 동안 딱 1회만
    }
}

void Button_Init(void)
{
    /* 문/창문 모터 제어 */
    IfxPort_setPinModeInput(BTN_WIN_UP_PORT,   BTN_WIN_UP_PIN,   IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(BTN_WIN_DOWN_PORT, BTN_WIN_DOWN_PIN, IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(BTN_DOOR_PORT,     BTN_DOOR_PIN,     IfxPort_InputMode_pullUp);

    btn_win_up   = (InBtnState_t){FALSE, FALSE, 0u, FALSE};
    btn_win_down = (InBtnState_t){FALSE, FALSE, 0u, FALSE};
    btn_door     = (InBtnState_t){FALSE, FALSE, 0u, FALSE};

    /* 문 잠금/해제 제어 */
//    IfxPort_setPinModeInput(BTN_DOOR_LOCK_PORT,   BTN_DOOR_LOCK_PIN,   IfxPort_InputMode_pullUp);
//
//    btn_door_lock   = (InBtnState_t){FALSE, FALSE, 0u, FALSE};
//
//    /* 스피커 ON/OFF 제어 */
//    IfxPort_setPinModeInput(BTN_SPEAKER_ONOFF_PORT,   BTN_SPEAKER_ONOFF_PIN,   IfxPort_InputMode_pullUp);
//
//    btn_speaker   = (InBtnState_t){FALSE, FALSE, 0u, FALSE};
//
//    IfxPort_setPinModeOutput(LED_DOOR_LOCK_PORT, LED_DOOR_LOCK_PIN,
//                             IfxPort_OutputMode_pushPull,
//                             IfxPort_OutputIdx_general);
//    IfxPort_setPinLow(LED_DOOR_LOCK_PORT, LED_DOOR_LOCK_PIN);
}

void Button_Update(void)
{
    boolean raw_up   = !IfxPort_getPinState(BTN_WIN_UP_PORT, BTN_WIN_UP_PIN);
    boolean raw_down = !IfxPort_getPinState(BTN_WIN_DOWN_PORT, BTN_WIN_DOWN_PIN);
    boolean raw_door = !IfxPort_getPinState(BTN_DOOR_PORT, BTN_DOOR_PIN);

//    boolean raw_lock    = !IfxPort_getPinState(BTN_DOOR_LOCK_PORT, BTN_DOOR_LOCK_PIN);
//    boolean raw_speaker = !IfxPort_getPinState(BTN_SPEAKER_ONOFF_PORT, BTN_SPEAKER_ONOFF_PIN);


    dbg_raw_up   = raw_up;
    dbg_raw_down = raw_down;
    dbg_raw_door = raw_door;

    Btn_Debounce(&btn_win_up,   raw_up);
    Btn_Debounce(&btn_win_down, raw_down);
    Btn_Debounce(&btn_door,     raw_door);
//    Btn_Debounce(&btn_door_lock, raw_lock);
//    Btn_Debounce(&btn_speaker,   raw_speaker);

    /* ── 창문 DOWN ── */
    if (btn_win_down.edge_detected)
    {
        if (Window_Motor_GetState() == MOTOR_RUNNING)
        {
            Window_Motor_Stop();
        }
        else
        {
            sint32 win_max_count = (sint32)(WINDOW_MAX_ANGLE * WINDOW_COUNTS_PER_DEG + 0.5f);
            IfxCpu_disableInterrupts();
            sint32 cur = window_encoder_count;
            IfxCpu_enableInterrupts();

            if (cur < win_max_count - WINDOW_ARRIVE_THRESH)
                Window_Motor_SetTarget(WINDOW_MAX_ANGLE);
        }
    }

    /* ── 창문 UP ── */
    if (btn_win_up.edge_detected)
    {
        if (Window_Motor_GetState() == MOTOR_RUNNING)
        {
            Window_Motor_Stop();
        }
        else
        {
            IfxCpu_disableInterrupts();
            sint32 cur = window_encoder_count;
            IfxCpu_enableInterrupts();

            if (cur > -WINDOW_ARRIVE_THRESH)
                Window_Motor_SetTarget(0.0f);
        }
    }

    /* ── 문 토글 ── */
    if (btn_door.edge_detected)
    {
        if (Door_Motor_GetState() == MOTOR_RUNNING)
        {
            Door_Motor_Stop();
        }
        else
        {
            IfxCpu_disableInterrupts();
            sint32 door_cur = door_encoder_count;
            IfxCpu_enableInterrupts();

            if (door_cur > DOOR_ARRIVE_THRESH)
                Door_Motor_SetTarget(0.0f);
            else
                Door_Motor_SetTarget(DOOR_MAX_ANGLE);
        }
    }

//    /* ── 문 잠금 토글 ── */
//    if (btn_door_lock.edge_detected)
//    {
//        if (g_doorLockState == LOCK_CMD_UNLOCK)
//            g_doorLockState = LOCK_CMD_LOCK;
//        else
//            g_doorLockState = LOCK_CMD_UNLOCK;
//    }
//
//    /* ── 스피커 토글 ── */
//    if (btn_speaker.edge_detected)
//    {
//        if (g_speakerState == SPEAKER_CMD_OFF)
//            g_speakerState = SPEAKER_CMD_ON;
//        else
//            g_speakerState = SPEAKER_CMD_OFF;
//    }
}
