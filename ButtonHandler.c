#include "ButtonHandler.h"
#include "IfxCpu.h"

typedef struct {
    boolean last_raw;
    boolean debounced;
    uint32  timer_ms;
    boolean edge_detected;
} BtnState_t;

static BtnState_t btn_win_up;
static BtnState_t btn_win_down;
static BtnState_t btn_door;

// ButtonHandler.c 파일 상단에 추가
volatile boolean dbg_raw_up   = FALSE;
volatile boolean dbg_raw_down = FALSE;
volatile boolean dbg_raw_door = FALSE;

/* 10ms 주기 호출 기준 디바운스 */
static void Btn_Debounce(BtnState_t *b, boolean raw_active)
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
    IfxPort_setPinModeInput(BTN_WIN_UP_PORT,   BTN_WIN_UP_PIN,   IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(BTN_WIN_DOWN_PORT, BTN_WIN_DOWN_PIN, IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(BTN_DOOR_PORT,     BTN_DOOR_PIN,     IfxPort_InputMode_pullUp);

    btn_win_up   = (BtnState_t){FALSE, FALSE, 0u, FALSE};
    btn_win_down = (BtnState_t){FALSE, FALSE, 0u, FALSE};
    btn_door     = (BtnState_t){FALSE, FALSE, 0u, FALSE};
}

void Button_Update(void)
{
    boolean raw_up   = !IfxPort_getPinState(&MODULE_P15, 5);
    boolean raw_down = !IfxPort_getPinState(&MODULE_P15, 4);
    boolean raw_door = !IfxPort_getPinState(&MODULE_P20, 14);

    dbg_raw_up   = raw_up;
    dbg_raw_down = raw_down;
    dbg_raw_door = raw_door;

    Btn_Debounce(&btn_win_up,   raw_up);
    Btn_Debounce(&btn_win_down, raw_down);
    Btn_Debounce(&btn_door,     raw_door);

    /* ── 창문 UP ── */
    if (btn_win_up.edge_detected)
    {
        if (Window_Motor_GetState() == MOTOR_RUNNING)
        {
            Window_Motor_Stop();
        }
        else
        {
            sint32 win_max_count = (sint32)(WINDOW_MAX_ANGLE * COUNTS_PER_DEG + 0.5f);
            IfxCpu_disableInterrupts();
            sint32 cur = window_encoder_count;
            IfxCpu_enableInterrupts();

            if (cur < win_max_count)
                Window_Motor_SetTarget(WINDOW_MAX_ANGLE);
        }
    }

    /* ── 창문 DOWN ── */
    if (btn_win_down.edge_detected)
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

            if (cur > 0)
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
}
