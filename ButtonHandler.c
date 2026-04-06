#include "ButtonHandler.h"

/* 내부 상태 */
typedef struct {
    boolean last_raw;
    boolean debounced;
    uint32  timer_ms;
    boolean edge_detected;   /* 눌림 엣지 1회 펄스 */
} BtnState_t;

static BtnState_t btn_win_up;
static BtnState_t btn_win_down;
static BtnState_t btn_door;

/*------------------------------------------------------------
 * 내부: 단일 버튼 디바운스 처리
 * raw_active: TRUE = 지금 눌려있음 (active-low 핀이면 NOT 처리 후 전달)
 *------------------------------------------------------------*/
static void Btn_Debounce(BtnState_t *b, boolean raw_active)
{
    b->edge_detected = FALSE;

    // 이전에 안눌렸다가 지금 눌렸을 때만 엣지
    if (raw_active == TRUE && b->last_raw == FALSE)
    {
        b->edge_detected = TRUE;
    }

    b->last_raw = raw_active;  // debounced 말고 last_raw 로 이전 상태 추적
}
/*------------------------------------------------------------
 * Button_Init
 *------------------------------------------------------------*/
void Button_Init(void)
{
    IfxPort_setPinModeInput(BTN_WIN_UP_PORT,   BTN_WIN_UP_PIN,   IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(BTN_WIN_DOWN_PORT, BTN_WIN_DOWN_PIN, IfxPort_InputMode_pullUp);
    IfxPort_setPinModeInput(BTN_DOOR_PORT,     BTN_DOOR_PIN,     IfxPort_InputMode_pullUp);

    btn_win_up.last_raw   = FALSE;
    btn_win_up.debounced  = FALSE;
    btn_win_up.timer_ms   = 0u;
    btn_win_down.last_raw = FALSE;
    btn_win_down.debounced= FALSE;
    btn_win_down.timer_ms = 0u;
    btn_door.last_raw     = FALSE;
    btn_door.debounced    = FALSE;
    btn_door.timer_ms     = 0u;
}

/*------------------------------------------------------------
 * Button_Update  (5ms 주기로 호출 권장)
 *------------------------------------------------------------*/
void Button_Update(void)
{
    const uint32 PERIOD_MS = 5u;

    /* 핀 읽기 (active-low → NOT) */
    boolean raw_up   = !IfxPort_getPinState(&MODULE_P20, 14);
    boolean raw_down = !IfxPort_getPinState(&MODULE_P15, 4);
    boolean raw_door = !IfxPort_getPinState(&MODULE_P15, 5);

    Btn_Debounce(&btn_win_up,   raw_up);
    Btn_Debounce(&btn_win_down, raw_down);
    Btn_Debounce(&btn_door,     raw_door);

    /* ── 창문 UP 버튼 ─────────────────────────────────────── */
    if (btn_win_up.edge_detected)
    {
        if (Window_Motor_GetState() == MOTOR_RUNNING)
        {
            /* 이동 중 → 그 자리에서 정지 */
            Window_Motor_Stop();
        }
        else
        {
            /* 완전 열림 여부: encoder ≥ MAX_COUNT이면 무시 */
            sint32 win_max_count = (sint32)(WINDOW_MAX_ANGLE * COUNTS_PER_DEG + 0.5f);
            IfxCpu_disableInterrupts();
            sint32 cur = window_encoder_count;
            IfxCpu_enableInterrupts();

            if (cur < win_max_count)
            {
                Window_Motor_SetTarget(WINDOW_MAX_ANGLE);
            }
            /* else: 이미 끝 → 무시 */
        }
    }

    /* ── 창문 DOWN 버튼 ───────────────────────────────────── */
    if (btn_win_down.edge_detected)
    {
        if (Window_Motor_GetState() == MOTOR_RUNNING)
        {
            /* 이동 중 → 그 자리에서 정지 */
            Window_Motor_Stop();
        }
        else
        {
            /* 완전 닫힘 여부: encoder ≤ 0이면 무시 */
            IfxCpu_disableInterrupts();
            sint32 cur = window_encoder_count;
            IfxCpu_enableInterrupts();

            if (cur > 0)
            {
                Window_Motor_SetTarget(0.0f);
            }
            /* else: 이미 닫힘 → 무시 */
        }
    }

    /* ── 문 열림/닫힘 토글 버튼 ──────────────────────────── */
    if (btn_door.edge_detected)
    {
        IfxCpu_disableInterrupts();
        sint32 door_cur = door_encoder_count;
        IfxCpu_enableInterrupts();

        if (door_cur > 0)
        {
            /* 조금이라도 열려있으면 → 닫기 */
            Door_Motor_SetTarget(0.0f);
        }
        else
        {
            /* 완전 닫혀있으면 → 80도 최대로 열기 */
            Door_Motor_SetTarget(DOOR_MAX_ANGLE);
        }
    }
}
