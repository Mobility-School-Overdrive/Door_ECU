/**********************************************************************************************************************
 * \file MotorDriver.c
 * \brief DC Motor (Door / Window) PWM + Quadrature Encoder Driver
 *        논블로킹 상태머신 구조 - 센서 드라이버와 연동 가능
 *********************************************************************************************************************/
#include "IfxCpu.h"
#include "MotorDriver.h"
#include "DoorApp.h"

/*============================================================
 * PWM Driver 인스턴스 (내부 전용)
 *============================================================*/
static IfxGtm_Tom_Pwm_Driver PWM_Door_Driver;
static IfxGtm_Tom_Pwm_Config PWM_Door_Config;
static IfxGtm_Tom_Pwm_Driver PWM_Window_Driver;
static IfxGtm_Tom_Pwm_Config PWM_Window_Config;

/*============================================================
 * Encoder TIM 핸들 (내부 전용)
 *============================================================*/
static IfxGtm_Tim_In Door_Tim_A_Handle;
static IfxGtm_Tim_In Door_Tim_B_Handle;
static IfxGtm_Tim_In Window_Tim_A_Handle;
static IfxGtm_Tim_In Window_Tim_B_Handle;

/*============================================================
 * 상태머신 내부 상태 (내부 전용)
 *============================================================*/
static MotorState_t door_state   = MOTOR_IDLE;
static MotorState_t window_state = MOTOR_IDLE;
static sint32       door_target_count   = 0;
static sint32       window_target_count = 0;
static sint32       door_last_dir       = 0;   // 방향 전환 감지용
static sint32       window_last_dir     = 0;

/*============================================================
 * 공유 엔코더 카운트
 *============================================================*/
volatile sint32 door_encoder_count   = 0;
volatile sint32 window_encoder_count = 0;

/*============================================================
 * 디버그용 전역변수
 *============================================================*/
volatile sint32 dbg_door_target = 0;
volatile sint32 dbg_door_error  = 0;
volatile sint32 dbg_door_abs    = 0;
volatile sint32 dbg_win_target  = 0;
volatile sint32 dbg_win_error   = 0;
volatile sint32 dbg_win_abs     = 0;
volatile uint32 dbg_duty        = 0;
volatile uint32 dbg_reach       = 0;
volatile uint32 dbg_brake       = 100;

/*============================================================
 * 내부 함수 선언
 *============================================================*/
static void GPIO_Init(void);
static void GTM_Init(void);
static void Door_PWM_Init(void);
static void Window_PWM_Init(void);
static void Encoder_TIM_Init(void);
static void Door_PWM_Update(uint32 duty);
static void Window_PWM_Update(uint32 duty);
static void Door_Motor_Drive(sint32 error, sint32 abs_err);
static void Window_Motor_Drive(sint32 error, sint32 abs_err);
void Door_Motor_SetDuty(uint32 duty);
void Window_Motor_SetDuty(uint32 duty);

/*============================================================
 * ISR - Door 엔코더 A (P13.1)
 *============================================================*/
IFX_INTERRUPT(door_encoder_a_isr, 0, DOOR_ENC_A_ISR_PRIO)
{
    boolean b = IfxPort_getPinState(&MODULE_P13, 2);
    if (b == 0) door_encoder_count--;
    else        door_encoder_count++;
}

/*============================================================
 * ISR - Door 엔코더 B (P13.2)
 *============================================================*/
IFX_INTERRUPT(door_encoder_b_isr, 0, DOOR_ENC_B_ISR_PRIO)
{
    boolean a = IfxPort_getPinState(&MODULE_P13, 1);
    if (a == 0) door_encoder_count++;
    else        door_encoder_count--;
}

/*============================================================
 * ISR - Window 엔코더 A (P02.4)
 *============================================================*/
IFX_INTERRUPT(window_encoder_a_isr, 0, WINDOW_ENC_A_ISR_PRIO)
{
    boolean b = IfxPort_getPinState(&MODULE_P02, 5);
    if (b == 0) window_encoder_count--;
    else        window_encoder_count++;
}

/*============================================================
 * ISR - Window 엔코더 B (P02.5)
 *============================================================*/
IFX_INTERRUPT(window_encoder_b_isr, 0, WINDOW_ENC_B_ISR_PRIO)
{
    boolean a = IfxPort_getPinState(&MODULE_P02, 4);
    if (a == 0) window_encoder_count++;
    else        window_encoder_count--;
}

/*============================================================
 * GPIO 초기화 (내부)
 *============================================================*/
static void GPIO_Init(void)
{
    // Door Motor
    IfxPort_setPinModeOutput(IfxPort_P02_7.port, IfxPort_P02_7.pinIndex,
            IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general); // BRAKE A
    IfxPort_setPinModeOutput(IfxPort_P10_1.port, IfxPort_P10_1.pinIndex,
            IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general); // DIR A

    // Window Motor
    IfxPort_setPinModeOutput(IfxPort_P02_6.port, IfxPort_P02_6.pinIndex,
            IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general); // BRAKE B
    IfxPort_setPinModeOutput(IfxPort_P10_2.port, IfxPort_P10_2.pinIndex,
            IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general); // DIR B

    // 초기 상태: BRAKE ON (안전), DIR CW
    IfxPort_setPinHigh(IfxPort_P02_7.port, IfxPort_P02_7.pinIndex);  // BRAKE A ON
    IfxPort_setPinHigh(IfxPort_P10_1.port, IfxPort_P10_1.pinIndex);  // DIR A CW
    IfxPort_setPinHigh(IfxPort_P02_6.port, IfxPort_P02_6.pinIndex);  // BRAKE B ON
    IfxPort_setPinHigh(IfxPort_P10_2.port, IfxPort_P10_2.pinIndex);  // DIR B CW
}

/*============================================================
 * GTM 초기화 (내부)
 *============================================================*/
static void GTM_Init(void)
{
    IfxGtm_enable(&MODULE_GTM);
    IfxGtm_Cmu_enableClocks(&MODULE_GTM, IFXGTM_CMU_CLKEN_FXCLK | IFXGTM_CMU_CLKEN_CLK0);
}

/*============================================================
 * PWM 초기화 (내부)
 *============================================================*/
static void Door_PWM_Init(void)
{
    IfxGtm_Tom_Pwm_initConfig(&PWM_Door_Config, &MODULE_GTM);
    PWM_Door_Config.clock                    = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk1;
    PWM_Door_Config.tom                      = DOOR_PWM_PIN.tom;
    PWM_Door_Config.tomChannel               = DOOR_PWM_PIN.channel;
    PWM_Door_Config.period                   = DOOR_PWM_PERIOD;
    PWM_Door_Config.pin.outputPin            = &DOOR_PWM_PIN;
    PWM_Door_Config.synchronousUpdateEnabled = TRUE;
    PWM_Door_Config.dutyCycle                = 0u;
    IfxGtm_Tom_Pwm_init(&PWM_Door_Driver, &PWM_Door_Config);
    IfxGtm_Tom_Pwm_start(&PWM_Door_Driver, TRUE);
}

static void Window_PWM_Init(void)
{
    IfxGtm_Tom_Pwm_initConfig(&PWM_Window_Config, &MODULE_GTM);
    PWM_Window_Config.clock                    = IfxGtm_Tom_Ch_ClkSrc_cmuFxclk1;
    PWM_Window_Config.tom                      = WINDOW_PWM_PIN.tom;
    PWM_Window_Config.tomChannel               = WINDOW_PWM_PIN.channel;
    PWM_Window_Config.period                   = WINDOW_PWM_PERIOD;
    PWM_Window_Config.pin.outputPin            = &WINDOW_PWM_PIN;
    PWM_Window_Config.synchronousUpdateEnabled = TRUE;
    PWM_Window_Config.dutyCycle                = 0u;
    IfxGtm_Tom_Pwm_init(&PWM_Window_Driver, &PWM_Window_Config);
    IfxGtm_Tom_Pwm_start(&PWM_Window_Driver, TRUE);
}

/*============================================================
 * Encoder TIM 초기화 (내부)
 *============================================================*/
static void Encoder_TIM_Init(void)
{
    IfxGtm_Tim_In_Config timConfig;

    // Door A (P13.1)
    IfxGtm_Tim_In_initConfig(&timConfig, &MODULE_GTM);
    timConfig.filter.inputPin     = &DOOR_ENC_A_PIN;
    timConfig.filter.inputPinMode = IfxPort_InputMode_pullUp;
    timConfig.capture.clock       = IfxGtm_Cmu_Clk_0;
    timConfig.capture.activeEdge  = IfxGtm_Tim_In_ActiveEdge_both;
    timConfig.capture.irqOnNewVal = TRUE;
    timConfig.isrPriority         = DOOR_ENC_A_ISR_PRIO;
    timConfig.isrProvider         = IfxSrc_Tos_cpu0;
    IfxGtm_Tim_In_init(&Door_Tim_A_Handle, &timConfig);

    // Door B (P13.2)
    IfxGtm_Tim_In_initConfig(&timConfig, &MODULE_GTM);
    timConfig.filter.inputPin     = &DOOR_ENC_B_PIN;
    timConfig.filter.inputPinMode = IfxPort_InputMode_pullUp;
    timConfig.capture.clock       = IfxGtm_Cmu_Clk_0;
    timConfig.capture.activeEdge  = IfxGtm_Tim_In_ActiveEdge_both;
    timConfig.capture.irqOnNewVal = TRUE;
    timConfig.isrPriority         = DOOR_ENC_B_ISR_PRIO;
    timConfig.isrProvider         = IfxSrc_Tos_cpu0;
    IfxGtm_Tim_In_init(&Door_Tim_B_Handle, &timConfig);

    // Window A (P02.4)
    IfxGtm_Tim_In_initConfig(&timConfig, &MODULE_GTM);
    timConfig.filter.inputPin     = &WINDOW_ENC_A_PIN;
    timConfig.filter.inputPinMode = IfxPort_InputMode_pullUp;
    timConfig.capture.clock       = IfxGtm_Cmu_Clk_0;
    timConfig.capture.activeEdge  = IfxGtm_Tim_In_ActiveEdge_both;
    timConfig.capture.irqOnNewVal = TRUE;
    timConfig.isrPriority         = WINDOW_ENC_A_ISR_PRIO;
    timConfig.isrProvider         = IfxSrc_Tos_cpu0;
    IfxGtm_Tim_In_init(&Window_Tim_A_Handle, &timConfig);

    // Window B (P02.5)
    IfxGtm_Tim_In_initConfig(&timConfig, &MODULE_GTM);
    timConfig.filter.inputPin     = &WINDOW_ENC_B_PIN;
    timConfig.filter.inputPinMode = IfxPort_InputMode_pullUp;
    timConfig.capture.clock       = IfxGtm_Cmu_Clk_0;
    timConfig.capture.activeEdge  = IfxGtm_Tim_In_ActiveEdge_both;
    timConfig.capture.irqOnNewVal = TRUE;
    timConfig.isrPriority         = WINDOW_ENC_B_ISR_PRIO;
    timConfig.isrProvider         = IfxSrc_Tos_cpu0;
    IfxGtm_Tim_In_init(&Window_Tim_B_Handle, &timConfig);
}

/*============================================================
 * Motor_Init (public)
 *============================================================*/
void Motor_Init(void)
{
    GPIO_Init();
    GTM_Init();
    Door_PWM_Init();
    Window_PWM_Init();
    Door_Motor_Stop();
    Window_Motor_Stop();
    Encoder_TIM_Init();
    door_encoder_count   = 0;
    window_encoder_count = 0;
    door_state           = MOTOR_IDLE;
    window_state         = MOTOR_IDLE;
    door_last_dir        = 0;
    window_last_dir      = 0;
}

/*============================================================
 * PWM 업데이트 (내부)
 *============================================================*/
static void Door_PWM_Update(uint32 duty)
{
    PWM_Door_Config.dutyCycle = duty;
    IfxGtm_Tom_Pwm_init(&PWM_Door_Driver, &PWM_Door_Config);
    IfxGtm_Tom_Pwm_start(&PWM_Door_Driver, TRUE);
}

static void Window_PWM_Update(uint32 duty)
{
    PWM_Window_Config.dutyCycle = duty;
    IfxGtm_Tom_Pwm_init(&PWM_Window_Driver, &PWM_Window_Config);
    IfxGtm_Tom_Pwm_start(&PWM_Window_Driver, TRUE);
}
/*============================================================
 * Motor_Delay_ms (public)
 *============================================================*/
void Motor_Delay_ms(uint32 ms)
{
    uint32 ticks = IfxStm_getTicksFromMilliseconds(&MODULE_STM0, ms);
    uint32 start = IfxStm_getLower(&MODULE_STM0);
    while ((IfxStm_getLower(&MODULE_STM0) - start) < ticks)
    {
        __nop();
    }
}

/*============================================================
 * 내부 드라이브 로직 (속도 결정)
 *============================================================*/
static void Door_Motor_Drive(sint32 error, sint32 abs_err)
{
    sint32 dir = (error > 0) ? 1 : -1;

    if (door_last_dir != 0 && door_last_dir != dir)
    {
        Door_PWM_Update(0);
        IfxPort_setPinHigh(IfxPort_P02_7.port, IfxPort_P02_7.pinIndex);
        Motor_Delay_ms(10);
    }
    door_last_dir = dir;

    uint32 duty;
    if      (abs_err >= DOOR_MAX_COUNT * 0.5) duty = (uint32)(0.15f * DOOR_PWM_PERIOD);
    else if (abs_err >= DOOR_MAX_COUNT * 0.25) duty = (uint32)(0.10f * DOOR_PWM_PERIOD);
    else                   duty = (uint32)(MIN_PWM / 100.0f * DOOR_PWM_PERIOD);

    dbg_duty  = duty;   // ← 추가
    dbg_brake = 0;      // ← BRAKE OFF 직전 확인용

    IfxPort_setPinLow(IfxPort_P02_7.port, IfxPort_P02_7.pinIndex);
    if (dir > 0) IfxPort_setPinHigh(IfxPort_P10_1.port, IfxPort_P10_1.pinIndex);
    else         IfxPort_setPinLow(IfxPort_P10_1.port,  IfxPort_P10_1.pinIndex);
    Door_PWM_Update(duty);
}

static void Window_Motor_Drive(sint32 error, sint32 abs_err)
{
    sint32 dir = (error > 0) ? 1 : -1;

    if (window_last_dir != 0 && window_last_dir != dir)
    {
        Window_PWM_Update(0);
        IfxPort_setPinHigh(IfxPort_P02_6.port, IfxPort_P02_6.pinIndex); // BRAKE ON
        Motor_Delay_ms(10);
        // Window_Motor_Stop() 대신 직접 처리 → state 안 바꿈
    }
    window_last_dir = dir;

    uint32 duty;
    if      (abs_err >= WINDOW_MAX_COUNT * 0.5) duty = (uint32)(0.15f * WINDOW_PWM_PERIOD);
    else if (abs_err >= WINDOW_MAX_COUNT * 0.25) duty = (uint32)(0.10f * WINDOW_PWM_PERIOD);
    else                   duty = (uint32)(MIN_PWM / 100.0f * WINDOW_PWM_PERIOD);

    IfxPort_setPinLow(IfxPort_P02_6.port, IfxPort_P02_6.pinIndex);   // BRAKE OFF
    if (dir > 0) IfxPort_setPinHigh(IfxPort_P10_2.port, IfxPort_P10_2.pinIndex); // CW
    else         IfxPort_setPinLow(IfxPort_P10_2.port,  IfxPort_P10_2.pinIndex); // CCW
    Window_PWM_Update(duty);
}

/*============================================================
 * Door Motor API (public)
 *============================================================*/

// 목표 각도 설정 → 즉시 리턴 (논블로킹)
void Door_Motor_SetTarget(float target_angle)
{
    door_target_count = (sint32)(target_angle * DOOR_COUNTS_PER_DEG + 0.5f);
    door_last_dir     = 0;
    door_state        = MOTOR_RUNNING;
}

// 메인루프에서 5ms마다 호출 → 상태 한 스텝 진행
void Door_Motor_Update(void)
{
    if (door_state != MOTOR_RUNNING) return;

    IfxCpu_disableInterrupts();
    sint32 current = door_encoder_count;
    IfxCpu_enableInterrupts();

    sint32 error   = door_target_count - current;
    sint32 abs_err = (error < 0) ? -error : error;

    dbg_door_target = door_target_count;
    dbg_door_error  = error;
    dbg_door_abs    = abs_err;

    if (abs_err <= DOOR_ARRIVE_THRESH)
    {
        Door_Motor_Stop();
        door_state = MOTOR_DONE;
        return;
    }

    Door_Motor_Drive(error, abs_err);
}

// 즉시 정지 - 센서 드라이버나 외부에서 직접 호출
void Door_Motor_Stop(void)
{
    Door_PWM_Update(0);
    IfxPort_setPinHigh(IfxPort_P02_7.port, IfxPort_P02_7.pinIndex);  // BRAKE ON
    door_state    = MOTOR_STOPPED;
    door_last_dir = 0;
}

// 블로킹 홈 복귀 - Motor_Init 직후 1회만 사용
void Door_Motor_Home(void)
{
    Door_Motor_Stop();
    Motor_Delay_ms(1000);
    door_encoder_count = 0;
    door_state         = MOTOR_IDLE;
}

MotorState_t Door_Motor_GetState(void)
{
    return door_state;
}

/*============================================================
 * Window Motor API (public)
 *============================================================*/

void Window_Motor_SetTarget(float target_angle)
{
    window_target_count = (sint32)(target_angle * WINDOW_COUNTS_PER_DEG + 0.5f);
    window_last_dir     = 0;
    window_state        = MOTOR_RUNNING;
}

void Window_Motor_Update(void)
{
    if (window_state != MOTOR_RUNNING) return;

    IfxCpu_disableInterrupts();
    sint32 current = window_encoder_count;
    IfxCpu_enableInterrupts();

    sint32 error   = window_target_count - current;
    sint32 abs_err = (error < 0) ? -error : error;

    dbg_win_target = window_target_count;
    dbg_win_error  = error;
    dbg_win_abs    = abs_err;

    if (abs_err <= WINDOW_ARRIVE_THRESH)
    {
        Window_Motor_Stop();
        window_state = MOTOR_DONE;
        return;
    }

    Window_Motor_Drive(error, abs_err);
}

void Window_Motor_Stop(void)
{
    Window_PWM_Update(0);
    IfxPort_setPinHigh(IfxPort_P02_6.port, IfxPort_P02_6.pinIndex);  // BRAKE ON
    window_state    = MOTOR_STOPPED;
    window_last_dir = 0;
}

void Window_Motor_Home(void)
{
    Window_Motor_Stop();
    Motor_Delay_ms(1000);
    window_encoder_count = 0;
    window_state         = MOTOR_IDLE;
}

MotorState_t Window_Motor_GetState(void)
{
    return window_state;
}

void Door_Motor_SetDuty(uint32 duty)
{
    Door_PWM_Update(duty);
}

void Window_Motor_SetDuty(uint32 duty)
{
    Window_PWM_Update(duty);
}

/***************/

float32 Door_Motor_GetCurrentAngle(void)
{
    return ((float32)door_encoder_count / DOOR_ENCODER_COUNT_PER_DEG);
}

float32 Window_Motor_GetCurrentAngle(void)
{
    return ((float32)window_encoder_count / WINDOW_ENCODER_COUNT_PER_DEG);
}
