/**********************************************************************************************************************
 * \file MotorDriver.c
 * \brief DC Motor (Door / Window) PWM + Quadrature Encoder Driver
 *********************************************************************************************************************/
#include "MotorDriver.h"
#include "IfxCpu.h"

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
 * 공유 엔코더 카운트
 *============================================================*/
volatile sint32 door_encoder_count   = 0;
volatile sint32 window_encoder_count = 0;

/*============================================================
 * 디버그용 전역변수
 *============================================================*/
volatile sint32 dbg_target = 0;
volatile sint32 dbg_error  = 0;
volatile sint32 dbg_abs    = 0;
volatile uint32 dbg_duty   = 0;

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

    // 초기 상태: BRAKE ON, DIR CW
    IfxPort_setPinHigh(IfxPort_P02_7.port,  IfxPort_P02_7.pinIndex);
    IfxPort_setPinHigh(IfxPort_P10_1.port, IfxPort_P10_1.pinIndex);
    IfxPort_setPinHigh(IfxPort_P02_6.port,  IfxPort_P02_6.pinIndex);
    IfxPort_setPinHigh(IfxPort_P10_2.port, IfxPort_P10_2.pinIndex);
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
}

/*============================================================
 * PWM 업데이트 (내부)
 *============================================================*/
static void Door_PWM_Update(uint32 duty)
{
    Ifx_GTM_TOM *tom = PWM_Door_Driver.tom;
    uint8 ch = PWM_Door_Driver.tomChannel;
    IfxGtm_Tom_Ch_setCompareOneShadow(tom, ch, duty);
    IfxGtm_Tom_Tgc_trigger(PWM_Door_Driver.tgc);
}

static void Window_PWM_Update(uint32 duty)
{
    Ifx_GTM_TOM *tom = PWM_Window_Driver.tom;
    uint8 ch = PWM_Window_Driver.tomChannel;
    IfxGtm_Tom_Ch_setCompareOneShadow(tom, ch, duty);
    IfxGtm_Tom_Tgc_trigger(PWM_Window_Driver.tgc);
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
 * Door Motor (public)
 *============================================================*/
void Door_Motor_Start_CW(float percent)
{
    if (percent > 100.0f) percent = 100.0f;
    if (percent < 0.0f)   percent = 0.0f;

    uint32 duty = (uint32)((percent / 100.0f) * DOOR_PWM_PERIOD);
    dbg_duty = duty;

    IfxPort_setPinLow(IfxPort_P02_7.port,  IfxPort_P02_7.pinIndex);  // BRAKE OFF
    Motor_Delay_ms(10);
    IfxPort_setPinHigh(IfxPort_P10_1.port, IfxPort_P10_1.pinIndex);  // DIR CW
    Door_PWM_Update(duty);
}

void Door_Motor_Start_CCW(float percent)
{
    if (percent > 100.0f) percent = 100.0f;
    if (percent < 0.0f)   percent = 0.0f;

    uint32 duty = (uint32)((percent / 100.0f) * DOOR_PWM_PERIOD);
    dbg_duty = duty;

    IfxPort_setPinLow(IfxPort_P02_7.port, IfxPort_P02_7.pinIndex);   // BRAKE OFF
    Motor_Delay_ms(10);
    IfxPort_setPinLow(IfxPort_P10_1.port, IfxPort_P10_1.pinIndex);   // DIR CCW
    Door_PWM_Update(duty);
}

void Door_Motor_Stop(void)
{
    Door_PWM_Update(0);
    IfxPort_setPinHigh(IfxPort_P02_7.port, IfxPort_P02_7.pinIndex);  // BRAKE ON
}

void Door_Motor_Home(void)
{
    Door_Motor_Stop();
    Motor_Delay_ms(1000);
    door_encoder_count = 0;
}

void Door_Motor_Angle_Control(float target_angle)
{
    sint32 target_count = (sint32)(target_angle * COUNTS_PER_DEG + 0.5f);

    while (1)
    {
        IfxCpu_disableInterrupts();
        sint32 current = door_encoder_count;
        IfxCpu_enableInterrupts();

        sint32 error   = target_count - current;
        sint32 abs_err = (error < 0) ? -error : error;

        dbg_target = target_count;
        dbg_error  = error;
        dbg_abs    = abs_err;

        if (abs_err <= DOOR_ARRIVE_THRESH)
        {
            Door_Motor_Stop();
            break;
        }
        else if (abs_err >= 8)
        {
            if (error > 0) Door_Motor_Start_CW(40.0f);
            else           Door_Motor_Start_CCW(40.0f);
        }
        else if (abs_err >= 4)
        {
            if (error > 0) Door_Motor_Start_CW(30.0f);
            else           Door_Motor_Start_CCW(30.0f);
        }
        else
        {
            if (error > 0) Door_Motor_Start_CW(MIN_PWM);
            else           Door_Motor_Start_CCW(MIN_PWM);
        }

        Motor_Delay_ms(5);
    }
}

/*============================================================
 * Window Motor (public)
 *============================================================*/
void Window_Motor_Start_CW(float percent)
{
    if (percent > 100.0f) percent = 100.0f;
    if (percent < 0.0f)   percent = 0.0f;

    uint32 duty = (uint32)((percent / 100.0f) * WINDOW_PWM_PERIOD);

    IfxPort_setPinLow(IfxPort_P02_6.port,  IfxPort_P02_6.pinIndex);  // BRAKE OFF
    Motor_Delay_ms(10);
    IfxPort_setPinHigh(IfxPort_P10_2.port, IfxPort_P10_2.pinIndex);  // DIR CW
    Window_PWM_Update(duty);
}

void Window_Motor_Start_CCW(float percent)
{
    if (percent > 100.0f) percent = 100.0f;
    if (percent < 0.0f)   percent = 0.0f;

    uint32 duty = (uint32)((percent / 100.0f) * WINDOW_PWM_PERIOD);

    IfxPort_setPinLow(IfxPort_P02_6.port, IfxPort_P02_6.pinIndex);   // BRAKE OFF
    Motor_Delay_ms(10);
    IfxPort_setPinLow(IfxPort_P10_2.port, IfxPort_P10_2.pinIndex);   // DIR CCW
    Window_PWM_Update(duty);
}

void Window_Motor_Stop(void)
{
    Window_PWM_Update(0);
    IfxPort_setPinHigh(IfxPort_P02_6.port, IfxPort_P02_6.pinIndex);  // BRAKE ON
}

void Window_Motor_Home(void)
{
    Window_Motor_Stop();
    Motor_Delay_ms(1000);
    window_encoder_count = 0;
}

void Window_Motor_Angle_Control(float target_angle)
{
    sint32 target_count = (sint32)(target_angle * COUNTS_PER_DEG + 0.5f);

    while (1)
    {
        IfxCpu_disableInterrupts();
        sint32 current = window_encoder_count;
        IfxCpu_enableInterrupts();

        sint32 error   = target_count - current;
        sint32 abs_err = (error < 0) ? -error : error;

        if (abs_err <= WINDOW_ARRIVE_THRESH)
        {
            Window_Motor_Stop();
            break;
        }
        else if (abs_err >= 8)
        {
            if (error > 0) Window_Motor_Start_CW(40.0f);
            else           Window_Motor_Start_CCW(40.0f);
        }
        else if (abs_err >= 4)
        {
            if (error > 0) Window_Motor_Start_CW(30.0f);
            else           Window_Motor_Start_CCW(30.0f);
        }
        else
        {
            if (error > 0) Window_Motor_Start_CW(MIN_PWM);
            else           Window_Motor_Start_CCW(MIN_PWM);
        }

        Motor_Delay_ms(5);
    }
}
