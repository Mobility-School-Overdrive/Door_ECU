/* PinTest.c - P11.11, P15.4, P15.5 LED 토글 테스트 */
#include "IfxPort.h"
#include "Stm/Std/IfxStm.h"

static void delay_ms(uint32 ms)
{
    uint32 ticks = IfxStm_getTicksFromMilliseconds(&MODULE_STM0, ms);
    uint32 start = IfxStm_getLower(&MODULE_STM0);
    while ((IfxStm_getLower(&MODULE_STM0) - start) < ticks) { __nop(); }
}

void PinTest_Run(void)
{
    /* 출력 모드로 설정 */
    IfxPort_setPinModeOutput(&MODULE_P20, 14,
        IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(&MODULE_P15, 4,
        IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);
    IfxPort_setPinModeOutput(&MODULE_P15, 5,
        IfxPort_OutputMode_pushPull, IfxPort_OutputIdx_general);

    /* 초기 LOW */
    IfxPort_setPinLow(&MODULE_P20, 14);
    IfxPort_setPinLow(&MODULE_P15, 4);
    IfxPort_setPinLow(&MODULE_P15, 5);

    for (;;)
    {
        IfxPort_togglePin(&MODULE_P20, 14);
        delay_ms(1000);

        IfxPort_togglePin(&MODULE_P15, 4);
        delay_ms(1000);

        IfxPort_togglePin(&MODULE_P15, 5);
        delay_ms(1000);
    }
}
