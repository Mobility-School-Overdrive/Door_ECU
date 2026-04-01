/**********************************************************************************************************************
 * \file Cpu0_Main.c
 *********************************************************************************************************************/
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "Ifx_Cfg_Ssw.h"
#include "MotorDriver.h"

IFX_ALIGN(4) IfxCpu_syncEvent cpuSyncEvent = 0;

void core0_main(void)
{
    IfxCpu_enableInterrupts();
    IfxScuWdt_disableCpuWatchdog(IfxScuWdt_getCpuWatchdogPassword());
    IfxScuWdt_disableSafetyWatchdog(IfxScuWdt_getSafetyWatchdogPassword());
    IfxCpu_emitEvent(&cpuSyncEvent);
    IfxCpu_waitEvent(&cpuSyncEvent, 1);

    Motor_Init();

    Door_Motor_Home();
    Window_Motor_Home();

    while (1)
    {
        Door_Motor_Angle_Control(45.0f);
        Window_Motor_Angle_Control(45.0f);
        Motor_Delay_ms(2000);

        Door_Motor_Angle_Control(0.0f);
        Window_Motor_Angle_Control(0.0f);
        Motor_Delay_ms(2000);
    }
}
