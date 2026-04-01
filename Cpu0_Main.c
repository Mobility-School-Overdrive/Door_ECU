/**********************************************************************************************************************
 * \file Cpu0_Main.c
 *********************************************************************************************************************/
#include "Ifx_Types.h"
#include "IfxCpu.h"
#include "IfxScuWdt.h"
#include "Ifx_Cfg_Ssw.h"
#include "MotorDriver.h"

// TODO: 센서 드라이버 합칠 때 아래 주석 해제
// #include "SensorDriver.h"

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
        // ── 센서 폴링 ──────────────────────────────────────
        // TODO: 센서 드라이버 합칠 때 아래 블록 채워넣기
        //
        // float ultrasonic = Sensor_GetUltrasonic();
        // float current    = Sensor_GetCurrent();
        //
        // if (ultrasonic < ULTRASONIC_THRESH || current > CURRENT_THRESH)
        // {
        //     Door_Motor_Stop();
        //     Window_Motor_Stop();
        // }
        // ────────────────────────────────────────────────────

        // ── 모터 상태머신 업데이트 (논블로킹) ───────────────
        Door_Motor_Update();
        Window_Motor_Update();

        // ── 상태 전환 예시 ───────────────────────────────────
        // DONE 상태가 되면 다음 목표로 전환
        if (Door_Motor_GetState()   == MOTOR_DONE &&
            Window_Motor_GetState() == MOTOR_DONE)
        {
            Motor_Delay_ms(2000);
            Door_Motor_SetTarget(0.0f);
            Window_Motor_SetTarget(0.0f);
        }

        Motor_Delay_ms(5);  // 루프 주기 5ms
    }
}
