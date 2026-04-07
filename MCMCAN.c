/********************************************************************************************************************
 * \file MCMCAN.c
 * \brief  Door ECU (TC375) — MCMCAN 드라이버 구현
 *
 * 수신 필터 설계:
 *   Filter 0  range   0x103~0x104  Obstacle Detection, Theft Detection
 *   Filter 1  range   0x141~0x143  Door Lock, Door Open, Safe Exit Assist
 *   Filter 2  classic 0x200        Smartkey Proximity Event
 *   Filter 3  classic 0x202        Window Control
 *   Filter 4  dual    0x205, 0x301 Ignition State Broadcast, User Setting Broadcast
 *********************************************************************************************************************/

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include "MCMCAN.h"
#include "Buzzer.h"
#include "DoorApp.h"

/*********************************************************************************************************************/
/*-------------------------------------------------Global variables--------------------------------------------------*/
/*********************************************************************************************************************/
DoorCanType g_doorCan;

/* 자동차 상태 */
VehicleControlState_t g_ctrlState = {
    .door_state     = OPEN_CLOSE_CLOSE,
    .window_state   = OPEN_CLOSE_CLOSE,
    .lock_state     = LOCK_CMD_LOCK,
    .speaker        = ON_OFF_OFF,
    .ignition_state = IGNITION_OFF,
    .theft_active   = FALSE,
    .safe_exit_active = FALSE,
    .current_user   = USER_ID_1
};

/* 사용자 설정 배열 — 0x301 User Setting Broadcast 수신 시 업데이트 */
UserSetting_t g_user_settings = {
    .door_angle = 90, .mirror_angle = 90,
    .led_color  = LED_COLOR_GREEN, .seat_pos = 50
};

/* Door ECU 최근 수신 명령/상태 */
volatile LockCmd_t   g_rxDoorLockCmd    = LOCK_CMD_UNLOCK;
volatile OpenClose_t g_rxDoorOpenCmd    = OPEN_CLOSE_CLOSE;
volatile OpenClose_t g_rxWindowCmd      = OPEN_CLOSE_CLOSE;
volatile boolean     g_rxSafeExitAssist = FALSE;
volatile boolean     g_rxTheftDetected  = FALSE;

/* 이벤트 pending 플래그 */
volatile boolean g_reqObstacleWarn      = FALSE;
volatile boolean g_reqTheftWarn         = FALSE;
volatile boolean g_reqMirrorLedUpdate   = FALSE;
volatile boolean g_reqSpeakerPlay       = FALSE;
volatile boolean g_reqIgnitionOnAction  = FALSE;
volatile boolean g_reqIgnitionOffAction = FALSE;
volatile boolean g_reqBuzzer            = FALSE;

/* 현재 사용자 기준 적용값 */
volatile uint8      g_currentDoorAngle  = 90U;
volatile LedColor_t g_currentLedColor   = LED_COLOR_GREEN;

/* 최근 수신 상태 */
volatile boolean g_canObstacleDetected        = FALSE;
volatile boolean g_canSmartKeyProximityUpdated = FALSE;
volatile boolean g_canIgnitionUpdated        = FALSE;
volatile boolean g_canUserSettingUpdated     = FALSE;

/* 최근 수신 메시지 캐시 */
SmtkProximityEvent_t g_rxSmtkProximity;
IgnitionStateBcast_t g_rxIgnitionState = { .ignition_state = IGNITION_OFF };
uint8                g_rxCurrentUserId = USER_ID_1;

/*********************************************************************************************************************/
/*--------------------------------------------------ISR Definitions--------------------------------------------------*/
/*********************************************************************************************************************/
IFX_INTERRUPT(canIsrTxHandler, 0, ISR_PRIORITY_CAN_TX);
IFX_INTERRUPT(canIsrRxHandler, 0, ISR_PRIORITY_CAN_RX);

static const IfxCan_Can_Pins canPins =
{
    &TX_PIN, IfxPort_OutputMode_pushPull,
    &RX_PIN, IfxPort_InputMode_pullUp,
    IfxPort_PadDriver_cmosAutomotiveSpeed1
};

volatile uint32 g_dbgTxIsrCount = 0;
void canIsrTxHandler(void)
{
    g_dbgTxIsrCount++;

    IfxCan_Node_clearInterruptFlag(g_doorCan.canSrcNode.node,
                                   IfxCan_Interrupt_transmissionCompleted);
}

void canIsrRxHandler(void)
{
    IfxCan_Node_clearInterruptFlag(g_doorCan.canSrcNode.node,
                                   IfxCan_Interrupt_rxFifo0NewMessage);

    g_doorCan.rxMsg.readFromRxFifo0 = TRUE;
    IfxCan_Can_readMessage(&g_doorCan.canSrcNode,
                           &g_doorCan.rxMsg,
                           g_doorCan.rxData);

    processCanRxMessage(&g_doorCan.rxMsg, g_doorCan.rxData);
}

/*********************************************************************************************************************/
/*--------------------------------------------Initialization---------------------------------------------------------*/
/*********************************************************************************************************************/

/*
 * Door ECU 수신 필터 설정
 *
 * 필터 번호 | 타입    | 범위·대상                    | 메시지
 * ----------+---------+-----------------------------+-----------------------------
 *     0     | range   | 0x103 ~ 0x104               | Obstacle Detection, Theft Detection
 *     1     | range   | 0x141 ~ 0x143               | Door Lock, Door Open, Safe Exit Assist
 *     2     | classic | 0x200                        | Smartkey Proximity Event
 *     3     | classic | 0x202                        | Window Control
 *     4     | dual    | 0x205  &  0x301              | Ignition State, User Setting Broadcast
 *
 * 수신하지 않는 이유:
 *   0x140 Vehicle Control — Door ECU와 무관 (기어·EPB·전후진은 Car Control 담당)
 *   0x105 Pinch Detection — Door ECU가 송신하는 메시지, 수신 불필요
 */
static void initCanFilters_Door(void)
{
    IfxCan_Filter f;
    f.elementConfiguration = IfxCan_FilterElementConfiguration_storeInRxFifo0;

    /* 필터 0 — 비상 이벤트 (경고음·LED 처리 담당) */
    f.number = 0;
    f.type   = IfxCan_FilterType_range;
    f.id1    = CAN_ID_OBSTACLE_DETECTION;   /* 0x103 */
    f.id2    = CAN_ID_THEFT_DETECTION;      /* 0x104 */
    IfxCan_Can_setStandardFilter(&g_doorCan.canSrcNode, &f);

    /* 필터 1 — Door ECU 전용 커맨드 (문·창문·하차보조) */
    f.number = 1;
    f.type   = IfxCan_FilterType_range;
    f.id1    = CAN_ID_DOOR_LOCK_CONTROL;    /* 0x141 */
    f.id2    = CAN_ID_SAFE_EXIT_ASSIST;     /* 0x143 */
    IfxCan_Can_setStandardFilter(&g_doorCan.canSrcNode, &f);

    /* 필터 2 — Smartkey Proximity Event (문·LED·스피커·창문 비트 담당) */
    f.number = 2;
    f.type   = IfxCan_FilterType_classic;
    f.id1    = CAN_ID_SMTK_PROXIMITY;       /* 0x200 */
    f.id2    = 0x7FFu;                      /* mask: 단일 ID */
    IfxCan_Can_setStandardFilter(&g_doorCan.canSrcNode, &f);

    /* 필터 3 — Window Control */
    f.number = 3;
    f.type   = IfxCan_FilterType_classic;
    f.id1    = CAN_ID_WINDOW_CONTROL;       /* 0x202 */
    f.id2    = 0x7FFu;
    IfxCan_Can_setStandardFilter(&g_doorCan.canSrcNode, &f);

    /* 필터 4 — Ignition State Broadcast & User Setting Broadcast */
    f.number = 4;
    f.type   = IfxCan_FilterType_dualId;
    f.id1    = CAN_ID_IGNITION_STATE_BCAST; /* 0x205 */
    f.id2    = CAN_ID_USER_SETTING_BCAST;   /* 0x301 */
    IfxCan_Can_setStandardFilter(&g_doorCan.canSrcNode, &f);
}

void initMcmcan(void)
{
    IfxPort_setPinModeOutput(&MODULE_P20, 6,
                             IfxPort_OutputMode_pushPull,
                             IfxPort_OutputIdx_general);
    IfxPort_setPinLow(&MODULE_P20, 6);

    IfxCan_Can_initModuleConfig(&g_doorCan.canConfig, &MODULE_CAN0);
    IfxCan_Can_initModule(&g_doorCan.canModule, &g_doorCan.canConfig);
    IfxCan_Can_initNodeConfig(&g_doorCan.canNodeConfig, &g_doorCan.canModule);

    g_doorCan.canNodeConfig.pins               = &canPins;
    g_doorCan.canNodeConfig.busLoopbackEnabled = FALSE;
    g_doorCan.canNodeConfig.nodeId             = IfxCan_NodeId_0;
    g_doorCan.canNodeConfig.frame.type         = IfxCan_FrameType_transmitAndReceive;

    /* TX 인터럽트 */
    g_doorCan.canNodeConfig.interruptConfig.transmissionCompletedEnabled  = TRUE;
    g_doorCan.canNodeConfig.interruptConfig.traco.priority                = ISR_PRIORITY_CAN_TX;
    g_doorCan.canNodeConfig.interruptConfig.traco.interruptLine           = IfxCan_InterruptLine_0;
    g_doorCan.canNodeConfig.interruptConfig.traco.typeOfService           = IfxSrc_Tos_cpu0;

    /* RX FIFO0 인터럽트 */
    g_doorCan.canNodeConfig.interruptConfig.rxFifo0NewMessageEnabled      = TRUE;
    g_doorCan.canNodeConfig.interruptConfig.rxf0n.priority                = ISR_PRIORITY_CAN_RX;
    g_doorCan.canNodeConfig.interruptConfig.rxf0n.interruptLine           = IfxCan_InterruptLine_1;
    g_doorCan.canNodeConfig.interruptConfig.rxf0n.typeOfService           = IfxSrc_Tos_cpu0;

    /* Message RAM */
    g_doorCan.canNodeConfig.messageRAM.baseAddress                        = (uint32)&MODULE_CAN0;
    g_doorCan.canNodeConfig.messageRAM.standardFilterListStartAddress     = 0x100;
    g_doorCan.canNodeConfig.messageRAM.rxFifo0StartAddress               = 0x200;

    /* RX FIFO0 */
    g_doorCan.canNodeConfig.rxConfig.rxMode               = IfxCan_RxMode_fifo0;
    g_doorCan.canNodeConfig.rxConfig.rxFifo0Size          = 16;
    g_doorCan.canNodeConfig.rxConfig.rxFifo0OperatingMode = IfxCan_RxFifoMode_blocking;

    IfxCan_Can_initNode(&g_doorCan.canSrcNode, &g_doorCan.canNodeConfig);

    /* Door ECU 수신 필터 적용 */
    initCanFilters_Door();
}

/*********************************************************************************************************************/
/*--------------------------------------저수준 송신 (내부 공용)------------------------------------------------------*/
/*********************************************************************************************************************/

void canSendRaw(uint32 txId, IfxCan_DataLengthCode dlc,
                const uint8 *payload, uint8 len)
{
    memset(g_doorCan.txData, 0x00, sizeof(g_doorCan.txData));

    if ((payload != NULL) && (len > 0u))
    {
        memcpy(g_doorCan.txData, payload, len);
    }

    IfxCan_Can_initMessage(&g_doorCan.txMsg);
    g_doorCan.txMsg.messageId      = txId;
    g_doorCan.txMsg.dataLengthCode = dlc;

    while (IfxCan_Status_notSentBusy ==
           IfxCan_Can_sendMessage(&g_doorCan.canSrcNode,
                                  &g_doorCan.txMsg,
                                  g_doorCan.txData))
    {}
}

/*********************************************************************************************************************/
/*------------------------------------------Door ECU 송신 래퍼-------------------------------------------------------*/
/*********************************************************************************************************************/

/*
 * 0x105  Pinch Detection  DLC=1  (→ ALL)
 * Door ECU 로컬에서 역방향 처리 후 CAN으로 보고.
 * Main ECU: 보류 / Car Control: LCD 표시
 */
void can_send_PinchDetection(PinchType_t type)
{
    uint8 payload = (uint8)type;
    canSendRaw(CAN_ID_PINCH_DETECTION,
               IfxCan_DataLengthCode_1,
               &payload, 1u);
}

/* 0x115 Door Status  (→ Main) */
void can_send_DoorStatus(void)
{
    DoorStatus_Frame_t frame;

    memset(&frame, 0, sizeof(frame));

    frame.fields.door_open_status   = g_ctrlState.door_state;
    frame.fields.door_lock_status   = g_ctrlState.lock_state;
    frame.fields.window_open_status = g_ctrlState.window_state;

    canSendRaw(CAN_ID_DOOR_STATUS,
               IfxCan_DataLengthCode_3,
               frame.raw,
               3u);
}

/* 0x211 Smartkey RSSI Event  (→ Main) */
void can_send_SmtkRssiEvent(UserId_t userId, RssiType_t rssiType)
{
    SmtkRssi_Frame_t frame;

    memset(&frame, 0, sizeof(frame));
    frame.fields.user_id = userId;
    frame.fields.rssi_type = rssiType;

    canSendRaw(CAN_ID_SMTK_RSSI_EVENT,
               IfxCan_DataLengthCode_2,
               frame.raw,
               2u);
}

/* 0x220  Handle Pressure Event  DLC=0  (→ Main) */
void can_send_HandlePressure(void)
{
    canSendRaw(CAN_ID_HANDLE_PRESSURE,
               IfxCan_DataLengthCode_0,
               NULL, 0u);
}

/* 0x221  Door Button Event  DLC=1  (→ Main) */
void can_send_DoorButtonEvent(BtnState_t state)
{
    uint8 payload = (uint8)state;
    canSendRaw(CAN_ID_DOOR_BUTTON_EVENT,
               IfxCan_DataLengthCode_1,
               &payload, 1u);
}

/* 0x222  Window Button Event  DLC=1  (→ Main) */
void can_send_WindowButtonEvent(BtnState_t state)
{
    uint8 payload = (uint8)state;
    canSendRaw(CAN_ID_WINDOW_BUTTON_EVENT,
               IfxCan_DataLengthCode_1,
               &payload, 1u);
}

/* 0x300  User Setting Value  DLC=1  (→ Car Control)
 * 사용자가 Door ECU UI에서 문 각도를 변경했을 때 Car Control에 알림 */
void can_send_UserSettingValue(uint8 doorAngle)
{
    canSendRaw(CAN_ID_USER_SETTING_VALUE,
               IfxCan_DataLengthCode_1,
               &doorAngle, 1u);
}

/* 0x210  Smartkey Button Bypass  DLC=2  (→ Main)
 * ESP32 UART 수신 후 CAN으로 Main ECU에 전달 */
void can_send_SmtkButtonBypass(UserId_t userId, BtnType_t btnType)
{
    SmtkButton_Frame_t frame;

    memset(&frame, 0, sizeof(frame));
    frame.fields.user_id = userId;
    frame.fields.btn_type = btnType;

    canSendRaw(CAN_ID_SMTK_BUTTON_EVENT,
               IfxCan_DataLengthCode_2,
               frame.raw,
               2u);
}

/*********************************************************************************************************************/
/*------------------------------------------수신 메시지 dispatch-----------------------------------------------------*/
/*********************************************************************************************************************/

void processCanRxMessage(const IfxCan_Message *msg, const uint32 *data)
{
    const uint8 *raw  = (const uint8 *)data;   /* uint32[] → uint8[] 재해석 */
    uint32       rxId = msg->messageId;

    switch (rxId)
    {
        /* ── 0x103  Obstacle Detection  DLC=0 ── */
        case CAN_ID_OBSTACLE_DETECTION:
            handler_ObstacleDetection_Door();
            break;

        /* ── 0x104  Theft Detection  DLC=0 ── */
        case CAN_ID_THEFT_DETECTION:
            handler_TheftDetection();
            break;

        /* ── 0x141  Door Lock Control  DLC=1 ── */
        case CAN_ID_DOOR_LOCK_CONTROL:
        {
            LockCmd_t cmd = (LockCmd_t)raw[0];
            handler_DoorLockControl(cmd);
            break;
        }

        /* ── 0x142  Door Open Control  DLC=1 ── */
        case CAN_ID_DOOR_OPEN_CONTROL:
        {
            OpenClose_t cmd = (OpenClose_t)raw[0];
            handler_DoorOpenControl(cmd);
            break;
        }

        /* ── 0x143  Safe Exit Assist  DLC=0 ── */
        case CAN_ID_SAFE_EXIT_ASSIST:
            handler_SafeExitAssist();
            break;

        /* ── 0x200  Smartkey Proximity Event  DLC=8 ── */
        case CAN_ID_SMTK_PROXIMITY:
        {
            SmtkProximityEvent_t smtk;
            memcpy(&smtk, raw, sizeof(SmtkProximityEvent_t));
            handler_SmtkProximity_Door(&smtk);
            break;
        }

        /* ── 0x202  Window Control  DLC=1 ── */
        case CAN_ID_WINDOW_CONTROL:
        {
            WindowControl_t winMsg;
            memcpy(&winMsg, raw, sizeof(WindowControl_t));
            handler_WindowControl(winMsg.window_cmd);
            break;
        }

        /* ── 0x205  Ignition State Broadcast  DLC=1 ── */
        case CAN_ID_IGNITION_STATE_BCAST:
        {
            IgnitionStateBcast_t ignMsg;
            memcpy(&ignMsg, raw, sizeof(IgnitionStateBcast_t));
            handler_IgnitionStateBcast_Door(ignMsg.ignition_state);
            break;
        }
        /* ── 0x301  User Setting Broadcast  DLC=5 ── */
        case CAN_ID_USER_SETTING_BCAST:
        {
            uint8 userId = raw[0];
            UserSetting_t setting;

            setting.door_angle   = raw[1];
            setting.mirror_angle = raw[2];
            setting.led_color    = (LedColor_t)raw[3];
            setting.seat_pos     = raw[4];

            handler_UserSettingBcast_Door(userId, &setting);
            break;
        }

        default:
            break;
    }

    (void)msg;
}

/*********************************************************************************************************************/
/*--------------------------------------수신 핸들러 구현-------------------------------------------------------------*/
/*********************************************************************************************************************/

static inline uint8 userId_toIndex(uint8 userId)
{
    return (userId > 0u) ? (userId - 1u) : 0u;
}

/* 0x103 — 장애물 감지 */
void handler_ObstacleDetection_Door(void)
{
    g_canObstacleDetected = TRUE;
    g_reqObstacleWarn = TRUE;
    g_reqBuzzer = TRUE;
}

/* 0x104 — 도난 감지 */
void handler_TheftDetection(void)
{
    g_rxTheftDetected = TRUE;
    g_ctrlState.theft_active = TRUE;
    g_ctrlState.speaker = ON_OFF_ON;

    g_reqTheftWarn = TRUE;
    g_reqMirrorLedUpdate = TRUE;
    g_reqBuzzer = TRUE;
}

/* 0x141 — 문 잠금/해제 */
void handler_DoorLockControl(LockCmd_t cmd)
{
    g_rxDoorLockCmd = cmd;
    g_ctrlState.lock_state = cmd;

    if (cmd == LOCK_CMD_UNLOCK)
    {
        g_ctrlState.speaker = ON_OFF_ON;
    }
}

/* 0x142 — 문 열기/닫기 */
void handler_DoorOpenControl(OpenClose_t cmd)
{
    g_rxDoorOpenCmd = cmd;
    g_ctrlState.door_state = cmd;
}

/* 0x143 — 안전 하차 보조 */
void handler_SafeExitAssist(void)
{
    g_rxSafeExitAssist = TRUE;
    g_ctrlState.safe_exit_active = TRUE;
    g_ctrlState.speaker = ON_OFF_ON;
    g_ctrlState.door_state = OPEN_CLOSE_CLOSE;
    g_rxDoorOpenCmd = OPEN_CLOSE_CLOSE;

    g_reqObstacleWarn = TRUE;
    g_reqBuzzer = TRUE;
}

/*
 * 0x200 — Smartkey Proximity Event
 * Door ECU 담당 비트만 처리
 */
void handler_SmtkProximity_Door(const SmtkProximityEvent_t *msg)
{
    uint8 idx;

    if (msg == NULL)
    {
        return;
    }

    g_rxSmtkProximity = *msg;
    g_rxCurrentUserId = (uint8)msg->user_id;
    g_canSmartKeyProximityUpdated = TRUE;
    g_ctrlState.current_user = (UserId_t)msg->user_id;

    idx = userId_toIndex((uint8)msg->user_id);

    if (idx < USER_COUNT)
    {
        g_currentDoorAngle = g_user_settings.door_angle;
        g_currentLedColor  = g_user_settings.led_color;
    }

    if (msg->ctrl_flags.bits.lock)
    {
        g_ctrlState.lock_state = msg->lock_val;

        if (msg->lock_val == LOCK_CMD_UNLOCK)
        {
            g_ctrlState.speaker = ON_OFF_ON;
        }
    }

    if (msg->ctrl_flags.bits.door)
    {
        g_ctrlState.door_state = msg->door_val;
    }

    if (msg->ctrl_flags.bits.window)
    {
        g_ctrlState.window_state = msg->window_val;
    }

    if (msg->ctrl_flags.bits.speaker)
    {
        g_ctrlState.speaker = msg->speaker;
    }

    if (msg->ctrl_flags.bits.mirror_led)
    {
        g_reqMirrorLedUpdate = TRUE;
    }
}

/* 0x202 — 창문 열기/닫기 */
void handler_WindowControl(OpenClose_t cmd)
{
    g_rxWindowCmd = cmd;
    g_ctrlState.window_state = cmd;
}

/* 0x205 — 시동 상태 전파 */
void handler_IgnitionStateBcast_Door(Ignition_t state)
{
    g_rxIgnitionState.ignition_state = state;
    g_canIgnitionUpdated = TRUE;
    g_ctrlState.ignition_state = state;

    if (state == IGNITION_ON)
    {
        g_reqIgnitionOnAction = TRUE;
        g_reqIgnitionOffAction = FALSE;
    }
    else
    {
        g_reqIgnitionOffAction = TRUE;
        g_reqIgnitionOnAction = FALSE;
    }
}

/* 0x301 — 사용자 설정 전파 */
void handler_UserSettingBcast_Door(uint8 userId, const UserSetting_t *setting)
{
    uint8 idx = userId_toIndex(userId);

    if ((setting == NULL) || (idx >= USER_COUNT))
    {
        return;
    }

    /* 현재 구조는 g_user_settings 1개만 있으므로
       우선 최근 수신값 저장 */
    g_user_settings = *setting;
    g_canUserSettingUpdated = TRUE;

    /* 현재 사용자면 즉시 적용 */
    if (g_rxCurrentUserId == userId)
    {
        g_currentDoorAngle = setting->door_angle;
        g_currentLedColor  = setting->led_color;

        /* 필요 시 미러 LED 갱신 요청 */
        g_reqMirrorLedUpdate = TRUE;
    }
}
