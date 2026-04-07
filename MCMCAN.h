/********************************************************************************************************************
 * \file MCMCAN.h
 * \brief  Door ECU (TC375) — MCMCAN 드라이버 헤더
 *
 * Door ECU 담당 하드웨어:
 *   LED (사이드미러 LED, 차내 LED), 스피커/부저
 *   DC 모터 × 2 (문, 창문), 압력 센서, 전류 센서 × 2
 *   차내 문/창문 버튼, UART ← ESP32 스마트키
 *********************************************************************************************************************/

#ifndef MCMCAN_H_
#define MCMCAN_H_

/*********************************************************************************************************************/
/*-----------------------------------------------------Includes------------------------------------------------------*/
/*********************************************************************************************************************/
#include <string.h>
#include "Ifx_Types.h"
#include "IfxCan_Can.h"
#include "IfxCan.h"
#include "IfxCpu_Irq.h"
#include "IfxPort.h"
#include "can_type_def.h"

/*********************************************************************************************************************/
/*------------------------------------------------------Macros-------------------------------------------------------*/
/*********************************************************************************************************************/
#define ISR_PRIORITY_CAN_TX         111
#define ISR_PRIORITY_CAN_RX         110
#define CAN_DATA_BUF_WORDS          2       /* CAN 최대 8바이트 = uint32 × 2 */

#define TX_PIN  IfxCan_TXD00_P20_8_OUT
#define RX_PIN  IfxCan_RXD00B_P20_7_IN

/* Door ECU 수신 필터 개수 */
#define DOOR_ECU_FILTER_COUNT       5

#define CAN_TX_TIMEOUT_MAX          1000

/*********************************************************************************************************************/
/*--------------------------------------------------Data Structures--------------------------------------------------*/
/*********************************************************************************************************************/
typedef struct
{
    IfxCan_Can_Config       canConfig;
    IfxCan_Can              canModule;
    IfxCan_Can_Node         canSrcNode;
    IfxCan_Can_NodeConfig   canNodeConfig;
    IfxCan_Filter           canFilter;
    IfxCan_Message          txMsg;
    IfxCan_Message          rxMsg;
    uint32                  txData[CAN_DATA_BUF_WORDS];
    uint32                  rxData[CAN_DATA_BUF_WORDS];
} DoorCanType;

typedef struct
{
    OpenClose_t door_state;
    OpenClose_t window_state;
    LockCmd_t   lock_state;
    OnOff_t     speaker;

    Ignition_t  ignition_state;
    boolean     theft_active;
    boolean     safe_exit_active;
    UserId_t    current_user;
} VehicleControlState_t;

/* Door ECU 최근 수신 명령/상태 */
extern volatile LockCmd_t   g_rxDoorLockCmd;
extern volatile OpenClose_t g_rxDoorOpenCmd;
extern volatile OpenClose_t g_rxWindowCmd;
extern volatile boolean     g_rxSafeExitAssist;
extern volatile boolean     g_rxTheftDetected;

/* 이벤트 pending 플래그 */
extern volatile boolean g_reqObstacleWarn;
extern volatile boolean g_reqTheftWarn;
extern volatile boolean g_reqMirrorLedUpdate;
extern volatile boolean g_reqSpeakerPlay;
extern volatile boolean g_reqIgnitionOnAction;
extern volatile boolean g_reqIgnitionOffAction;

/* 현재 사용자 기준 적용값 */
extern volatile uint8      g_currentDoorAngle;
extern volatile LedColor_t g_currentLedColor;

/* 최근 수신 상태 */
extern volatile boolean g_canObstacleDetected;
extern volatile boolean g_canSmartKeyProximityUpdated;
extern volatile boolean g_canIgnitionUpdated;
extern volatile boolean g_canUserSettingUpdated;
extern volatile boolean g_reqBuzzer;

/* 최근 수신 메시지 캐시 */
extern SmtkProximityEvent_t  g_rxSmtkProximity;
extern IgnitionStateBcast_t  g_rxIgnitionState;
extern uint8                 g_rxCurrentUserId;

/* 사용자 설정 배열 */
extern UserSetting_t g_user_settings;

extern VehicleControlState_t g_ctrlState;

/*********************************************************************************************************************/
/*-----------------------------------------------Function Prototypes-------------------------------------------------*/
/*********************************************************************************************************************/

/* 초기화 */
void initMcmcan(void);

/* 저수준 송신 (내부 공용) */
void canSendRaw(uint32                txId,
                        IfxCan_DataLengthCode  dlc,
                        const uint8           *payload,
                        uint8                  len);

/* ── Door ECU 송신 함수 ── */

/* 끼임 감지 → ALL 브로드캐스트 (로컬 역방향 처리 후 보고) */
void can_send_PinchDetection      (PinchType_t  type);          /* 0x105  DLC=1 */

/* 이벤트 → Main ECU 보고 */
void can_send_HandlePressure      (void);                       /* 0x220  DLC=0 */
void can_send_DoorButtonEvent     (BtnState_t   state);         /* 0x221  DLC=1 */
void can_send_WindowButtonEvent   (BtnState_t   state);         /* 0x222  DLC=1 */

/* 사용자 설정 변경 → Control ECU 전달 */
void can_send_UserSettingValue    (uint8         doorAngle);    /* 0x300  DLC=1 */

/* ESP32 UART 수신 → Main ECU CAN Bypass */
void can_send_DoorStatus(void);
void can_send_SmtkButtonBypass(UserId_t userId, BtnType_t btnType); /* 0x210  DLC=2 */
void can_send_SmtkRssiEvent(UserId_t userId, RssiType_t rssiType);

/* ── 수신 처리 (ISR → dispatch) ── */
void processCanRxMessage(const IfxCan_Message *msg, const uint32 *data);

/* ── 수신 핸들러 (담당자 구현) ── */
void handler_ObstacleDetection_Door (void);
void handler_TheftDetection         (void);
void handler_DoorLockControl        (LockCmd_t                   cmd);
void handler_DoorOpenControl        (OpenClose_t                 cmd);
void handler_SafeExitAssist         (void);
void handler_SmtkProximity_Door     (const SmtkProximityEvent_t *msg);
void handler_WindowControl          (OpenClose_t                 cmd);
void handler_IgnitionStateBcast_Door(Ignition_t                  state);
void handler_UserSettingBcast_Door  (uint8                       userId,
                                     const UserSetting_t        *setting);

#endif /* MCMCAN_H_ */
