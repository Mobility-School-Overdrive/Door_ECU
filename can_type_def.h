/**
 * @file    can_type_def.h
 * @brief   스마트 도어 시스템 CAN 프로토콜 자료형 정의
 *
 * ECU 구성 : Main ECU / Door ECU / Control ECU / SMK ECU / SENS ECU
 */

#ifndef CAN_TYPE_DEF_H
#define CAN_TYPE_DEF_H

#include "Ifx_Types.h"

/* ============================================================
   CAN ID 정의
   낮은 ID = 높은 우선순위 (CAN 버스 중재)
   ============================================================ */

/* ── ① 비상 ── */
#define CAN_ID_FIRE_DETECTION       0x100U  /* SENS  → Main  | DLC=0 | 화재 발생        */
#define CAN_ID_FLOOD_DETECTION      0x101U  /* SENS  → Main  | DLC=0 | 침수 발생        */
#define CAN_ID_ROLLOVER_DETECTION   0x102U  /* SENS  → Main  | DLC=0 | 전복 발생        */
#define CAN_ID_OBSTACLE_DETECTION   0x103U  /* Main  → ALL   | DLC=0 | 전·후방 장애물   */
#define CAN_ID_THEFT_DETECTION      0x104U  /* Main  → Door  | DLC=0 | 도난 감지        */
#define CAN_ID_PINCH_DETECTION      0x105U  /* Door  → ALL   | DLC=1 | 문·창문 끼임     */

/* ── ② 주기 상태 ── */
#define CAN_ID_VEHICLE_STATUS       0x110U  /* Control→ Main | DLC=3 | 10 ms 주기       */
#define CAN_ID_DOOR_STATUS          0x115U  /* Door   → Main | DLC=3 | 10 ms 주기       */
#define CAN_ID_SENSOR_STATUS        0x120U  /* SENS  → Main  | DLC=2 | 10 ms 주기       */

/* ── ③ 안전 커맨드 ── */
#define CAN_ID_VEHICLE_CONTROL      0x140U  /* Main  → ALL   | DLC=6 | 차량 제어        */
#define CAN_ID_DOOR_LOCK_CONTROL    0x141U  /* Main  → Door  | DLC=1 | 문 잠금·해제     */
#define CAN_ID_DOOR_OPEN_CONTROL    0x142U  /* Main  → Door  | DLC=1 | 문 열기·닫기     */
#define CAN_ID_SAFE_EXIT_ASSIST     0x143U  /* Main  → Door  | DLC=0 | 안전 하차 보조   */
#define CAN_ID_BRAKE_STATUS         0x144U  /* Control→ Main | DLC=1 | 브레이크 상태    */
#define CAN_ID_EPB_BUTTON_EVENT     0x145U  /* Control→ Main | DLC=0 | EPB 버튼 누름    */
#define CAN_ID_GEAR_INPUT           0x146U  /* Control→ Main | DLC=1 | 기어 조작        */

/* ── ④ 편의 커맨드·이벤트 ── */
#define CAN_ID_SMTK_PROXIMITY       0x200U  /* Main  → ALL   | DLC=8 | 스마트키 접근·이탈 */
#define CAN_ID_WINDOW_CONTROL       0x202U  /* Main  → Door  | DLC=1 | 창문 열기·닫기   */
#define CAN_ID_SEAT_POSITION_CTRL   0x203U  /* Main  → Ctrl  | DLC=1 | 시트 위치 조정   */
#define CAN_ID_IGNITION_STATE_BCAST 0x205U  /* Main  → ALL   | DLC=1 | 시동 상태 전파   */
#define CAN_ID_SMTK_BUTTON_EVENT    0x210U  /* Door  → Main  | DLC=2 | 스마트키 버튼    */
#define CAN_ID_SMTK_RSSI_EVENT      0x211U  /* Door  → Main  | DLC=2 | 스마트키 RSSI    */
#define CAN_ID_HANDLE_PRESSURE      0x220U  /* Door  → Main  | DLC=0 | 손잡이 압력 상승 */
#define CAN_ID_DOOR_BUTTON_EVENT    0x221U  /* Door  → Main  | DLC=1 | 문 버튼 입력     */
#define CAN_ID_WINDOW_BUTTON_EVENT  0x222U  /* Door  → Main  | DLC=1 | 창문 버튼 입력   */

/* ── ⑤ 저우선 ── */
#define CAN_ID_USER_SETTING_VALUE   0x300U  /* Door  → Ctrl  | DLC=1 | 사용자 설정 값   */
#define CAN_ID_USER_SETTING_BCAST   0x301U  /* Main  → ALL   | DLC=5 | 사용자 설정 전파 */
#define CAN_ID_USER_SETTING_SAVE    0x302U  /* Control→ Main | DLC=5 | 사용자 설정 저장 */

/* 오류·범위 초과 sentinel 값 */
#define CAN_SPEED_ERROR             0xFFU   /* VEHICLE_SPEED: 통신 오류       */
#define CAN_DIST_OUT_OF_RANGE       0xFFU   /* OBSTACLE_DIST / SIDE_DIST: 범위 초과 */
#define CAN_GEAR_NO_CHANGE          0xFFU   /* GEAR_CMD: 기어 변경 없음 (현재 유지) */

#define USER_COUNT                  3U


/* ============================================================
   공용 열거형 (여러 메시지에서 재사용)
   ============================================================ */

/** 문·창문·미러 열림/닫힘 방향 */
typedef enum {
    OPEN_CLOSE_CLOSE = 0x00,   /* 닫기 / 접힘 */
    OPEN_CLOSE_OPEN  = 0x01    /* 열기 / 전개 */
} OpenClose_t;

/** 잠금 명령 */
typedef enum {
    LOCK_CMD_LOCK   = 0x00,    /* 잠금        */
    LOCK_CMD_UNLOCK = 0x01     /* 잠금 해제   */
} LockCmd_t;

/** ON / OFF (LED, 스피커) */
typedef enum {
    ON_OFF_OFF = 0x00,         /* 끄기        */
    ON_OFF_ON  = 0x01          /* 켜기        */
} OnOff_t;

/** 시동 제어·상태 */
typedef enum {
    IGNITION_OFF = 0x00,       /* 시동 끄기 / 꺼진 상태 */
    IGNITION_ON  = 0x01        /* 시동 켜기 / 켜진 상태 */
} Ignition_t;

/** 기어 위치 (커맨드·상태 공용) */
typedef enum {
    GEAR_P = 0x00,             /* P 주차 */
    GEAR_R = 0x01,             /* R 후진 */
    GEAR_N = 0x02,             /* N 중립 */
    GEAR_D = 0x03              /* D 전진 */
} Gear_t;

/** 전후진 명령 */
typedef enum {
    DRIVE_CMD_STOP    = 0x00,  /* 정지   */
    DRIVE_CMD_FORWARD = 0x01,  /* 전진   */
    DRIVE_CMD_REVERSE = 0x02   /* 후진   */
} DriveCmd_t;

/** EPB 명령 */
typedef enum {
    EPB_CMD_RELEASE   = 0x00,  /* EPB 해제       */
    EPB_CMD_ENGAGE    = 0x01,  /* EPB 체결       */
    EPB_CMD_EMERGENCY = 0x02   /* 긴급 제동 (즉시 체결) */
} EpbCmd_t;

/** 사용자 ID */
typedef enum {
    USER_ID_1 = 0x00,
    USER_ID_2 = 0x01,
    USER_ID_3 = 0x02
} UserId_t;

/** LED 색상 */
typedef enum {
    LED_COLOR_RED   = 0x00,    /* 빨강 */
    LED_COLOR_GREEN = 0x01,    /* 초록 */
    LED_COLOR_BLUE  = 0x02     /* 파랑 */
} LedColor_t;

/** 스마트키 버튼 종류 */
typedef enum {
    BTN_TYPE_LOCK          = 0x00, /* 문 잠금 버튼       */
    BTN_TYPE_UNLOCK        = 0x01, /* 문 잠금 해제 버튼  */
    BTN_TYPE_FORWARD       = 0x02, /* 전진 버튼          */
    BTN_TYPE_REVERSE       = 0x03, /* 후진 버튼          */
    BTN_TYPE_WINDOW_CLOSE  = 0x04, /* 창문 닫기 버튼     */
    BTN_TYPE_WINDOW_OPEN   = 0x05  /* 창문 열기 버튼     */
} BtnType_t;

/** Rssi */
typedef enum {
    RSSI_TYPE_FAR  = 0x00,   /* 스마트키 이탈 */
    RSSI_TYPE_NEAR = 0x01    /* 스마트키 접근 */
} RssiType_t;

/** 문/창문 버튼 상태 (누름 방향) */
typedef enum {
    BTN_STATE_CLOSE = 0x00,    /* 닫음(내림) 버튼 누름 */
    BTN_STATE_OPEN  = 0x01     /* 열림(올림) 버튼 누름 */
} BtnState_t;

/** 끼임 감지 대상 */
typedef enum {
    PINCH_TYPE_DOOR   = 0x00,  /* 문 끼임    */
    PINCH_TYPE_WINDOW = 0x01   /* 창문 끼임  */
} PinchType_t;

/** 브레이크 상태 */
typedef enum {
    BRAKE_STATE_RELEASED = 0x00, /* 브레이크 미입력  */
    BRAKE_STATE_PRESSED  = 0x01  /* 브레이크 입력 중 */
} BrakeState_t;

/** 운전석 착석 상태 */
typedef enum {
    SEAT_STATE_EMPTY    = 0x00,  /* 착석 없음  */
    SEAT_STATE_OCCUPIED = 0x01   /* 착석 감지  */
} SeatState_t;


/* ============================================================
   비트필드 (bitfield union)
   raw: CAN 수신 바이트 그대로, bits: 개별 필드 접근
   ============================================================ */

/**
 * Vehicle Control (0x140) B0 — 차량 제어 비트 플래그
 * bit0: 시동 제어 포함 여부
 * bit1: 전후진 명령 포함 여부
 * bit2: 기어 변경 포함 여부
 * bit3: EPB 동작 포함 여부
 * bit4~7: 예비
 */
typedef union {
    uint8 raw;
    struct {
        uint8 ignition : 1;   /* bit0: 시동 제어   (0=미포함, 1=포함) */
        uint8 drive    : 1;   /* bit1: 전후진 제어 (0=미포함, 1=포함) */
        uint8 gear     : 1;   /* bit2: 기어 제어   (0=미포함, 1=포함) */
        uint8 epb      : 1;   /* bit3: EPB 동작    (0=미포함, 1=포함) */
        uint8 reserved : 4;   /* bit4~7: 예비      */
    } bits;
} VehicleCtrlFlags_t;

/**
 * Smartkey Proximity Event (0x200) B0 — 스마트키 접근·이탈 제어 플래그
 * 각 비트가 1이면 해당 바이트(B2~B7)의 값을 적용
 * bit0: 문 잠금/해제 (→ B2 LOCK_VAL)
 * bit1: 문 열림/닫힘 (→ B3 DOOR_VAL)
 * bit2: 미러 열림/닫힘 (→ B4 MIRROR_VAL)
 * bit3: 미러 LED (→ B5 MIRROR_LED)
 * bit4: 스피커 (→ B6 SPEAKER)
 * bit5: 창문 (→ B7 WINDOW_VAL)
 * bit6~7: 예비
 */
typedef union {
    uint8 raw;
    struct {
        uint8 lock       : 1; /* bit0: 문 잠금·해제  */
        uint8 door       : 1; /* bit1: 문 열림·닫힘  */
        uint8 mirror     : 1; /* bit2: 미러 열림·닫힘 */
        uint8 mirror_led : 1; /* bit3: 미러 LED      */
        uint8 speaker    : 1; /* bit4: 스피커        */
        uint8 window     : 1; /* bit5: 창문          */
        uint8 reserved   : 2; /* bit6~7: 예비        */
    } bits;
} SmtkProxFlags_t;


/* ============================================================
   메시지 구조체 (packed)
   CAN 수신 버퍼와 memcpy 또는 포인터 캐스팅으로 사용
   ============================================================ */

/* ── MAIN → Door ── */

/** 0x141  Door Lock Control  DLC=1 */
typedef struct {
    LockCmd_t lock_cmd;         /* B0: 잠금 명령 */
} __attribute__((packed)) DoorLockControl_t;

/** 0x142  Door Open Control  DLC=1 */
typedef struct {
    OpenClose_t door_cmd;       /* B0: 문 열림 명령 */
} __attribute__((packed)) DoorOpenControl_t;

/** 0x202  Window Control  DLC=1 */
typedef struct {
    OpenClose_t window_cmd;     /* B0: 창문 명령 */
} __attribute__((packed)) WindowControl_t;

/** 0x105  Pinch Detection  DLC=1  (Door → ALL) */
typedef struct {
    PinchType_t pinch_type;     /* B0: 끼임 종류 */
} __attribute__((packed)) PinchDetection_t;

/* ── MAIN → Control ── */

/** 0x203  Seat Position Control  DLC=1 */
typedef struct {
    UserId_t user_id;           /* B0: 사용자 ID (저장된 시트 위치 조회 키) */
} __attribute__((packed)) SeatPositionControl_t;

/* ── MAIN → ALL ── */

/** 0x140  Vehicle Control  DLC=6 */
typedef struct {
    VehicleCtrlFlags_t ctrl_flags;  /* B0: 제어 비트 플래그 */
    Ignition_t         ignition;    /* B1: 시동             */
    DriveCmd_t         drive_cmd;   /* B2: 전후진 명령      */
    Gear_t             gear_cmd;    /* B3: 기어 명령        */
    EpbCmd_t           epb_cmd;     /* B4: 주차 브레이크    */
    uint8            reserved;    /* B5: 예비 (0x00 고정) */
} __attribute__((packed)) VehicleControl_t;

/** 0x200  Smartkey Proximity Event  DLC=8 */
typedef struct {
    SmtkProxFlags_t ctrl_flags;     /* B0: 제어 비트 플래그 (어떤 필드를 적용할지) */
    UserId_t        user_id;        /* B1: 사용자 ID       */
    LockCmd_t       lock_val;       /* B2: 잠금 값         */
    OpenClose_t     door_val;       /* B3: 문 열림 값      */
    OpenClose_t     mirror_val;     /* B4: 미러 열림 값    */
    OnOff_t         mirror_led;     /* B5: 미러 LED        */
    OnOff_t         speaker;        /* B6: 스피커          */
    OpenClose_t     window_val;     /* B7: 창문 값         */
} __attribute__((packed)) SmtkProximityEvent_t;

/** 0x205  Ignition State Broadcast  DLC=1 */
typedef struct {
    Ignition_t ignition_state;      /* B0: 시동 상태 */
} __attribute__((packed)) IgnitionStateBcast_t;

/** 0x301  User Setting Broadcast  DLC=5  (Main → ALL) */
/** 0x302  User Setting Save       DLC=5  (Control → Main) — 동일 구조 */
typedef struct {
    uint8    door_angle;      /* B0: 문 최대 개방 각도   (0~180°) */
    uint8    mirror_angle;    /* B1: 사이드미러 목표 각도 (0~180°) */
    LedColor_t led_color;       /* B2: LED 색상             */
    uint8    seat_pos;        /* B3: 시트 위치            (0=최전방 ~ 100=최후방 %) */
} __attribute__((packed)) UserSetting_t;

/* ── SMK ECU → Door ── */

/** 0x210  Smartkey Button Event  DLC=1 */
typedef struct {
    UserId_t  user_id;   /* B0: 사용자 ID */
    BtnType_t btn_type;  /* B1: 버튼 종류 */
} __attribute__((packed)) SmtkButtonEvent_t;

typedef struct {
    UserId_t   user_id;    /* B0: 사용자 ID */
    RssiType_t rssi_type;  /* B1: RSSI 상태 */
} __attribute__((packed)) SmtkRssiEvent_t;

/* ── Door ECU → Main ── */

/** 0x221  Door Button Event  DLC=1 */
typedef struct {
    BtnState_t btn_state;       /* B0: 버튼 상태 (0=닫음, 1=열림) */
} __attribute__((packed)) DoorButtonEvent_t;

/** 0x222  Window Button Event  DLC=1 */
typedef struct {
    BtnState_t btn_state;       /* B0: 버튼 상태 (0=닫음, 1=열림) */
} __attribute__((packed)) WindowButtonEvent_t;

/** 0x300  User Setting Value  DLC=1  (Door → Control) */
typedef struct {
    uint8 door_angle;         /* B0: 문 최대 개방 각도 (0~180°) */
} __attribute__((packed)) UserSettingValue_t;

typedef struct {
    OpenClose_t door_open_status;     /* B0: 문 열림 상태 */
    LockCmd_t   door_lock_status;     /* B1: 문 잠김 상태 */
    OpenClose_t window_open_status;   /* B2: 창문 열림 상태 */
} __attribute__((packed)) DoorStatus_t;

/* ── Control ECU → Main ── */

/** 0x110  Vehicle Status  DLC=3  (10 ms 주기) */
typedef struct {
    uint8 accel_pressure;     /* B0: 엑셀 페달 압력    (0=미입력 ~ 100=최대 %) */
    uint8 vehicle_speed;      /* B1: 현재 차량 속도    (km/h, 0xFF=통신 오류)  */
    uint8 obstacle_dist;      /* B2: 전방 장애물 거리  (cm,   0xFF=범위 초과)  */
} __attribute__((packed)) VehicleStatus_t;

/** 0x144  Brake Status  DLC=1 */
typedef struct {
    BrakeState_t brake_state;   /* B0: 브레이크 상태 */
} __attribute__((packed)) BrakeStatus_t;

/** 0x146  Gear Input  DLC=1 */
typedef struct {
    Gear_t gear_state;          /* B0: 기어 상태 */
} __attribute__((packed)) GearInput_t;

/** 0x231  Seat Occupancy Status  DLC=1 */
typedef struct {
    SeatState_t seat_state;     /* B0: 착석 상태 */
} __attribute__((packed)) SeatOccupancyStatus_t;

/* ── SENS ECU → Main ── */

/** 0x120  Sensor Status  DLC=2  (10 ms 주기) */
typedef struct {
    uint8 indoor_temp;        /* B0: 실내 온도          (0~100 °C)             */
    uint8 side_dist;          /* B1: 후측방 장애물 거리 (cm, 0xFF=범위 초과)   */
} __attribute__((packed)) SensorStatus_t;


/* ============================================================
   CAN 프레임 수신용 union
   raw[]에 memcpy 후 fields로 접근
   ============================================================ */

#define CAN_FRAME_UNION(TypeName, StructType, DLC) \
    typedef union {                                 \
        uint8      raw[(DLC)];                    \
        StructType   fields;                        \
    } TypeName

CAN_FRAME_UNION(VehicleControl_Frame_t,     VehicleControl_t,       6);
CAN_FRAME_UNION(DoorLockControl_Frame_t,    DoorLockControl_t,      1);
CAN_FRAME_UNION(DoorOpenControl_Frame_t,    DoorOpenControl_t,      1);
CAN_FRAME_UNION(DoorStatus_Frame_t,         DoorStatus_t,           3);
CAN_FRAME_UNION(WindowControl_Frame_t,      WindowControl_t,        1);
CAN_FRAME_UNION(PinchDetection_Frame_t,     PinchDetection_t,       1);
CAN_FRAME_UNION(SeatPositionCtrl_Frame_t,   SeatPositionControl_t,  1);
CAN_FRAME_UNION(SmtkProximity_Frame_t,      SmtkProximityEvent_t,   8);
CAN_FRAME_UNION(IgnitionStateBcast_Frame_t, IgnitionStateBcast_t,   1);
CAN_FRAME_UNION(UserSetting_Frame_t,        UserSetting_t,          5);
CAN_FRAME_UNION(SmtkButton_Frame_t,         SmtkButtonEvent_t,      2);
CAN_FRAME_UNION(SmtkRssi_Frame_t,           SmtkRssiEvent_t,        2);
CAN_FRAME_UNION(DoorButton_Frame_t,         DoorButtonEvent_t,      1);
CAN_FRAME_UNION(WindowButton_Frame_t,       WindowButtonEvent_t,    1);
CAN_FRAME_UNION(UserSettingValue_Frame_t,   UserSettingValue_t,     1);
CAN_FRAME_UNION(VehicleStatus_Frame_t,      VehicleStatus_t,        3);
CAN_FRAME_UNION(BrakeStatus_Frame_t,        BrakeStatus_t,          1);
CAN_FRAME_UNION(GearInput_Frame_t,          GearInput_t,            1);
CAN_FRAME_UNION(SeatOccupancy_Frame_t,      SeatOccupancyStatus_t,  1);
CAN_FRAME_UNION(SensorStatus_Frame_t,       SensorStatus_t,         2);

#endif /* CAN_TYPE_DEF_H */
