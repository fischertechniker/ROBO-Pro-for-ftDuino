#ifndef _FX1_H_
#define _FX1_H_

#include "common.h"

#define TX_BAUD      38400
#define BUF_MAXLEN   95  // one data packet: max. 95 byte

// Selection of used constants from ROBO_TX_FW.h

#define N_CNT                   4           // Number of counters
#define N_PWM_CHAN              8           // Number of PWM channels
#define N_MOTOR                 4           // Number of motors
#define N_UNI                   8           // Number of universal inputs

#define R_MIN                   10          // 5kOhm range [Ohm] 
#define R_MAX                   4999        // [Ohm]
#define R_OVR                   5000        // [Ohm] overload

#define U_MIN                   0           // 10V range [mV]
#define U_MAX                   9999        // [mV]
#define U_OVR                   10000       // [mV] overload

#define ULTRASONIC_MIN          2           // Ultrasonic Sensor range [cm]
#define ULTRASONIC_MAX          1023        // [cm]
#define ULTRASONIC_OVR          1024        // [cm] overload
#define NO_ULTRASONIC           4096        // Not present

#define DUTY_MIN                0           // Motor outputs PWM values range
#define DUTY_MAX                512

#define DEV_NAME_LEN_MAX        16          // "ROBO TX-xxxxxxxx"
#define BT_ADDR_STR_LEN         17          // "xx:xx:xx:xx:xx:xx"

#define BT_CNT_MAX              8           // Number of Bluetooth Channels

// Identifiers of the Transfer Area

#define TA_LOCAL                0           // Transfer Area for local Controller
#define TA_COUNT                9           // Number of Transfer Areas in array = 9

#define N_EXT                   (TA_COUNT - 1)  // Number of extension Controllers = 8

#define STX                     2           // start transmission
#define U                       0x55
#define ETX                     3           // end of transmission

// Fish.X1 command codes

enum eX1CmdCode {
  CMD_001               = 1,
  CMD_101               = 101, // reply, 0x65
  CMD_002               = 2,
  CMD_102               = 102, // reply, 0x66
  CMD_005               = 5,
  CMD_105               = 105, // reply, 0x69
  CMD_006               = 6,
  CMD_106               = 106, // reply, 0x6A
  CMD_007               = 7,
  CMD_107               = 107, // reply, 0x6B
};

// device connect state

enum ext_dev_connect_state_e {
  EXT_DEV_OFFLINE = 0,
  EXT_DEV_ONLINE,
  EXT_DEV_INVALID
};

// bluetooth connection state

enum BtConnState
{
  BT_STATE_IDLE = 0,              // BT channel is disconnected
  BT_STATE_CONN_ONGOING,          // BT channel is being connected
  BT_STATE_CONNECTED,             // BT channel is connected
  BT_STATE_DISC_ONGOING           // BT channel is being disconnected
};

// Version structure definition, 4 bytes

typedef union {
  UINT32        abcd;
  struct {
    UCHAR8      a;
    UCHAR8      b;
    UCHAR8      c;
    UCHAR8      d;
  } part;
} FT_VER;

// fischertechnik versions of hardware and firmware components, 16 bytes

typedef struct {
  FT_VER          hardware;   // Version of hardware (hardware.part.a = 'A' or 'B' or 'C')
  FT_VER          firmware;   // Version of firmware ("V %d.%02d, DLL %d", firmware.part.c,
                              // firmware.part.d, firmware.part.b)
  FT_VER          ta;         // Version of transfer area ("V %d.%02d", ta.part.c, ta.part.d)
  UINT8           reserved[4];
} FT_VERSION;

// Structures for Transfer Area

// TA_OUTPUT: Output structure, 44 bytes, used by CMD002_Request

typedef struct {
  UINT16  cnt_reset_cmd_id[N_CNT];       // Counter reset requests (should be increased by 1 each time
                                         // counter reset is needed)
  UINT8   master[N_MOTOR];               // If not 0, synchronize this channel with the given channel
                                         // (1:channel 0, ...)
  INT16   duty[N_PWM_CHAN];              // Selected motor outputs PWM values
  UINT16  distance[N_MOTOR];             // Selected distance (counter value) at which motor shall stop
  UINT16  cnt_ext_motor_cmd_id[N_MOTOR]; // Should be increased by 1 each time settings for extended
                                         // motor control mode (duty and/or distance) are changed             
} TA_OUTPUT;

// TA_INPUT: Input structure, 48 bytes, used by CMD002_Reply

typedef struct {
  INT16   uni[N_UNI];                  // Current values of the universal inputs
  BOOL8   cnt_in[N_CNT];               // Current levels (0 or 1) on the counter inputs according
                                       // to their configuration (normal or inverse)
  INT16   counter[N_CNT];              // Current values of the counter inputs
  INT16   display_button_left;         // Number of milliseconds during which the left display button
                                       // is being kept pressed
  INT16   display_button_right;        // Number of milliseconds during which the right display button
                                       // is being kept pressed
  BOOL16  cnt_resetted[N_CNT];         // set to 1 when last requested counter reset was fulfilled
  BOOL16  motor_pos_reached[N_MOTOR];  // set to 1 if target position is reached
} TA_INPUT;

// TA_CONFIG: Config structure, 44 bytes, used by CMD005_Request

typedef struct
{
  BOOL8   motor[N_MOTOR]; // TRUE = corresponding outputs are used as a pair of motor outputs M1...M4,
                          // FALSE = corresponding outputs are used as a pair of separate digital
                          //         PWM outputs O1...O8
  UINT8   uni[N_UNI];     // type of universal inputs: 0x00: 10V analog, 0x01: 5kOhm analog, 
                          // 0x03: Ultrasonic, 0x80: 10V digital, 0x81: 5kOhm digital
  UINT8   cnt[N_CNT];     // 0 = normal counter mode (change 0 -> 1 is counted)
                          // 1 = inverse counter mode (change 1 -> 0 is counted)
  UINT8   reserved[32];
} TA_CONFIG;

// TA_INFO: Info structure, 64 bytes, used by CMD006_Reply

typedef struct {
  char        device_name[DEV_NAME_LEN_MAX + 1];  // Controller name
  char        bt_addr[BT_ADDR_STR_LEN + 1];       // Bluetooth address as a string
  UINT8       reserved;
  UINT32      ta_array_start_addr;
  UINT32      pgm_area_start_addr;
  UINT32      pgm_area_size;
  FT_VERSION  version;
} TA_INFO;

// TA_STATE: State structure, 100 bytes, used by CMD007_Reply

typedef struct {
  BOOL8   ext_dev_connect_state[N_EXT];  // See enum ext_dev_connect_state_e
  UINT8   btstatus[BT_CNT_MAX];          // Status of Bluetooth connections
  UINT8   reserved[8];
} TA_STATE;

// Structures of CMD Data Packets

//  CMD_002_Request structure 

typedef struct {
  UINT32      TAId;
  TA_OUTPUT   X1Data;
} CMD_002_Data;

//  CMD_102_Reply structure

typedef struct {
  UINT32    TAId;
  TA_INPUT  X1Data;
} CMD_102_Data;

//  CMD_005_Request structure

typedef struct {
  UINT32      TAId;
  TA_CONFIG   X1Data;
} CMD_005_Data;

// CMD_105_Reply structure

typedef struct {
  UINT32	TAId;
} CMD_105_Data;

//  CMD_006_Request structure

typedef struct {
  UINT32  TAId;
} CMD_006_Data;

//  CMD_106_Reply structure

typedef struct {
  UINT32    TAId;
  TA_INFO   X1Data;
} CMD_106_Data;

//  CMD_107_Reply structure

typedef struct {
  UINT32	  TAId;
  TA_STATE	X1Data;
} CMD_107_Data;

// Parser Fish.X1 State Definitions

enum eX1FSMState {
  UNKNOWN_STATE = 0,
  FSM_WAIT_STX,
  FSM_WAIT_55,
  FSM_WAIT_L_1,
  FSM_WAIT_L_2,
  FSM_WAIT_DATA,
  FSM_WAIT_CS_1,
  FSM_WAIT_CS_2,
  FSM_WAIT_ETX
};

// Parser Remote Shell State Definitions

enum eSHFSMState {
  S_STATE_UNKNOWN = 0,
  GET_0D_START,
  GET_67,
  GET_0D_END
};

#endif
