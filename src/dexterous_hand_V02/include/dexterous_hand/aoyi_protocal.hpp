#ifndef DEXTEROUS_AOYI_PROTOCAL_HPP_
#define DEXTEROUS_AOYI_PROTOCAL_HPP_
// #include "dexterous_hand/OHandSerialAPI.h"
// #include "dexterous_hand/aoyi_hand.hpp"

#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <cstdint>

#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef NULL
#define NULL ((void *)0)
#endif


/* Exported constants --------------------------------------------------------*/

#define MAX_THUMB_ROOT_POS 3
#define MAX_MOTOR_CNT 6
#define MAX_FORCE_ENTRIES 12 * 5 /* Max force entries for one finger */


typedef enum
{
  HAND_PROTOCOL_UART,
  HAND_PROTOCOL_I2C
} HAND_PROTOCOL;


#define PROTOCOL_VERSION_MAJOR 3


/*
 * OHand error codes
 */

/*
 * Error codes for remote peer, protocol part
 */
#define ERR_PROTOCOL_WRONG_LRC                          0x01


/*
 * Error codes for remote peer, command part
 */
#define ERR_COMMAND_INVALID                             0x11
#define ERR_COMMAND_INVALID_BYTE_COUNT                  0x12
#define ERR_COMMAND_INVALID_DATA                        0x13


/*
 * Error codes for remote peer, status part
 */
#define ERR_STATUS_INIT                                 0x21
#define ERR_STATUS_CALI                                 0x22
#define ERR_STATUS_STUCK                                0x23


/*
 * Error codes for remote peer, operation result part
 */
#define ERR_OP_FAILED                                   0x31
#define ERR_SAVE_FAILED                                 0x32


/*
 * API return values
 */
#define HAND_RESP_HAND_ERROR                            0xFF           /* device error, error call back will be called with OHand error codes listed above */

#define HAND_RESP_SUCCESS                               0x00
#define HAND_RESP_TIMER_FUNC_NOT_SET                    0x01           /* local error, timer function not set, call HAND_SetTimerFunction(...) first */
#define HAND_RESP_INVALID_CONTEXT                       0x02           /* local error, invalid context, NULL or send data function not set */
#define HAND_RESP_TIMEOUT                               0x03           /* local error, time out when waiting node response */
#define HAND_RESP_INVALID_OUT_BUFFER_SIZE               0x04           /* local error, out buffer size not matched to returned data */
#define HAND_RESP_UNMATCHED_ADDR                        0x05           /* local error, unmatched node id between returned and waiting */
#define HAND_RESP_UNMATCHED_CMD                         0x06           /* local error, unmatched command between returned and waiting */
#define HAND_RESP_DATA_SIZE_TOO_BIG                     0x07           /* local error, size of data to send exceeds the buffer size */
#define HAND_RESP_DATA_INVALID                          0x08           /* local error, data content invalid */


/* Sub-command for HAND_CMD_SET_CUSTOM */
#define SUB_CMD_SET_SPEED     (1 << 0)
#define SUB_CMD_SET_POS       (1 << 1)
#define SUB_CMD_SET_ANGLE     (1 << 2)
#define SUB_CMD_GET_POS       (1 << 3)
#define SUB_CMD_GET_ANGLE     (1 << 4)
#define SUB_CMD_GET_CURRENT   (1 << 5)
#define SUB_CMD_GET_FORCE     (1 << 6)
#define SUB_CMD_GET_STATUS    (1 << 7)


//----------------------------------------
// Private data & functions

#define MAX_PROTOCOL_DATA_SIZE 64

typedef enum
{
  WAIT_ON_HEADER_0,
  WAIT_ON_HEADER_1,
  WAIT_ON_ADDRESSED_NODE_ID,
  WAIT_ON_OWN_NODE_ID,
  WAIT_ON_COMMAND_ID,
  WAIT_ON_BYTECOUNT,
  WAIT_ON_DATA,
  WAIT_ON_LRC
} HAND_PROTOCOL_DECODER_STATE;

/*
 * OHand commands
 */

 /* Chief GET commands */
#define HAND_CMD_GET_PROTOCOL_VERSION       0x00     /* Get protocol version, Please don't modify! */
#define HAND_CMD_GET_FW_VERSION             0x01     /* Get firmware version */
#define HAND_CMD_GET_HW_VERSION             0x02     /* Get hardware version, [HW_TYPE, HW_VER, BOOT_VER_MAJOR, BOOT_VER_MINOR] */
#define HAND_CMD_GET_CALI_DATA              0x03     /* Get calibration data */
#define HAND_CMD_GET_FINGER_PID             0x04     /* Get PID of finger */
#define HAND_CMD_GET_FINGER_CURRENT_LIMIT   0x05     /* Get motor current limit of finger */
#define HAND_CMD_GET_FINGER_CURRENT         0x06     /* Get motor current of finger */
#define HAND_CMD_GET_FINGER_FORCE_TARGET    0x07     /* Get force limit of finger */
#define HAND_CMD_GET_FINGER_FORCE           0x08     /* Get force of finger */
#define HAND_CMD_GET_FINGER_POS_LIMIT       0x09     /* Get absolute position limit of finger */
#define HAND_CMD_GET_FINGER_POS_ABS         0x0A     /* Get current absolute position of finger */
#define HAND_CMD_GET_FINGER_POS             0x0B     /* Get current logical position of finger */
#define HAND_CMD_GET_FINGER_ANGLE           0x0C     /* Get first joint angle of finger */
#define HAND_CMD_GET_THUMB_ROOT_POS         0x0D     /* Get preset position of thumb root, [0, 1, 2, 255], 255 as invalid */
#define HAND_CMD_GET_FINGER_POS_ABS_ALL     0x0E     /* Get current absolute position of all fingers */
#define HAND_CMD_GET_FINGER_POS_ALL         0x0F     /* Get current logical position of all fingers */
#define HAND_CMD_GET_FINGER_ANGLE_ALL       0x10     /* Get first joint angle of all fingers */
#define HAND_CMD_GET_FINGER_STOP_PARAMS     0x11     /* Get finger finger stop parametres */
#define HAND_CMD_GET_FINGER_FORCE_PID       0x12     /* Get finger force PID */


/* Auxiliary GET commands */
#define HAND_CMD_GET_SELF_TEST_LEVEL        0x20    /* Get self-test level state */
#define HAND_CMD_GET_BEEP_SWITCH            0x21    /* Get beep switch state */
#define HAND_CMD_GET_BUTTON_PRESSED_CNT     0x22    /* Get button press count */
#define HAND_CMD_GET_UID                    0x23    /* Get 96 bits UID */
#define HAND_CMD_GET_BATTERY_VOLTAGE        0x24    /* Get battery voltage */
#define HAND_CMD_GET_USAGE_STAT             0x25    /* Get usage stat */

#define HAND_CMD_GET_SPEED_CTRL_PARAMS      0x3D    /* Get speed control parameters */
#define HAND_CMD_GET_MANUFACTURE_DATA       0x3E    /* Get manufacture data */
#define HAND_CMD_GET_VENDOR_ID              0x3F    /* Get vendor id */


/* Chief SET commands */
#define HAND_CMD_RESET                      0x40    /* Please don't modify */
#define HAND_CMD_POWER_OFF                  0x41    /* Power off */
#define HAND_CMD_SET_NODE_ID                0x42    /* Set node ID */
#define HAND_CMD_CALIBRATE                  0x43    /* Recalibrate hand */
#define HAND_CMD_SET_CALI_DATA              0x44    /* Set finger pos range & thumb pos set */
#define HAND_CMD_SET_FINGER_PID             0x45    /* Set PID of finger */
#define HAND_CMD_SET_FINGER_CURRENT_LIMIT   0x46    /* Set motor current limit of finger */
#define HAND_CMD_SET_FINGER_FORCE_TARGET    0x47    /* Set force limit of finger */
#define HAND_CMD_SET_FINGER_POS_LIMIT       0x48    /* Get current absolute position of finger */
#define HAND_CMD_FINGER_START               0x49    /* Start motor */
#define HAND_CMD_FINGER_STOP                0x4A    /* Stop motor */
#define HAND_CMD_SET_FINGER_POS_ABS         0x4B    /* Move finger to physical position, [0, 65535] */
#define HAND_CMD_SET_FINGER_POS             0x4C    /* Move finger to logical position, [0, 65535] */
#define HAND_CMD_SET_FINGER_ANGLE           0x4D    /* Set first joint angle of finger */
#define HAND_CMD_SET_THUMB_ROOT_POS         0x4E    /* Move thumb root to preset position, {0, 1, 2} */
#define HAND_CMD_SET_FINGER_POS_ABS_ALL     0x4F    /* Set current absolute position of all fingers */
#define HAND_CMD_SET_FINGER_POS_ALL         0x50    /* Set current logical position of all fingers */
#define HAND_CMD_SET_FINGER_ANGLE_ALL       0x51    /* Set first joint angle of all fingers */
#define HAND_CMD_SET_FINGER_STOP_PARAMS     0x52    /* Set finger finger stop parametres */
#define HAND_CMD_SET_FINGER_FORCE_PID       0x53    /* Set finger force PID */
#define HAND_CMD_RESET_FORCE                0x54    /* Reset force */

#define HAND_CMD_SET_CUSTOM                 0x5F    /* Custom set command */

/* Auxiliary SET commands */
#define HAND_CMD_SET_SELF_TEST_LEVEL        0x60    /* Set self-test level, level, 0: wait command, 1: semi self-test, 2: full self-test */
#define HAND_CMD_SET_BEEP_SWITCH            0x61    /* Set beep ON/OFF */
#define HAND_CMD_BEEP                       0x62    /* Beep for duration if beep switch is on */
#define HAND_CMD_SET_BUTTON_PRESSED_CNT     0x63    /* Set button press count, for ROH calibration only */
#define HAND_CMD_START_INIT                 0x64    /* Start init in case of SELF_TEST_LEVEL=0 */
#define HAND_CMD_SET_MANUFACTURE_DATA       0x65    /* Set manufacture data */
#define HAND_CMD_SET_SPEED_CTRL_PARAMS      0x66    /* Set speed control parameters */


#define CMD_ERROR_MASK (1 << 7) /* bit mask for command error */

/*
 * Byte name for data in _packet_data
 */
#define NODE_ID_BYTE_NUM 0
#define OWN_ID_BYTE_NUM 1
#define CMD_ID_BYTE_NUM 2
#define DATA_CNT_BYTE_NUM 3
#define DATA_START_BYTE_NUM 4

//  /*
//   * private_data should have same address with struct, i.e., shoule be first element in struct
//   */
// typedef struct
// {
//   const void* private_data; /* context private data pointer, can point to anything external,
//                              * e.g., char array of port name, or a struct contains some data,
//                              * resource should be always available, i.e., memory not recycled when calling API. */
//   uint8_t address_master;
//   HAND_PROTOCOL protocol;
//   HAND_PROTOCOL_DECODER_STATE initial_state;

//   uint16_t timeout; /* ms */
//   uint8_t is_whole_packet;
//   HAND_PROTOCOL_DECODER_STATE decode_state;
//   uint8_t packet_data[MAX_PROTOCOL_DATA_SIZE + 5]; /* node_id, own_id, command_id, byte_cnt, data[], lrc */
//   uint8_t send_buf[MAX_PROTOCOL_DATA_SIZE + 7]; /* extra 7 bytes for header0, header1, address, master address, cmd, nb_data, lrc */
//   uint8_t byte_count; /* data bytes left in packet */

//   void (*send_data_impl)(uint8_t addr, uint8_t* data, uint8_t size, void* ctx);
//   void (*recv_data_impl)(void* ctx);
// } ohand_context_t;

// void (*_delay_milli_seconds_impl)(uint32_t ms);
// uint32_t(*_get_milli_seconds_impl)(void);

// static uint8_t HAND_ProtocolLRC(uint8_t* lrcBytes, uint8_t lrcByteCount)
// {
//   uint8_t lrc = 0;
//   uint8_t i;

//   for (i = 0; i < lrcByteCount; i++)
//   {
//     lrc ^= lrcBytes[i];
//   }

//   return lrc;
// }

#endif // DEXTEROUS_AOYI_PROTOCAL_HPP_