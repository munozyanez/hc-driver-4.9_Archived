/*EM_LICENSE*/
/* 
 * $Id: hico_api.h 1141 2009-03-16 14:51:31Z ny $
 * Author: Martin Nylund
 */
#if !defined(_DPM_H)
#define _DPM_H

#define NUMBER_OF_CAN_NODES 2 //for minipci

#if defined(__GNUC__)

#define PACKED __attribute__((packed))

 #ifdef __KERNEL__
  #include <linux/types.h>
  typedef uint8_t reg8_t;
  typedef uint16_t reg16_t;
  typedef uint32_t reg32_t;
 #else 
  #include <stdint.h>
  typedef volatile uint8_t reg8_t;
  typedef volatile uint16_t reg16_t;
  typedef volatile uint32_t reg32_t;
 #endif

#else /* __GNUC__ */

/* Not a gnu compiler - use pragma packing for this file at include time*/
#define PACKED

/* These are actually part of C99 standard int stdint.h ... */
typedef unsigned long uint32_t;
typedef unsigned short uint16_t;
typedef unsigned char uint8_t;

#endif

typedef struct can_msg BUF_UNIT;
#ifdef __KERNEL__
 #define BUF_PTR(buf,ptr) ( (buf)->base + ioread16(&(buf)->vars->ptr) )
#else
 #define BUF_PTR(buf,ptr) ( (buf)->base + (buf)->vars->ptr )
#endif
#define BUF_WPTR(buf) BUF_PTR(buf,wptr)
#define BUF_RPTR(buf) BUF_PTR(buf,rptr)
struct buffer_vars{
    reg16_t base;
    reg16_t wptr;
    reg16_t rptr;
    reg16_t size;
}PACKED;

struct buffer{
    BUF_UNIT *base;
    struct buffer_vars *vars;
};


struct can_status{
    reg16_t _reserved[11];

    /* node_type gives information of the physical CAN interface */
#define CAN_TYPE_EMPTY 0       /* Tranceiver not mounted on the PCB */
#define CAN_TYPE_HS 1          /* High-Speed tranceiver */
#define CAN_TYPE_FT 2          /* Fault Tolerant tranceiver */
#define CAN_TYPE_RES 3         /* reserved */
#define CAN_TYPE_UNKNOWN 0xff
    reg8_t can_type;

    /* Value of the CAN nodes io pin. On Fault Tolerant (FT) CAN boards, this
     * is the LineError signal (low active -> '1' means no error) */
    reg8_t iopin;

    /* Number of messages in the receive buffer (DPM buffer + the HiCO.mPCI
     * internal receive buffer */
    reg16_t msgs_in_sram;
    reg16_t srambuf_size;

    reg16_t received;
    reg16_t sent;
    reg16_t filtered;

    /* GSR register of the CAN controller */
    reg8_t can_mod;
    reg8_t can_gsr;
    reg8_t can_rxerr;
    reg8_t can_txerr;

    /* Current bitrate index and bitrate in kbps */
    reg16_t bitrate_i;
    reg16_t bitrate;

    /* Current mode of the can node (reset, baudscan, active, etc.). These
     * definitons are recognized by the CMD_SET_MODE command  */
#define CM_BAUDSCAN 1
#define CM_PASSIVE 2
#define CM_ACTIVE 3
#define CM_RESET 4
#define CM_DISABLED 0xdead
    reg16_t mode;

    /* Diverse flags set by the host for the board*/
#define CF_AUTOCLR_OVERRUN (1<<0)
#define CF_FILTERS_ACTIVE (1<<1)
    reg16_t flags2hico;

    /* Diverse flags set by the board for the host */
    reg16_t flags2host;
    
}PACKED;


/* This structure needs to be 32bit aligned */
struct board_status{
    reg16_t _reserved[9];

    /* Boot code version of the processor */
    reg16_t lpcbc_rev;

    /* Relevant only for HiCO.CAN-PCI-104 boards. This is the jumpered
     * position value (0-3) which identifies the board in the PCI104 stack. */
    reg8_t pci104_pos;

    /* This is an emtrion specific value which identifies on what kind of
     * board we are running on */
#define HW_HICOCAN_MPCI 0x10
#define HW_HICOCAN_MPCI_4C 0x11
#define HW_HICOCAN_PCI104 0x13
#define HW_HICOCAN_UNKNOWN 0xff
    reg8_t hw_id;

    reg16_t cmd_ack_cnt;
    
/* These are "defined exceptions", meaning that the cause of the exception is
 * excactly known. They can be raised if false data is given to the board */
#define BE_OK_STR "no error"
#define BE_OK 0  

#define BE_INV_FW_IMAGE_IN_DPM_STR "invalid fw image in DPM during FW update"
#define BE_INV_FW_IMAGE_IN_DPM 2 

#define BE_INV_FW2_IMAGE_STR "no valid firmware (fw2) image found"
#define BE_INV_FW2_IMAGE 3 
    

/* These are "undefined exceptions" which means that the cause of the
 * exception is not clear. It can be a firmware bug or caused by the a
 * 'misbehaving' host driver. These exceptions should, of course, never be
 * raise but in the real world...*/
    
#define BE_EXCPT_WATCHDOG_STR "watchdog exception"
#define BE_EXCPT_WATCHDOG 0x8001

#define BE_EXCPT_SOFTWARE_STR "undefined software exception"
#define BE_EXCPT_SOFTWARE 0x8002

#define BE_EXCPT_DATA_ABORT_STR "data abort exception"
#define BE_EXCPT_DATA_ABORT 0x8003

#define BE_EXCPT_UNDEF_INSTR_STR "undefined instruction exception"
#define BE_EXCPT_UNDEF_INSTR 0x8004

#define BE_EXCPT_INVALID_STR "invalid firmware exception"
#define BE_EXCPT_INVALID 0x80FF
    
    reg16_t error;

    /* FW release/version number. */
#define FW_DEBUG_VERSION 0xFFFF
    reg16_t fw_version;

    /* 0=day, 1=month, 2=year, 3=hour */
    reg8_t fw_date[4];

    /* Variable to read which firmware is running */
#define FW1_RUNNING 0xf1f1
#define FW2_RUNNING 0xf2f2
#define EXCPT_RUNNING 0xfefe
    reg16_t fw_running;

    /* By mistake this used to contain the device PCI id. It really doesn't
     * belong here, since it's already written in the PCI eeprom and can be
     * read from the PCI configuration. This variable is not used and contains
     * value 0xdead */
    reg16_t device_id;

}PACKED;


#ifndef _HICO_API_H
/* structure of a CAN messages. Due to the bit definitions it looks more
 * complicated than it actually is...*/
struct can_msg {
    /* Frame info */
    union{
	uint16_t fi;
	struct {
	    /* Data length 0 to 8 */
	    uint16_t dlc:4;

	    /* remote transmission request flag */
	    uint16_t rtr:1;

	    /* Frame format flag 0=normal, 1=extended */ 
	    uint16_t ff:1;

	    /* Data overrun status flag */
	    uint16_t dos:1;

	    /* io pin status or can error signal on fault tolerant can */
	    uint16_t iopin:1;

	    /* CAN node number which received the message */
	    uint16_t node:2;

	    uint16_t reserved:6;
	}PACKED;
    }PACKED;

    /* Timestamp in microseconds */
    uint32_t ts;

    /* CAN identifier */
    uint32_t id;

    /* CAN message data */
    uint8_t data[8];
}PACKED;
#endif

/* Size of the dual ported memory in bytes */
#define DPM_MSG_AREA_SIZE(dpm_size) (dpm_size - sizeof(struct dpm))

/* Size of data blocks when updating firmware via dpm */
#define FW_UPDATE_BLOCK_SIZE 0x1000


/* Macro to set a pointer to the control area of the dpm. */
#define SET_DPM_PTR(dpm_base,dpm_size) ((struct dpm *)((uint8_t *)dpm_base + dpm_size - sizeof(struct dpm)))


/* DPM control area. This structure is overlayed at the end of DPM. The area
 * between DPM base address and start of this structure is for the message
 * queues */
struct dpm {
    /* Tx and Rx buffers for all of the nodes */
    struct buffer_vars tx_buffers[NUMBER_OF_CAN_NODES]; 
    struct buffer_vars rx_buffers[NUMBER_OF_CAN_NODES]; 
    struct can_status can_status[NUMBER_OF_CAN_NODES];
    

    struct board_status board_status;
    
    reg32_t args[2];

    /* Bits in this register correspond the INT_* Definitions */
    reg16_t int_enable;

    /* int_count can be used on PCI bus to check if the board really sent an
     * interrupt or if it was some other device sharing the same irq */
    reg16_t int_count;
    reg16_t mb_hico2host;
    reg16_t mb_host2hico;
}PACKED;


struct dpm_2ch {
    /* Tx and Rx buffers for all of the nodes */
    struct buffer_vars tx_buffers[2]; 
    struct buffer_vars rx_buffers[2]; 
    struct can_status can_status[2];
    

    struct board_status board_status;
    
    reg32_t args[2];

    /* Bits in this register correspond the INT_* Definitions */
    reg16_t int_enable;

    /* int_count can be used on PCI bus to check if the board really sent an
     * interrupt or if it was some other device sharing the same irq */
    reg16_t int_count;
    reg16_t mb_hico2host;
    reg16_t mb_host2hico;
}PACKED;

struct dpm_4ch {
    /* Tx and Rx buffers for all of the nodes */
    struct buffer_vars tx_buffers[4]; 
    struct buffer_vars rx_buffers[4]; 
    struct can_status can_status[4];
    

    struct board_status board_status;
    
    reg32_t args[2];

    /* Bits in this register correspond the INT_* Definitions */
    reg16_t int_enable;

    /* int_count can be used on PCI bus to check if the board really sent an
     * interrupt or if it was some other device sharing the same irq */
    reg16_t int_count;
    reg16_t mb_hico2host;
    reg16_t mb_host2hico;
}PACKED;


/****************************************/
/* Commands from host to the HiCO Board */
/****************************************/
#define CMD_NONE        0

#define CMD_SET_BITRATE 2   // BITRATE_* as first argument

#define CMD_SET_MODE    3   // CM_* as first argument

#define CMD_CLR_OVERRUN 4   // no arguments

#define CMD_CLR_FILTERS 5   // no arguments

#define CMD_SET_RANGE_FILTER 6 // arg1 is lower id, arg2 upper id

#define CMD_SET_AMASK_FILTER 7 // arg1 is mask, arg2 is code

#define CMD_RESET_TIMESTAMP 8  // no arguments

#define CMD_SET_BTR 9  // arg1 is btr settings 

#define CMD_SET_SJW_INCREMENT 10  // arg1 is value to add to the default SJW value

#define CMD_GET_ERR_STAT 11  // arg1 is the index to the stats table

#define CMD_CLR_ERR_STAT 12  // Clear all error statistic counters

#define CMD_SET_CAN_TYPE 13  // Set can type to CAN_TYPE_HS or CAN_TYPE_FT

/* In case of an exception, a debug string is output into the DPM starting
 * from offset 0. This command has only meaning if and exception has occured
 * (i.e. fw_running==EXCPT_RUNNING) */
#define CMD_PRINT_EXCEPTION 0x81

#define CMD_SERIAL_DBG 0x82

/* put a pattern to the leds which signal that production was succesfull. The
 * board has to be rese in order to get out of this state */
#define CMD_PRODUCTION_OK 0x83

/*************************/
/* Command return values */
/*************************/
#define E_OK 1
#define E_INVARG 2
#define E_INVCMD 3
#define E_IGNORED 4

/******************************************************/
/* Bitrates recognized by the CMD_SET_BITRATE command */
/******************************************************/
#define BITRATE_10k	 0 
#define BITRATE_20k	 1 
#define BITRATE_50k	 2 
#define BITRATE_100k	 3 
#define BITRATE_125k	 4 
#define BITRATE_250k	 5 
#define BITRATE_500k	 6 
#define BITRATE_800k	 7 
#define BITRATE_1000k	 8 

/*************************************************/
/* Interrupt reasons from HiCO board to the host */
/*************************************************/
#define INT_CAN1_RX	(1<<0)
#define INT_CAN1_TX	(1<<1)
/* 1<<2 reserved */
#define INT_CAN2_RX	(1<<4)
#define INT_CAN2_TX	(1<<5)
/* 1<<6 reserved */
#define INT_CMD_ACK     (1<<8)
#define INT_ERROR       (1<<9)
#define INT_EXCEPION    (1<<10)
#define INT_CAN3_RX	(1<<11)
#define INT_CAN3_TX	(1<<12)
#define INT_CAN4_RX	(1<<13)
#define INT_CAN4_TX	(1<<14)
/* 1<<15 reserved */

/* dpm.c */
int buf_real_size(struct buffer *buf);
int buf_is_full(struct buffer *buf);
int buf_is_empty(struct buffer *buf);
int buf_message_cnt(struct buffer *buf);
int buf_increment_wptr(struct buffer *buf);
int buf_increment_rptr(struct buffer *buf);

#define buf_not_empty(B) !buf_is_empty(B)
#define buf_not_full(B) !buf_is_full(B)

#if !defined(FW1) && !defined(FW2) && !defined(__KERNEL__)  && !defined(_WINDOWS) && !defined(__ALLOW_DPM_H)
#error -- do not include dpm.h to your application - use hico_api.h instead!
#endif

#endif
