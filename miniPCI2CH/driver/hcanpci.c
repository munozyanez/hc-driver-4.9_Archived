/* Linux kernel module for HiCO.CAN-MiniPCI CAN-bus cards. Only 2.6 kernels
 * are supported 
 *
 * Copyright (c) 2007: emtrion GmbH
 * 
 * $Id: hcanpci.c 1207 2009-05-11 13:36:51Z ny $
 * */
#include <linux/init.h>
#include <linux/sched.h>
#include <linux/module.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/string.h>
#include <linux/pci.h>
#include <linux/proc_fs.h>
#include <linux/delay.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/version.h>
#include <asm/uaccess.h>
#include <asm/io.h>

#include "version.h"
#include "_hico_api.h"
#include "dpm.h"
#include "dpm.c"

MODULE_LICENSE("Dual BSD/GPL");
MODULE_AUTHOR("Martin Nylund (emtrion GmbH)");
MODULE_DESCRIPTION("Driver for emtrion HiCO.CAN-MiniPCI CAN-bus cards");

/* PCI Subdevice ID for HiCO.CAN-MiniPCI and HiCO.CAN-PCI-104 */
#define HICOMPCI_PCI_SUBDEV_ID 0x3267
#define HICOPCI104_PCI_SUBDEV_ID 0x3325

#define REQUIRED_FW2_VERSION 1124

static unsigned int fw_update = 0;
module_param(fw_update, int, 0664);

static unsigned int irqtrace = 0;
module_param(irqtrace, int, 0664);

#define FIRST_MINOR 0
#define MINOR_COUNT 64
#define DRV_NAME "hcanpci"

static int major = 0;
module_param(major, int, S_IRUGO);

#define iosetbits16(mask,address) iowrite16(ioread16(address)|(mask),(address))
#define ioclrbits16(mask,address) iowrite16(ioread16(address)&~(mask),(address))

struct hcan_board;


struct hcan_node{
    struct cdev cdev;
    int cdev_added;
    struct hcan_board *board;
    int minor;

    /* CAN node number */
    int number;
    char proc_name[50];


    /* Interrupt bits (correspond the INT_CAN*_ definitions */
    uint16_t rx_int;
    uint16_t tx_int;
    uint16_t err_int;
    
    struct proc_dir_entry *proc_file;

    /* Tx and Rx buffers in DPM */
    struct buffer dpm_txbuf;
    struct buffer dpm_rxbuf;
    wait_queue_head_t ev_tx_ready;
    wait_queue_head_t ev_rx_ready;

    /* Pointers to CAN node status structures in DPM */
    struct can_status *can_status;
};

struct hcan_board{
    struct pci_dev *pdev;
    uint8_t *dpm_base;
    uint16_t *dpm_sem_base;
    uint8_t *cfg_base;
    struct dpm *dpm;
    struct hcan_node node[NUMBER_OF_CAN_NODES];
    char proc_name[50];
    int number;
    
    /* these are updated in get_fw1_version */
    int fw1_version;
    char fw1_date[4];
    
    /* Directory where the proc files are */
    struct proc_dir_entry *proc_dir;

    /* Proc file for board specific data */
    struct proc_dir_entry *proc_file;

    wait_queue_head_t ev_cmd_ack;

    /* Indicates tha CMD ack has been received */
    volatile int cmd_ack;
    volatile int last_ack_count;

    /* Semaphore used when sending commands to the board */
    struct semaphore sem;
//struct mutex sem;

    int cmd_timeout;
    int latte_timeout;
};

struct proc_dir_entry *hcan_proc_dir=NULL;

int board_count=0;



void reset_mode(struct hcan_board *board, int status)
{
    uint32_t val;
    
    /* CNTRL[30] is reset */
    
    val=readl(board->cfg_base+0x50);
    
    if(status){
	val|=(1<<30);
    } else {
	val&=~(1<<30);
    }

    writel(val,board->cfg_base+0x50);

}

void set_fw_update_enable_pin(struct hcan_board *board, int state)
{
    uint32_t val;

    /* GPIO control register is at 0x54 */
    
    val=readl(board->cfg_base+0x54);

    if(state){
	val|=(1<<2);
    } else {
	val&=~(1<<2);
    }

    writel(val,board->cfg_base+0x54);

}

int error_map[]={
    [E_OK] = 0,
    [E_INVARG] = -EINVAL,
    [E_INVCMD] = -EINVAL,
    [E_IGNORED] = -EBUSY,
};

int board_cmd(struct hcan_board *board, uint16_t msg_code, 
	uint32_t arg1, uint32_t arg2, uint32_t *retval)
{
    int ret=0;
    int saved;


    /* aquire board semaphore. Only one command allowed at a time */
    if(down_interruptible(&board->sem)){
	return -ERESTARTSYS;
    }
    
    iowrite32(arg1,&board->dpm->args[0]);
    iowrite32(arg2,&board->dpm->args[1]);

    /* Wait for an answer */
    board->cmd_ack=0;

    saved=ioread16(&board->dpm->board_status.cmd_ack_cnt);

    barrier();
    /* Put the message in the mailbox. This will generate interrupt on the
     * board */
    iowrite16(msg_code,&board->dpm->mb_host2hico);
    
    if(wait_event_timeout(board->ev_cmd_ack, board->cmd_ack, board->cmd_timeout)==0){
	printk(KERN_INFO "%s: No ack from board %s - timed out\n",
		__FUNCTION__,pci_name(board->pdev));
	emI(saved);
	emI(ioread16(&board->dpm->board_status.cmd_ack_cnt));
	ret= -EIO;
	goto out;
    }


    /* Get the return value from the error cell */
    ret=ioread16(&board->dpm->args[0]);
    ret|=ioread16( ((uint16_t *)&board->dpm->args[0])+1 )<<16;
    if(ret>sizeof(error_map)/4){
	printk(KERN_WARNING "%s: invalid return value %d for command %d\n",
		__FUNCTION__,ret,msg_code);
	ret=-EIO;
    } else {
	ret=error_map[ret];
    }

    /* FIXME: Give the FW some time to update DPM variables. We get an ACK sooner
     * than they are all updated in main() of FW. This should be fixed in the
     * firmware */
    msleep(1);
    if(retval!=NULL){
	*retval=ioread32(&board->dpm->args[1]);
    }

out:
    /* free board semaphore */
    up(&board->sem);
    return ret;

}

int node_cmd(struct hcan_node *node, uint16_t cmd, 
	uint32_t arg1, uint32_t arg2,uint32_t *retval)
{
    /* Check that the firmware is running */
    if(ioread16(&node->board->dpm->board_status.fw_running)!=FW2_RUNNING){
	printk(KERN_WARNING "%s: Firmware fw2 not running on board %s (fw_running=%04x)\n",
		__FUNCTION__,pci_name(node->board->pdev),
		ioread16(&node->board->dpm->board_status.fw_running));
	return -EIO;
    }
    /* put the CAN node number into the command */
    cmd= (cmd&0xff)|(node->number<<8);

    return board_cmd(node->board, cmd, arg1, arg2, retval);
}


//int hcan_board_proc(char *buf, char **start, off_t offset, int count,
//		      int *eof, void *data)
//data is now filp
static ssize_t hcan_board_read(struct file *filp,   /* see include/linux/fs.h   */
                           char *buf,        /* buffer to fill with data */
                           size_t length,       /* length of the buffer     */
                           loff_t * offset)
{
    int len = 0,i;
    
    //struct hcan_board *board = filp;
    struct hcan_board *board;//copied from newer 4CH driver
    board=PDE_DATA(file_inode(filp));//copied from newer 4CH driver
    struct board_status *bs;
    struct can_status *cs;
    char *str;
    uint8_t byte;
    uint16_t word;

    bs = &board->dpm->board_status;

    len+=sprintf(buf+len,"\nboard %s\n",pci_name(board->pdev));
    len+=sprintf(buf+len,"nodes: can%d, can%d\n",
	    board->node[0].minor, board->node[1].minor);

//    len+=sprintf(buf+len,"Linux driver: v%d (compiled on %s %s)\n",
//           HCANPCI_DRIVER_VERSION,__DATE__,__TIME__);

    word=ioread16(&bs->fw_running);
    switch(word){
	case FW1_RUNNING: str="bootloader (fw1)"; break;
	case FW2_RUNNING: str="firmware (fw2)"; break;
	case EXCPT_RUNNING: str="exception!"; break;
	default: str="invalid!"; break;
    }
    len+=sprintf(buf+len,"fw state: %04x - %s\n",
	    word,str);
    if(word==FW1_RUNNING){
    //*eof = 1; //used on previous code
	return len;
    }

    len+=sprintf(buf+len,"fw1 version: %d %s\n",board->fw1_version,
	    (board->fw1_version==FW_DEBUG_VERSION)?"- debug release":"");
    
    len+=sprintf(buf+len,"fw1 date: %02d %02d.%02d.%02d\n",
	    board->fw1_date[3],board->fw1_date[0],
	    board->fw1_date[1],board->fw1_date[2]);
    
    word=ioread16(&bs->fw_version);
    len+=sprintf(buf+len,"fw2 version: %d %s\n",word,
	    (word==FW_DEBUG_VERSION)?"- debug release":"");
    if(word!=FW_DEBUG_VERSION && word<REQUIRED_FW2_VERSION){
	len+=sprintf(buf+len,
		"WARNING: This version of Linux driver "
		"requires FW2 version 1120 or above!!\n\n");
    }
    
    len+=sprintf(buf+len,"fw2 date: %02d %02d.%02d.%02d\n",
	    ioread8(&bs->fw_date[3]),
	    ioread8(&bs->fw_date[0]),
	    ioread8(&bs->fw_date[1]),
	    ioread8(&bs->fw_date[2]));
    len+=sprintf(buf+len,"lpcbc rev: 0x%0x\n",ioread16(&bs->lpcbc_rev));

    /* PCI eeprom version */
    pci_read_config_byte(board->pdev,8,&byte);
    len+=sprintf(buf+len,"pci rev: 0x%x\n",byte);
    
    switch(ioread8(&bs->hw_id)){
	case HW_HICOCAN_MPCI:
	    str="HiCO.CAN-MiniPCI";
	    break;
	case HW_HICOCAN_PCI104:
	    str="HiCO.CAN-PCI-104";
	    break;
	case HW_HICOCAN_UNKNOWN:
	    str="unknown hardware";
	    break;
	default:
	    str="Invalid!";
	    break;
    }
    len+=sprintf(buf+len,"hardware id: 0x%x (%s)\n",ioread8(&bs->hw_id),str);

    for(i=0;i<NUMBER_OF_CAN_NODES;i++){
	cs=board->node[i].can_status;
	switch(ioread8(&cs->can_type)){
	    case CAN_TYPE_EMPTY: str="empty - not populated"; break;
	    case CAN_TYPE_HS: str="High Speed (HS)"; break;
	    case CAN_TYPE_FT: str="Fault Tolerant (FT)"; break;
	    default: str="Invalid/Unknown"; break;
	}

	len+=sprintf(buf+len,"can%d: %s - ",i,str);

	switch(ioread8(&cs->mode)){
	    case 1: str="baudscan"; break;
	    case 2: str="passive"; break;
	    case 3: str="active"; break;
	    case 4: str="reset"; break;
	    default: str="invalid"; break;
	}
	len+=sprintf(buf+len,"%s ",str);
	len+=sprintf(buf+len,"%dkbps ",((int)ioread16(&cs->bitrate)));

	byte=ioread8(&cs->can_gsr);
	len+=sprintf(buf+len,"%s%s%s\n",
		byte&((1<<6)|(1<<7))?"":"ok",
		byte&(1<<6)?"ErrPassive! ":"", 
		byte&(1<<7)?"BusOff! ":"");
    }

    word=ioread16(&bs->hw_id);
    if((word==HW_HICOCAN_PCI104)||
	    (word==HW_HICOCAN_UNKNOWN)){

	byte=ioread8(&bs->pci104_pos);
	len+=sprintf(buf+len,"pci104_pos: %d",byte);
	if(byte > 3){
	    len+=sprintf(buf+len," <- invalid!\n");
	} else {
	    len+=sprintf(buf+len,"\n");
	}
    }





    word=ioread16(&bs->error);
    switch(word){
	case BE_OK:                  str=BE_OK_STR; break;
	case BE_INV_FW_IMAGE_IN_DPM: str=BE_INV_FW_IMAGE_IN_DPM_STR; break;
	case BE_INV_FW2_IMAGE:       str=BE_INV_FW2_IMAGE_STR; break;
	case BE_EXCPT_SOFTWARE:      str=BE_EXCPT_SOFTWARE_STR; break;
	case BE_EXCPT_WATCHDOG:      str=BE_EXCPT_WATCHDOG_STR; break;
	case BE_EXCPT_UNDEF_INSTR:   str=BE_EXCPT_UNDEF_INSTR_STR; break;
	case BE_EXCPT_DATA_ABORT:    str=BE_EXCPT_DATA_ABORT_STR; break;
	case BE_EXCPT_INVALID:       str=BE_EXCPT_INVALID_STR; break;
	default: str="invalid error code!"; break;
    }
    len+=sprintf(buf+len,"error code: %04x - %s\n",
	    word,str);

    /* exception string would have to converted to bigendian */
#ifndef __BIG_ENDIAN
    if(ioread16(&bs->fw_running)==EXCPT_RUNNING){
	if(board_cmd(board, CMD_PRINT_EXCEPTION, 0, 0,NULL)){
	    len+=sprintf(buf+len,"failed to get exception string\n");
	} else {
	    char *str = kmalloc(SIZE_OF_DPM, GFP_KERNEL);
	    if(!str){
		len+=sprintf(buf+len,"failed to get memory for exception string!!\n");
	    } else {
		memcpy_fromio(str,board->dpm_base,SIZE_OF_DPM);
		len+=sprintf(buf+len,"-------------------------------------------------------------\n");
		len+=sprintf(buf+len,str);
		len+=sprintf(buf+len,"-------------------------------------------------------------\n");
		kfree(str);
	    }
	}
    }
#endif

    len+=sprintf(buf+len,"interrupt: %d\n",
	    board->pdev->irq);

    //*eof = 1; //used on previous code
    return len;
}

//int hcan_node_proc(char *buf, char **start, off_t offset, int count,
//		      int *eof, void *data)
//data is now filp
static ssize_t hcan_node_read(struct file *filp,   /* see include/linux/fs.h   */
                           char *buf,        /* buffer to fill with data */
                           size_t length,       /* length of the buffer     */
                           loff_t * offset)
{
    int len = 0;
    
    char *mode=NULL,*type=NULL;
    struct hcan_node *node;//copied from newer 4CH code
    node=PDE_DATA(file_inode(filp));//copied from newer 4CH code
    //struct hcan_node *node = filp;
    struct hcan_board *board = node->board;
    struct can_status *cs=node->can_status;
    uint8_t byte;

    switch(ioread16(&cs->mode)){
	case 1: mode="baudscan"; break;
	case 2: mode="passive"; break;
	case 3: mode="active"; break;
	case 4: mode="reset"; break;
	default: mode="invalid"; break;
    }

    len+=sprintf(buf+len,"can%d: node %d on board %s\n",node->minor,node->number,
	    pci_name(board->pdev));

    switch(ioread8(&cs->can_type)){
	case CAN_TYPE_EMPTY: type="empty - not populated"; break;
	case CAN_TYPE_HS: type="High Speed (HS)"; break;
	case CAN_TYPE_FT: type="Fault Tolerant (FT)"; break;
	default: type="Invalid/Unknown"; break;
    }

    len+=sprintf(buf+len,"type: %s\n",type);
    len+=sprintf(buf+len,"mode: %s\n",mode);
    
    len+=sprintf(buf+len,"bitrate: %dkbps\n",((int)ioread16(&cs->bitrate)));

    byte=ioread8(&cs->can_gsr);
    len+=sprintf(buf+len,"status: %x - %s%s%s\n",
	    byte,
	    byte&((1<<6)|(1<<7))?"":"ok",
	    byte&(1<<6)?"ErrPassive! ":"", 
	    byte&(1<<7)?"BusOff! ":"");

    len+=sprintf(buf+len,"iopin: %x", ioread8(&cs->iopin));
    if(ioread8(&cs->can_type) == CAN_TYPE_FT){
	if(ioread8(&cs->iopin)==0){
	    len+=sprintf(buf+len," <- line error!");
	} else {
	    len+=sprintf(buf+len," <- line ok");
	}
    }
    len+=sprintf(buf+len,"\n");

    len+=sprintf(buf+len,"errCnt tx/rx: %d/%d\n", 
	    ioread8(&cs->can_txerr),ioread8(&cs->can_rxerr));

    len+=sprintf(buf+len,"dpm Tx buf: %d/%d %s\n",
	    buf_message_cnt(&node->dpm_txbuf),buf_real_size(&node->dpm_txbuf),
	    buf_is_full(&node->dpm_txbuf)?"full!":"");

    len+=sprintf(buf+len,"dpm Rx buf: %d/%d %s\n",
	    buf_message_cnt(&node->dpm_rxbuf),buf_real_size(&node->dpm_rxbuf),
	    buf_is_full(&node->dpm_rxbuf)?"full!":"");

    len+=sprintf(buf+len,"sram Rx buf: %d/%d %s\n",
	    ioread16(&cs->msgs_in_sram),
	    ioread16(&cs->srambuf_size),
	    (ioread16(&cs->msgs_in_sram)==ioread16(&cs->srambuf_size))?"full!":"");

    len+=sprintf(buf+len,"rec/snt/flt: %d/%d/%d\n",
	    ioread16(&cs->received),ioread16(&cs->sent),
	    ioread16(&cs->filtered));

	    len+=sprintf(buf+len,"int_enable=%04x\n",ioread16(&board->dpm->int_enable));

    //*eof = 1; //used in former code
    return len;
}


//static int hcan_ioctl(struct inode *inode, struct file *filp,
//			 unsigned int cmd, unsigned long arg)
static long hcan_ioctl(struct file *filp,
             unsigned int cmd, unsigned long arg)
{

    struct hcan_board *board;
    struct hcan_node *node;
    struct can_filter filter;
    struct err_stat err_stat;
    int timeout,ret=0,val,i;

    node = (struct hcan_node *) filp->private_data;
    board = (struct hcan_board *) node->board;

    //check if a valid ioctl command for this driver
    if (_IOC_TYPE(cmd) != IOC_MAGIC)
	return -ENOTTY;

    switch (cmd) {

    case IOC_RESET_BOARD:

	/* Save the current int mask */
	val=ioread16(&board->dpm->int_enable);

	iowrite16(0,&board->dpm->board_status.fw_running);
	reset_mode(board,1);
	reset_mode(board,0);

	/* Wait for the boot firmware to get up and running  */
	timeout=100;
	while(ioread16(&board->dpm->board_status.fw_running)!=FW2_RUNNING && timeout--){
	    msleep(10);
	    if(--timeout==0)break;
	}
	if(!timeout){
	    printk(KERN_WARNING "%s: IOC_RESET_BOARD: could not get firmware running board %s\n",
		    __FUNCTION__,pci_name(board->pdev));
	    ret = -EIO;
	    break;
	}

	board->last_ack_count=ioread16(&board->dpm->board_status.cmd_ack_cnt);

	/* restore the interupt mask */
	iowrite16(val,&board->dpm->int_enable);
	break;


    case IOC_GET_CAN_STATUS:
	val=ioread8(&node->can_status->can_gsr);
	val|=ioread8(&node->can_status->can_rxerr)<<16;
	val|=ioread8(&node->can_status->can_txerr)<<24;
	
	if (copy_to_user((uint32_t *)arg, &val, sizeof(uint32_t))) {
	    ret = -EFAULT;
	    break;
	}
	break;

    case IOC_GET_CAN_TYPE:
	val=ioread8(&node->can_status->can_type);
	if (copy_to_user((uint32_t *)arg, &val, sizeof(uint32_t))) {
	    ret = -EFAULT;
	    break;
	}
	break;

    case IOC_GET_BOARD_STATUS:
	val=ioread16(&board->dpm->board_status.fw_running)<<16;
	val|=ioread16(&board->dpm->board_status.error);
	if (copy_to_user((uint32_t *)arg, &val, sizeof(uint32_t))) {
	    ret = -EFAULT;
	    break;
	}
	break;

    case IOC_GET_HW_ID:
	val=ioread8(&board->dpm->board_status.hw_id);
	if (copy_to_user((uint32_t *)arg, &val, sizeof(uint32_t))) {
	    ret = -EFAULT;
	    break;
	}
	break;

    case IOC_GET_PCI104_POS:
	val=ioread8(&board->dpm->board_status.pci104_pos);
	if (copy_to_user((uint32_t *)arg, &val, sizeof(uint32_t))) {
	    ret = -EFAULT;
	    break;
	}
	break;

    case IOC_GET_FW2_VERSION:
	val=ioread16(&board->dpm->board_status.fw_version);
	if (copy_to_user((uint32_t *)arg, &val, sizeof(uint32_t))) {
	    ret = -EFAULT;
	    break;
	}
	break;
    case IOC_GET_DRIVER_VERSION:
	val=HCANPCI_DRIVER_VERSION;
	if (copy_to_user((uint32_t *)arg, &val, sizeof(uint32_t))) {
	    ret = -EFAULT;
	    break;
	}
	break;
    case IOC_GET_LPCBC_REV:
	val=ioread16(&board->dpm->board_status.lpcbc_rev);
	if (copy_to_user((uint32_t *)arg, &val, sizeof(uint32_t))) {
	    ret = -EFAULT;
	    break;
	}
	break;

    case IOC_SET_BITRATE:
	if(copy_from_user(&val, (void *)arg, sizeof(int))){
	    ret = -EFAULT;
	    break;
	}
	ret=node_cmd(node,CMD_SET_BITRATE,val,0,NULL);
	break;

    case IOC_SET_SJW_INCREMENT:
	if(copy_from_user(&val, (void *)arg, sizeof(int))){
	    ret = -EFAULT;
	    break;
	}
	ret=node_cmd(node,CMD_SET_SJW_INCREMENT,val,0,NULL);
	break;
    case IOC_GET_ERR_STAT:
	for(i=0;i<0x3f;i++){
	    ret=node_cmd(node,CMD_GET_ERR_STAT,i,0,&val);
	    if(ret){
		break;
	    }
	    err_stat.values[i]=val;
	}
	if (copy_to_user((uint32_t *)arg, &err_stat, sizeof(struct err_stat))) {
	    ret = -EFAULT;
	    break;
	}
	break;
    case IOC_CLEAR_ERR_STAT:
	ret=node_cmd(node,CMD_CLR_ERR_STAT,0,0,NULL);
	break;

#ifdef IOC_SET_MODE
    case IOC_SET_MODE:
	if(copy_from_user(&val, (void *)arg, sizeof(int))){
	    ret = -EFAULT;
	    break;
	}
	ret=node_cmd(node,CMD_SET_MODE,val,0,NULL);
	break;
#endif

    case IOC_GET_BITRATE:
	val=ioread16(&node->can_status->bitrate_i);
	if (copy_to_user((uint32_t *)arg, &val, sizeof(uint32_t))) {
	    ret = -EFAULT;
	    break;
	}
	break;

    case IOC_GET_IOPIN_STATUS:
	val=ioread8(&node->can_status->iopin);
	if (copy_to_user((uint32_t *)arg, &val, sizeof(uint32_t))) {
	    ret = -EFAULT;
	    break;
	}
	break;

    case IOC_START:
	ret=node_cmd(node,CMD_SET_MODE,CM_ACTIVE,0,NULL);

	if(ret==0){
	    if((val=ioread16(&node->can_status->mode))!=CM_ACTIVE){
		printk(KERN_ERR "IOC_START: node in wrong mode %d\n",val);
		ret=-EIO;
	    }
	}
	break;

    case IOC_START_BAUDSCAN:
	ret=node_cmd(node,CMD_SET_MODE,CM_RESET,0,NULL);
	if(ret==0) ret=node_cmd(node,CMD_SET_MODE,CM_BAUDSCAN,0,NULL);

	if(ret==0){
	    if((val=ioread16(&node->can_status->mode))!=CM_BAUDSCAN){
		printk(KERN_ERR "IOC_START_BAUDSCAN: node in wrong mode %d\n",val);
		ret=-EIO;
	    }
	}
	break;

    case IOC_START_PASSIVE:
	ret=node_cmd(node,CMD_SET_MODE,CM_PASSIVE,0,NULL);
	if(ret==0){
	    if((val=ioread16(&node->can_status->mode))!=CM_PASSIVE){
		printk(KERN_ERR "IOC_START_PASSIVE: node in wrong mode %d\n",val);
		ret=-EIO;
	    }
	}
	break;

    case IOC_STOP:
	ret=node_cmd(node,CMD_SET_MODE,CM_RESET,0,NULL);
	if(ret==0){
	    if((val=ioread16(&node->can_status->mode))!=CM_RESET){
		printk(KERN_ERR "IOC_STOP: node in wrong mode %d\n",val);
		ret=-EIO;
	    }
	}
	break;

    case IOC_GET_MODE:
	val=ioread16(&node->can_status->mode);
	if (copy_to_user((uint32_t *)arg, &val, sizeof(uint32_t))) {
	    ret = -EFAULT;
	    break;
	}
	break;

    case IOC_MSGS_IN_RXBUF:
	val=buf_message_cnt(&node->dpm_rxbuf)+ioread16(&node->can_status->msgs_in_sram);
	if (copy_to_user((uint32_t *)arg, &val, sizeof(uint32_t))) {
	    ret = -EFAULT;
	    break;
	}
	break;

    case IOC_MSGS_IN_TXBUF:
	val=buf_message_cnt(&node->dpm_txbuf);
	if (copy_to_user((uint32_t *)arg, &val, sizeof(uint32_t))) {
	    ret = -EFAULT;
	    break;
	}
	break;

    case IOC_GET_TXBUF_SIZE:
	val=buf_real_size(&node->dpm_txbuf);
	if (copy_to_user((uint32_t *)arg, &val, sizeof(uint32_t))) {
	    ret = -EFAULT;
	    break;
	}
	break;

    case IOC_GET_RXBUF_SIZE:
	val=buf_real_size(&node->dpm_rxbuf)+ioread16(&node->can_status->srambuf_size);
	if (copy_to_user((uint32_t *)arg, &val, sizeof(uint32_t))) {
	    ret = -EFAULT;
	    break;
	}
	break;

    case IOC_RESET_TIMESTAMP:
	ret=node_cmd(node,CMD_RESET_TIMESTAMP,0,0,NULL);
	break;

    case IOC_SERIAL_DBG:
	if(copy_from_user(&val, (void *)arg, sizeof(int))){
	    ret = -EFAULT;
	    break;
	}
	ret=node_cmd(node,CMD_SERIAL_DBG,val,0,NULL);
	break;

    case IOC_PRODUCTION_OK:
	ret=node_cmd(node,CMD_PRODUCTION_OK,0,0,NULL);
	break;

    case IOC_LATTE_INIT:
	{
	    int timeout;
	    if(copy_from_user(&timeout, (void *)arg, sizeof(int))){
		ret = -EFAULT;
		break;
	    }
	    ret=node_cmd(node,CMD_INIT_LATTE,0,0,NULL);
	    /* Set the command timeout to a greater value than the sample
	     * timeout */
	    node->board->latte_timeout=timeout;
	    node->board->cmd_timeout=HZ*(timeout+1);
	    break;
	}
    case IOC_LATTE_INITIALIZED:
	if (copy_to_user((uint32_t *)arg, &node->board->latte_timeout, sizeof(uint32_t))) {
	    ret = -EFAULT;
	    break;
	}
	break;
    case IOC_LATTE_SAMPLE:
	{
	    struct latte_sample sample;
	    uint32_t *results;

	    /* Must be initialized first */
	    if(!node->board->latte_timeout){
		ret = -EPERM;
		break;
	    }

	    results=(uint32_t *)node->board->dpm_base;
	    
	    ret=node_cmd(node,CMD_LATTE,node->board->latte_timeout,0,NULL);
	    if(!ret){
		sample.t0 = 0;
		sample.t1 = ioread32(&results[0]);
		sample.t2 = ioread32(&results[1]);
		if (copy_to_user((uint32_t *)arg, &sample, sizeof(struct latte_sample))) {
		    ret = -EFAULT;
		    break;
		}
	    }
	}
	break;

    case IOC_SET_FILTER:
	if(copy_from_user(&filter,(void *)arg,sizeof(struct can_filter))){
	    ret=-EFAULT;
	    break;
	}

	switch(filter.type){
	case FTYPE_RANGE:
	    ret=node_cmd(node,CMD_SET_RANGE_FILTER,filter.lower,filter.upper,NULL);
	    break;
	case FTYPE_AMASK:
	    ret=node_cmd(node,CMD_SET_AMASK_FILTER,filter.mask,filter.code,NULL);
	    break;
	default:
	    ret=-EINVAL;
	    break;
	}
	iosetbits16(CF_FILTERS_ACTIVE, &node->can_status->flags2hico);
	break;

    case IOC_CLEAR_FILTERS:
	ret=node_cmd(node,CMD_CLR_FILTERS,0,0,NULL);
	ioclrbits16(CF_FILTERS_ACTIVE, &node->can_status->flags2hico);
	break;

    default:
	ret = -ENOTTY;
	break;

    }

    return ret;
}


/* Write firmware into the board */
ssize_t hcan_fw_write(struct file *filp, const char __user *buf, size_t count, loff_t *fpos)
{
    struct hcan_node *node=filp->private_data;
    struct hcan_board *board=node->board;
    uint8_t *data=NULL,*rptr;
    int ret=0;
    int blocks,block_nr=0;
    unsigned long timeout;

    blocks=count/FW_UPDATE_BLOCK_SIZE;
    if(count%FW_UPDATE_BLOCK_SIZE) blocks ++;

    data=kmalloc(blocks*FW_UPDATE_BLOCK_SIZE, GFP_KERNEL);
    if(!data){
	ret = -ENOMEM;
	goto out;
    }

    if(copy_from_user(data,buf,count)){
	ret=-EFAULT;
	goto out;
    }

    printk(KERN_DEBUG "%s: Writing firmware to board %s (image size %lu bytes, %d blocks)\n",
	    __FUNCTION__,pci_name(board->pdev),count,blocks);

    /* Set read pointer to the beginning of data */
    rptr=data;

    /* Reset the firmware running status variable */
    iowrite16(0,&board->dpm->board_status.fw_running);

    /* Set PCI reset active */
    reset_mode(board,1);

    /* Set the firmware update enable IO pin high */
    set_fw_update_enable_pin(board, 1);

    /* turn on the board */
    reset_mode(board,0);

    /* Wait for the boot firmware to get up and running  */
    timeout=100;
    iowrite16(0,&board->dpm->board_status.fw_running);
    while(ioread16(&board->dpm->board_status.fw_running)!=FW1_RUNNING){
	msleep(10);
	if(--timeout==0)break;
    }
    if(!timeout){
	printk(KERN_WARNING "%s: Boot firmware not running on board %s\n",
		__FUNCTION__,pci_name(board->pdev));
	ret=-EIO;
	goto out;
    }

    /* Enable command ack interrupts */
    iowrite16(INT_CMD_ACK,&board->dpm->int_enable);

    block_nr=0;
    while(blocks){

	block_nr++;
	/* Write one block to the dpm */
	memcpy_toio(board->dpm_base,rptr,FW_UPDATE_BLOCK_SIZE); 

	/* Put the block number into the mailbox */
	iowrite16(0,&board->dpm->mb_hico2host);
	iowrite16(block_nr,&board->dpm->mb_host2hico);
	

	/* Wait for an answer */
	board->cmd_ack=0;
	if(wait_event_timeout(board->ev_cmd_ack, board->cmd_ack, HZ)==0){
	    printk(KERN_WARNING "%s: No response from board %s. Timed out\n",
		    __FUNCTION__,pci_name(board->pdev));
	    ret=-EIO;
	    goto out;
	}

	blocks--;
	rptr+=FW_UPDATE_BLOCK_SIZE;
    }

    /* Wait for the new firmware to get running  */
    timeout=100;
    while(ioread16(&board->dpm->board_status.fw_running)!=FW2_RUNNING ){
	msleep(10);
	if(--timeout==0) break;
    }
    if(!timeout){
	printk(KERN_WARNING "%s: Application firmware not running on board %s\n",
		__FUNCTION__,pci_name(board->pdev));
	ret=-EIO;
	goto out;
    }

    ret=count;
	    

out:
    if(data) kfree(data);
    set_fw_update_enable_pin(board, 0);
    return ret;
}

/* In order to get the bootloader version info - we first need to reset the
 * board into fw1 */ 
int get_fw1_version(struct hcan_board *board)
{
    int ret=0;
    unsigned long timeout;

    /* Reset the firmware running status variable */
    iowrite16(0,&board->dpm->board_status.fw_running);

    /* Set PCI reset active */
    reset_mode(board,1);

    /* Set the firmware update enable IO pin high */
    set_fw_update_enable_pin(board, 1);

    /* turn on the board */
    reset_mode(board,0);

    /* Wait for the boot firmware to get up and running  */
    timeout=100;
    iowrite16(0,&board->dpm->board_status.fw_running);
    while(ioread16(&board->dpm->board_status.fw_running)!=FW1_RUNNING){
	msleep(10);
	if(--timeout==0)break;
    }
    if(!timeout){
	printk(KERN_WARNING "%s: Boot firmware not running on board %s\n",
		__FUNCTION__,pci_name(board->pdev));
	ret=-EIO;
	goto out;
    }

    board->fw1_version = ioread16(&board->dpm->board_status.fw_version);
    board->fw1_date[0] = ioread8(&board->dpm->board_status.fw_date[0]);
    board->fw1_date[1] = ioread8(&board->dpm->board_status.fw_date[1]);
    board->fw1_date[2] = ioread8(&board->dpm->board_status.fw_date[2]);
    board->fw1_date[3] = ioread8(&board->dpm->board_status.fw_date[3]);

    set_fw_update_enable_pin(board, 0);

    /* Set PCI reset active */
    reset_mode(board,1);

    /* turn on the board */
    reset_mode(board,0);


    /* Wait for the new firmware to get running  */
    timeout=100;
    while(ioread16(&board->dpm->board_status.fw_running)!=FW2_RUNNING ){
	msleep(10);
	if(--timeout==0) break;
    }
    if(!timeout){
	printk(KERN_WARNING "%s: Application firmware not running on board %s\n",
		__FUNCTION__,pci_name(board->pdev));
	ret=-EIO;
	goto out;
    }

    ret = 0;

out:
    set_fw_update_enable_pin(board, 0);
    return ret;
}

ssize_t hcan_read(struct file *filp, char __user *buff, size_t count, loff_t *offp)
{
    struct hcan_node *node=filp->private_data;
    struct hcan_board *board=node->board;
    struct can_msg _msg, *msg;
    int ret,i;

    /* Only CAN telegrams can be read */
    if (count != sizeof(struct can_msg))
	return -EINVAL;
    
    if(ioread16(&board->dpm->board_status.fw_running)!=FW2_RUNNING){
	printk(KERN_WARNING "%s: Firmware no running on board %s (fw_running=%x)\n",
		__FUNCTION__,pci_name(board->pdev),
		ioread16(&board->dpm->board_status.fw_running));
	return -EIO;
    }

    if(buf_is_empty(&node->dpm_rxbuf)){

	/* return if the read is set as non-blocking */
	if (filp->f_flags & O_NONBLOCK)
	    return -EAGAIN;

	/* Enable rx interrupts */
	iosetbits16(node->rx_int,&node->board->dpm->int_enable);
    
	/* Wait for data. Return with "restat sys command" error if the
	 * process received a signal */
	if (wait_event_interruptible(node->ev_rx_ready, buf_not_empty(&node->dpm_rxbuf))){
	    return -ERESTARTSYS;	
	}

    }

    /* Get the read pointer */
    msg=(struct can_msg *)BUF_RPTR(&node->dpm_rxbuf);

    /* Copy the message from the DPM into memory */
    _msg.fi = ioread16(&msg->fi);
    _msg.ts = ioread32(&msg->ts);
    _msg.id = ioread32(&msg->id);
    for(i=0;i<MSG_DLC(&_msg) && i<8;i++){
	_msg.data[i] = ioread8(&msg->data[i]);
    }


    /* Copy the CAN message in userspace. Return value is a number of bytes
     * left to be copied, which has to be zero */
    if (copy_to_user(buff, &_msg, sizeof(struct can_msg))) {
	ret = -EFAULT;
	goto out;
    }

    /* Increment read pointer */
    buf_increment_rptr(&node->dpm_rxbuf);

    ret=sizeof(struct can_msg);

out:
    return ret;
}

ssize_t hcan_write(struct file *filp, const char __user *buf, size_t count, loff_t *fpos)
{
    struct hcan_node *node=filp->private_data;
    struct hcan_board *board=node->board;
    struct can_msg *msg,_msg;
    int ret,i;

    if(fw_update){
	return hcan_fw_write(filp,buf,count,fpos);
    }

    if(ioread16(&board->dpm->board_status.fw_running)!=FW2_RUNNING){
	printk(KERN_INFO "%s: Firmware no running on board %s (fw_running=%x)\n",
		__FUNCTION__,pci_name(board->pdev),
		ioread16(&board->dpm->board_status.fw_running));
	return -EIO;
    }

    /* Only CAN telegrams can be written */
    if (count != sizeof(struct can_msg))
	return -EINVAL;

    if(buf_is_full(&node->dpm_txbuf)){

	/* return if the read is set as non-blocking */
	if (filp->f_flags & O_NONBLOCK)
	    return -EAGAIN;


	/* Enable Tx interrupts */
	iosetbits16(node->tx_int,&node->board->dpm->int_enable);

	/* Wait for free space in the tx buffer. Return with "restat sys
	 * command" error if the process received a signal */
	if (wait_event_interruptible(node->ev_tx_ready, buf_not_full(&node->dpm_txbuf))){
	    return -ERESTARTSYS;	
	}
    }
    
    /* Get the write pointer */
    msg=(struct can_msg *)BUF_WPTR(&node->dpm_txbuf);
    
    /* Copy the message from user space */
    if(copy_from_user(&_msg,buf,sizeof(struct can_msg))){
	ret=-EFAULT;
	goto out;
    }

    /* ..write it into dpm.. */
    iowrite16(_msg.fi,&msg->fi);
    iowrite32(_msg.ts,&msg->ts);
    iowrite32(_msg.id,&msg->id);
    for(i=0;i<MSG_DLC(&_msg) && i<8;i++){
	iowrite8(_msg.data[i],&msg->data[i]);
    }

    /*.. and increment write pointer */
    buf_increment_wptr(&node->dpm_txbuf);

    ret=sizeof(struct can_msg);

out:
    return ret;
}

int hcan_open(struct inode *inode, struct file *filp)
{
    struct hcan_node *node;

    node = container_of(inode->i_cdev, struct hcan_node, cdev);

    filp->private_data = node;

    return 0;
}

unsigned int hcan_poll(struct file *filp, poll_table * wait)
{
    unsigned int mask = 0;
    struct hcan_node *node = filp->private_data;

    /* Add the read and write wait queues to the polled wait queues */
    poll_wait(filp, &node->ev_rx_ready, wait);
    poll_wait(filp, &node->ev_tx_ready, wait);

    if (buf_not_full(&node->dpm_txbuf)){
	mask |= POLLOUT | POLLWRNORM;
    } else {
	iosetbits16(node->tx_int,&node->board->dpm->int_enable);
    }

    if (buf_not_empty(&node->dpm_rxbuf)){
	mask |= POLLIN | POLLRDNORM;
    } else {
	iosetbits16(node->rx_int,&node->board->dpm->int_enable);
    }

    return mask;
}

int hcan_release(struct inode *inode, struct file *filp)
{
    return 0;
}


struct file_operations hcan_fops = {
    .owner = THIS_MODULE,
    .read = hcan_read,
    .write = hcan_write,
    .unlocked_ioctl = hcan_ioctl,
    .poll = hcan_poll,
    .open = hcan_open,
    .release = hcan_release,
};

struct file_operations hboard_fops = {
    .owner = THIS_MODULE,
    .read = hcan_board_read,
//    .write = hcan_write,
//    .unlocked_ioctl = hcan_ioctl,
//    .poll = hcan_poll,
//    .open = hcan_open,
//    .release = hcan_release,
};
struct file_operations hnode_fops = {
    .owner = THIS_MODULE,
    .read = hcan_node_read,
//    .write = hcan_write,
//    .unlocked_ioctl = hcan_ioctl,
//    .poll = hcan_poll,
//    .open = hcan_open,
//    .release = hcan_release,
};

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,19) 
static irqreturn_t hcan_interrupt(int irq, void *__host)
#else
static irqreturn_t hcan_interrupt(int irq, void *__host, struct pt_regs *regs)
#endif
{
    int i;
    struct hcan_board *board=__host;
    uint16_t reason,tmp,fw_state;


    /* Get the irq reason and clear the cell. The reason can't be trusted
     * 100%, since we can't read and reset the cell atomically (not on all
     * platforms at least), so we always check the tx/rx queue status as well.
     * We use it just for performance. */
    reason = ioread16(&board->dpm->mb_hico2host);
    iowrite16(0,&board->dpm->mb_hico2host);

    /* During reset, the board sends some not wanted interrupts. If FW2 is not
     * running - only command ack interrupts are let through */
    fw_state=ioread16(&board->dpm->board_status.fw_running);
    if(fw_state!=FW2_RUNNING){
	if(fw_state==FW1_RUNNING || fw_state==EXCPT_RUNNING){
	    reason&=INT_CMD_ACK;
	} else {
	    reason=0;
	}
    }

    if(irqtrace){
	printk("reason=%04x",reason);
	if(reason&INT_CAN1_RX   ) printk(" INT_CAN1_RX");
	if(reason&INT_CAN1_TX   ) printk(" INT_CAN1_TX");
	if(reason&INT_CAN1_ERROR) printk(" INT_CAN1_ERROR");
	if(reason&INT_CAN2_RX   ) printk(" INT_CAN2_RX");
	if(reason&INT_CAN2_TX   ) printk(" INT_CAN2_TX");
	if(reason&INT_CAN2_ERROR) printk(" INT_CAN2_ERROR");
	if(reason&INT_CMD_ACK)    printk(" INT_CMD_ACK");
	if(reason&INT_ERROR)      printk(" INT_ERROR");
	if(reason&INT_EXCEPION)   printk(" INT_EXCEPION");
	printk("\n");
    }
    
    if(!reason){
	return IRQ_NONE;
    }
    for(i=0;i<NUMBER_OF_CAN_NODES;i++){
	struct hcan_node *node=&board->node[i];

	if(reason&node->rx_int ){
	    if(buf_not_empty(&node->dpm_rxbuf)){
		wake_up_interruptible(&node->ev_rx_ready);

		/* Disable rx interrupts */
		ioclrbits16(node->rx_int,&node->board->dpm->int_enable);
	    }
	}

	if(reason&node->tx_int){
	    wake_up_interruptible(&node->ev_tx_ready);
	    if(buf_is_empty(&node->dpm_txbuf)){
		/* Disable Tx interrupts */
		ioclrbits16(node->tx_int,&node->board->dpm->int_enable);
	    }
	} 

	if(reason&node->err_int){
	    wake_up_interruptible(&node->ev_tx_ready);
	    wake_up_interruptible(&node->ev_rx_ready);
	} 
    }

    tmp=ioread16(&board->dpm->board_status.cmd_ack_cnt);
    if(tmp!=board->last_ack_count){
        board->last_ack_count=tmp;
	board->cmd_ack=1;
	wake_up(&board->ev_cmd_ack);
    } 
	
    return IRQ_HANDLED;
}

    
void enable_pci_interrupts(struct hcan_board *board)
{
    uint16_t val;
    /* Enable PCI interrupts for the board. */
    val=ioread16(board->cfg_base+0x4c);
    val|=(1<<6);
    iowrite16(val,board->cfg_base+0x4c);
}
void disable_pci_interrupts(struct hcan_board *board)
{
    uint16_t val;
    /* Disable PCI interrupts for the board */
    val=ioread16(board->cfg_base+0x4c);
    val&=~(1<<6);
    iowrite16(val,board->cfg_base+0x4c);
}

static int hcan_pci_probe (struct pci_dev *pdev, const struct pci_device_id *pci_id)
{
    struct hcan_board *board;
    int ret,i;
    char name[50];
    if( (pci_id->subdevice != HICOMPCI_PCI_SUBDEV_ID) &&  
	    (pci_id->subdevice != HICOPCI104_PCI_SUBDEV_ID)){
	printk(KERN_WARNING "%s: called for a device with wrong subdevice id 0x%x\n",
		__FUNCTION__,pci_id->subdevice);
	return -EINVAL;
    }

    ret = pci_enable_device(pdev);
    if (ret){
	printk(KERN_WARNING "%s: Failed to enable device\n",
		__FUNCTION__);
	return ret;
    }

    ret = pci_request_regions(pdev, DRV_NAME);
    if (ret){
	printk(KERN_WARNING "%s: Failed to request regions\n",
		__FUNCTION__);
	goto err_out;
    }

    
    board = kmalloc(sizeof(*board), GFP_KERNEL);
    if (!board) {
	printk(KERN_ERR "%s: memory alloc failure\n",
	       __FUNCTION__);
	ret = -ENOMEM;
	goto err_out_regions;
    }

    memset(board, 0, sizeof(*board));
    board->pdev = pdev;
    
    board->number = board_count;
    board_count++;

    /* Default timeout for board commands */
    board->cmd_timeout=HZ;

    init_waitqueue_head(&board->ev_cmd_ack);

    /* PCI configuration registers */
    board->cfg_base = ioremap(pci_resource_start(pdev, 0),
			 pci_resource_len(pdev, 0));
    if (!board->cfg_base) {
	    printk(KERN_ERR "%s: could not map PCI configuration registers for board %s\n",
		   __FUNCTION__,pci_name(pdev));
	    ret = -ENOMEM;
	    goto err_out_kfree;
    }

    /* DPM memory 8k */
    board->dpm_base = ioremap(pci_resource_start(pdev, 2),
			 pci_resource_len(pdev, 2));
    if (!board->dpm_base) {
	    printk(KERN_ERR "%s: could not map DPM region for board %s\n",
		   __FUNCTION__,pci_name(pdev));
	    ret = -ENOMEM;
	    goto err_out_kfree;
    }


    /* Set the dpm pointer to point to the control area */
    board->dpm=SET_DPM_PTR(board->dpm_base);


    /* all boards share the same directory for now */
    board->proc_dir = hcan_proc_dir;


    /* Initialse board mutex */
    sema_init(&board->sem,1); //copied from 4CH driver

    /* Create a proc file entry for the board. Use the bus and device numbers
     * for the filename. The must be a more intelligent way to get the bus/dev
     * Ids, but i was too lazy to figure it out...*/
    {
	char name[20];
	int busn, devn,dummy;
	if(sscanf(pci_name(pdev),"%x:%x:%x.%x",&dummy,&busn,&devn,&dummy)!=4){
        sprintf(board->proc_name,"board_%s",pci_name(pdev));
	} else {
        sprintf(board->proc_name,"board_%02x_%02x",busn,devn);
	}
    //old code commented
//	board->proc_file = create_proc_read_entry(name,	//name
//				   0666,	//default mode
//				   board->proc_dir,	//parent dir
//				   hcan_board_proc,
//    				   board);
    board->proc_file = proc_create_data(board->proc_name,	//name
                                        0666,	//default mode
                                        board->proc_dir,	//parent dir
                                        &hboard_fops,
                                        board);
	if (!board->proc_file){
	    printk(KERN_WARNING "%s: Failed to create proc entry '%s'\n",
            __FUNCTION__,board->proc_name);
	}
    }


    for(i=0;i<NUMBER_OF_CAN_NODES;i++){
	struct hcan_node *node=&board->node[i];

	node->board=board;
	node->minor=FIRST_MINOR + ((board->number*NUMBER_OF_CAN_NODES)+i);
	node->number=i;

	/* Set the DPM message queue pointers */
	node->dpm_rxbuf.vars=&board->dpm->rx_buffers[i];
	node->dpm_txbuf.vars=&board->dpm->tx_buffers[i];

	node->can_status=&board->dpm->can_status[i];

	/* Do some checking on the variables, otherwise we could create a
	 * wild pointer */
	if(ioread16(&node->dpm_rxbuf.vars->base) > DPM_MSG_AREA_SIZE || 
	    ioread16(&node->dpm_txbuf.vars->base) > DPM_MSG_AREA_SIZE ||
	    ioread16(&node->dpm_rxbuf.vars->size) > (DPM_MSG_AREA_SIZE/sizeof(BUF_UNIT)) ||
	    ioread16(&node->dpm_txbuf.vars->size) > (DPM_MSG_AREA_SIZE/sizeof(BUF_UNIT))){
	    if(!fw_update){
	        printk(KERN_INFO "%s: DPM contains invalid message queue information on board %s\n",
		    __FUNCTION__,pci_name(board->pdev));
		ret=-EIO;
		goto err_out_kfree_nodes;
	    } else {
		node->dpm_rxbuf.base = NULL;
		node->dpm_txbuf.base = NULL;
	    }
	} else {
	    node->dpm_rxbuf.base = (BUF_UNIT *)((uint8_t *)board->dpm_base + ioread16(&node->dpm_rxbuf.vars->base));
	    node->dpm_txbuf.base = (BUF_UNIT *)((uint8_t *)board->dpm_base + ioread16(&node->dpm_txbuf.vars->base));
	}


	switch(i){
	    case 0: 
		node->tx_int=INT_CAN1_TX; 
		node->rx_int=INT_CAN1_RX; 
		node->err_int=INT_CAN1_ERROR; 
		break;
	    case 1:
		node->tx_int=INT_CAN2_TX; 
		node->rx_int=INT_CAN2_RX; 
		node->err_int=INT_CAN2_ERROR; 
		break;
	}

	
	init_waitqueue_head(&node->ev_tx_ready);
	init_waitqueue_head(&node->ev_rx_ready);

	cdev_init(&node->cdev, &hcan_fops);
	node->cdev.owner = THIS_MODULE;
	node->cdev.ops = &hcan_fops;

	ret = cdev_add(&node->cdev, MKDEV(major,node->minor), 1);
	if(ret){
	    printk(KERN_NOTICE "%s: Error %d adding can%d\n",
		    __FUNCTION__,ret,node->minor);
	    goto err_out_kfree_nodes;
	} else {
	    node->cdev_added=1;
	}

    sprintf(&node->proc_name[0],"can%d",node->minor);
//	node->proc_file = create_proc_read_entry(name,	//name
//				   0666,	//default mode
//				   board->proc_dir,	//parent dir
//				   hcan_node_proc,
//				   node);
    node->proc_file = proc_create_data(node->proc_name,	//name
                   0666,	//default mode
                   board->proc_dir,	//parent dir
                   &hnode_fops,
                   node);
	if (!node->proc_file){
	    printk(KERN_WARNING "%s: Failed to create proc entry '%s'\n",
            __FUNCTION__,node->proc_name);
	}
    }

    pci_set_drvdata(pdev, board);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,24)
    ret = request_irq(pdev->irq, hcan_interrupt, IRQF_SHARED, DRV_NAME, board);
#else
    ret = request_irq(pdev->irq, hcan_interrupt, SA_SHIRQ, DRV_NAME, board);
#endif
    if (ret) {
	    printk(KERN_ERR "%s: Could not allocati irq %d for board %s\n",
		   __FUNCTION__,pdev->irq,pci_name(board->pdev));
	    goto err_out_kfree_nodes;
    }

    enable_pci_interrupts(board);

    /* Enable command ackowledge interrupts */
    iosetbits16(INT_CMD_ACK, &board->dpm->int_enable);

    printk(KERN_INFO "%s: board %s with can nodes can%d and can%d initialized.\n",
	       __FUNCTION__, pci_name(pdev),board->node[0].minor,board->node[1].minor);
    
    if(!fw_update)
        get_fw1_version(board);
    
    board->last_ack_count=ioread16(&board->dpm->board_status.cmd_ack_cnt);

    /* Enable command ackowledge interrupts */
    iosetbits16(INT_CMD_ACK, &board->dpm->int_enable);

    return 0;



err_out_kfree_nodes:
    for(i=0;i<NUMBER_OF_CAN_NODES;i++){
	struct hcan_node *node=&board->node[i];
	if(node->cdev_added){
	    cdev_del(&node->cdev);
	}

	if(node->proc_file){
        remove_proc_entry(node->proc_name,board->proc_dir);
	    node->proc_file=NULL;
	}
    }

    if(board->dpm_base) iounmap(board->dpm_base);
    if(board->cfg_base) iounmap(board->cfg_base);

    if(board->proc_file){
    remove_proc_entry(board->proc_name,board->proc_dir);
	board->proc_file=NULL;
    }

err_out_kfree:
    kfree(board);
    board_count--;
err_out_regions:
    pci_release_regions(pdev);
err_out:
    pci_disable_device(pdev);
    return ret;
}



static void hcan_pci_remove (struct pci_dev *pdev)
{
    int i;
    struct hcan_board *board = pci_get_drvdata(pdev);

    disable_pci_interrupts(board);
    
    free_irq(pdev->irq, board);


    for(i=0;i<NUMBER_OF_CAN_NODES;i++){
	struct hcan_node *node=&board->node[i];
	if(node->cdev_added){
	    cdev_del(&node->cdev);
	}
	if(node->proc_file){
        remove_proc_entry(node->proc_name,board->proc_dir);
	    node->proc_file=NULL;
	}
    }

    if(board->proc_file){
    remove_proc_entry(board->proc_name,board->proc_dir);
	board->proc_file=NULL;
    }

    if(board->dpm_base) iounmap(board->dpm_base);
    if(board->cfg_base) iounmap(board->cfg_base);


    kfree(board);
    board_count--;
    pci_release_regions(pdev);
    pci_disable_device(pdev);
    pci_set_drvdata(pdev, NULL);
}

static struct pci_device_id hcan_pci_tbl[] = {
        { PCI_VENDOR_ID_PLX, 0x9030, PCI_VENDOR_ID_PLX, HICOMPCI_PCI_SUBDEV_ID, 0, 0, },
        { PCI_VENDOR_ID_PLX, 0x9030, PCI_VENDOR_ID_PLX, HICOPCI104_PCI_SUBDEV_ID, 0, 0, },

        { } 
};

MODULE_DEVICE_TABLE(pci, hcan_pci_tbl);

static struct pci_driver hcan_pci_driver = {
        .name           = DRV_NAME,
        .id_table       = hcan_pci_tbl,
        .probe          = hcan_pci_probe,
        .remove         = hcan_pci_remove,
};


static int __init hcan_init(void)
{
    dev_t devNo;
    int ret;

    if(major){
	devNo=MKDEV(major,FIRST_MINOR);
	ret=register_chrdev_region(devNo,MINOR_COUNT,DRV_NAME);
    }else{
	ret = alloc_chrdev_region(&devNo,FIRST_MINOR,MINOR_COUNT,DRV_NAME);
	major = MAJOR(devNo);
    }

    if(ret<0){
	printk(KERN_WARNING "%s: can't get major %d\n",
		__FUNCTION__,major);
	return ret;
    }
    

    hcan_proc_dir = proc_mkdir(DRV_NAME, NULL);
    if(!hcan_proc_dir){
	printk(KERN_WARNING "%s: could not create proc directory '%s'\n",
		__FUNCTION__,DRV_NAME);
	ret=-EINVAL;
	goto fail;
    }

    ret=pci_register_driver(&hcan_pci_driver);
    if(ret){
	printk(KERN_WARNING "%s: coul not register PCI driver\n",
		__FUNCTION__);
	goto fail_register;	
    }
    
    

    printk(KERN_INFO "%s: module initialized. Using major %d\n",
	    __FUNCTION__,major);
    
    return 0;

fail_register:
    //remove_proc_entry(hcan_proc_dir->name,NULL);
    remove_proc_entry(DRV_NAME,NULL);

fail:
    unregister_chrdev_region(MKDEV(major,FIRST_MINOR),MINOR_COUNT);
    return ret;
}

static void hcan_exit(void)
{
    pci_unregister_driver(&hcan_pci_driver);

    //remove_proc_entry(hcan_proc_dir->name,NULL);
    remove_proc_entry(DRV_NAME,NULL);
    
    unregister_chrdev_region(MKDEV(major,FIRST_MINOR),MINOR_COUNT);
}

module_init(hcan_init);
module_exit(hcan_exit);

