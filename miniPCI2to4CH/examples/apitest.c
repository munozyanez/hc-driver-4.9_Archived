/*EM_LICENSE*/
/* 
 * $Id$
 * Author: Martin Nylund
 */

/* This programs purpose is to test the drivers API with as many as possible
 * ioctl calls and combinations. It is _not_ meant to be a reference for
 * writing clean and well structured programs */



#include <sys/types.h>
#include <sys/wait.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <err.h>
#include <sys/time.h>
#ifndef __QNX__
 #include <poll.h>
#endif
#include <assert.h>
#include "hico_api.h"

#if defined(IOC_GET_ERR_STAT)
static char *err_stat_names[]={
    [0x2]="ID28:21",
    [0x3]="Start of Frame",
    [0x4]="SRTR bit",
    [0x5]="IDE bit",
    [0x6]="ID20:18",
    [0x7]="ID17:13",
    [0x8]="CRC",
    [0x9]="res bit 0",
    [0xa]="Data Field",
    [0xb]="DLC",
    [0xc]="RTR bit",
    [0xd]="res bit 1",
    [0xe]="ID4:0",
    [0xf]="ID12:5",
    [0x11]="Active error flag",
    [0x12]="Intermission",
    [0x13]="Dominant OK bits",
    [0x16]="Passive Error flag",
    [0x17]="Error Delimiter",
    [0x18]="CRC delimiter",
    [0x19]="ACK slot",
    [0x1a]="End of Frame",
    [0x1b]="ACK delimiter",
    [0x1c]="Overload flag",
};
#endif

/* Baudrates for High Speed can boards (i.e. "normal" boards) */
#define BITRATE_HS_1 BITRATE_500k
#define BITRATE_HS_2 BITRATE_1000k

/* Bitrates for Fault Tolerant boards */
#define BITRATE_FT_1 BITRATE_50k
#define BITRATE_FT_2 BITRATE_125k

#define REQUIRED_FW2_VERSION 1024
#define REQUIRED_DRIVER_VERSION 1124

int bitrate_1,bitrate_2;

#define TEST_SELECT
#define TEST_WITH_PINGPONG 
#ifndef __QNX__
#define TEST_POLL
#endif


char *msg2str(struct can_msg *msg);
int read_timeout(int fd, struct can_msg *buf, unsigned int timeout);

struct can_node{
    char *dev;
    int fd;
    int type;
    int hw_id;
    int lpcbc_rev;
    int fw2_version;
};

void timeout_handler(int sig)
{
    fprintf(stderr,"apitest TIMEOUT!\n");
    exit(1);
}

int main(int argc, char *argv[])
{
    int ret,node_i,i,val,state,n_read,n_write,size;
    struct can_node nodes[2],*TxNode,*RxNode;
    struct can_msg tx_msg, rx_msg;
    struct can_filter filter;
    
    struct timeval timeout_value={40,0};
    struct timeval timeout_interval={40,0};
    struct itimerval timeout_timer={timeout_interval,timeout_value};

#if defined(IOC_GET_ERR_STAT)
    struct err_stat err_stat;
#endif

#ifdef NDEBUG
    errx(1,"This test isn't much worth if compiled "
	    "with NDEBUG !!!\nOn QNX use the o-g variant.");
#endif

    setitimer(ITIMER_REAL,&timeout_timer,0);
    signal(SIGALRM,timeout_handler);

    if(argc!=3){
	errx(1, "usage: %s <can1> <can2>",argv[0]);
    }


    /* First of all check that our bit field definitions are correct. If
     * one of these tests fail the compiler is aligning something wrong.
     * maybe the endianess was not detected correctly... */
    rx_msg.fi=0;
    rx_msg.dlc=5;
    assert(MSG_DLC(&rx_msg)==5);

    rx_msg.fi=0;
    rx_msg.ff=1;
    assert(MSG_FF(&rx_msg)==1);

    rx_msg.fi=0;
    rx_msg.rtr=1;
    assert(MSG_RTR(&rx_msg)==1);

    rx_msg.fi=0;
    rx_msg.iopin=1;
    assert(MSG_IOPIN(&rx_msg)==1);

    rx_msg.fi=0;
    rx_msg.dos=1;
    assert(MSG_DOS(&rx_msg)==1);

    rx_msg.fi=0;
    rx_msg.node=3;
    assert(MSG_NODE(&rx_msg)==3);


    /* Open both CAN nodes for reading and writing */

    for(i=0;i<2;i++){
	struct can_node *node = &nodes[i];

	node->dev=argv[i+1];
	node->fd = open(node->dev, O_RDWR);
	if(node->fd < 0){
	    err(1, "could not open can node '%s'",node->dev);
	}
    }

    RxNode=&nodes[0];
    TxNode=&nodes[1];

    
    /* Reset the board using both nodes. If we have only one board giving
     * the reset command on one of the nodes would be enough. The nodes
     * can be located on different boards, though. */
    for(node_i=0;node_i<2;node_i++){
	struct can_node *node = &nodes[node_i];

	ret=ioctl(node->fd,IOC_RESET_BOARD);
	if(ret!=0){
	    err(1, "could not reset board");
	}

	ret=ioctl(node->fd,IOC_GET_FW2_VERSION,&val);
	if(ret!=0){
	    err(1, "could not get fw2 version");
	}
	if(val<REQUIRED_FW2_VERSION){
	    errx(1,"This version of apitest requires board firmware %d or higher (version %d detected)",
		    REQUIRED_DRIVER_VERSION,val);
	}

	node->fw2_version = val;

	ret=ioctl(node->fd,IOC_GET_DRIVER_VERSION,&val);
	if(ret!=0){
	    err(1, "could not get driver version");
	}
	if(val==0){
	    warnx("%s: USED DRIVER IS NOT A RELEASE",node->dev);
	} else {
	    if(val<REQUIRED_DRIVER_VERSION){
		errx(1,"This version of apitest requires Linux driver %d or higher (version %d detected)",
			REQUIRED_DRIVER_VERSION,val);
	    }
	}

	/* Check the node and board status */
	ret=ioctl(node->fd,IOC_GET_BOARD_STATUS,&val);
	assert(ret==0);

	if(val!=BS_RUNNING_OK){
	    errx(1,"board not running (IOC_GET_BOARD_STATUS=0x%x)\n",(uint32_t)val);
	}

	ret=ioctl(node->fd,IOC_GET_LPCBC_REV,&val);
	assert(ret==0);
	assert(val>0);
	node->lpcbc_rev=val;

	/* Check the hardware ID */
	ret=ioctl(node->fd,IOC_GET_HW_ID,&node->hw_id);
	assert(ret==0);

	switch(node->hw_id){
	    case HW_HICOCAN_UNKNOWN:
		warnx("unknown hardware ID 0x%x\n",node->hw_id);
	    case HW_HICOCAN_MPCI:
		break;
	    case HW_HICOCAN_MPCI_4C:
		break;
	    case HW_HICOCAN_PCI104:
		break;
	    default:
		errx(1,"Invalid Hardware identifier 0x%x\n",node->hw_id);
	}

	if(node->hw_id==HW_HICOCAN_PCI104){
	    /* Check the PCI ID (i.e. position in the stack from 0 to 3*/
	    ret=ioctl(node->fd,IOC_GET_PCI104_POS,&val);
	    assert(ret==0);
	    assert( (val>=0) && (val<=3) );
	}
    }

#ifdef ALLOW_UNKNOWN_HW
    bitrate_1 = BITRATE_FT_1;
    bitrate_2 = BITRATE_FT_2;
    for(node_i=0;node_i<2;node_i++){
	struct can_node *node = &nodes[node_i];
	node->type = CAN_TYPE_HS;
    }
#else
    /* Default to HS bitrates. If any of nodes is a FT, change to FT
     * bitrates */
    bitrate_1 = BITRATE_HS_1;
    bitrate_2 = BITRATE_HS_2;
    for(node_i=0;node_i<2;node_i++){
	struct can_node *node = &nodes[node_i];

	/* Check node type */
	ret=ioctl(node->fd,IOC_GET_CAN_TYPE,&node->type);
	assert(ret==0);

	switch(node->type){
	    case CAN_TYPE_EMPTY:
		errx(1,"CAN tranceiver not populated on %s (CAN_TYPE_EMPTY) - cannot run apitest\n",node->dev);
	    case CAN_TYPE_HS:
		break;
	    case CAN_TYPE_FT:
		bitrate_1 = BITRATE_FT_1;
		bitrate_2 = BITRATE_FT_2;
		break;
	    default:
		errx(1,"Invalid CAN_TYPE %d for node %s\n",node->type,node->dev);
	}
    }
#endif


    for(node_i=0;node_i<2;node_i++){
	struct can_node *node = &nodes[node_i];

	if(node->type==CAN_TYPE_FT){
	    /* Check the IO pin status which, on Fault Tolerant boards signals
	     * LineError (low active) */
	    ret=ioctl(node->fd,IOC_GET_IOPIN_STATUS,&val);
	    assert(ret==0);
	    assert(val==1 || val==0);
	    if(val==0) warnx("Fault tolerant tranceiver for node %s signals line error",node->dev);

	} else {
	    /* just check that the ioctl call returns a valid value */
	    ret=ioctl(node->fd,IOC_GET_IOPIN_STATUS,&val);
	    assert(ret==0);
	    assert(val==1 || val==0);
	}
    }

    /* Set baudrate/bitrate for both nodes */
    for(node_i=0;node_i<2;node_i++){
	struct can_node *node = &nodes[node_i];

	val=bitrate_1;
	ret=ioctl(node->fd,IOC_SET_BITRATE,&val);
	if(ret!=0){
	    err(1, "could not set bitrate");
	}

	/* Check that we can set the SJW value. To prove that it really
	 * has an effect, would be more difficult... */
	val=3;
	ret=ioctl(node->fd,IOC_SET_SJW_INCREMENT,&val);
	assert(ret==0);
	val=0; /* Set back the default value */
	ret=ioctl(node->fd,IOC_SET_SJW_INCREMENT,&val);
	assert(ret==0);
    }

    
    /* Set both nodes in active state */
    ret=ioctl(TxNode->fd,IOC_START);
    assert(ret==0);

    ret=ioctl(RxNode->fd,IOC_START);
    assert(ret==0);

    /* Check the mode value and check that it's correct */
    ret=ioctl(TxNode->fd,IOC_GET_MODE,&val);
    assert(ret==0);
    assert(val==CM_ACTIVE);

    ret=ioctl(RxNode->fd,IOC_GET_MODE,&val);
    assert(ret==0);
    assert(val==CM_ACTIVE);


    /* Compose a dummy CAN message to send */
    memset(&tx_msg,0,sizeof(tx_msg));
    memset(&rx_msg,0,sizeof(tx_msg));
    tx_msg.ff = FF_EXTENDED;
    tx_msg.id = 0xaabbcc;
    tx_msg.dlc = 8;
    tx_msg.rtr = 0;
    for(i=0;i<tx_msg.dlc;i++){
	tx_msg.data[i]=i;
    }


    /* Write the message to node 1 and receive it with node 2 */
    ret=write(TxNode->fd,&tx_msg,sizeof(struct can_msg));
    assert(ret==sizeof(struct can_msg));


    ret=read_timeout(RxNode->fd, &rx_msg, 1000);
    if(ret==0){	    /* Timeout */

	/* Check the Tx CAN node status */
	ret=ioctl(TxNode->fd,IOC_GET_CAN_STATUS,&val);
	assert(ret==0);

	if(val&CS_ERROR_PASSIVE || val&CS_ERROR_BUS_OFF){
	    errx(1,"TxNode->fd in %s state after first write - "
		    "check the cable and make sure you have a "
		    "termination resitor on the bus!",
		    val&CS_ERROR_BUS_OFF?"bus off":"error passive");
	}
	
	errx(1, "Failed to read the first message - check the cable!");
    } else {
	assert(ret==sizeof(struct can_msg));
	assert(rx_msg.dlc==8);
	assert(rx_msg.ff==FF_EXTENDED);

	/* Check that the bitmask macros return the same value */
	assert(MSG_DLC(&rx_msg)==8);
	assert(MSG_FF(&rx_msg)==FF_EXTENDED);
	assert(MSG_RTR(&rx_msg)==0);

	assert(rx_msg.id==0xaabbcc);
	assert(rx_msg.data[0]==0);
	assert(rx_msg.data[1]==1);
	assert(rx_msg.data[2]==2);
	assert(rx_msg.data[3]==3);
	assert(rx_msg.data[4]==4);
	assert(rx_msg.data[5]==5);
	assert(rx_msg.data[6]==6);
	assert(rx_msg.data[7]==7);
    }

    /* Make a double check for the IO pin / CAN error after sending a message */
    for(node_i=0;node_i<2;node_i++){
	struct can_node *node = &nodes[node_i];

	if(node->type==CAN_TYPE_FT){
	    /* Check the IO pin status which, on Fault Tolerant boards signals
	     * LineError (low active) */
	    ret=ioctl(node->fd,IOC_GET_IOPIN_STATUS,&val);
	    assert(ret==0);
	    assert(val==1 || val==0);
	    if(val==0) errx(1,"Fault tolerant tranceiver for node %s signals line error",node->dev);

	} else {
	    /* just check that the ioctl call returns a valid value */
	    ret=ioctl(node->fd,IOC_GET_IOPIN_STATUS,&val);
	    assert(ret==0);
	    assert(val==1 || val==0);
	}
    }

    /* ------------------ */
    /* Assert that we send/receive sane identifiers */
    tx_msg.ff = FF_EXTENDED;
    tx_msg.id = 0x1aaaaaaa;

    ret=write(TxNode->fd,&tx_msg,sizeof(struct can_msg));
    assert(ret==sizeof(struct can_msg));

    ret=read_timeout(RxNode->fd, &rx_msg, 1000);
    assert(ret!=0);

    assert(rx_msg.id == 0x1aaaaaaa);
    assert(rx_msg.ff == FF_EXTENDED);
    assert(MSG_FF(&rx_msg) == FF_EXTENDED);


    tx_msg.ff = FF_NORMAL;
    tx_msg.id = 0x1a;

    ret=write(TxNode->fd,&tx_msg,sizeof(struct can_msg));
    assert(ret==sizeof(struct can_msg));

    ret=read_timeout(RxNode->fd, &rx_msg, 1000);
    assert(ret!=0);

    assert(rx_msg.id == 0x1a);
    assert(rx_msg.ff == FF_NORMAL);


    tx_msg.ff = FF_NORMAL;
    tx_msg.id = 0;

    ret=write(TxNode->fd,&tx_msg,sizeof(struct can_msg));
    assert(ret==sizeof(struct can_msg));

    ret=read_timeout(RxNode->fd, &rx_msg, 1000);
    assert(ret!=0);

    assert(rx_msg.id == 0);
    assert(rx_msg.ff == FF_NORMAL);


    tx_msg.ff = FF_NORMAL;
    tx_msg.dlc = 0;
    tx_msg.rtr = 1;

    ret=write(TxNode->fd,&tx_msg,sizeof(struct can_msg));
    assert(ret==sizeof(struct can_msg));

    ret=read_timeout(RxNode->fd, &rx_msg, 1000);
    assert(ret!=0);

    assert(rx_msg.id == 0);
    assert(rx_msg.ff == FF_NORMAL);
    assert(rx_msg.dlc == 0);
    assert(rx_msg.rtr == 1);

    /* ------------------ */


    /* Set the node in non-blocking mode */
    ret=fcntl(RxNode->fd, F_SETFL, (fcntl(RxNode->fd, F_GETFL) | O_NONBLOCK));
    assert(ret==0);

    /* Try to read a message from a empty Rx buffer */
    ret=read(RxNode->fd,&rx_msg,sizeof(rx_msg));
    assert(ret==-1 && errno==EAGAIN);
    

    /* Now send a message with Tx node and read in non-blocking mode by
     * looping */
    tx_msg.id=0xbeef;
    tx_msg.ff = FF_EXTENDED;
    tx_msg.dlc = 8;
    tx_msg.rtr = 0;
    ret=write(TxNode->fd,&tx_msg,sizeof(tx_msg));
    assert(ret==sizeof(tx_msg));
    
    i=10;
    do{
	ret=read(RxNode->fd,&rx_msg,sizeof(rx_msg));
	usleep(1000);
	assert(i>0);
    }while(ret==-1 && errno==EAGAIN);


    //emS(msg2str(&rx_msg));

    
    /* Now set the transmittin node on non-blocking mode, set the CAN node in
     * passive mode and write the Tx buffer full */

    ret=fcntl(TxNode->fd, F_SETFL, (fcntl(TxNode->fd, F_GETFL) | O_NONBLOCK));
    assert(ret==0);

    /* Following should work identically for passive mode and reset mode. The
     * transmit buffer can be written full, and after setting the node in
     * active mode, it should transmit all the messages */
    for(state=0;state<2;state++){
	int mode_cmd;
	int expected_mode;
	if(state==0) {
	    mode_cmd = IOC_START_PASSIVE;
	    expected_mode= CM_PASSIVE;
	    printf("writing tx buffer full after IOC_START_PASSIVE");
	} else {
	    mode_cmd = IOC_STOP;
	    expected_mode= CM_RESET;
	    printf("writing tx buffer full after IOC_STOP");
	}

	ret=ioctl(TxNode->fd,IOC_STOP);
	assert(ret==0);

	ret=ioctl(TxNode->fd,IOC_GET_MODE,&val);
	assert(ret==0);
	assert(val==CM_RESET);

	ret=ioctl(TxNode->fd,mode_cmd);
	assert(ret==0);

	ret=ioctl(TxNode->fd,IOC_GET_MODE,&val);
	assert(ret==0);
	assert(val==expected_mode);

	ret=ioctl(TxNode->fd,IOC_MSGS_IN_TXBUF, &val);
	assert(ret==0);
	printf(" %d",val);

	n_write=0;
	tx_msg.ff=FF_NORMAL;
	while(1){
	    tx_msg.id=n_write;
	    ret=write(TxNode->fd,&tx_msg, sizeof(tx_msg));
	    if(ret!=sizeof(tx_msg)){
		break;
	    }
	    n_write++;
	};

	assert(ret==-1 && errno==EAGAIN);


	/* Make sure that the whole transmit buffer is full */
	ret=ioctl(TxNode->fd,IOC_MSGS_IN_TXBUF, &val);
	assert(ret==0);

	ret=ioctl(TxNode->fd,IOC_GET_TXBUF_SIZE, &size);
	assert(ret==0);

	assert(val==size);

	/* Now check that the rx buffer of the rx node is empty */
	ret=read(RxNode->fd,&rx_msg,sizeof(rx_msg));
	assert(ret==-1 && errno==EAGAIN);


	/* Set the tx node back to active (through reset mode ) mode and read
	 * the messages */
	ret=ioctl(TxNode->fd,IOC_STOP);
	assert(ret==0);

	ret=ioctl(TxNode->fd,IOC_GET_MODE,&val);
	assert(ret==0);
	assert(val==CM_RESET);

	ret=ioctl(TxNode->fd,IOC_START);
	assert(ret==0);

	ret=ioctl(TxNode->fd,IOC_GET_MODE,&val);
	assert(ret==0);
	assert(val==CM_ACTIVE);

	ret=ioctl(RxNode->fd,IOC_MSGS_IN_RXBUF, &val);
	assert(ret==0);

	/* Wait for the messages to be received */
	i=0;
	while(1){
	    usleep(1000*10);
	    ret=ioctl(RxNode->fd,IOC_MSGS_IN_RXBUF, &val);
	    assert(ret==0);

	    if(val==n_write){
		break;
	    }

	    /* Timeout check */
	    assert(++i<200);
	}

	ret=ioctl(RxNode->fd,IOC_MSGS_IN_RXBUF, &val);
	assert(ret==0);

	ret=ioctl(TxNode->fd,IOC_GET_TXBUF_SIZE, &size);
	assert(ret==0);

	assert(val==size);
	    
	n_read=0;
	while(1){
	    ret=read(RxNode->fd,&rx_msg,sizeof(rx_msg));
	    if(ret!=sizeof(rx_msg)){
		break;
	    }
	    n_read++;
	};

	assert(ret==-1 && errno==EAGAIN);

	/* Check that we sent and received the same amount of messages */
	assert(n_write == n_read);

	printf("\n");
    }

    
#ifdef TEST_POLL
    {
	struct pollfd pfd;

	/* Initialise the pollfd structure for waiting the POLLIN event from the
	 * RxNode->fd (i.e. data available) */
	pfd.fd = RxNode->fd;
	pfd.events = POLLIN;
	pfd.revents = 0;

	/* The rx buffer is now empty so poll should return with timeout */
	ret=poll(&pfd,1,100);
	assert(ret==0);


	/* Now send a message and wait with poll */
	tx_msg.id=0xcafe;
	tx_msg.ff=FF_EXTENDED;

	ret=write(TxNode->fd,&tx_msg, sizeof(tx_msg));
	assert(ret==sizeof(struct can_msg));

	ret=poll(&pfd,1,100);
	assert(ret==1 && pfd.revents&POLLIN);

	ret=read(RxNode->fd,&rx_msg,sizeof(rx_msg));
	assert(ret==sizeof(struct can_msg));
	
    }
#else
    printf("not testing poll()\n");
#endif

    /* Set both nodes back to blocking mode */
    ret=fcntl(RxNode->fd, F_SETFL, (fcntl(RxNode->fd, F_GETFL) & ~O_NONBLOCK));
    assert(ret==0);
    ret=fcntl(TxNode->fd, F_SETFL, (fcntl(TxNode->fd, F_GETFL) & ~O_NONBLOCK));
    assert(ret==0);


    /*******************************************************************/
    printf("testing passive error\n");
    /*******************************************************************/
    ret=ioctl(TxNode->fd,IOC_START);
    assert(ret==0);

    ret=ioctl(RxNode->fd,IOC_STOP);
    assert(ret==0);

    ret=write(TxNode->fd,&tx_msg,sizeof(tx_msg));
    assert(ret==sizeof(struct can_msg));

    /* Wait in a loop for the status to get to error passive */
    i=0;
    while(1){
	ioctl(TxNode->fd,IOC_GET_CAN_STATUS, &val);
	if(val&CS_ERROR_PASSIVE && (CS_GET_TXERRCNT(val)==128))break;
	usleep(10*1000);
	i++;
	if(i>100){
	    printf("Failed to generate passive error state. Make sure there " 
		    "are no other CAN nodes connected to the bus and try again\n");
	    assert(i<100);
	}
    }

    /* Check that the tx error count is 128 which is limit for error passive
     * state */
    assert(CS_GET_TXERRCNT(val)==128);

    /* Now put the receive node back to active state and read the message */
    ret=ioctl(RxNode->fd,IOC_START);
    assert(ret==0);

    ret=read(RxNode->fd,&rx_msg,sizeof(rx_msg));
    assert(ret==sizeof(struct can_msg));

    /* Check that the error counter has been reduced by one */
    ioctl(TxNode->fd,IOC_GET_CAN_STATUS, &val);
    assert(CS_GET_TXERRCNT(val)==127);

    /* The default error status treshold is 96 (0x60). Generate some more
     * traffic and make sure that we get out of the error status mode. */
    for(i=0;i<50;i++){
	ret=write(TxNode->fd,&tx_msg,sizeof(tx_msg));
	assert(ret==sizeof(struct can_msg));

	ret=read(RxNode->fd,&rx_msg,sizeof(rx_msg));
	assert(ret==sizeof(struct can_msg));
    }

    /* Check that the error counter has been reduced by 50 */
    ioctl(TxNode->fd,IOC_GET_CAN_STATUS, &val);
    assert(CS_GET_TXERRCNT(val)==(127-i));

    /* Make sure that the tx node is not any more in error passive mode */
    ioctl(TxNode->fd,IOC_GET_CAN_STATUS, &val);
    assert((val&CS_ERROR_PASSIVE)==0);

    /********************************************************************/
    /* Acceptance filtering */
    /********************************************************************/

    /* Set a acceptance filter */
    filter.type=FTYPE_RANGE;
    filter.lower=0xa;
    filter.upper=0xf;
    ret=ioctl(RxNode->fd,IOC_SET_FILTER,&filter);
    assert(ret==0);

    printf("testing acceptance filtering\n");

    /* Send some messages with diverse IDs */
    for(i=0;i<0xff;i++){
	tx_msg.id=i;
	ret=write(TxNode->fd,&tx_msg,sizeof(struct can_msg));
	assert(ret==sizeof(struct can_msg));
    }

    i=0;
    while(1){
	ret=read_timeout(RxNode->fd,&rx_msg,1000);
	
	if(ret==0) break; /* timeout */

	assert(ret==sizeof(struct can_msg));

	/* No messages outside the range are allowed */
	assert(rx_msg.id >= filter.lower && rx_msg.id <= filter.upper);
	i++;
	//emS(msg2str(&rx_msg));
    }

    /* Check that only the given id range was let through */
    assert(i==(filter.upper - filter.lower + 1));

    /* Clear acceptance filters */
    ret=ioctl(RxNode->fd,IOC_CLEAR_FILTERS,&filter);
    assert(ret==0);
    
    /* Set another acceptance filter */
    filter.type=FTYPE_AMASK;
    filter.mask=0x70; // bits of interest...
    filter.code=0x20; // ...the value they should have
    ret=ioctl(RxNode->fd,IOC_SET_FILTER,&filter);
    assert(ret==0);


    /* Send again some messages with diverse IDs */
    for(i=0;i<0xff;i++){
	tx_msg.id=i;
	ret=write(TxNode->fd,&tx_msg,sizeof(struct can_msg));
	assert(ret==sizeof(struct can_msg));
    }

    i=0;
    while(read_timeout(RxNode->fd,&rx_msg,100)==sizeof(struct can_msg)){

	assert((rx_msg.id&filter.mask) == (filter.mask&filter.code));
	i++;
    }

    
    /* Clear filters */
    ret=ioctl(RxNode->fd,IOC_CLEAR_FILTERS,&filter);
    assert(ret==0);
    
    /* Set two acceptance filters */
    filter.type=FTYPE_RANGE;
    filter.lower=10;
    filter.upper=15;
    ret=ioctl(RxNode->fd,IOC_SET_FILTER,&filter);
    assert(ret==0);

    filter.type=FTYPE_RANGE;
    filter.lower=120;
    filter.upper=125;
    ret=ioctl(RxNode->fd,IOC_SET_FILTER,&filter);
    assert(ret==0);

    /* Send again some messages with diverse IDs */
    for(i=0;i<0xff;i++){
	tx_msg.id=i;
	ret=write(TxNode->fd,&tx_msg,sizeof(struct can_msg));
	assert(ret==sizeof(struct can_msg));
    }

    i=0;
    while(read_timeout(RxNode->fd,&rx_msg,100)==sizeof(struct can_msg)){
	i++;
    }

    /* Clear filters */
    ret=ioctl(RxNode->fd,IOC_CLEAR_FILTERS,&filter);
    assert(ret==0);

    /* Check that we received the right amount of messages */
    assert(i==6+6);

    /********************************************************************/
    /* Test timestamps */
    /********************************************************************/
    {
	int saved_ts=0;
	printf("testing timestamps\n");
	ret=write(TxNode->fd,&tx_msg,sizeof(tx_msg));
	assert(ret==sizeof(struct can_msg));

	ret=read_timeout(RxNode->fd,&rx_msg,1000);
	assert(ret==sizeof(struct can_msg));

	/* Just check that it's not zero */
	assert(rx_msg.ts>100);
	saved_ts=rx_msg.ts;

	/* Which node is used for resetting doesn't matter. The nodes have a
	 * common timer for timestamps */
	ret=ioctl(RxNode->fd,IOC_RESET_TIMESTAMP);
	assert(ret==0);

	ret=write(TxNode->fd,&tx_msg,sizeof(tx_msg));
	assert(ret==sizeof(struct can_msg));

	ret=read_timeout(RxNode->fd,&rx_msg,1000);
	assert(ret==sizeof(struct can_msg));

	/* Check that the timestamp got reset*/
	assert(rx_msg.ts<saved_ts);
    }

    /********************************************************************/
    /* Test receiving messages in passive mode */

    /* We need to distinguish here between different processor revisions
     * of lpc2292. Revision B receives messages in passive mode even
     * though nobody acknowledges then. In revision 'C' messages are not
     * received. This makes a big difference in this test if only two CAN
     * nodes are connected to the bus. Either we receive nothing (revision
     * 'C', or we receive continuously the same message (i.e. the sending
     * node puts the message coninuously on the bus and it newer gets
     * acknowledged).
     *
     * The chip revision cannot be read in the processor, so we use the
     * internal bootcode information.
     */
#define CHIPREV_C_LPCBC 0x146
    /********************************************************************/
    printf("testing passive mode\n");
    ret=ioctl(TxNode->fd,IOC_STOP);
    assert(ret==0);
    ret=ioctl(RxNode->fd,IOC_STOP);
    assert(ret==0);

    /* Start receive node in passive state */
    ret=ioctl(RxNode->fd,IOC_START_PASSIVE);
    assert(ret==0);

    /* Tx node in acitive state */
    ret=ioctl(TxNode->fd,IOC_START);
    assert(ret==0);

    /* Send one message */
    ret=write(TxNode->fd,&tx_msg,sizeof(tx_msg));
    assert(ret==sizeof(struct can_msg));

    /* Since the message newer gets ackowledged (assuming there are no
     * other CAN nodes on the bus), we should not receive any messages */

    for(i=0;i<100;i++){
	ret=read_timeout(RxNode->fd,&rx_msg,1000);
	if(ret!=sizeof(struct can_msg))
	    break;
    }

    if(RxNode->lpcbc_rev >= CHIPREV_C_LPCBC){
	assert(i==0);
    }else{
	/* We should not receive not ack'd messages, but it's difficult to
	 * fix a buggy processor */
	assert(i==100);
	printf("\tignoring result because of old chip revision\n");
    }

    ret=ioctl(TxNode->fd,IOC_STOP);
    assert(ret==0);
    ret=ioctl(RxNode->fd,IOC_STOP);
    assert(ret==0);


    /********************************************************************/
    /* Test baudscan */
    /********************************************************************/
    printf("testing baudscan\n");

    /* Transition to baudscan mode only allowed through the reset mode */
    ret=ioctl(RxNode->fd,IOC_STOP,&val);
    assert(ret==0);

    ret=ioctl(RxNode->fd,IOC_START_BAUDSCAN,&val);
    assert(ret==0);

    ret=ioctl(TxNode->fd,IOC_START);
    assert(ret==0);

    ret=ioctl(RxNode->fd,IOC_GET_MODE,&val);
    assert(ret==0);
    assert(val==CM_BAUDSCAN);


    /* Send again some messages with diverse IDs */
    for(i=0;i<200;i++){
	tx_msg.id=i;
	ret=write(TxNode->fd,&tx_msg,sizeof(struct can_msg));
	assert(ret==sizeof(struct can_msg));
    }

    i=0;
    while(1){
	ret=read_timeout(RxNode->fd,&rx_msg,500);
	if(ret==0) break; /* timeout */
	assert(ret==sizeof(struct can_msg));
	i++;
    }

    /* make sure we received some messages */ 
    assert(i>0);

    ret=ioctl(RxNode->fd,IOC_GET_BITRATE, &val);
    assert(ret==0);

    /* both nodes should have the same baudrate now */
    assert(val==bitrate_1);


    /* Change to baudrate 2 */

    ret=ioctl(TxNode->fd,IOC_STOP);
    assert(ret==0);
    ret=ioctl(TxNode->fd,IOC_GET_MODE,&val);
    assert(ret==0);
    assert(val==CM_RESET);

    ret=ioctl(RxNode->fd,IOC_STOP);
    assert(ret==0);
    ret=ioctl(RxNode->fd,IOC_GET_MODE,&val);
    assert(ret==0);
    assert(val==CM_RESET);

    /* Set baudrate for both nodes and set them to active state*/
    val=bitrate_2;
    ret=ioctl(TxNode->fd,IOC_SET_BITRATE,&val);
    assert(ret==0);
    ret=ioctl(RxNode->fd,IOC_SET_BITRATE,&val);
    assert(ret==0);

    ret=ioctl(TxNode->fd,IOC_START);
    assert(ret==0);
    ret=ioctl(RxNode->fd,IOC_START);
    assert(ret==0);


#if defined(IOC_GET_ERR_STAT)
    /* Test status */
    for(node_i=0;node_i<2;node_i++){
	struct can_node *node = &nodes[node_i];

	/* Test error statistics only if we have fw2 version above 1135 */
	if(node->fw2_version<1135)
	    continue;
	/***************************************************************/

	printf("error stat for %s\n",node->dev);

	ret=ioctl(node->fd,IOC_GET_ERR_STAT,&err_stat);
	assert(ret==0);
	state=0;
	for(i=0;i<0x3f;i++){
	    if(err_stat.values[i]){
		printf("\t%s %s: %04x\n",i&0x20?"Rx":"Tx",err_stat_names[i&0x1f],err_stat.values[i]);
		state=1;
	    }
	}
	/* make sure that some errors were detected during the baudscan */
	assert(state==1);

	/* Clear the error statistics */
	ret=ioctl(node->fd,IOC_CLEAR_ERR_STAT);
	if(ret!=0){
	    err(1,"IOC_CLEAR_ERR_STAT");
	}

	/* Wait for a second for any errors on CAN bus (these a<re
	 * possible on interferences on the signal) */
	sleep(1);

	/* Get the statistics again and make sure all are zero */
	ret=ioctl(node->fd,IOC_GET_ERR_STAT,&err_stat);
	assert(ret==0);
	for(i=0;i<0x3f;i++){
	    if(err_stat.values[i]){
		printf("ERR: \t%s %s: %04x\n",i&0x20?"Rx":"Tx",err_stat_names[i&0x1f],err_stat.values[i]);
		ret=1;
	    }
	}
	assert(ret==0);
    }
#endif

#ifdef TEST_SELECT
    /********************************************************************/
    /* Test function with select() */
    /********************************************************************/
    printf("testing select()\n");


    for(i=0;i<2;i++){

	fd_set rfds,wfds;
        struct timeval tv;
	FD_ZERO(&rfds);
	FD_ZERO(&wfds);
	tv.tv_sec = 1;
	tv.tv_usec = 0;
	
	printf("testing select() in ");
	if(i==0){
	    printf("nonblocking mode\n");
	    /* Set both nodes in non-blocking mode */
	    ret=fcntl(RxNode->fd, F_SETFL, (fcntl(RxNode->fd, F_GETFL) | O_NONBLOCK));
	    assert(ret==0);
	    ret=fcntl(TxNode->fd, F_SETFL, (fcntl(TxNode->fd, F_GETFL) | O_NONBLOCK));
	    assert(ret==0);
	}else{
	    printf("blocking mode\n");
	    /* Set both nodes back to blocking mode */
	    ret=fcntl(RxNode->fd, F_SETFL, (fcntl(RxNode->fd, F_GETFL) & ~O_NONBLOCK));
	    assert(ret==0);
	    ret=fcntl(TxNode->fd, F_SETFL, (fcntl(TxNode->fd, F_GETFL) & ~O_NONBLOCK));
	    assert(ret==0);
	}

	/* TxNode->fd should be writable immediately */
	FD_SET(TxNode->fd,&wfds);
	ret=select(TxNode->fd+1, NULL,&wfds,NULL,NULL);
	assert(ret==1);
	assert(FD_ISSET(TxNode->fd,&wfds));

	/* Check that it works with timeout too */
	tv.tv_sec=1;
	FD_SET(TxNode->fd,&wfds);
	ret=select(TxNode->fd+1, NULL,&wfds,NULL,&tv);
	assert(ret==1);
	assert(FD_ISSET(TxNode->fd,&wfds));

	/* select on RxNode->fd should block and return with timeout since
	 * there's nothing in the Rx buffer */
	tv.tv_sec=1;
	FD_SET(RxNode->fd,&rfds);
	ret=select(RxNode->fd+1,&rfds,NULL,NULL,&tv);
	assert(ret==0);
	assert(!FD_ISSET(RxNode->fd,&rfds));


	/* Now send one message and test that select() returns with
	 * appropriate fd set */
	ret=write(TxNode->fd,&tx_msg,sizeof(struct can_msg));
	assert(ret==sizeof(struct can_msg));

	tv.tv_sec=1;
	FD_SET(RxNode->fd,&rfds);
	ret=select(RxNode->fd+1,&rfds,NULL,NULL,&tv);
	assert(ret==1);
	assert(FD_ISSET(RxNode->fd,&rfds));

	/* Read the message and make sure that select returns after that again
	 * with timeout */
	ret=read(RxNode->fd,&rx_msg,sizeof(rx_msg));
	assert(ret==sizeof(struct can_msg));

	tv.tv_sec=1;
	FD_SET(RxNode->fd,&rfds);
	ret=select(RxNode->fd+1,&rfds,NULL,NULL,&tv);
	assert(ret==0);
	assert(!FD_ISSET(RxNode->fd,&rfds));

	/* Now test the same without timeout */
	ret=write(TxNode->fd,&tx_msg,sizeof(struct can_msg));
	assert(ret==sizeof(struct can_msg));

	tv.tv_sec=1;
	FD_SET(RxNode->fd,&rfds);
	ret=select(RxNode->fd+1,&rfds,NULL,NULL,NULL);
	assert(ret==1);
	assert(FD_ISSET(RxNode->fd,&rfds));

	/* read the message */
	ret=read(RxNode->fd,&rx_msg,sizeof(rx_msg));
	assert(ret==sizeof(struct can_msg));

    }

#else
    printf("not testing select()\n");
#endif

#ifdef TEST_WITH_PINGPONG

    /* The select() calls here are just to make sure the API works ok. For the
     * pingpong test below, they could be commented out */
    printf("testing pingpong\n");
    {
	struct can_msg msg;
	int fd;
	int pid;
	fd_set fds;
	int limit=100;
        //struct timeval tv;
	FD_ZERO(&fds);
	switch(pid=fork()){
	    case 0:
		/* child */
		fd=RxNode->fd;

		/* send initial msg */
		tx_msg.id=0xa;
		ret=write(fd,&tx_msg,sizeof(struct can_msg));
		assert(ret==sizeof(struct can_msg));

		while(1){
		    FD_SET(fd,&fds);
		    ret=select(fd+1,&fds,NULL,NULL,NULL);
		    assert(ret==1);
		    assert(FD_ISSET(fd,&fds));

		    ret=read(fd,&msg,sizeof(struct can_msg));
		    assert(ret==sizeof(struct can_msg));

		    if(msg.id==0) exit(0);

		    FD_SET(fd,&fds);
		    ret=select(fd+1,NULL,&fds,NULL,NULL);
		    assert(ret==1);
		    assert(FD_ISSET(fd,&fds));

		    ret=write(fd,&msg,sizeof(struct can_msg));
		    assert(ret==sizeof(struct can_msg));
		}
		exit(0);
	    default:
		/* parent */
		fd=TxNode->fd;
		assert(pid>0);
		i=0;
		while(1){

		    FD_SET(fd,&fds);
		    ret=select(fd+1,&fds,NULL,NULL,NULL);
		    assert(ret==1);
		    assert(FD_ISSET(fd,&fds));

		    ret=read(fd,&msg,sizeof(struct can_msg));
		    assert(ret==sizeof(struct can_msg));

		    printf("%d ",msg.ts/1000);
		    fflush(stdout);

		    FD_SET(fd,&fds);
		    ret=select(fd+1,NULL,&fds,NULL,NULL);
		    assert(ret==1);
		    assert(FD_ISSET(fd,&fds));

		    i++;
		    if(i>limit){
			/* send a 'terminate id' */
			msg.id=0x0;
		    }

		    ret=write(fd,&msg,sizeof(struct can_msg));
		    assert(ret==sizeof(struct can_msg));

		    if(i>limit){
			ret=wait(&i);
			assert(ret==pid);
			assert(WIFEXITED(i));
			break;
		    }
		}
	}

    }

#else
    printf("not testing with pingpong\n");
#endif

    /* Blink LEDs to indicate succesful test */
    if(getenv("HCANPCI_PRODUCTION")){
	ret=ioctl(TxNode->fd,IOC_PRODUCTION_OK);
    }


    close(TxNode->fd);
    close(RxNode->fd);

    printf("- test through -\n");

    return 0;
}

/* Function to make a string presentation of a CAN message */
char *msg2str(struct can_msg *msg)
{
    int i = 0;
    static char buf[100],*ptr=NULL;

    ptr=&buf[0]; 

    ptr+=sprintf(ptr,"%s=%08x rtr=%d ts=%d data %d bytes: ",
	    msg->ff?"extID":"   ID",
	    msg->id,
	    msg->rtr,
	    msg->ts,
	    msg->dlc);
    
    for (i = 0; (i < msg->dlc) && (i<8); i++) {
	ptr+=sprintf(ptr,"%02x ",msg->data[i]);
    }
    if(msg->dos)ptr+=sprintf(ptr,"dos!");
    return &buf[0];

}


/* Read a message from the driver. If the Rx buffer is empty - wait for the
 * time given by timeout argument for Rx data.
 *
 * parameters:
 *     fd - file descriptor of a CAN node
 *     buf - CAN message where the data is to be written
 *     timeout - timeout for the read operation in milliseconds
 *
 *  return value:
 *     < 0  -  Error value
 *     0    -  timeout occured
 *     > 0  -  number of read bytes (i.e. sizeof(struct can_msg)
 */
int read_timeout(int fd, struct can_msg *buf, unsigned int timeout)
{
    fd_set fds;
    struct timeval tv;
    int sec,ret;
    FD_ZERO(&fds);

    sec=timeout/1000;
    tv.tv_sec=sec;
    tv.tv_usec=(timeout-(sec*1000))*1000;

    FD_SET(fd,&fds);

    ret=select(fd+1,&fds,0,0,&tv);
    if(ret==0){
	return 0; /* timeout */
    } else if (ret<0) {
	return errno;
    } else {
	assert(FD_ISSET(fd,&fds));
	ret=read(fd,buf,sizeof(struct can_msg));
	return ret;
    }
}
