/*EM_LICENSE*/
/* $Id$
 * Author: Martin Nylund
 */

/* This program generates traffic and tries to use the board "on its
 * limits".  The CAN nodes will likely have many times data overruns, and
 * error passive states (i.e. the error leds are likely to be blinking).
 * How often the nodes go to error passive state depends a lot, whether
 * there are other CAN nodes attached to the bus. 
 *
 * If the program exits after 40 seconds without "hanging" in any point
 * and without giving any error messages, the test has been run
 * succesfully.
 *
 * NOTE: This program is not meant to serve as an example, how to write
 * "good" software. It's only purpose is to try to abuse the driver */


#include <sys/types.h>
#include <sys/wait.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <err.h>
#include <pthread.h>
#include <sys/time.h>
#ifndef __QNX__
 #include <poll.h>
#endif
#include <assert.h>
#include "hico_api.h"

/* Baudrates for High Speed can boards (i.e. "normal" boards) */
#define BITRATE_HS BITRATE_1000k

/* Bitrates for Fault Tolerant boards */
#define BITRATE_FT BITRATE_125k

#ifdef ALLOW_UNKNOWN_HW
#undef BITRATE_HS
# define BITRATE_HS BITRATE_FT
#endif

#define NUM_THREADS 6

/* Depending on your hardware performance, you might need to adjust this
 * timeout to get the test run through */
#define TIMEOUT 180

char *msg2str(struct can_msg *msg, int fault_tolerant, char *buf);
int read_timeout(int fd, struct can_msg *buf, unsigned int timeout);

/* Some CAN node specific variables, which are passed to the threads */
struct can_node{
    int fd;
    char *dev;
    int type;
    int start_stop_pause;
    int status_read_pause;
    int row;
};

/* Global variables */
pthread_mutex_t stdout_mutex;
int fault_tolerant=0;
int verbose=1;
int running = 1;

/* Thread to read diverse status variables from the CAN node */
void *status_read(void *data)
{
    int ret,val;
    struct can_node *node=(struct can_node *)data;
    int can_status;
    int board_status;
    int iopin;
    int mode;
    
    while(running){

	ret=ioctl(node->fd,IOC_GET_MODE,&mode);
	assert(ret==0);

	ret=ioctl(node->fd,IOC_GET_BOARD_STATUS,&board_status);
	assert(ret==0);

	ret=ioctl(node->fd,IOC_GET_IOPIN_STATUS,&iopin);
	assert(ret==0);

	ret=ioctl(node->fd,IOC_GET_CAN_STATUS,&can_status);
	assert(ret==0);

	ret=ioctl(node->fd,IOC_GET_BITRATE,&val);
	assert(ret==0);

	if(verbose){
	    pthread_mutex_lock(&stdout_mutex);
	    if(isatty(1)){
		printf("\033[u"); /* restore cursor position */
		printf("\033[%dA",node->row); /* cursor down X rows */
		printf("%s: %08x %x %x", node->dev,
			can_status,mode,iopin);
	    } else {
		printf("%s status: %08x %x %x\n", node->dev, 
			can_status,mode,iopin);
	    }
	    pthread_mutex_unlock(&stdout_mutex);
	}

	usleep(node->status_read_pause);

    }

    pthread_exit((void *)0);
    return NULL;
}

/* Thread to start and stop a CAN node (i.e. set to 
 * active and reset mode) */
void *stop_and_start(void *data)
{
    int ret,val;
    struct can_node *node=(struct can_node *)data;
    
    while(running){

	if(verbose>1){
	    pthread_mutex_lock(&stdout_mutex);
	    printf("%s: STOP\n",node->dev);
	    pthread_mutex_unlock(&stdout_mutex);
	}
	ret=ioctl(node->fd,IOC_STOP);
	assert(ret==0);

	ret=ioctl(node->fd,IOC_GET_MODE,&val);
	assert(ret==0);
	assert(val==CM_RESET);

	/* don't stay that long in reset mode */
	usleep(node->start_stop_pause/4);

	if(verbose>1){
	    pthread_mutex_lock(&stdout_mutex);
	    printf("%s: START\n",node->dev);
	    pthread_mutex_unlock(&stdout_mutex);
	}
	ret=ioctl(node->fd,IOC_START);
	assert(ret==0);

	ret=ioctl(node->fd,IOC_GET_MODE,&val);
	assert(ret==0);
	assert(val==CM_ACTIVE);

	usleep(node->start_stop_pause);
    }

    pthread_exit((void *)0);
    return NULL;
}

/* Thread to generate traffic. Write the tx buffer full and then read all
 * messages from rx buffer */
void *tx_and_rx(void *data)
{
    struct can_node *node=(struct can_node *)data;
    int count=0,ret;
    struct can_msg msg;
    char buf[100];

    /* Set the node in non-blocking mode */
    ret=fcntl(node->fd, F_SETFL, (fcntl(node->fd, F_GETFL) | O_NONBLOCK));
    assert(ret==0);

    while(running){

	/* Compose a dummy CAN message to send */
	memset(&msg,0,sizeof(msg));
	msg.ff = FF_NORMAL;
	msg.id = node->fd;
	msg.dlc = 4;

	/* Write messages into the nodes transmit buffer, until it's full
	 * (i.e. it returns with -1 and errno is set to EAGAIN) */
	while(1){
	    count++;
	    msg.data[0] = (uint8_t)(count>>24);
	    msg.data[1] = (uint8_t)(count>>16);
	    msg.data[2] = (uint8_t)(count>>8);
	    msg.data[3] = (uint8_t)(count>>0);
	    ret=write(node->fd,&msg,sizeof(struct can_msg));
	    if(ret==-1 && errno==EAGAIN){
		count--;
		break;
	    } else if (ret != sizeof(msg)){
		err(1,"write returned %d",ret);
	    }
	}


	while(1){
	    ret=read(node->fd,&msg,sizeof(msg));
	    if(ret==-1 && errno==EAGAIN){
		break;
	    }
	    if(ret!=sizeof(msg)){
		err(1,"read");
	    }
	    

	    if(verbose){
		pthread_mutex_lock(&stdout_mutex);
		msg2str(&msg,fault_tolerant,&buf[0]);
		if(isatty(1)){
		    printf("\033[u"); /* restore cursor position */
		    printf("\033[%dA",node->row); /* cursor down X rows */
		    printf("\033[%dC",25);
		    printf("%s",&buf[0]);
		} else {
		    printf("%s: ",node->dev);
		    printf(&buf[0]);
		    printf("\n");
		}
		pthread_mutex_unlock(&stdout_mutex);
	    }
	}


    }

    pthread_exit((void *)0);
    return NULL;
}

void ctrl_c()
{ 	
    /* turn cursor on again */
    if(isatty(1)){
	printf("\n\n");
	printf("\033[?25h");
    }
    exit(1);
}
void timeout_handler(int sig)
{
    /* turn cursor on again */
    if(isatty(1)){
	printf("\n\n");
	printf("\033[?25h");
    }
    fprintf(stderr,"TIMEOUT!\n");
    exit(1);
}


int main(int argc, char *argv[])
{
    int ret,i,val,bitrate;
    int TxNode, RxNode;
    struct can_msg tx_msg, rx_msg;
    int hw_id;
    struct can_node nodes[2];

    pthread_t threads[NUM_THREADS];
    pthread_attr_t attr;

    struct timeval timeout_value={TIMEOUT,0};
    struct timeval timeout_interval={TIMEOUT,0};
    struct itimerval timeout_timer={timeout_interval,timeout_value};

    
#ifdef NDEBUG
    errx(1,"This test isn't much worth if compiled "
	    "with NDEBUG !!!\nOn QNX use the o-g variant.");
#endif

    setitimer(ITIMER_REAL,&timeout_timer,0);
    signal(SIGALRM,timeout_handler);


    if(argc!=3){
	errx(1, "usage: %s <can1> <can2>",argv[0]);
    }

    /* Turn cursor off */
    if(isatty(1)){
	printf("\n\n");
	printf("\033[?25l");

	/* Save current cursor position */
	printf("\033[s");
    }
    signal(SIGINT, ctrl_c);

    /* The first part of the program is just to check that everything is
     * basically working (nodes are connected) */

    /* Open both CAN nodes for reading and writing */

    TxNode = open(argv[1], O_RDWR);
    if(TxNode<0){
	err(1, "could not open can node '%s'",argv[1]);
    }

    RxNode = open(argv[2], O_RDWR);
    if(RxNode<0){
	err(1, "could not open can node '%s'",argv[2]);
    }
    nodes[0].fd = TxNode;
    nodes[0].dev = argv[1];
    nodes[1].fd = RxNode;
    nodes[1].dev = argv[2];

    
    /* Reset the board. Which node is used for this doesn't matter */

    ret=ioctl(TxNode,IOC_RESET_BOARD);
    if(ret!=0){
	err(1, "could not reset board");
    }

    /* Check the node and board status */
    ret=ioctl(TxNode,IOC_GET_BOARD_STATUS,&i);
    assert(ret==0);

    if(i!=BS_RUNNING_OK){
	err(1,"board not running (status=0x%x)\n",(uint32_t)i);
    }

    /* Check the hardware ID */
    ret=ioctl(TxNode,IOC_GET_HW_ID,&hw_id);
    assert(ret==0);

    switch(hw_id){
	case HW_HICOCAN_UNKNOWN:
	    warnx("unknown hardware ID 0x%x",hw_id);
	case HW_HICOCAN_MPCI:
	case HW_HICOCAN_MPCI_4C:
	case HW_HICOCAN_PCI104:
	    break;
	default:
#ifndef ALLOW_UNKNOWN_HW
	    errx(1,"Invalid Hardware identifier 0x%x",hw_id);
#endif
	    break;
    }

    bitrate = BITRATE_HS;
    for(i=0;i<2;i++){
	struct can_node *node = &nodes[i];

	/* Check node type */
	ret=ioctl(node->fd,IOC_GET_CAN_TYPE,&node->type);
	assert(ret==0);

	switch(node->type){
	    case CAN_TYPE_EMPTY:
		errx(1,"CAN tranceiver not populated on %s (CAN_TYPE_EMPTY) - cannot run apitest\n",node->dev);
	    case CAN_TYPE_HS:
		break;
	    case CAN_TYPE_FT:
		bitrate = BITRATE_FT;
		break;
	    default:
#ifndef ALLOW_UNKNOWN_HW
		errx(1,"Invalid CAN_TYPE %d for node %s\n",node->type,node->dev);
#endif
		break;
	}

	if(node->type==CAN_TYPE_EMPTY){
	    /* Check the IO pin status which, on Fault Tolerant boards signals
	     * LineError (low active) */
	    ret=ioctl(node->fd,IOC_GET_IOPIN_STATUS,&val);
	    assert(ret==0);
	    assert(val==1 || val==0);
	    if(val==1) errx(1,"Fault tolerant tranceiver signals line error");
	}
    }

    /* Set baudrate for both nodes */
    ret=ioctl(TxNode,IOC_SET_BITRATE,&bitrate);
    if(ret!=0){
	err(1, "IOC_SET_BITRATE");
    }

    ret=ioctl(RxNode,IOC_SET_BITRATE,&bitrate);
    if(ret!=0){
	err(1, "IOC_SET_BITRATE");
    }

    /* Set both nodes in active state */
    ret=ioctl(TxNode,IOC_START);
    assert(ret==0);

    ret=ioctl(RxNode,IOC_START);
    assert(ret==0);

    /* Check the mode value and check that it's correct */
    ret=ioctl(TxNode,IOC_GET_MODE,&val);
    assert(ret==0);
    assert(val==CM_ACTIVE);

    ret=ioctl(RxNode,IOC_GET_MODE,&val);
    assert(ret==0);
    assert(val==CM_ACTIVE);


    /* Compose a dummy CAN message to send */
    memset(&tx_msg,0,sizeof(tx_msg));
    memset(&rx_msg,0,sizeof(tx_msg));
    tx_msg.ff = FF_EXTENDED;
    tx_msg.id = 0xaabbcc;
    tx_msg.dlc = 8;
    for(i=0;i<tx_msg.dlc;i++){
	tx_msg.data[i]=i;
    }


    /* Write the message to node 1 and receive it with node 2 */
    ret=write(TxNode,&tx_msg,sizeof(struct can_msg));
    assert(ret==sizeof(struct can_msg));


    ret=read_timeout(RxNode, &rx_msg, 1000);
    if(ret==0){	    /* Timeout */

	/* Check the Tx CAN node status */
	ret=ioctl(TxNode,IOC_GET_CAN_STATUS,&i);
	assert(ret==0);

	if(i&CS_ERROR_PASSIVE || i&CS_ERROR_BUS_OFF){
	    errx(1,"TxNode in %s state after first write - check the cable!",
		    i&CS_ERROR_BUS_OFF?"bus off":"error passive");
	}
	
	errx(1, "Failed to read the first message - check the cable!");
    } else {
	assert(ret==sizeof(struct can_msg));
    }


    /* Now, start the actual multithreaded abuse test. First initialize
     * the thread data and attributes, then start the threads. */

    pthread_attr_init(&attr);
    pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);

    /* Initialize the mutex */
    pthread_mutex_init(&stdout_mutex,NULL);

    /* We give a bit varying pause times for the threads, just to get some
     * more "randomness" */
    nodes[0].start_stop_pause=1000*1000;
    nodes[1].start_stop_pause=1000*900;

    nodes[0].status_read_pause=1000;
    nodes[1].status_read_pause=1100;

    nodes[0].row=1;
    nodes[1].row=2;

    for(i=0;i<2;i++){

	/* Start the transmit and receive thread */
	ret=pthread_create(&threads[i],&attr,tx_and_rx,&nodes[i]);
	assert(ret==0);
    }

    for(i=0;i<2;i++){

	/* Start the status read thread */
	ret=pthread_create(&threads[i+2],&attr,status_read,&nodes[i]);
	assert(ret==0);
    }

    /* Test continuous traffic before starting to give start/stop commands
     * */

    sleep(20);

    for(i=0;i<2;i++){

	/* Start the start and stop thread */
	ret=pthread_create(&threads[i+4],&attr,stop_and_start,&nodes[i]);
	assert(ret==0);
    }

    sleep(20);

    /* set the global flag running to 0, which makes the child threads
     * exit */
    running=0;

    for(i=0;i<NUM_THREADS;i++){
	ret=pthread_join(threads[i],NULL);
	assert(ret==0);
    }

    close(nodes[0].fd);
    close(nodes[1].fd);

    /* turn cursor on again */
    if(isatty(1)){
	printf("\n");
	printf("\033[?25h");
    }
    printf("\n");

    return 0;
}


/* Make a printable presentation of a CAN telegram */
char *msg2str(struct can_msg *msg, int fault_tolerant, char *buf)
{
    int i = 0;
    char *ptr=NULL;

    ptr=&buf[0]; 

    ptr+=sprintf(ptr," %04x %d %d",
	    msg->id,msg->dlc, msg->ts);
    
    for (i = 0; (i < msg->dlc) && (i<8); i++) {
	ptr+=sprintf(ptr," %02x",msg->data[i]);
    }
    if(msg->dos)ptr+=sprintf(ptr," dos!");
    else ptr+=sprintf(ptr,"     ");

    /* if this is a message from fault tolerant CAN, check the iopin status,
     * which is low active (i.e. iopin==1 -> no error) */
    if(fault_tolerant){
	if(!msg->iopin)ptr+=sprintf(ptr," LineErr");
    }
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
