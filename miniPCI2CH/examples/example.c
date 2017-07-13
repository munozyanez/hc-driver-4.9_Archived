/* A simple example how to initialize and use the HiCO.CAN driver */

#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <err.h>
#include <assert.h>
#include "hico_api.h"

#define USE_TIMEOUT 1

char *msg2str(struct can_msg *msg);
int read_timeout(int fd, struct can_msg *buf, unsigned int timeout);

int main(int argc, char *argv[])
{
    int ret,i,val;
    int TxNode, RxNode;
    struct can_msg tx_msg, rx_msg;

    if(argc!=3){
	errx(1, "usage: %s <can1> <can2>",argv[0]);
    }

    /* Open both CAN nodes for reading and writing */
    TxNode = open(argv[1], O_RDWR);
    if(TxNode<0){
	err(1, "could not open can node '%s'",argv[1]);
    }

    RxNode = open(argv[2], O_RDWR);
    if(RxNode<0){
	err(1, "could not open can node '%s'",argv[2]);
    }

    
    /* Reset the board. Which node is used for this doesn't matter */
    ret=ioctl(TxNode,IOC_RESET_BOARD);
    if(ret!=0){
	err(1, "could not reset board");
    }


    /* Set baudrate for both nodes */
    val=BITRATE_500k;
    ret=ioctl(TxNode,IOC_SET_BITRATE,&val);
    if(ret!=0){
	err(1, "could not set bitrate");
    }

    ret=ioctl(RxNode,IOC_SET_BITRATE,&val);
    if(ret!=0){
	err(1, "could not set bitrate");
    }

    
    /* Start both CAN nodes */
    ret=ioctl(TxNode,IOC_START);
    if(ret!=0){
	err(1, "IOC_START");
    }

    ret=ioctl(RxNode,IOC_START);
    if(ret!=0){
	err(1, "IOC_START");
    }

    memset(&tx_msg,0,sizeof(struct can_msg));
    memset(&rx_msg,0,sizeof(struct can_msg));

    /* Compose a CAN message with some dummy data */
    tx_msg.ff = FF_NORMAL;
    tx_msg.id = 0xab;
    tx_msg.dlc = 8;
    for(i=0;i<tx_msg.dlc;i++){
	tx_msg.data[i]=i;
    }


    /* Write the message to the TxNode and receive it with RxNode */
    ret=write(TxNode,&tx_msg,sizeof(struct can_msg));
    if(ret!=sizeof(tx_msg)){
	err(1, "Failed to send message");
    }

#if USE_TIMEOUT
    ret=read_timeout(RxNode,&rx_msg,500);
    if(ret==0){
	errx(1,"timeout - could not read the message. Check the cable!");
    }
#else
    ret=read(RxNode,&rx_msg,sizeof(struct can_msg));
    if(ret!=sizeof(rx_msg)){
	err(1,"read");
    }
#endif

    assert(ret==sizeof(struct can_msg));

    printf("received message %s\n",msg2str(&rx_msg));

    /* Close the nodes after usage */
    close(TxNode);
    close(RxNode);

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
