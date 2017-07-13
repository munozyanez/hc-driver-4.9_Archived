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
#include "canopentypes.h"

#define USE_TIMEOUT 1

char *msg2str(struct can_msg *msg);
char *msgcO2str(struct canOpen_msg *msgcO);
int read_timeout(int fd, struct can_msg *buf, unsigned int timeout);
int read_timeoutcO(int fdcO, struct canOpen_msg *bufcO, unsigned int timeoutcO);


int main(int argc, char *argv[])
{
    int ret,i,val;
    int TxNode, RxNode;
    struct can_msg tx_msg, rx_msg;
    struct canOpen_msg tx_msgOpen, rx_msgOpen;

    uint8_t msg1 = 0x81;
    uint8_t msg2 = 0x00;
    uint8_t msg3 = 0x00;
    uint8_t msg4 = 0x00;
    uint8_t msg5 = 0x00;
    uint8_t msg6 = 0x00;
    uint8_t msg7 = 0x00;
    uint8_t msg8 = 0x00;
    uint8_t msg_start[] = {msg1,msg2,msg3,msg4,msg5,msg6,msg7,msg8};


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
    val=BITRATE_1000k;
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



//    /* Write the message to the TxNode and receive it with RxNode */
    //ret=write(TxNode,&tx_msg,sizeof(struct can_msg));
    ret=write(TxNode,&tx_msg,sizeof(struct can_msg));
    if(ret!=sizeof(tx_msg)){
    err(1, "Failed to send message");
    }

#if USE_TIMEOUT
    ret=read_timeout(TxNode,&rx_msg,500);
    if(ret==0){
    errx(1,"timeout - could not read the message. Check the cable!");
    }
#else
    ret=read(TxNode,&rx_msg,sizeof(struct can_msg)                                                                                                                                                                                                                                                        );
    if(ret!=sizeof(rx_msg)){
    err(1,"read");
    }
#endif

    assert(ret==sizeof(struct can_msg));

    printf("received message %s\n",msg2str(&tx_msg));
    //printf("received message %s\n",msg2str(&rx_msg));


    /* Close the nodes after usage */
    close(TxNode);
    close(RxNode);

    return 0;
}

///* Function to make a string presentation of a CAN message */
char *msg2str(struct can_msg *msg)
{
    uint32_t quit_start = (msg->id)<<1;
    int Fnc_Code =(quit_start & 0xF00)>>8;
    int Node= ((quit_start<<4)&0xFE0)>>5;

    int i = 0;
    static char buf[100],*ptr=NULL;

    ptr=&buf[0];

    ptr+=sprintf(ptr,"%s=%08x Funcion-Code=%d Node=%d rtr=%d ts=%d data %d bytes : ",
        msg->ff?"extID":"   ID",
        msg->id,
        Fnc_Code,
        Node,
        msg->rtr,
        msg->ts,
        msg->dlc
        );

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
canOpen_msg a;
/* Funcion que utilizo para entrar los datos que quiero escribir*/
canOpen_msg value_canOpen(uint16_t dlc_data,uint16_t rtr_data,uint16_t fun_cod_data,uint16_t id_data,uint8_t msg_data[])
{
    long ret;
    canOpen_msg tx_msgOpen;
    tx_msgOpen.dlc_co = dlc_data;
    tx_msgOpen.rtr_co = rtr_data;
    tx_msgOpen.fun_cod_co = fun_cod_data;
    ret=ioctl(TxNode,IOC_SET_BITRATE,BITRATE_1000k);
    if(ret!=0)
    {
        err(1, "could not set bitrate");
    }

    ret=ioctl(RxNode,IOC_SET_B,
              tx_msgOpen.id_co= id_data);
    for( int i=0; i < 8; i++)
    {
        tx_msgOpen.data_co[i] = msg_data[i];
    }


return tx_msgOpen;

}

void write_Can (uint8_t msg_start[], can_msg tx_msg ,int tipo_esc, canOpen_msg tx_cO,uint16_t CRC_H,uint16_t CRC_L){

    //memset(&tx_msg,0,sizeof(struct can_msg));
    switch( tipo_esc)
    {
    case 0: //msg de red

        tx_msg.ff = FF_NORMAL;
        tx_msg.id = 0x00;
        tx_msg.dlc = 2;
        memcpy(tx_msg.data, msg_start, 2*sizeof(uint8_t));
        for( int i=0; i < tx_msg.dlc; i++){
            tx_msg.data[i] = msg_start[i];

        }
     case 1:
        tx_msg.ff = FF_NORMAL;
        tx_msg.id = ((0x00 << 4) | tx_cO.fun_cod_co) << 7 | tx_cO.id_co;
        tx_msg.rtr = tx_cO.rtr_co;
        tx_msg.dlc = tx_cO.dlc_co;
        memcpy(tx_msg.data, msg_start, tx_msg.dlc*sizeof(uint8_t));
        for( int i=0; i < tx_msg.dlc; i++){
            tx_msg.data[i] = tx_cO.data_co[i];

        }
    }
}


//int main(int argc, char *argv[])
//{
//    int ret,i,val;
//    int TxNode, RxNode;
//    struct can_msg tx_msg, rx_msg;

//    if(argc!=3){
//	errx(1, "usage: %s <can1> <can2>",argv[0]);
//    }

//    /* Open both CAN nodes for reading and writing */
//    TxNode = open(argv[1], O_RDWR);
//    if(TxNode<0){
//    err(1, "could not open can node '%s'",argv[1]);
//    }

//    RxNode = open(argv[2], O_RDWR);
//    if(RxNode<0){
//    err(1, "could not open can node '%s'",argv[2]);
//    }

    
//    /* Reset the board. Which node is used for this doesn't matter */
//    ret=ioctl(TxNode,IOC_RESET_BOARD);
//    if(ret!=0){
//    err(1, "could not reset board");
//    }


//    /* Set baudrate for both nodes */
//    val=BITRATE_1000k;
//    ret=ioctl(TxNode,IOC_SET_BITRATE,&val);
//    if(ret!=0){
//    err(1, "could not set bitrate");
//    }

//    ret=ioctl(RxNode,IOC_SET_BITRATE,&val);
//    if(ret!=0){
//    err(1, "could not set bitrate");
//    }

    
//    /* Start both CAN nodes */
//    ret=ioctl(TxNode,IOC_START);
//    if(ret!=0){
//    err(1, "IOC_START");
//    }

//    ret=ioctl(RxNode,IOC_START);
//    if(ret!=0){
//    err(1, "IOC_START");
//    }

//    memset(&tx_msg,0,sizeof(struct can_msg));
//    memset(&rx_msg,0,sizeof(struct can_msg));

//        uint8_t msg_start[] = {0x81,0x09};// en amensajes de red primer byte el comando y segundo el ID
//        //uint8_t msg_stop[] = {0x81,0x00};//Ver que mandar
//        /* Compose a CAN message with some dummy data */
//        tx_msg.ff = FF_NORMAL;
//        tx_msg.id = 0x00;//x0B; 00 mensaje de red
//        //tx_msg.id = 0x08;
//        tx_msg.dlc = 2;
//        memcpy(tx_msg.data, msg_start, 2*sizeof(uint8_t));
//        for(i=0;i<tx_msg.dlc;i++){
//        tx_msg.data[i]=msg_start[i];
//        }


////    /* Write the message to the TxNode and receive it with RxNode */
//    ret=write(TxNode,&tx_msg,sizeof(struct can_msg));
//    ret=write(TxNode,&tx_msg,sizeof(struct can_msg));
//    if(ret!=sizeof(tx_msg)){
//    err(1, "Failed to send message");
//    }

//#if USE_TIMEOUT
//    ret=read_timeout(TxNode,&rx_msg,500);
//    if(ret==0){
//    errx(1,"timeout - could not read the message. Check the cable!");
//    }
//#else
//    ret=read(TxNode,&rx_msg,sizeof(struct can_msg)                                                                                                                                                                                                                                                        );
//    if(ret!=sizeof(rx_msg)){
//    err(1,"read");
//    }
//#endif

//    assert(ret==sizeof(struct can_msg));

//    printf("received message %s\n",msg2str(&rx_msg));
//    //printf("received message %s\n",msgCanOp(&rx_msg));


//    /* Close the nodes after usage */
//    close(TxNode);
//    close(RxNode);

//    return 0;
//}

/////* Function can->CanOpen*/



/////* Function to make a string presentation of a CAN message */
//char *msg2str(struct can_msg *msg)
//{
//    uint32_t quit_start = (msg->id)<<1;
//    int Fnc_Code =(quit_start & 0xF00)>>8;
//    int Node= ((quit_start<<4)&0xFE0)>>5;

//    int i = 0;
//    static char buf[100],*ptr=NULL;

//    ptr=&buf[0];

//    ptr+=sprintf(ptr,"%s=%08x Funcion-Code=%d Node=%d rtr=%d ts=%d data %d bytes : ",
//        msg->ff?"extID":"   ID",
//        msg->id,
//        Fnc_Code,
//        Node,
//        msg->rtr,
//        msg->ts,
//        msg->dlc
//        );

//    for (i = 0; (i < msg->dlc) && (i<8); i++) {
//    ptr+=sprintf(ptr,"%02x ",msg->data[i]);
//    }
//    if(msg->dos)ptr+=sprintf(ptr,"dos!");
//    return &buf[0];

//}



///* Read a message from the driver. If the Rx buffer is empty - wait for the
// * time given by timeout argument for Rx data.
// *
// * parameters:
// *     fd - file descriptor of a CAN node
// *     buf - CAN message where the data is to be written
// *     timeout - timeout for the read operation in milliseconds
// *
// *  return value:
// *     < 0  -  Error value
// *     0    -  timeout occured
// *     > 0  -  number of read bytes (i.e. sizeof(struct can_msg)
// */
//int read_timeout(int fd, struct can_msg *buf, unsigned int timeout)
//{
//    fd_set fds;
//    struct timeval tv;
//    int sec,ret;
//    FD_ZERO(&fds);

//    sec=timeout/1000;
//    tv.tv_sec=sec;
//    tv.tv_usec=(timeout-(sec*1000))*1000;

//    FD_SET(fd,&fds);

//    ret=select(fd+1,&fds,0,0,&tv);
//    if(ret==0){
//	return 0; /* timeout */
//    } else if (ret<0) {
//	return errno;
//    } else {
//	assert(FD_ISSET(fd,&fds));
//	ret=read(fd,buf,sizeof(struct can_msg));
//	return ret;
//    }
//}


