/*EM_LICENSE*/
/* 
 * $Id$
 * Author: Martin Nylund
 *
 * test.c: A small program for debugging and testing the
 * hicocan-isa/pci driver.  See pingpong.c for a more straightforward example
 * how to start application development.
 * 
 */


#include <sys/types.h>
#include <stdio.h>
#include <stdlib.h>
#include <stdbool.h>
#include <sys/un.h>
#include <sys/poll.h>
#include <unistd.h>
#include <signal.h>
#include <strings.h>
#include <assert.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <errno.h>
#include <err.h>
#include <sys/ioctl.h>
#include <sys/poll.h>

#include "hico_api.h"

int verbose = 0;
int canFd = -1;

/* End of transfer message. By default this is an empty CAN message
 * (i.e. dlc=0)
 */
struct can_msg canEOF;

struct can_node{
    char *dev;
    int fd;
    int type;
};

char *msg2str(struct can_msg *msg, int num, int fault_tolerant)
{
    int i = 0;
    static char buf[100],*ptr=NULL;

    ptr=&buf[0]; 

    ptr+=sprintf(ptr,"% 8d %08x %d %d %d can=%d ts=%d",num,
	    msg->id,msg->ff,msg->rtr,msg->dlc,
	    msg->node, msg->ts);
    
    /* RTR Messages contain a DLC, but no data */
    if(!msg->rtr){
	    for (i = 0; (i < msg->dlc) && (i<8); i++) {
		ptr+=sprintf(ptr," %02x",msg->data[i]);
	    }
    }
    if(msg->dos)ptr+=sprintf(ptr," dos!");

    /* if this is a message from fault tolerant CAN, check the iopin status,
     * which is low active (i.e. iopin==1 -> no error) */
    if(fault_tolerant){
	if(!msg->iopin)ptr+=sprintf(ptr," LineErr");
    }
    return &buf[0];

}


int main(int argc, char *argv[])
{
    int opt;
    int ret;
    int repeat=1,write_pause=0;
    int inc_data=0;
    int can_type =0 ;

    char *options="ho:Om:xrMw:ib:E:kvr:z:p:";
    char *helppi=
"-h	        : print this help\n" 
"-o <can_node>   : open a can node (e.g. /dev/canx)\n"
"-m <mode>       : set mode (baudscan, passive, start, stop)\n"
"-x              : reset board\n"
"-b <bitrate>    : set bitrate (10k,20k,50k,100k,125k,250k,500k,800k,1000k)\n"
"-r              : read from the CAN node and write data to stdout\n"
"-M              : print CAN bus traffic on stdout (Monitor)\n"
"-w <write_spec> : write data to can bus where <write_spec> is\n"
"                 id.xxxxx  - id in hex + immediate data as string\n"
"		  id,xxxxx  - id in hex + immediate data as hex\n"
"		  id,       - id in hex + no data (dlc=0)\n"
"		  id,+      - id in hex + incrementing int value as data\n"
"		  id        - id in hex + write data from stdin\n"
"		  id:<file> - id in hex + write data from file <file>\n"
"		  idR[1:8]  - id in hex + rtr for given number of bytes\n"
"		  NOTE: When reading from a file or stream, last CAN \n"
"		  message will have a data lenth (dlc) of 0. This is \n"
"		  also the default 'eof' message for the read command (-r)\n"
"-i              : print CAN status info\n"
"-E <eof>        : eof message data in hex string\n"
"-z <repeat>     : set number of repeats for the next write command\n"
"-p <pause>      : set a pause between writes in milliseconds\n"
"-v <level>      : set verbosity level\n"
"\n"
"IMPORTANT: Order of the options matters! The commands/options are \n"
"executed in the same order they are given\n"
"\n";

    while( ( opt = getopt( argc, argv, options ) )!= -1 )
    {
	switch (opt) {

	case 'h':
	    printf(helppi);
	    break;


	case 'v':		// open the CAN device node
	    verbose++;
	    break;

	case 'o':		// open the CAN device node
	    canFd = open(optarg, O_RDWR);
	    if (canFd == -1)
		err(1, optarg);

	    /* Get CAN type (HS/FT/EMPTY) */
	    ret = ioctl(canFd,IOC_GET_CAN_TYPE,&can_type);
	    if(ret!=0){
		err(1,"IOC_GET_CAN_TYPE");
	    }
	    if(can_type!=CAN_TYPE_HS && can_type!=CAN_TYPE_FT){
#ifdef ALLOW_UNKNOWN_HW
		can_type=CAN_TYPE_FT;
#else 
		errx(1,"Invalid CAN type");
#endif
	    }

	    break;

	case 'O':		// close the CAN device node
	    ret = close(canFd);
	    if (ret == -1)
		warn("canFd");
	    break;


	    //It doesn't matter to which of the board's
	    //CAN nodes this command goes
	case 'x':		// reset the hicocan board. 
	    ret = ioctl(canFd, IOC_RESET_BOARD);
	    if (ret == -1)
		err(1, "IOC_RESET_BOARD");
	    break;

	case 'z':
	    if(sscanf(optarg,"%d",&repeat)!=1){
		errno=EINVAL;
		err(1,optarg);
	    }
	    break;
	case 'p':
	    if(sscanf(optarg,"%d",&write_pause)!=1){
		errno=EINVAL;
		err(1,optarg);
	    }
	    break;

	case 'E':		//set 'end of transfer' can message
	    {
		int i = 0, val = 0;
		while (sscanf((optarg + (i * 2)), "%02x", &val) == 1) {
		    canEOF.data[i] = (uint8_t) val;
		    i++;
		    if (i == 8)
			break;
		}
		canEOF.dlc = i;
	    }
	    break;

	case 'w':		// read data from stdin and write to CAN
	    inc_data=0;

	    /* A bit messy due to command line parsing ... */
	    while( repeat-- > 0){
		struct can_msg msg;
		char buf[100], c = 0;
		int i;
		FILE *dataStream=NULL;

		memset(&msg,0,sizeof(msg));

		// format: id:<data_in_hex>
		i = 0;
		ret = sscanf(optarg, "%x%c%s", &msg.id, &c, buf);

		if (ret == 3 && c == '.') {	/* '<id>.xxxxxx' ->  immediate data as string */
		    strncpy((char *) &msg.data[0], (char *) &buf[0], 8);
		    msg.dlc = strlen(buf);
		    if (msg.dlc > 8)
			msg.dlc = 8;

		} else if (ret == 3 && c == ',') {	/* '<id>,xxxxxx' -> immediate data in hex */
		    int val;                            /* or '<id>,+' -> upcounting integer value as data */
		    if(buf[0]=='+'){
			msg.data[3]=(uint8_t)(inc_data>>0);
			msg.data[2]=(uint8_t)(inc_data>>8);
			msg.data[1]=(uint8_t)(inc_data>>16);
			msg.data[0]=(uint8_t)(inc_data>>24);
			inc_data++;
			msg.dlc=4;
		    } else {
			while (sscanf(&buf[i * 2], "%02x", &val) == 1) {
			    msg.data[i] = (uint8_t) val;
			    i++;
			    if (i == 8)
				break;
			}
			msg.dlc = i;
		    }
		} else if (ret == 2 && c == ',') {	/* '<id>,' -> empty message (i.e. dlc=0) */
		    msg.dlc = 0;
		} else if (ret == 1) {	/* '<id>' -> read from stdin */
		    dataStream = stdin;
		} else if (ret == 3 && c == ':') {	/* '<id>:<filename>' open a file for reading */
		    dataStream = fopen(buf, "r");
		    if (dataStream==NULL)
			err(1, buf);
		} else if (ret == 3 && c == 'R') {	/* '<id>![1:8]' rtr for given number of bytes */
			int dlc;
			dlc=atol(buf);
			if(dlc>8 || dlc<1){
			    errx(1, "Invalid dlc for rtr message in argument '%s'", optarg);
			}
			msg.dlc = dlc;
			msg.rtr = 1;
		} else {
		    errx(1, "invalid argument '%s'", optarg);
		}
		
		/* Set the extended frame flag if the id is longer than 11
		 * bits */
                if(msg.id>0x7ff)msg.ff=1;

		if (dataStream == NULL) {
		    if (write(canFd, &msg, sizeof(struct can_msg)) == -1)
			warn("write");
		} else {

		    do {
			ret=fread(&msg.data[0],1,8,dataStream);
			if(ret<0) err(1,"dataStream");

			/* Last message will have a length of 0, which is the
			 * default 'eof' for the command */
			msg.dlc=ret;
			
			if (write(canFd, &msg, sizeof(struct can_msg)) == -1){
			    warn("write");
			    if(errno==EIO) break;  
			} else {
			    if (verbose == 1) {
				fprintf(stderr, "w");
				fflush(stderr);
			    }
			}

		    } while (ret != 0);
		    
		    if(dataStream!=stdin)fclose(dataStream);
		}

		if(write_pause){
		    usleep(1000*write_pause);
		}
	    };
	    repeat=1;
	    break;

	case 'r':		// read from CAN and write to stdout
	    {
		struct can_msg msg;
		int count=0;
		FILE *stream=stdout;
		do {
		    ret = read(canFd, &msg, sizeof(struct can_msg));
		    if (ret != sizeof(struct can_msg)) {
			warn("read");
			if(errno==EIO) break;
		    } else {
			count++;

			// stop on the 'EOF' CAN telegram
			if (msg.dlc == canEOF.dlc) {
			    if (memcmp(&canEOF.data[0], &msg.data[0],
				 msg.dlc) == 0) {
				if (verbose) {
				    fprintf(stderr, "EOF\n");
				    fflush(stderr);
				}
				fflush(stream);
				break;
			    }
			}

			fwrite(&msg.data[0],1,msg.dlc,stream);
			fflush(stream);
			if(msg.dos){
			    fprintf(stderr,"dos! ts=%d\n",msg.ts);
			}
		    }
		    if (verbose) {
			fprintf(stderr, "r");
			fflush(stderr);
		    }

		} while (1);
		fflush(stream);
		//fprintf(stderr,"received %d telegrams\n",count);

	    }
	    break;
	
	case 'M':		// read from CAN and write to stdout
	    {
		struct can_msg msg;
		int num=0;
		FILE *stream=stdout;
		do {
		    ret = read(canFd, &msg, sizeof(struct can_msg));
		    if (ret != sizeof(struct can_msg)) {
			err(1,"read");
		    } else {
			printf("%s\n",msg2str(&msg,++num, (can_type==CAN_TYPE_FT)));	
		    }
		} while (1);
		fflush(stream);
	    }
	    break;




	case 'b':		// set bitrate
	    {
		int bitrate = (int)atof(optarg);
		switch(bitrate){
		case 10:    bitrate = BITRATE_10k;   break; 
		case 20:    bitrate = BITRATE_20k;   break; 
		case 50:    bitrate = BITRATE_50k;   break; 
		case 100:   bitrate = BITRATE_100k;  break; 
		case 125:   bitrate = BITRATE_125k;  break; 
		case 250:   bitrate = BITRATE_250k;  break; 
		case 500:   bitrate = BITRATE_500k;  break; 
		case 800:   bitrate = BITRATE_800k;  break; 
		case 1000:  bitrate = BITRATE_1000k; break; 
		default: errx(1, "invalid argument '%s' for option '-%c'",optarg,opt); 
		}

		ret = ioctl(canFd, IOC_SET_BITRATE, &bitrate);
		if (ret == -1)
		    err(1, "IOC_SET_BITRATE");

	    }
	    break;

	case 'm':		// set mode
	    {
		/* make all transitions via reset mode */

		if (strcasecmp(optarg,"baudscan")==0){
		    ret=ioctl(canFd,IOC_STOP);
		    if (ret == -1) err(1, "IOC_STOP");
		    ret=ioctl(canFd,IOC_START_BAUDSCAN);
		    if (ret == -1) err(1, "IOC_START_BAUDSCAN");
		}
		else if (strcasecmp(optarg,"active")==0 || strcasecmp(optarg,"start")==0 ){
		    ret=ioctl(canFd,IOC_STOP);
		    if (ret == -1) err(1, "IOC_STOP");
		    ret=ioctl(canFd,IOC_START);
		    if (ret == -1) err(1, "IOC_START");
		}
		else if (strcasecmp(optarg,"passive")==0){
		    ret=ioctl(canFd,IOC_STOP);
		    if (ret == -1) err(1, "IOC_STOP");
		    ret=ioctl(canFd,IOC_START_PASSIVE);
		    if (ret == -1) err(1, "IOC_START_PASSIVE");
		}
		else if (strcasecmp(optarg,"reset")==0 || strcasecmp(optarg,"stop")==0 ){
		    ret=ioctl(canFd,IOC_STOP);
		    if (ret == -1) err(1, "IOC_STOP");
		} else {
		    errx(1, "invalid argument '%s' for option '-%c'",optarg,opt);
		}
	    }
	    break;
	    
#ifdef IOC_SERIAL_DBG
	case 'D':		// set serial debug output
	    {
		int val = 0;
		if (sscanf(optarg, "%d", &val) != 1) {
		    errx(1, "invalid argument '%s' for option '-%c'",optarg,opt);
		}

		ret = ioctl(canFd, IOC_SERIAL_DBG, val);
		if (ret == -1)
		    err(1, "IOC_SERIAL_DBG");

	    }
	    break;
#endif
#ifdef IOC_PRODUCTION_OK
	case 'k':		
	    {
		ret = ioctl(canFd, IOC_PRODUCTION_OK);
		if (ret == -1)
		    err(1, "IOC_PRODUCTION_OK");

	    }
	    break;
#endif

	case ':':
	case '?':
	    fprintf(stderr,"use option '-h' for help\n");
	    exit(1);
	    break;
	}
    }


    exit(0);
}				//end of main
