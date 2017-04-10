
#include "dpm.h"

#ifdef __HICO_FW__
#include "globals.h"
#elif __KERNEL__
#include <linux/kernel.h>
#include <asm/io.h>
#endif


/* Rest of the buffer handling functions are written as macros in dpm.h */

#ifdef __HICO_FW__

#define CHECK_POSITION if(wptr>size || rptr > size) except(wptr,rptr,size);

#else
 #define CHECK_POSITION if(wptr>size || rptr > size) printk(KERN_ERR "%s: wptr=%d rptr=%d size=%d\n",__FUNCTION__,wptr,rptr,size);
 #define BARRIER()
#endif


inline int __buf_is_full(int wptr, int rptr, int size)
{

    /* read pointer has just wrapped and is at the start. In this case the
     * buffer is full if write pointer is at the end  */
    if( rptr == 0 && wptr == (size-1) ) 
	return 1;

    /* The 'normal' case. write pointer is one unit behind read pointer */
    if( (rptr-wptr)==1 ) return 1;
    
    /* Otherwise, there's space left in the buffer */
    return 0;
}

inline int __buf_is_empty(int wptr, int rptr, int size)
{

    if(wptr==rptr) return 1;

    return 0;
}

int buf_is_full(struct buffer *buf)
{
    int wptr,rptr,size;
#ifdef __KERNEL__
    wptr=ioread16(&buf->vars->wptr);
    rptr=ioread16(&buf->vars->rptr);
    size=ioread16(&buf->vars->size);
#else
    wptr=buf->vars->wptr;
    rptr=buf->vars->rptr;
    size=buf->vars->size;
#endif

    CHECK_POSITION

    return __buf_is_full(wptr,rptr,size);
}

int buf_is_empty(struct buffer *buf)
{
    int wptr,rptr,size;
#ifdef __KERNEL__
    wptr=ioread16(&buf->vars->wptr);
    rptr=ioread16(&buf->vars->rptr);
    size=ioread16(&buf->vars->size);
#else
    wptr=buf->vars->wptr;
    rptr=buf->vars->rptr;
    size=buf->vars->size;
#endif

    CHECK_POSITION

    return __buf_is_empty(wptr,rptr,size);
}

int buf_real_size(struct buffer *buf)
{
#ifdef __KERNEL__
    return ioread16(&buf->vars->size)-1;
#else
    return buf->vars->size-1;
#endif
}


int buf_message_cnt(struct buffer *buf)
{
    int wptr,rptr,size;
#ifdef __KERNEL__
    wptr=ioread16(&buf->vars->wptr);
    rptr=ioread16(&buf->vars->rptr);
    size=ioread16(&buf->vars->size);
#else
    wptr=buf->vars->wptr;
    rptr=buf->vars->rptr;
    size=buf->vars->size;
#endif

    CHECK_POSITION

    if(__buf_is_full(wptr,rptr,size)){
	return size-1;
    } else if (wptr==rptr){ // if empty
	return 0;
    } else if (rptr < wptr){
	return wptr - rptr;
    } else {
	return size - (rptr - wptr);
    }
}



int buf_increment_wptr(struct buffer *buf)
{
    int wptr,rptr,size;
#ifdef __KERNEL__
    wptr=ioread16(&buf->vars->wptr);
    rptr=ioread16(&buf->vars->rptr);
    size=ioread16(&buf->vars->size);
#else
    wptr=buf->vars->wptr;
    rptr=buf->vars->rptr;
    size=buf->vars->size;
#endif

    CHECK_POSITION

    /* This function should never be called if the buffer is full */
    if(__buf_is_full(wptr,rptr,size)){
#ifdef __HICO_FW__
        except(__buf_is_full(wptr,rptr,size),rptr,wptr);
#else
	printk(KERN_ERR "%s: Buf is already full wptr=%d rptr=%d\n",__FUNCTION__,wptr,rptr);
	return -1;
#endif
    }

    if(wptr==(size-1)){
	wptr=0;
    } else {
	wptr++;
    }

    /* This function should never be called if the buffer is already empty */
    if(__buf_is_empty(wptr,rptr,size)){
#ifdef __HICO_FW__
        except(__buf_is_empty(wptr,rptr,size),rptr,wptr);
#else
	printk(KERN_ERR "%s: Buf is empty after incrementing wptr!! wptr=%d rptr=%d\n",__FUNCTION__,wptr,rptr);
	return -1;
#endif
    }

#ifdef __KERNEL__
    iowrite16((uint16_t)wptr,&buf->vars->wptr);
#else
    DPM_WRITE(&buf->vars->wptr, wptr);
#endif

    CHECK_POSITION

    return 0;
}

int buf_increment_rptr(struct buffer *buf)
{
    int wptr,rptr,size;
#ifdef __KERNEL__
    wptr=ioread16(&buf->vars->wptr);
    rptr=ioread16(&buf->vars->rptr);
    size=ioread16(&buf->vars->size);
#else
    wptr=buf->vars->wptr;
    rptr=buf->vars->rptr;
    size=buf->vars->size;
#endif

    CHECK_POSITION

    /* This function should never be called if the buffer is already empty */
    if(__buf_is_empty(wptr,rptr,size)){
#ifdef __HICO_FW__
        except(__buf_is_empty(wptr,rptr,size),rptr,wptr);
#else
	printk(KERN_ERR "%s: Buf is empty wptr=%d rptr=%d\n",__FUNCTION__,wptr,rptr);
	return -1;
#endif
	
    }

    if(rptr==(size-1)){
	rptr=0;
    } else {
	rptr++;
    }

    /* This function should never be called if the buffer is already empty */
    if(__buf_is_full(wptr,rptr,size)){
#ifdef __HICO_FW__
        except(__buf_is_full(wptr,rptr,size),rptr,wptr);
#else
	printk(KERN_ERR "%s: Buf is full after incrementing rptr!! wptr=%d rptr=%d\n",__FUNCTION__,wptr,rptr);
	return -1;
#endif
    }

#ifdef __KERNEL__
    iowrite16((uint16_t)rptr,&buf->vars->rptr);
#else
    DPM_WRITE(&buf->vars->rptr, rptr);
#endif
    
    return 0;
}
