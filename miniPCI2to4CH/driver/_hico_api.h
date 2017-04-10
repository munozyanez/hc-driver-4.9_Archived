#ifndef _PROD_HICO_API_H
#define _PROD_HICO_API_H

#include "hico_api.h"


/* unsupported ioctl calls and defines which are not part of the
 * HiCO.CAN-MiniPCI API. These are used for debugging and production */

#define IOC_SERIAL_DBG      _IOW     (IOC_MAGIC, 100, uint32_t)
/* PRODUCTION_OK (101) moved to the main api header */

#define IOC_LATTE_INIT      _IOW     (IOC_MAGIC, 102, int)
#define IOC_LATTE_SAMPLE    _IOR     (IOC_MAGIC, 103, struct latte_sample)
#define IOC_LATTE_INITIALIZED    _IOR     (IOC_MAGIC, 104, int)
#define LATTE_TIMEOUT 0xffffffff
#define LATTE_MODE 0xae
struct latte_sample{
    uint32_t t0,t1,t2;
};
    
    


/* Some handy debugging macros */
#define emX(var) do{printk("DBG %s:%d: ",__FUNCTION__,__LINE__);printk("%s=0x%08x\n",#var,(uint32_t)var);}while(0);
#define emI(var) do{printk("DBG %s:%d: ",__FUNCTION__,__LINE__);printk("%s=%d\n",#var,(int)var);}while(0);
#define emS(var) do{printk("DBG %s:%d: ",__FUNCTION__,__LINE__);printk("%s=%s\n",#var,var);}while(0);
#define emC(condition) do{if(condition){printk("DBG %s:%d: ",__FUNCTION__,__LINE__);printk("(%s)=%d\n",#condition,(condition));}}while(0);

#define _X(var) printk(" %s=%x",#var,var);
#define _I(var) printk(" %s=%i",#var,var);
#define _S(var) printk(" %s=%s",#var,var);

#endif
