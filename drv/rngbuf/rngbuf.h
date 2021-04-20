/******************************************************************************
**
**                      
**
*******************************************************************************
** File Name      : RngBuf.h
** Version        : v1.0
** Author         : wzq
** Created        : 2014.9.25
** Modification   : Created file
** Description    :  
******************************************************************************/

#ifndef __RNGBUF_H__
#define __RNGBUF_H__

#ifdef __cplusplus
    #if __cplusplus
extern "C"{
    #endif
#endif /* __cplusplus */

/* Header File */


/* Macro Define */


/* Global variable and struct/enum/union define */

typedef struct RNGBUF_s     
{
    int pWritePos;    
    int pReadPos; 
    int bufSize;    
    char * buf;
}RNGBUF_t;

/* Function List*/

extern int rng_is_empty (RNGBUF_t * pBuf);
extern int rng_is_full (RNGBUF_t * pBuf);
extern RNGBUF_t * rng_create (int nbytes);
extern int rng_get_buf (RNGBUF_t * pBuf, char *buffer, int maxbytes);
extern int rng_put_buf (RNGBUF_t * pBuf, char *buffer, int nbytes);
extern int rng_get_free_bytes (RNGBUF_t * pBuf);
extern int rng_get_used_bytes (RNGBUF_t * pBuf);
extern void rng_delete (RNGBUF_t * pBuf);
extern void rng_flush (RNGBUF_t * pBuf);
extern void rng_move_ahead (RNGBUF_t * pBuf, int n);
extern void rng_put_ahead (RNGBUF_t * pBuf, char byte, int offset);


#ifdef __cplusplus
    #if __cplusplus
}
    #endif
#endif /* __cplusplus */

#endif /* __RNGBUF_H__ */
